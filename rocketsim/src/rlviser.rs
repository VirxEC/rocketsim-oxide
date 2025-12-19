use std::{
    hash::{Hash, Hasher},
    io,
    net::{IpAddr, SocketAddr, UdpSocket},
    str::FromStr,
    time::Duration,
};

use ahash::{AHashSet, AHasher};
use rocketsim_flat::{
    AddRender, Message, Packet, PacketRef, RemoveRender, Render,
    planus::{Builder, ReadAsRoot},
};

use crate::sim::{Arena, GameState};

/// Pass this into rlviser as the first argument
/// default: 45243
pub const RLVISER_PORT: u16 = 45243;

/// Pass this into rlviser as the second argument
/// default: 34254
pub const ROCKETSIM_PORT: u16 = 34254;

#[derive(Default)]
pub struct RenderingManager {
    renders: AHashSet<i32>,
    render_buffer: Vec<Message>,
}

impl RenderingManager {
    #[allow(clippy::cast_possible_truncation)]
    const fn reduce_hash(hash: u64) -> u32 {
        let upper = (hash >> 32) as u32;
        let lower = hash as u32;
        upper ^ lower
    }

    #[allow(clippy::cast_possible_wrap)]
    pub fn add_renders(&mut self, id: &str, commands: Vec<Render>) {
        let mut hasher = AHasher::default();
        id.hash(&mut hasher);
        let id = Self::reduce_hash(hasher.finish()) as i32;

        self.renders.insert(id);
        self.render_buffer
            .push(Message::AddRender(Box::new(AddRender { id, commands })));
    }

    #[allow(clippy::cast_possible_wrap)]
    pub fn remove_renders(&mut self, id: &str) {
        let mut hasher = AHasher::default();
        id.hash(&mut hasher);
        let id = Self::reduce_hash(hasher.finish()) as i32;

        if self.renders.remove(&id) {
            self.render_buffer
                .push(Message::RemoveRender(Box::new(RemoveRender { id })));
        }
    }

    pub fn remove_all_renders(&mut self) {
        self.render_buffer.extend(
            self.renders
                .iter()
                .map(|&id| Message::RemoveRender(Box::new(RemoveRender { id }))),
        );
        self.renders.clear();
    }
}

/// Please note that you must launch the RLViser executable yourself.
pub struct RLViser {
    socket: UdpSocket,
    rlviser_addr: SocketAddr,
    udp_buffer: Vec<u8>,
    flat_builder: Builder,
    paused: bool,
    pub rendering_manager: RenderingManager,
}

impl RLViser {
    /// Please note that you must launch the RLViser executable yourself.
    pub fn new() -> io::Result<Self> {
        let socket = UdpSocket::bind(("0.0.0.0", ROCKETSIM_PORT))?;
        let rlviser_addr = SocketAddr::new(IpAddr::from_str("127.0.0.1").unwrap(), RLVISER_PORT);

        socket.set_nonblocking(true)?;

        let mut flat_builder = Builder::with_capacity(1024);
        send_msg(
            &socket,
            rlviser_addr,
            Message::Connection(Box::default()),
            &mut flat_builder,
        )?;

        Ok(Self {
            socket,
            rlviser_addr,
            flat_builder,
            paused: false,
            udp_buffer: Vec::with_capacity(1024),
            rendering_manager: RenderingManager::default(),
        })
    }

    /// Please note that you must launch the RLViser executable yourself,
    /// passing in the port for rlviser to bind to as the first argument
    /// and `rocketsim_port` as the second argument.
    pub fn new_with_addrs(rocketsim_port: u16, rlviser_addr: SocketAddr) -> io::Result<Self> {
        let socket = UdpSocket::bind(("0.0.0.0", rocketsim_port))?;
        socket.set_nonblocking(true)?;

        let mut flat_builder = Builder::with_capacity(1024);
        send_msg(
            &socket,
            rlviser_addr,
            Message::Connection(Box::default()),
            &mut flat_builder,
        )?;

        Ok(Self {
            socket,
            rlviser_addr,
            flat_builder,
            paused: false,
            udp_buffer: Vec::with_capacity(1024),
            rendering_manager: RenderingManager::default(),
        })
    }

    #[must_use]
    pub const fn is_paused(&self) -> bool {
        self.paused
    }

    pub fn send_state(&mut self, game_state: &GameState) -> io::Result<()> {
        send_msg(
            &self.socket,
            self.rlviser_addr,
            Message::GameState(game_state.into()),
            &mut self.flat_builder,
        )?;

        Ok(())
    }

    pub fn flush_render_buffer(&mut self) -> io::Result<()> {
        if self.rendering_manager.render_buffer.is_empty() {
            return Ok(());
        }

        for render_message in &self.rendering_manager.render_buffer {
            send_msg(
                &self.socket,
                self.rlviser_addr,
                render_message.clone(),
                &mut self.flat_builder,
            )?;
        }

        self.rendering_manager.render_buffer.clear();

        Ok(())
    }

    pub fn handle_state_settings(
        &mut self,
        arena: &mut Arena,
        render_interval: &mut Duration,
    ) -> io::Result<()> {
        'outer: loop {
            self.udp_buffer.clear();
            let mut packet_size_buffer = [0; 8];

            // read a u64 indicating the size of the incoming packet
            #[cfg(windows)]
            {
                if let Err(e) = self.socket.peek_from(&mut packet_size_buffer) {
                    if let Some(code) = e.raw_os_error() {
                        if code == 10040 {
                            break;
                        }
                    }

                    break 'outer;
                }
            }

            #[cfg(not(windows))]
            {
                if self.socket.peek_from(&mut packet_size_buffer).is_err() {
                    break 'outer;
                }
            }

            let packet_size = packet_size_buffer.len()
                + usize::try_from(u64::from_be_bytes(packet_size_buffer)).unwrap();
            self.udp_buffer.resize(packet_size, 0);
            if self.socket.recv_from(&mut self.udp_buffer).is_err() {
                continue;
            }

            let Ok(packet): Result<Packet, _> =
                PacketRef::read_as_root(&self.udp_buffer[packet_size_buffer.len()..])
                    .and_then(TryInto::try_into)
            else {
                continue;
            };

            match packet.message {
                Message::Speed(speed) => {
                    *render_interval = Duration::from_secs_f32(1.0 / (120. * speed.speed));
                }
                Message::Paused(paused) => {
                    self.paused = paused.paused;
                }
                Message::GameState(game_state) => {
                    arena.set_game_state(game_state.as_ref().into());
                }
                Message::Connection(_) => {}
                Message::Quit(_) | Message::AddRender(_) | Message::RemoveRender(_) => {
                    panic!("Unexpected message from RLViser");
                }
            }
        }

        Ok(())
    }

    pub fn quit(mut self) -> io::Result<()> {
        let packet = Packet {
            message: Message::Quit(Box::default()),
        };

        self.flat_builder.clear();
        let payload = self.flat_builder.finish(packet, None);
        let data_len_bin = u64::try_from(payload.len()).unwrap().to_be_bytes();
        let bytes = [&data_len_bin[..], payload].concat();
        self.socket.send_to(&bytes, self.rlviser_addr)?;

        Ok(())
    }
}

fn send_msg(
    socket: &UdpSocket,
    rlviser_addr: SocketAddr,
    message: Message,
    flat_builder: &mut Builder,
) -> Result<(), io::Error> {
    flat_builder.clear();

    let payload = flat_builder.finish(Packet { message }, None);
    let data_len_bin = u64::try_from(payload.len()).unwrap().to_be_bytes();
    let bytes = [&data_len_bin[..], payload].concat();

    socket.send_to(&bytes, rlviser_addr)?;
    Ok(())
}
