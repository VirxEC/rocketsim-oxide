use crate::sim;

impl From<super::GameMode> for crate::GameMode {
    fn from(value: super::GameMode) -> Self {
        match value {
            super::GameMode::Soccar => Self::Soccar,
            super::GameMode::Hoops => Self::Hoops,
            super::GameMode::Heatseeker => Self::Heatseeker,
            super::GameMode::Snowday => Self::Snowday,
            super::GameMode::Dropshot => Self::Dropshot,
            super::GameMode::TheVoid => Self::TheVoid,
        }
    }
}

impl From<crate::GameMode> for super::GameMode {
    fn from(value: crate::GameMode) -> Self {
        match value {
            crate::GameMode::Soccar => Self::Soccar,
            crate::GameMode::Hoops => Self::Hoops,
            crate::GameMode::Heatseeker => Self::Heatseeker,
            crate::GameMode::Snowday => Self::Snowday,
            crate::GameMode::Dropshot => Self::Dropshot,
            crate::GameMode::TheVoid => Self::TheVoid,
        }
    }
}

impl From<super::Team> for sim::Team {
    fn from(value: super::Team) -> Self {
        match value {
            super::Team::Blue => Self::Blue,
            super::Team::Orange => Self::Orange,
        }
    }
}

impl From<sim::Team> for super::Team {
    fn from(value: sim::Team) -> Self {
        match value {
            sim::Team::Blue => Self::Blue,
            sim::Team::Orange => Self::Orange,
        }
    }
}

impl From<sim::TileState> for super::TileState {
    fn from(value: sim::TileState) -> Self {
        match value {
            sim::TileState::Full => Self::Full,
            sim::TileState::Damaged => Self::Damaged,
            sim::TileState::Broken => Self::Broken,
        }
    }
}

impl From<super::TileState> for sim::TileState {
    fn from(value: super::TileState) -> Self {
        match value {
            super::TileState::Full => Self::Full,
            super::TileState::Damaged => Self::Damaged,
            super::TileState::Broken => Self::Broken,
        }
    }
}

impl From<super::Vec3> for glam::Vec3A {
    fn from(value: super::Vec3) -> Self {
        Self::new(value.x, value.y, value.z)
    }
}

impl From<glam::Vec3A> for super::Vec3 {
    fn from(value: glam::Vec3A) -> Self {
        Self {
            x: value.x,
            y: value.y,
            z: value.z,
        }
    }
}

impl From<&super::DropshotTile> for sim::DropshotTile {
    fn from(value: &super::DropshotTile) -> Self {
        Self {
            pos: value.pos.into(),
            state: value.state.into(),
        }
    }
}

impl From<&sim::DropshotTile> for super::DropshotTile {
    fn from(value: &sim::DropshotTile) -> Self {
        Self {
            pos: value.pos.into(),
            state: value.state.into(),
        }
    }
}

impl From<super::DropshotInfo> for sim::DropshotInfo {
    fn from(value: super::DropshotInfo) -> Self {
        Self {
            charge_level: value.charge_level,
            accumulated_hit_force: value.accumulated_hit_force,
            y_target_dir: value.y_target_dir,
            has_damaged: value.has_damaged,
            last_damage_tick: value.last_damage_tick,
        }
    }
}

impl From<sim::DropshotInfo> for super::DropshotInfo {
    fn from(value: sim::DropshotInfo) -> Self {
        Self {
            charge_level: value.charge_level,
            accumulated_hit_force: value.accumulated_hit_force,
            y_target_dir: value.y_target_dir,
            has_damaged: value.has_damaged,
            last_damage_tick: value.last_damage_tick,
        }
    }
}

impl From<super::BoostPadConfig> for sim::BoostPadConfig {
    fn from(value: super::BoostPadConfig) -> Self {
        Self {
            pos: value.pos.into(),
            is_big: value.is_big,
        }
    }
}

impl From<sim::BoostPadConfig> for super::BoostPadConfig {
    fn from(value: sim::BoostPadConfig) -> Self {
        Self {
            pos: value.pos.into(),
            is_big: value.is_big,
        }
    }
}

impl From<super::BoostPadState> for sim::BoostPadState {
    fn from(value: super::BoostPadState) -> Self {
        Self {
            is_active: value.is_active,
            cooldown: value.cooldown,
            cur_locked_car: value.cur_locked_car,
            prev_locked_car_id: value.prev_locked_car_id,
        }
    }
}

impl From<sim::BoostPadState> for super::BoostPadState {
    fn from(value: sim::BoostPadState) -> Self {
        Self {
            is_active: value.is_active,
            cooldown: value.cooldown,
            cur_locked_car: value.cur_locked_car,
            prev_locked_car_id: value.prev_locked_car_id,
        }
    }
}

impl From<&super::BoostPadInfo> for sim::BoostPadInfo {
    fn from(value: &super::BoostPadInfo) -> Self {
        Self {
            config: value.config.into(),
            state: value.state.into(),
        }
    }
}

impl From<&sim::BoostPadInfo> for super::BoostPadInfo {
    fn from(value: &sim::BoostPadInfo) -> Self {
        Self {
            config: value.config.into(),
            state: value.state.into(),
        }
    }
}

impl From<super::WheelPairConfig> for sim::WheelPairConfig {
    fn from(value: super::WheelPairConfig) -> Self {
        Self {
            wheel_radius: value.wheel_radius,
            suspension_rest_length: value.suspension_rest_length,
            connection_point_offset: value.connection_point_offset.into(),
        }
    }
}

impl From<sim::WheelPairConfig> for super::WheelPairConfig {
    fn from(value: sim::WheelPairConfig) -> Self {
        Self {
            wheel_radius: value.wheel_radius,
            suspension_rest_length: value.suspension_rest_length,
            connection_point_offset: value.connection_point_offset.into(),
        }
    }
}

impl From<super::CarConfig> for sim::CarConfig {
    fn from(value: super::CarConfig) -> Self {
        Self {
            hitbox_size: value.hitbox_size.into(),
            hitbox_pos_offset: value.hitbox_pos_offset.into(),
            front_wheels: value.front_wheels.into(),
            back_wheels: value.back_wheels.into(),
            three_wheels: value.three_wheels,
            dodge_deadzone: value.dodge_deadzone,
        }
    }
}

impl From<sim::CarConfig> for super::CarConfig {
    fn from(value: sim::CarConfig) -> Self {
        Self {
            hitbox_size: value.hitbox_size.into(),
            hitbox_pos_offset: value.hitbox_pos_offset.into(),
            front_wheels: value.front_wheels.into(),
            back_wheels: value.back_wheels.into(),
            three_wheels: value.three_wheels,
            dodge_deadzone: value.dodge_deadzone,
        }
    }
}

impl From<super::Mat3> for glam::Mat3A {
    fn from(value: super::Mat3) -> Self {
        Self {
            x_axis: value.forward.into(),
            y_axis: value.right.into(),
            z_axis: value.up.into(),
        }
    }
}

impl From<glam::Mat3A> for super::Mat3 {
    fn from(value: glam::Mat3A) -> Self {
        Self {
            forward: value.x_axis.into(),
            right: value.y_axis.into(),
            up: value.z_axis.into(),
        }
    }
}

impl From<super::PhysState> for sim::PhysState {
    fn from(value: super::PhysState) -> Self {
        Self {
            pos: value.pos.into(),
            rot_mat: value.rot_mat.into(),
            vel: value.vel.into(),
            ang_vel: value.ang_vel.into(),
        }
    }
}

impl From<sim::PhysState> for super::PhysState {
    fn from(value: sim::PhysState) -> Self {
        Self {
            pos: value.pos.into(),
            rot_mat: value.rot_mat.into(),
            vel: value.vel.into(),
            ang_vel: value.ang_vel.into(),
        }
    }
}

impl From<&super::CarContact> for sim::CarContact {
    fn from(value: &super::CarContact) -> Self {
        Self {
            other_car_id: value.other_car_id,
            cooldown_timer: value.cooldown_timer,
        }
    }
}

impl From<sim::CarContact> for Box<super::CarContact> {
    fn from(value: sim::CarContact) -> Self {
        let mut new = Self::default();
        new.other_car_id = value.other_car_id;
        new.cooldown_timer = value.cooldown_timer;

        new
    }
}

impl From<super::CarControls> for sim::CarControls {
    fn from(value: super::CarControls) -> Self {
        Self {
            throttle: value.throttle,
            steer: value.steer,
            pitch: value.pitch,
            yaw: value.yaw,
            roll: value.roll,
            jump: value.jump,
            boost: value.boost,
            handbrake: value.handbrake,
        }
    }
}

impl From<sim::CarControls> for super::CarControls {
    fn from(value: sim::CarControls) -> Self {
        Self {
            throttle: value.throttle,
            steer: value.steer,
            pitch: value.pitch,
            yaw: value.yaw,
            roll: value.roll,
            jump: value.jump,
            boost: value.boost,
            handbrake: value.handbrake,
        }
    }
}

impl From<&super::BallHitInfo> for sim::BallHitInfo {
    fn from(value: &super::BallHitInfo) -> Self {
        Self {
            relative_pos_on_ball: value.relative_pos_on_ball.into(),
            ball_pos: value.ball_pos.into(),
            extra_hit_vel: value.extra_hit_vel.into(),
            tick_count_when_hit: value.tick_count_when_hit,
            tick_count_when_extra_impulse_applied: value.tick_count_when_extra_impulse_applied,
        }
    }
}

impl From<sim::BallHitInfo> for Box<super::BallHitInfo> {
    fn from(value: sim::BallHitInfo) -> Self {
        let mut new = Self::default();
        new.relative_pos_on_ball = value.relative_pos_on_ball.into();
        new.ball_pos = value.ball_pos.into();
        new.extra_hit_vel = value.extra_hit_vel.into();
        new.tick_count_when_hit = value.tick_count_when_hit;
        new.tick_count_when_extra_impulse_applied = value.tick_count_when_extra_impulse_applied;

        new
    }
}

impl From<super::WheelsWithContact> for [bool; 4] {
    fn from(value: super::WheelsWithContact) -> Self {
        [
            value.front_left,
            value.front_right,
            value.rear_left,
            value.rear_right,
        ]
    }
}

impl From<[bool; 4]> for super::WheelsWithContact {
    fn from(value: [bool; 4]) -> Self {
        Self {
            front_left: value[0],
            front_right: value[1],
            rear_left: value[2],
            rear_right: value[3],
        }
    }
}

impl From<&super::CarState> for sim::CarState {
    fn from(value: &super::CarState) -> Self {
        Self {
            phys: value.physics.into(),
            tick_count_since_update: value.tick_count_since_update,
            is_on_ground: value.is_on_ground,
            wheels_with_contact: value.wheels_with_contact.into(),
            has_jumped: value.has_jumped,
            has_double_jumped: value.has_double_jumped,
            has_flipped: value.has_flipped,
            flip_rel_torque: value.flip_rel_torque.into(),
            jump_time: value.jump_time,
            flip_time: value.flip_time,
            is_flipping: value.is_flipping,
            is_jumping: value.is_jumping,
            air_time: value.air_time,
            air_time_since_jump: value.air_time_since_jump,
            boost: value.boost,
            time_since_boosted: value.time_since_boosted,
            is_boosting: value.is_boosting,
            boosting_time: value.boosting_time,
            is_supersonic: value.is_supersonic,
            supersonic_time: value.supersonic_time,
            handbrake_val: value.handbrake_val,
            is_auto_flipping: value.is_auto_flipping,
            auto_flip_timer: value.auto_flip_timer,
            auto_flip_torque_scale: value.auto_flip_torque_scale,
            world_contact_normal: value.world_contact_normal.map(Into::into),
            car_contact: value.car_contact.as_deref().map(Into::into),
            is_demoed: value.is_demoed,
            demo_respawn_timer: value.demo_respawn_timer,
            ball_hit_info: value.ball_hit_info.as_deref().map(Into::into),
            last_controls: value.last_controls.into(),
        }
    }
}

impl From<sim::CarState> for Box<super::CarState> {
    fn from(value: sim::CarState) -> Self {
        let mut new = Self::default();
        new.physics = value.phys.into();
        new.tick_count_since_update = value.tick_count_since_update;
        new.is_on_ground = value.is_on_ground;
        new.wheels_with_contact = value.wheels_with_contact.into();
        new.has_jumped = value.has_jumped;
        new.has_double_jumped = value.has_double_jumped;
        new.has_flipped = value.has_flipped;
        new.flip_rel_torque = value.flip_rel_torque.into();
        new.jump_time = value.jump_time;
        new.flip_time = value.flip_time;
        new.is_flipping = value.is_flipping;
        new.is_jumping = value.is_jumping;
        new.air_time = value.air_time;
        new.air_time_since_jump = value.air_time_since_jump;
        new.boost = value.boost;
        new.time_since_boosted = value.time_since_boosted;
        new.is_boosting = value.is_boosting;
        new.boosting_time = value.boosting_time;
        new.is_supersonic = value.is_supersonic;
        new.supersonic_time = value.supersonic_time;
        new.handbrake_val = value.handbrake_val;
        new.is_auto_flipping = value.is_auto_flipping;
        new.auto_flip_timer = value.auto_flip_timer;
        new.auto_flip_torque_scale = value.auto_flip_torque_scale;
        new.world_contact_normal = value.world_contact_normal.map(Into::into);
        new.car_contact = value.car_contact.map(Into::into);
        new.is_demoed = value.is_demoed;
        new.demo_respawn_timer = value.demo_respawn_timer;
        new.ball_hit_info = value.ball_hit_info.map(Into::into);
        new.last_controls = value.last_controls.into();

        new
    }
}

impl From<&super::CarInfo> for sim::CarInfo {
    fn from(value: &super::CarInfo) -> Self {
        Self {
            id: value.id,
            team: value.team.into(),
            state: value.state.as_ref().into(),
            config: value.config.into(),
        }
    }
}

impl From<&sim::CarInfo> for super::CarInfo {
    fn from(value: &sim::CarInfo) -> Self {
        Self {
            id: value.id,
            team: value.team.into(),
            state: value.state.into(),
            config: value.config.into(),
        }
    }
}

impl From<super::HeatseekerInfo> for sim::HeatseekerInfo {
    fn from(value: super::HeatseekerInfo) -> Self {
        Self {
            y_target_dir: value.y_target_dir,
            cur_target_speed: value.cur_target_speed,
            time_since_hit: value.time_since_hit,
        }
    }
}

impl From<sim::HeatseekerInfo> for super::HeatseekerInfo {
    fn from(value: sim::HeatseekerInfo) -> Self {
        Self {
            y_target_dir: value.y_target_dir,
            cur_target_speed: value.cur_target_speed,
            time_since_hit: value.time_since_hit,
        }
    }
}

impl From<super::BallState> for sim::BallState {
    fn from(value: super::BallState) -> Self {
        Self {
            phys: value.physics.into(),
            ticks_since_update: value.tick_count_since_update,
            hs_info: value.hs_info.into(),
            ds_info: value.ds_info.into(),
        }
    }
}

impl From<sim::BallState> for super::BallState {
    fn from(value: sim::BallState) -> Self {
        Self {
            physics: value.phys.into(),
            tick_count_since_update: value.ticks_since_update,
            hs_info: value.hs_info.into(),
            ds_info: value.ds_info.into(),
        }
    }
}

impl From<&super::DropshotTilesByTeam> for [Vec<sim::DropshotTile>; 2] {
    fn from(value: &super::DropshotTilesByTeam) -> Self {
        [
            value.blue_tiles.iter().map(Into::into).collect(),
            value.orange_tiles.iter().map(Into::into).collect(),
        ]
    }
}

impl From<&[Vec<sim::DropshotTile>; 2]> for Box<super::DropshotTilesByTeam> {
    fn from([blue_tiles, orange_tiles]: &[Vec<sim::DropshotTile>; 2]) -> Self {
        let mut new = Self::default();
        new.blue_tiles = blue_tiles.iter().map(Into::into).collect();
        new.orange_tiles = orange_tiles.iter().map(Into::into).collect();

        new
    }
}

impl From<&super::GameState> for sim::GameState {
    fn from(value: &super::GameState) -> Self {
        Self {
            tick_rate: value.tick_rate,
            tick_count: value.tick_count,
            game_mode: value.game_mode.into(),
            cars: value
                .cars
                .as_ref()
                .map(|cars| cars.iter().map(Into::into).collect()),
            ball: value.ball.into(),
            pads: value
                .pads
                .as_ref()
                .map(|pads| pads.iter().map(Into::into).collect()),
            tiles: value.tiles.as_deref().map(Into::into),
        }
    }
}

impl From<&sim::GameState> for Box<super::GameState> {
    fn from(value: &sim::GameState) -> Self {
        let mut new = Self::default();
        new.tick_rate = value.tick_rate;
        new.tick_count = value.tick_count;
        new.game_mode = value.game_mode.into();
        new.cars = value
            .cars
            .as_ref()
            .map(|cars| cars.iter().map(Into::into).collect());
        new.ball = value.ball.into();
        new.pads = value
            .pads
            .as_ref()
            .map(|pads| pads.iter().map(Into::into).collect());
        new.tiles = value.tiles.as_ref().map(Into::into);

        new
    }
}
