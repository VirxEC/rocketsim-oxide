#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord)]
pub(crate) enum UserInfoTypes {
    #[default]
    None,
    Car,
    Ball,
    DropshotTile,
}