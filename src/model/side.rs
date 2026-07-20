#[derive(Debug, Clone, Copy)]
pub enum Side {
    Left,
    Right,
    Single,
}

impl Side {
    pub fn label(&self) -> &'static str {
        match self {
            Self::Left => "left",
            Self::Right => "right",
            Self::Single => "single",
        }
    }

    pub fn label_short(&self) -> &'static str {
        match self {
            Self::Left => "l",
            Self::Right => "r",
            Self::Single => "s",
        }
    }
}
