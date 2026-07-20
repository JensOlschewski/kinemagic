use ::serde::Deserialize;

use crate::model::Side;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum YamlSide {
    Left,
    Right,
    Single,
}

impl From<YamlSide> for Side {
    fn from(value: YamlSide) -> Self {
        match value {
            YamlSide::Left => Self::Left,
            YamlSide::Right => Self::Right,
            YamlSide::Single => Self::Single,
        }
    }
}
