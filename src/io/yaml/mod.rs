use serde::de::DeserializeOwned;

pub mod bodies;
pub mod hardpoints;
pub mod joints;
pub mod model;
pub mod orientation;
pub mod side;
pub mod motions;

pub use model::YamlModel;

pub fn read_yaml_str<T>(yaml: &str) -> Result<T, serde_yaml_ng::Error>
where
    T: DeserializeOwned,
{
    Ok(serde_yaml_ng::from_str::<T>(yaml)?)
}
