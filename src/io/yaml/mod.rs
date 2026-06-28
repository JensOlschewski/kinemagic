use serde::de::DeserializeOwned;

pub mod hardpoints;

pub use hardpoints::YamlHardpoints;

pub fn read_yaml_str<T>(yaml: &str) -> Result<T, serde_yaml_ng::Error>
where
    T: DeserializeOwned,
{
    Ok(serde_yaml_ng::from_str::<T>(yaml)?)
}
