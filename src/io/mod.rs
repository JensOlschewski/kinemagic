use std::path::{Path, PathBuf};

use thiserror::Error;

use crate::io::yaml::{YamlError, parse_yaml_file};
use crate::model::Input;

pub mod text;
pub mod yaml;

pub fn load_file(path: &Path) -> Result<Input, LoadInputError> {
    let yaml = parse_yaml_file(path).map_err(|source| LoadInputError::Load {
        path: path.to_owned(),
        source,
    })?;

    yaml.into_input().map_err(|source| LoadInputError::Load {
        path: path.to_owned(),
        source,
    })
}

#[derive(Debug, Error)]
#[non_exhaustive]
pub enum LoadInputError {
    #[error("failed to load `{path}`")]
    Load {
        path: PathBuf,
        #[source]
        source: YamlError,
    },
}
