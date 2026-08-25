use std::path::{Path, PathBuf};
use thiserror::Error;

use crate::io::yaml::{YamlError, parse_yaml_file};
use crate::problem::{self, PrepareError, PreparedProblem};

pub mod text;
pub mod yaml;

pub fn load_and_prepare(path: impl AsRef<Path>) -> Result<PreparedProblem, LoadPrepareError> {
    let path = path.as_ref().to_owned();
    let input = parse_yaml_file(&path)
        .map_err(|source| LoadPrepareError::Load {
            path: path.clone(),
            source,
        })?
        .into_input()
        .map_err(|source| LoadPrepareError::Load {
            path: path.clone(),
            source,
        })?;

    problem::prepare(input).map_err(|source| LoadPrepareError::Prepare { path, source })
}

#[derive(Debug, Error)]
#[non_exhaustive]
pub enum LoadPrepareError {
    #[error("failed to load `{path}`: {source}")]
    Load {
        path: PathBuf,
        #[source]
        source: YamlError,
    },
    #[error("failed to prepare `{path}`: {source}")]
    Prepare {
        path: PathBuf,
        #[source]
        source: PrepareError,
    },
}
