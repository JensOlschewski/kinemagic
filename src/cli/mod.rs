use std::{fs, io, path::PathBuf};

use clap::{Args, Parser, Subcommand};

pub mod check;
pub mod solve;

#[derive(Parser)]
#[command(version, about, long_about = None)]
pub struct Cli {
    #[command(subcommand)]
    pub command: Commands,
}

#[derive(Subcommand)]
pub enum Commands {
    /// Check the model
    Check(CheckArgs),
    /// Solve the model
    Solve(SolveArgs),
}

#[derive(Args)]
pub struct CheckArgs {
    input: PathBuf,
}

#[derive(Args)]
pub struct SolveArgs {
    input: PathBuf,
}

fn read_input(input_path: &PathBuf) -> io::Result<String> {
    fs::read_to_string(input_path)
}

#[cfg(test)]
mod tests {
    use super::*;
    use clap::Parser;

    #[test]
    fn parses_check_command() {
        let cli = Cli::try_parse_from(["km", "check", "model.yaml"]).unwrap();

        assert!(matches!(cli.command, Commands::Check(_)));
    }

    #[test]
    fn parses_solve_command() {
        let cli = Cli::try_parse_from(["km", "solve", "model.yaml"]).unwrap();

        assert!(matches!(cli.command, Commands::Solve(_)));
    }

    #[test]
    fn rejects_missing_subcommand() {
        assert!(Cli::try_parse_from(["km"]).is_err());
    }

    #[test]
    fn rejects_missing_input() {
        assert!(Cli::try_parse_from(["km", "check"]).is_err());
    }

    #[test]
    fn rejects_unknown_command() {
        assert!(Cli::try_parse_from(["km", "unknown", "model.yaml"]).is_err());
    }

    #[test]
    fn check_fails_for_missing_file() {
        let result = check::run(CheckArgs {
            input: "missing.yaml".into(),
        });

        assert!(result.is_err());
    }

    #[test]
    fn solve_fails_for_missing_file() {
        let result = solve::run(SolveArgs {
            input: "missing.yaml".into(),
        });

        assert!(result.is_err());
    }

    #[test]
    fn check_accepts_readable_file() {
        let result = check::run(CheckArgs {
            input: "tests/fixtures/spherical_one_body_parse.yaml".into(),
        });

        assert!(result.is_ok());
    }
}
