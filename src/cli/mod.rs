use std::path::PathBuf;

use clap::{Args, Parser, Subcommand, ValueEnum};

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
    /// View validated reference geometry in the terminal
    #[arg(long)]
    view: bool,
}

#[derive(Args)]
pub struct SolveArgs {
    input: PathBuf,
    /// Render solve progress to stderr
    #[arg(long)]
    progress: bool,
    /// View solved frames after solving, or stream them live
    #[arg(
        long,
        value_enum,
        num_args = 0..=1,
        require_equals = true,
        default_missing_value = "after",
        conflicts_with = "progress"
    )]
    view: Option<SolveViewMode>,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq, ValueEnum)]
pub enum SolveViewMode {
    After,
    Live,
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
    fn parses_check_view_command() {
        let cli = Cli::try_parse_from(["km", "check", "model.yaml", "--view"]).unwrap();

        assert!(matches!(
            cli.command,
            Commands::Check(CheckArgs { view: true, .. })
        ));
    }

    #[test]
    fn parses_solve_command() {
        let cli = Cli::try_parse_from(["km", "solve", "model.yaml"]).unwrap();

        assert!(matches!(cli.command, Commands::Solve(_)));
    }

    #[test]
    fn parses_solve_view_command() {
        let cli = Cli::try_parse_from(["km", "solve", "model.yaml", "--view"]).unwrap();

        assert!(matches!(
            cli.command,
            Commands::Solve(SolveArgs {
                view: Some(SolveViewMode::After),
                ..
            })
        ));
    }

    #[test]
    fn parses_solve_live_view_command() {
        let cli = Cli::try_parse_from(["km", "solve", "model.yaml", "--view=live"]).unwrap();

        assert!(matches!(
            cli.command,
            Commands::Solve(SolveArgs {
                view: Some(SolveViewMode::Live),
                ..
            })
        ));
    }

    #[test]
    fn parses_bare_view_before_input() {
        let cli = Cli::try_parse_from(["km", "solve", "--view", "model.yaml"]).unwrap();

        assert!(matches!(
            cli.command,
            Commands::Solve(SolveArgs {
                view: Some(SolveViewMode::After),
                ..
            })
        ));
    }

    #[test]
    fn rejects_solve_view_with_progress() {
        assert!(
            Cli::try_parse_from(["km", "solve", "model.yaml", "--view", "--progress"]).is_err()
        );
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
            view: false,
        });

        assert!(result.is_err());
    }

    #[test]
    fn solve_fails_for_missing_file() {
        let result = solve::run(SolveArgs {
            input: "missing.yaml".into(),
            progress: false,
            view: None,
        });

        assert!(result.is_err());
    }

    #[test]
    fn check_accepts_valid_no_motion_file() {
        let result = check::run(CheckArgs {
            input: "tests/fixtures/spherical_one_body_parse.yaml".into(),
            view: false,
        });

        assert!(result.is_ok());
    }

    #[test]
    fn check_accepts_valid_motion_file() {
        let result = check::run(CheckArgs {
            input: "tests/fixtures/spherical_two_body_motion.yaml".into(),
            view: false,
        });

        assert!(result.is_ok());
    }

    #[test]
    fn check_rejects_malformed_yaml() {
        let result = check::run(CheckArgs {
            input: "tests/fixtures/malformed.yaml".into(),
            view: false,
        });

        assert!(result.is_err());
    }

    #[test]
    fn check_rejects_not_solve_ready_model() {
        let result = check::run(CheckArgs {
            input: "tests/fixtures/spherical_duplicate_motion.yaml".into(),
            view: false,
        });

        assert!(result.is_err());
    }
}
