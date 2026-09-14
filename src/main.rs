use std::process::ExitCode;

use anyhow::Result;
use clap::Parser;

use cli::{Cli, Commands};

mod cli;

fn main() -> ExitCode {
    match run() {
        Ok(()) => ExitCode::SUCCESS,
        Err(error) => {
            eprintln!("Error: {error:#}");
            ExitCode::FAILURE
        }
    }
}

fn run() -> Result<()> {
    match Cli::parse().command {
        Commands::Check(args) => crate::cli::check::run(args),
        Commands::Solve(args) => crate::cli::solve::run(args),
    }
}
