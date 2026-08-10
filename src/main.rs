use clap::Parser;
use cli::{Cli, Commands};
use std::error::Error;

pub mod cli;

fn main() -> Result<(), Box<dyn Error>> {
    match Cli::parse().command {
        Commands::Check(args) => crate::cli::check::run(args),
        Commands::Solve(args) => crate::cli::solve::run(args),
    }
}
