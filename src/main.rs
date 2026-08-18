mod cli;

use clap::Parser;
use std::error::Error;

use cli::{Cli, Commands};

fn main() -> Result<(), Box<dyn Error>> {
    match Cli::parse().command {
        Commands::Check(args) => crate::cli::check::run(args),
        Commands::Solve(args) => crate::cli::solve::run(args),
    }
}
