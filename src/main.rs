mod cli;
mod io;

use clap::Parser;
use cli::{Cli, Commands};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    match Cli::parse().command {
        Commands::Check(args) => cli::check::run_check(args),
        Commands::Solve(args) => cli::solve::run_solve(args),
    }
}
