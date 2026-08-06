use clap::Parser;
use kinemagic::cli::{Cli, Commands};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    match Cli::parse().command {
        Commands::Check(args) => kinemagic::cli::check::run_check(args),
        Commands::Solve(args) => kinemagic::cli::solve::run_solve(args),
    }
}
