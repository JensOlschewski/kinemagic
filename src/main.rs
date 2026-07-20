use clap::Parser;
use mbs_solver::cli::{Cli, Commands};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    match Cli::parse().command {
        Commands::Check(args) => mbs_solver::cli::check::run_check(args),
        Commands::Solve(args) => mbs_solver::cli::solve::run_solve(args),
    }
}
