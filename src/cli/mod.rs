pub mod check;
pub mod solve;

use std::path::{PathBuf};

use clap::{Args, Parser, Subcommand, ValueEnum};

#[derive(Parser)]
#[command(about, version)] //comes from Cargo.toml
pub struct Cli {
    #[command(subcommand)]
    pub command: Commands,
}

#[derive(Subcommand)]
pub enum Commands {
    /// Check the model for errors
    Check(CheckArgs),

    /// Solve the model and output the results
    Solve(SolveArgs),
}

#[derive(Args)]
pub struct IOArgs {
    // required input file path
    #[arg(value_name = "INPUT", help = "Input file path")]
    pub input: PathBuf,
    // optional output file path, defaults to stdout
    #[arg(long, short, value_name = "OUTPUT", help = "Output file path, defaults to stdout")]
    pub output: Option<PathBuf>,
}

#[derive(Args)]
pub struct CheckArgs {
    #[command(flatten)]
    pub io: IOArgs,
    #[arg(short, long, value_enum, default_value_t = InputFormat::Yaml)]
    pub input_format: InputFormat,
}

#[derive(Args)]
pub struct SolveArgs {
    #[command(flatten)]
    pub io: IOArgs,
    #[arg(short, long, value_enum, default_value = "pretty")]
    pub format: OutputFormat,
}

#[derive(Debug, Clone, ValueEnum)]
pub enum InputFormat {
    Yaml
}

#[derive(Clone, Debug, ValueEnum)]
pub enum OutputFormat {
    Hdf5,
    Pretty,
}
