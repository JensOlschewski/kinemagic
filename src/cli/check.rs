use crate::cli::CheckArgs;
use anyhow::{Context, Result};

use kinemagic::io::load_file;
use kinemagic::problem::prepare;

pub fn run(args: CheckArgs) -> Result<()> {
    let input = load_file(&args.input)?;

    let _problem =
        prepare(input).with_context(|| format!("failed to prepare `{}`", args.input.display()))?;

    println!("valid: {}", args.input.display());
    Ok(())
}
