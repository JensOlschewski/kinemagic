use crate::cli::CheckArgs;

use kinemagic::io::load_and_prepare;

pub fn run(args: CheckArgs) -> Result<(), Box<dyn std::error::Error>> {
    load_and_prepare(&args.input)?;

    println!("valid: {}", args.input.display());

    Ok(())
}
