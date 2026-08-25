use crate::cli::SolveArgs;

use kinemagic::io::load_and_prepare;

pub fn run(args: SolveArgs) -> Result<(), Box<dyn std::error::Error>> {
    load_and_prepare(&args.input)?;

    println!("solve dispatched: {}", args.input.display());

    Ok(())
}
