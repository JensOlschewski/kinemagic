use crate::cli::SolveArgs;

use super::read_input;

pub fn run(args: SolveArgs) -> Result<(), Box<dyn std::error::Error>> {
    let input = args.input;

    let _yaml_string = read_input(&input)?;

    println!("solve dispatched: {}", input.display());

    Ok(())
}
