use crate::cli::CheckArgs;

use super::read_input;

pub fn run(args: CheckArgs) -> Result<(), Box<dyn std::error::Error>> {
    let input = args.input;

    let _yaml_string = read_input(&input)?;

    println!("check dispatched: {}", input.display());

    Ok(())
}
