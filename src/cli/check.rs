use std::error::Error;

use crate::cli::{CheckArgs};
use crate::io::yaml::{YamlHardpoints, read_yaml_str};
use crate::io::{read_input, write_output};

pub fn run_check(args: CheckArgs) -> Result<(), Box<dyn Error>> {

    // Destructuring
    let CheckArgs {
        io,
        input_format,
    } = args;

    // yaml case
    // other cases later possible
    let yaml = read_input(&io.input)?;
    let hardpoints_input: YamlHardpoints = read_yaml_str(&yaml)?;

    let rendered = format!(
        "Checking model: {:?}, format: {:?}\n\n{hardpoints_input:#?}",
        io.input, input_format
    );


    write_output(io.output.as_deref(), &rendered)?;

    Ok(())
}
