use std::error::Error;

use crate::cli::SolveArgs;
use crate::io::yaml::{YamlHardpoints, read_yaml_str};
use crate::io::{read_input, write_output};

pub fn run_solve(args: SolveArgs) -> Result<(), Box<dyn Error>> {
    // Destructuring
    let SolveArgs { io, format } = args;

    // yaml case
    // other cases later possible
    let yaml = read_input(&io.input)?;
    let hardpoints_input: YamlHardpoints = read_yaml_str(&yaml)?;

    let rendered = format!("Solving model: {:?}, format: {:?}, \n\n{hardpoints_input:#?}", io.input, format);

    write_output(io.output.as_deref(), &rendered)?;

    Ok(())
}
