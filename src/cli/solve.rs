use std::error::Error;

use crate::cli::SolveArgs;
use crate::io::render::bodies::render_bodies_pretty;
use crate::io::yaml::{YamlModel, read_yaml_str};
use crate::io::{read_input, write_output};
use crate::model::Bodies;

pub fn run_solve(args: SolveArgs) -> Result<(), Box<dyn Error>> {
    // Destructuring
    let SolveArgs { io, format: _ } = args;

    // yaml case
    // other cases later possible
    let yaml = read_input(&io.input)?;
    let input: YamlModel = read_yaml_str(&yaml)?;
    let (hardpoints, body_specs) = input.into_model_parts();
    let bodies = Bodies::build(&body_specs, &hardpoints)?;
    let rendered = render_bodies_pretty(&bodies);

    write_output(io.output.as_deref(), &rendered)?;

    Ok(())
}
