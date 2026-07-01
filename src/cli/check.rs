use std::error::Error;

use crate::cli::CheckArgs;
use crate::io::render::bodies::render_bodies_pretty;
use crate::io::yaml::{YamlModel, read_yaml_str};
use crate::io::{read_input, write_output};
use crate::model::Bodies;

pub fn run_check(args: CheckArgs) -> Result<(), Box<dyn Error>> {
    // Destructuring
    let CheckArgs {
        io,
        input_format: _,
    } = args;

    // yaml layer
    // other cases later possible
    let yaml = read_input(&io.input)?;
    let input: YamlModel = read_yaml_str(&yaml)?;
    let (hardpoints, body_specs) = input.into_model_parts();

    // Building the model from the input
    let bodies = Bodies::build(&body_specs, &hardpoints)?;
    let rendered = render_bodies_pretty(&bodies);

    write_output(io.output.as_deref(), &rendered)?;

    Ok(())
}
