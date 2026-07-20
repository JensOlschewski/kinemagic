use std::error::Error;

use crate::cli::CheckArgs;
use crate::io::render::bodies::render_bodies_pretty;
use crate::io::render::joints::render_joints_pretty;
use crate::io::yaml::{YamlModel, read_yaml_str};
use crate::io::{read_input, write_output};
use crate::model::{Bodies, Joints};

pub fn run_check(args: CheckArgs) -> Result<(), Box<dyn Error>> {
    // Destructuring
    let CheckArgs {
        io,
        input_format: _,
    } = args;

    // yaml parsing layer
    // Other filetypes later possible
    let yaml = read_input(&io.input)?;
    let input: YamlModel = read_yaml_str(&yaml)?;
    let (hardpoints, bodies_spec, joints_spec) = input.into_model_parts()?;

    // Building bodies from the input
    let bodies = Bodies::build(&bodies_spec, &hardpoints)?;
    let body_rendered = render_bodies_pretty(&bodies);

    // Building joints from the input
    let joints = Joints::build(&joints_spec, &bodies)?;
    let joints_rendered = render_joints_pretty(&joints, &bodies);

    // Rendering the output
    let rendered = format!("{}/\n{}", &body_rendered, &joints_rendered);
    write_output(io.output.as_deref(), &rendered)?;

    Ok(())
}
