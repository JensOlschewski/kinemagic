use std::error::Error;

use crate::cli::SolveArgs;
use crate::io::yaml::{YamlModel, read_yaml_str};
use crate::io::{read_input, write_output};
use crate::kinematics::{forward::update_body_poses, state::KinematicState};
use crate::model::{Bodies, Joints};

pub fn run_solve(args: SolveArgs) -> Result<(), Box<dyn Error>> {
    let SolveArgs { io, format: _ } = args;

    let yaml = read_input(&io.input)?;
    let input: YamlModel = read_yaml_str(&yaml)?;
    let (hardpoints, body_specs, joint_specs) = input.into_model_parts()?;
    let bodies = Bodies::build(&body_specs, &hardpoints)?;
    let joints = Joints::build(&joint_specs, &bodies)?;
    let mut state = KinematicState::from_reference(&bodies, &joints)?;

    update_body_poses(&mut state, &joints)?;

    let rendered = crate::io::render::state::render_state_pretty(&bodies, &state)?;

    write_output(io.output.as_deref(), &rendered)?;

    Ok(())
}
