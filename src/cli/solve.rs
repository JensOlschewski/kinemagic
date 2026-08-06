use std::error::Error;

use crate::cli::SolveArgs;
use crate::io::yaml::{YamlModel, read_yaml_str};
use crate::io::{read_input, write_output};
use crate::kinematics::{forward::update_body_poses, state::KinematicState};
use crate::model::{Bodies, Joints, Model};

pub fn run_solve(args: SolveArgs) -> Result<(), Box<dyn Error>> {
    let SolveArgs { io, format: _ } = args;

    let yaml = read_input(&io.input)?;
    let input: YamlModel = read_yaml_str(&yaml)?;
    let (hardpoints, body_specs, joint_specs, motions) = input.into_model_parts()?;

    let bodies = Bodies::build(&body_specs, &hardpoints)?;
    let joints = Joints::build(&joint_specs, &bodies)?;
    let model = Model::new(bodies, joints);

    let bodies_from_model = model.bodies();
    let joints_from_model = model.joints();

    let mut state = KinematicState::from_reference(bodies_from_model, joints_from_model)?;

    for motion in &motions {
        state.apply_motion(motion)?;
    }

    update_body_poses(&mut state, joints_from_model)?;

    let rendered = crate::io::render::state::render_state_pretty(bodies_from_model, &state)?;

    write_output(io.output.as_deref(), &rendered)?;

    Ok(())
}
