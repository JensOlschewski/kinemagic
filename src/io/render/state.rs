use crate::kinematics::KinematicsError;
use crate::kinematics::state::{KinematicState};
use crate::model::{Bodies};

pub fn render_state_pretty(
    bodies: &Bodies,
    state: &KinematicState,
) -> Result<String, KinematicsError> {
    let mut rendered = String::new();

    for (idx, body) in bodies.iter().enumerate() {
        if idx > 0 {
            rendered.push('\n');
        }

    let pose = state
        .get_body_pose(body.id())
        .ok_or(KinematicsError::MissingBody(body.id()))?;

    super::bodies::render_body_header(&mut rendered, body);
    super::bodies::render_body_pose(&mut rendered, pose);
    super::bodies::render_points_on_body(&mut rendered, body, pose);
    }

    Ok(rendered)
}
