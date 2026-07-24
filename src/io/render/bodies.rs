use crate::model::{Bodies, Body, BodyPose};

pub fn render_bodies_pretty(bodies: &Bodies) -> String {
    let mut rendered = String::new();

    for (idx, body) in bodies.iter().enumerate() {
        if idx > 0 {
            rendered.push('\n');
        }
        render_body_header(&mut rendered, body);
        render_body_pose(&mut rendered, body.pose());
        render_points_on_body(&mut rendered, body, body.pose());
    }
    rendered
}

pub(super) fn render_body_header(rendered: &mut String, body: &Body) {
    rendered.push_str(&format!(
        "Body {}  id={}  side={}\n",
        body.name(),
        body.id().0,
        body.side().label(),
    ));
}

pub(super) fn render_body_pose(rendered: &mut String, pose: &BodyPose) {
    rendered.push_str(&format!(
        "position\n {}\n",
        super::format_point_row(&pose.position)
    ));
    rendered.push_str("orientation\n");
    rendered.push_str(&super::format_quaternion_row(&pose.orientation));
}

pub(super) fn render_points_on_body(rendered: &mut String, body: &Body, pose: &BodyPose) {
    rendered.push_str("points\n");

    for (name, point) in body.global_points_at(pose) {
        rendered.push_str(&format!("{name:<8} {}\n", super::format_point_row(&point)));
    }
}
