use nalgebra::{Rotation3, Vector3};

use crate::model::joints::{Joint, JointEndpoint};
use crate::model::{Bodies, Joints};

pub fn render_joints_pretty(joints: &Joints, bodies: &Bodies) -> String {
    let mut rendered = String::new();

    render_joint_section(
        &mut rendered,
        "Primary joints",
        joints.primary(),
        bodies,
    );
    rendered.push('\n');
    render_joint_section(
        &mut rendered,
        "Secondary joints",
        joints.secondary(),
        bodies,
    );

    rendered
}

fn render_joint_section<'a>(
    rendered: &mut String,
    title: &str,
    joints: impl Iterator<Item = &'a Joint>,
    bodies: &Bodies,
) {
    rendered.push_str(title);
    rendered.push('\n');

    for (index, joint) in joints.enumerate() {
        if index > 0 {
            rendered.push('\n');
        }

        rendered.push_str(&format!(
            "Joint {}  id={}  kind={:?}\n",
            joint.name(), joint.id().0, joint.kind()
        ));
        render_endpoint(rendered, "i", &joint.i_endpoint(), bodies);
        render_endpoint(rendered, "j", &joint.j_endpoint(), bodies);
    }
}

fn render_endpoint(rendered: &mut String, label: &str, endpoint: &JointEndpoint, bodies: &Bodies) {
    let body = bodies
        .get_by_id(endpoint.body_id)
        .expect("joint endpoint body must exist");

    let marker = &endpoint.local_marker;
    let global_position = marker.global_position(body.pose());
    let global_orientation = marker.global_orientation(body.pose());

    rendered.push_str(&format!(
        "  {label} endpoint  body={}\n",
        endpoint.body_id.0
    ));

    rendered.push_str("    local position  ");
    rendered.push_str(&format_point_row(&marker.local_position));
    rendered.push('\n');

    rendered.push_str("    global position ");
    rendered.push_str(&format_point_row(&global_position));
    rendered.push('\n');

    rendered.push_str("    local basis\n");
    render_basis(rendered, &marker.local_orientation);

    rendered.push_str("    global basis\n");
    render_basis(rendered, &global_orientation);
}

fn render_basis(rendered: &mut String, orientation: &Rotation3<f64>) {
    let names = ["ex", "ey", "ez"];

    for (name, column) in names.into_iter().zip(orientation.matrix().column_iter()) {
        let axis = column.into_owned();

        rendered.push_str(&format!("      {name} {}\n", format_point_row(&axis),));
    }
}

fn format_point_row(point: &Vector3<f64>) -> String {
    format!(
        "[ {}  {}  {} ]",
        format_number(point.x),
        format_number(point.y),
        format_number(point.z)
    )
}

fn format_number(value: f64) -> String {
    format!("{:>8.3}", sanitize_zero(value))
}

fn sanitize_zero(value: f64) -> f64 {
    if value.abs() < 1.0e-12 { 0.0 } else { value }
}
