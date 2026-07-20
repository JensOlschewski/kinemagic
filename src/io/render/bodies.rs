use nalgebra::Vector3;

use crate::model::Bodies;

pub fn render_bodies_pretty(bodies: &Bodies) -> String {
    let mut rendered = String::new();

    for (index, body) in bodies.iter().enumerate() {
        if index > 0 {
            rendered.push('\n');
        }

        rendered.push_str(&format!(
            "Body {}  id={}  side={}\n\n",
            body.name(),
            body.id().0,
            body.side().label()
        ));
        rendered.push_str("cm\n");
        rendered.push_str(&format_point_column(&body.position()));
        rendered.push('\n');
        rendered.push_str("points\n");

        for (name, point) in body.global_points() {
            rendered.push_str(&format!("{name:<8} {}\n", format_point_row(&point)));
        }
    }

    rendered
}

fn format_point_column(point: &Vector3<f64>) -> String {
    format!(
        "[ {} ]\n[ {} ]\n[ {} ]\n",
        format_number(point.x),
        format_number(point.y),
        format_number(point.z)
    )
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
    if value == 0.0 { 0.0 } else { value }
}
