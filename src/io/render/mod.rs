use nalgebra::{UnitQuaternion, Vector3};

pub mod bodies;
pub mod joints;
pub mod state;

fn format_point_row(point: &Vector3<f64>) -> String {
    format!(
        "[ {}  {}  {} ]",
        format_number(point.x),
        format_number(point.y),
        format_number(point.z)
    )
}

fn format_quaternion_row(q: &UnitQuaternion<f64>) -> String {
    format!(
        "[ {}  {}  {} {} ]\n",
        format_number(q.coords.w),
        format_number(q.coords.x),
        format_number(q.coords.y),
        format_number(q.coords.z),
    )
}

fn format_number(value: f64) -> String {
    format!("{:>8.3}", sanitize_zero(value))
}

fn sanitize_zero(value: f64) -> f64 {
    if value == 0.0 { 0.0 } else { value }
}
