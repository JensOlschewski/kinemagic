use crate::model::BodyId;
use crate::solve::{BodyPose, BodyPoses};

const ZERO_THRESHOLD: f64 = 0.5e-12;

pub fn render_body_poses(poses: &BodyPoses) -> String {
    let mut output = String::new();

    for (body_id, pose) in poses.iter() {
        output.push_str(&render_body_pose(*body_id, pose));
    }

    output
}

fn render_body_pose(body_id: BodyId, pose: &BodyPose) -> String {
    let [w, x, y, z] = canonical_quaternion(pose);

    format!(
        "body {body_id} position [{}, {}, {}] quaternion [{}, {}, {}, {}]\n",
        format_number(pose.position().x),
        format_number(pose.position().y),
        format_number(pose.position().z),
        format_number(w),
        format_number(x),
        format_number(y),
        format_number(z),
    )
}

fn canonical_quaternion(pose: &BodyPose) -> [f64; 4] {
    let orientation = pose.orientation();
    let mut components =
        [orientation.w, orientation.i, orientation.j, orientation.k].map(sanitize_zero);

    if components
        .iter()
        .copied()
        .find(|component| *component != 0.0)
        .is_some_and(|component| component.is_sign_negative())
    {
        components = components.map(|component| sanitize_zero(-component));
    }

    components
}

fn format_number(value: f64) -> String {
    format!("{:.12}", sanitize_zero(value))
}

fn sanitize_zero(value: f64) -> f64 {
    if value.abs() < ZERO_THRESHOLD {
        0.0
    } else {
        value
    }
}

#[cfg(test)]
mod tests {
    use nalgebra::{Quaternion, UnitQuaternion, Vector3};

    use super::*;
    use crate::io::yaml::parse_yaml_str;
    use crate::problem::prepare;
    use crate::solve::solve;

    #[test]
    fn renders_complete_poses_in_body_id_order() {
        let yaml = include_str!("../../tests/fixtures/spherical_two_body_parse.yaml")
            .replace("  B1:", "  ZBody:")
            .replace("  B2:", "  ABody:");
        let input = parse_yaml_str(&yaml).unwrap().into_input().unwrap();

        assert_eq!(
            input.model().bodies().iter().nth(1).unwrap().id(),
            BodyId::new(2)
        );

        let problem = prepare(input).unwrap();
        let output = render_body_poses(&solve(&problem));

        assert_eq!(
            output,
            concat!(
                "body 0 position [0.000000000000, 0.000000000000, 0.000000000000] quaternion [1.000000000000, 0.000000000000, 0.000000000000, 0.000000000000]\n",
                "body 1 position [0.000000000000, 0.000000000000, -100.000000000000] quaternion [1.000000000000, 0.000000000000, 0.000000000000, 0.000000000000]\n",
                "body 2 position [0.000000000000, 0.000000000000, -300.000000000000] quaternion [1.000000000000, 0.000000000000, 0.000000000000, 0.000000000000]\n",
            )
        );
    }

    #[test]
    fn normalizes_negative_zero_and_negative_scalar_sign() {
        let pose = BodyPose::new(
            Vector3::new(-0.0, -0.4e-12, 1.234_567_890_123_4),
            UnitQuaternion::new_unchecked(Quaternion::new(-1.0, -0.0, 0.0, 0.0)),
        );

        assert_eq!(
            render_body_pose(BodyId::new(7), &pose),
            "body 7 position [0.000000000000, 0.000000000000, 1.234567890123] quaternion [1.000000000000, 0.000000000000, 0.000000000000, 0.000000000000]\n"
        );
    }

    #[test]
    fn canonicalizes_vector_sign_when_scalar_is_zero() {
        let pose = BodyPose::new(
            Vector3::zeros(),
            UnitQuaternion::new_unchecked(Quaternion::new(0.0, -1.0, -0.0, 0.0)),
        );

        assert_eq!(
            render_body_pose(BodyId::new(1), &pose),
            "body 1 position [0.000000000000, 0.000000000000, 0.000000000000] quaternion [0.000000000000, 1.000000000000, 0.000000000000, 0.000000000000]\n"
        );
    }
}
