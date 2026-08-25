use std::process::{Command, Output};

const VALID_NO_MOTION: &str = "tests/fixtures/spherical_one_body_parse.yaml";
const VALID_MOTION: &str = "tests/fixtures/spherical_two_body_motion.yaml";
const MALFORMED: &str = "tests/fixtures/malformed.yaml";
const NOT_SOLVE_READY: &str = "tests/fixtures/spherical_duplicate_motion.yaml";

#[test]
fn check_accepts_valid_no_motion_problem() {
    assert_check_succeeds(VALID_NO_MOTION);
}

#[test]
fn check_accepts_valid_motion_problem() {
    assert_check_succeeds(VALID_MOTION);
}

#[test]
fn check_rejects_malformed_yaml_with_path_context() {
    assert_check_fails_with_path(MALFORMED);
}

#[test]
fn check_rejects_not_solve_ready_problem_with_path_context() {
    assert_check_fails_with_path(NOT_SOLVE_READY);
}

fn assert_check_succeeds(path: &str) {
    let output = check(path);

    assert!(output.status.success());
    assert_eq!(
        String::from_utf8(output.stdout).unwrap(),
        format!("valid: {path}\n")
    );
    assert!(output.stderr.is_empty());
}

fn assert_check_fails_with_path(path: &str) {
    let output = check(path);

    assert!(!output.status.success());
    assert!(output.stdout.is_empty());
    assert!(String::from_utf8(output.stderr).unwrap().contains(path));
}

fn check(path: &str) -> Output {
    Command::new(env!("CARGO_BIN_EXE_km"))
        .args(["check", path])
        .output()
        .unwrap()
}
