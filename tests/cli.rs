use std::process::{Command, Output};

const VALID_NO_MOTION: &str = "tests/fixtures/spherical_one_body_parse.yaml";
const VALID_MOTION: &str = "tests/fixtures/spherical_two_body_motion.yaml";
const MALFORMED: &str = "tests/fixtures/malformed.yaml";
const NOT_SOLVE_READY: &str = "tests/fixtures/spherical_duplicate_motion.yaml";
const MISSING: &str = "tests/fixtures/missing.yaml";
const DOCUMENTED_EXAMPLE: &str = "examples/spherical_one_body_motion.yaml";
const CLOSED_LOOP_EXAMPLE: &str = "tests/fixtures/spherical_one_body_closed_loop.yaml";
const TIME_MOTION: &str = "tests/fixtures/spherical_one_body_time_motion.yaml";

const EXPECTED_ONE_BODY: &str =
    include_str!("fixtures/spherical_one_body_parse_expected_output.txt");
const EXPECTED_TWO_BODY: &str =
    include_str!("fixtures/spherical_two_body_motion_expected_output.txt");
const EXPECTED_CLOSED_LOOP: &str =
    include_str!("fixtures/spherical_one_body_closed_loop_expected_output.txt");

#[test]
fn check_accepts_valid_no_motion_problem() {
    assert_check_succeeds(VALID_NO_MOTION);
}

#[test]
fn check_accepts_valid_motion_problem() {
    assert_check_succeeds(VALID_MOTION);
}

#[test]
fn check_accepts_valid_example_motion_problem() {
    assert_check_succeeds(DOCUMENTED_EXAMPLE);
}

#[test]
fn check_accepts_valid_closed_loop_problem() {
    assert_check_succeeds(CLOSED_LOOP_EXAMPLE);
}

#[test]
fn check_rejects_malformed_yaml_with_path_context() {
    assert_check_fails_with_path(MALFORMED);
}

#[test]
fn check_rejects_not_solve_ready_problem_with_path_context() {
    assert_check_fails_with_path(NOT_SOLVE_READY);
}

#[test]
fn solve_renders_one_body_reference_problem() {
    let output = run("solve", VALID_NO_MOTION);

    assert!(output.status.success());
    assert!(output.stderr.is_empty());
    assert_eq!(String::from_utf8(output.stdout).unwrap(), EXPECTED_ONE_BODY);
}

#[test]
fn solve_renders_two_body_motion_problem() {
    let output = run("solve", VALID_MOTION);

    assert!(output.status.success());
    assert!(output.stderr.is_empty());
    assert_eq!(String::from_utf8(output.stdout).unwrap(), EXPECTED_TWO_BODY);
}

#[test]
fn solve_renders_explicit_multi_frame_output() {
    let output = run("solve", TIME_MOTION);
    let stdout = String::from_utf8(output.stdout).unwrap();

    assert!(output.status.success());
    assert!(stdout.contains("frame 0 time 0.000000000000\n"));
    assert!(stdout.contains("frame 2 time 0.200000000000\n"));
}

#[test]
fn solve_renders_one_body_example_problem() {
    let output = run("solve", DOCUMENTED_EXAMPLE);

    assert!(output.status.success());
    assert!(output.stderr.is_empty());
    let stdout = String::from_utf8(output.stdout).unwrap();
    assert_eq!(stdout.matches("frame ").count(), 41);
    assert!(stdout.contains("frame 0 time 0.000000000000\n"));
    assert!(stdout.contains("frame 40 time 4.000000000000\n"));
}

#[test]
fn solve_renders_closed_loop_problem() {
    let output = run("solve", CLOSED_LOOP_EXAMPLE);

    assert!(output.status.success());
    assert!(output.stderr.is_empty());
    assert_eq!(
        String::from_utf8(output.stdout).unwrap(),
        EXPECTED_CLOSED_LOOP
    );
}

#[test]
fn solve_renders_closed_loop_progress_to_stderr() {
    let output = run_with_args("solve", CLOSED_LOOP_EXAMPLE, &["--progress"]);
    let stderr = String::from_utf8(output.stderr).unwrap();

    assert!(output.status.success());
    assert_eq!(
        String::from_utf8(output.stdout).unwrap(),
        EXPECTED_CLOSED_LOOP
    );
    assert!(stderr.contains("Current Step Current Time    Progress (%)"));
    assert!(stderr.contains("------------ -------------   ------------"));
    assert!(stderr.contains("1            0.00e0          100.00"));
}

#[test]
fn view_requires_an_interactive_terminal() {
    let output = run_with_args("check", VALID_NO_MOTION, &["--view"]);

    assert!(!output.status.success());
    assert!(output.stdout.is_empty());
    assert!(
        String::from_utf8(output.stderr)
            .unwrap()
            .contains("terminal viewer requires interactive stdin and stdout")
    );
}

#[test]
fn solve_view_requires_an_interactive_terminal_without_pose_output() {
    let output = run_with_args("solve", VALID_NO_MOTION, &["--view"]);

    assert!(!output.status.success());
    assert!(output.stdout.is_empty());
    assert!(
        String::from_utf8(output.stderr)
            .unwrap()
            .contains("terminal viewer requires interactive stdin and stdout")
    );
}

#[test]
fn solve_live_view_requires_an_interactive_terminal_without_pose_output() {
    let output = run_with_args("solve", VALID_NO_MOTION, &["--view=live"]);

    assert!(!output.status.success());
    assert!(output.stdout.is_empty());
    assert!(
        String::from_utf8(output.stderr)
            .unwrap()
            .contains("terminal viewer requires interactive stdin and stdout")
    );
}

#[test]
fn solve_rejects_view_with_progress() {
    let output = run_with_args("solve", VALID_NO_MOTION, &["--view", "--progress"]);

    assert!(!output.status.success());
    assert!(
        String::from_utf8(output.stderr)
            .unwrap()
            .contains("cannot be used with")
    );
}

#[test]
fn solve_output_is_deterministic() {
    let first = run("solve", VALID_MOTION);
    let second = run("solve", VALID_MOTION);

    assert!(first.status.success());
    assert!(second.status.success());
    assert!(first.stderr.is_empty());
    assert!(second.stderr.is_empty());
    assert_eq!(first.stdout, second.stdout);
}

#[test]
fn solve_rejects_missing_file_without_partial_output() {
    assert_solve_fails_without_output(MISSING);
}

#[test]
fn solve_rejects_invalid_problem_without_partial_output() {
    assert_solve_fails_without_output(NOT_SOLVE_READY);
}

fn assert_solve_fails_without_output(path: &str) {
    let output = run("solve", path);

    assert!(!output.status.success());
    assert!(output.stdout.is_empty());

    let stderr = String::from_utf8(output.stderr).unwrap();
    assert!(stderr.contains(path));
}

fn assert_check_succeeds(path: &str) {
    let output = run("check", path);

    assert!(output.status.success());
    assert_eq!(
        String::from_utf8(output.stdout).unwrap(),
        format!("valid: {path}\n")
    );
    assert!(output.stderr.is_empty());
}

fn assert_check_fails_with_path(path: &str) {
    let output = run("check", path);

    assert!(!output.status.success());
    assert!(output.stdout.is_empty());
    assert!(String::from_utf8(output.stderr).unwrap().contains(path));
}

fn run(command: &str, path: &str) -> Output {
    run_with_args(command, path, &[])
}

fn run_with_args(command: &str, path: &str, args: &[&str]) -> Output {
    Command::new(env!("CARGO_BIN_EXE_km"))
        .args([command, path])
        .args(args)
        .output()
        .unwrap()
}
