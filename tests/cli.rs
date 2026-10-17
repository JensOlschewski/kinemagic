use std::process::{Command, Output};

const VALID_STATIONARY: &str = "tests/fixtures/spherical_one_body_stationary_motion.yaml";
const VALID_MOTION: &str = "tests/fixtures/spherical_two_body_motion.yaml";
const UNDERDETERMINED_OPEN_TREE: &str = "tests/fixtures/open_tree_underdetermined.yaml";
const MALFORMED: &str = "tests/fixtures/malformed.yaml";
const NOT_SOLVE_READY: &str = "tests/fixtures/spherical_duplicate_motion.yaml";
const MISSING: &str = "tests/fixtures/missing.yaml";
const DOCUMENTED_EXAMPLE: &str = "examples/revolute_one_body_motion.yaml";
const CLOSED_LOOP_EXAMPLE: &str = "tests/fixtures/spherical_one_body_closed_loop.yaml";
const TIME_MOTION: &str = "tests/fixtures/spherical_one_body_time_motion.yaml";
const REVOLUTE_MOTION: &str = "tests/fixtures/revolute_one_body_motion.yaml";
const UNDERDETERMINED_CLOSED_LOOP: &str =
    "tests/fixtures/spherical_closed_loop_underdetermined.yaml";
const INCOMPATIBLE_CLOSED_LOOP: &str = "tests/fixtures/closed_loop_incompatible.yaml";
const FOUR_BAR_EXAMPLE: &str = "examples/four_bar_linkage.yaml";

const EXPECTED_ONE_BODY: &str =
    include_str!("fixtures/spherical_one_body_stationary_motion_expected_output.txt");
const EXPECTED_TWO_BODY: &str =
    include_str!("fixtures/spherical_two_body_motion_expected_output.txt");
const EXPECTED_CLOSED_LOOP: &str =
    include_str!("fixtures/spherical_one_body_closed_loop_expected_output.txt");
const EXPECTED_REVOLUTE: &str =
    include_str!("fixtures/revolute_one_body_motion_expected_output.txt");

const EXPECTED_CHECK_ONE_BODY: &str =
    include_str!("fixtures/spherical_one_body_stationary_motion_check_expected_output.txt");
const EXPECTED_CHECK_TWO_BODY: &str =
    include_str!("fixtures/spherical_two_body_motion_check_expected_output.txt");
const EXPECTED_CHECK_DOCUMENTED_EXAMPLE: &str =
    include_str!("fixtures/revolute_one_body_motion_check_expected_output.txt");
const EXPECTED_CHECK_CLOSED_LOOP: &str =
    include_str!("fixtures/spherical_one_body_closed_loop_check_expected_output.txt");
const EXPECTED_CHECK_UNDERDETERMINED_CLOSED_LOOP: &str =
    include_str!("fixtures/spherical_closed_loop_underdetermined_check_expected_output.txt");
const EXPECTED_CHECK_FOUR_BAR: &str =
    include_str!("fixtures/four_bar_linkage_check_expected_output.txt");

#[test]
fn check_accepts_valid_stationary_problem() {
    assert_check_succeeds(VALID_STATIONARY, EXPECTED_CHECK_ONE_BODY);
}

#[test]
fn check_accepts_valid_motion_problem() {
    assert_check_succeeds(VALID_MOTION, EXPECTED_CHECK_TWO_BODY);
}

#[test]
fn check_accepts_valid_example_motion_problem() {
    assert_check_succeeds(DOCUMENTED_EXAMPLE, EXPECTED_CHECK_DOCUMENTED_EXAMPLE);
}

#[test]
fn check_accepts_valid_closed_loop_problem() {
    assert_check_succeeds(CLOSED_LOOP_EXAMPLE, EXPECTED_CHECK_CLOSED_LOOP);
}

#[test]
fn check_accepts_four_bar_linkage_example() {
    assert_check_succeeds(FOUR_BAR_EXAMPLE, EXPECTED_CHECK_FOUR_BAR);
}

#[test]
fn check_reports_underdetermined_closed_loop_mobility() {
    assert_check_succeeds(
        UNDERDETERMINED_CLOSED_LOOP,
        EXPECTED_CHECK_UNDERDETERMINED_CLOSED_LOOP,
    );
}

#[test]
fn check_reports_incompatible_closed_loop_with_offending_joint() {
    let output = run("check", INCOMPATIBLE_CLOSED_LOOP);

    assert!(!output.status.success());
    assert!(output.stdout.is_empty());
    let stderr = String::from_utf8(output.stderr).unwrap();
    assert!(stderr.contains("JointId(4)"));
    assert!(stderr.contains("does not close"));
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
fn check_and_solve_reject_underdetermined_open_tree_with_offending_coordinates() {
    for command in ["check", "solve"] {
        let output = run(command, UNDERDETERMINED_OPEN_TREE);

        assert!(!output.status.success());
        assert!(output.stdout.is_empty());

        let stderr = String::from_utf8(output.stderr).unwrap();
        assert!(stderr.contains(UNDERDETERMINED_OPEN_TREE));
        assert!(stderr.contains("joint `1` component 0"));
        assert!(stderr.contains("joint `1` component 2"));
    }
}

#[test]
fn solve_renders_one_body_reference_problem() {
    let output = run("solve", VALID_STATIONARY);

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
fn solve_renders_revolute_motion_problem() {
    let output = run("solve", REVOLUTE_MOTION);

    assert!(output.status.success());
    assert!(output.stderr.is_empty());
    assert_eq!(String::from_utf8(output.stdout).unwrap(), EXPECTED_REVOLUTE);
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
    let output = run_with_args("check", VALID_STATIONARY, &["--view"]);

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
    let output = run_with_args("solve", VALID_STATIONARY, &["--view"]);

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
    let output = run_with_args("solve", VALID_STATIONARY, &["--view", "--progress"]);

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

fn assert_check_succeeds(path: &str, expected: &str) {
    let output = run("check", path);

    assert!(output.status.success());
    assert_eq!(String::from_utf8(output.stdout).unwrap(), expected);
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
