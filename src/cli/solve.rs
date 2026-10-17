use std::io::{self, Write};

use anyhow::Result;
use kinemagic::io::{load_file, text::render_frames};
use kinemagic::solve::SequenceSolver;
use kinemagic::view::{Scene, ViewFrame, run_viewer};

const PROGRESS_INTERVAL: usize = 20;

use crate::cli::SolveArgs;

pub fn run(args: SolveArgs) -> Result<()> {
    let problem = load_file(&args.input)?;
    let times = problem.solver().times().collect::<Vec<_>>();

    let mut frames = Vec::new();
    let mut solver = SequenceSolver::new(&problem);
    let mut stderr = io::stderr().lock();

    for (frame_index, time) in times.iter().copied().enumerate() {
        let step = frame_index + 1;

        // render progress to stderr
        if args.progress {
            if step == 1 {
                render_progress_header(&mut stderr);
            }

            if step == 1 || step.is_multiple_of(PROGRESS_INTERVAL) || step == times.len() {
                render_progress_step(&mut stderr, step, times.len(), time);
            }
        }

        // solve the frame
        let poses = solver.solve_at(time)?;

        // store the frame
        frames.push((time, poses));
    }

    if args.view {
        // render the frames to ratatui
        let view_frames = frames
            .iter()
            .map(|(time, poses)| {
                let scene = Scene::from_body_poses(problem.mechanism(), poses)?;
                Ok(ViewFrame::solved(*time, scene))
            })
            .collect::<Result<Vec<_>>>()?;

        run_viewer(view_frames)?;
    } else {
        // render the frames to stdout
        let output = render_frames(&frames);
        io::stdout().lock().write_all(output.as_bytes())?;
    }

    Ok(())
}

fn render_progress_header(stderr: &mut impl Write) {
    let _ = writeln!(stderr);
    let _ = writeln!(
        stderr,
        "{:<12} {:<15} {:<12}",
        "Current Step", "Current Time", "Progress (%)"
    );
    let _ = writeln!(
        stderr,
        "{:<12} {:<15} {:<12}",
        "------------", "-------------", "------------"
    );
}

fn render_progress_step(stderr: &mut impl Write, step: usize, steps: usize, time: f64) {
    let percent = 100.0 * step as f64 / steps as f64;
    let _ = writeln!(stderr, "{:<12} {:<15.2e} {:<12.2}", step, time, percent);
    let _ = stderr.flush();
}
