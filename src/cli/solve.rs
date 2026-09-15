use std::io::{self, Write};
use std::sync::atomic::Ordering;

use anyhow::{Context, Result};
use kinemagic::io::{load_file, text::render_frames};
use kinemagic::problem::prepare;
use kinemagic::solve::{SequenceSolver, SolverProgress};
use kinemagic::view::{LiveViewEvent, Scene, ViewFrame, run_live_viewer, run_viewer};

use crate::cli::{SolveArgs, SolveViewMode};

pub fn run(args: SolveArgs) -> Result<()> {
    let input = load_file(&args.input)?;

    let problem =
        prepare(input).with_context(|| format!("failed to prepare `{}`", args.input.display()))?;

    let times = problem.solver().times().collect::<Vec<_>>();
    if args.view == Some(SolveViewMode::Live) {
        let reference = ViewFrame::reference(Scene::from_reference(problem.model()));
        let total_frames = times.len();
        return run_live_viewer(reference, total_frames, move |updates, cancelled| {
            let mut solver = SequenceSolver::new(&problem);

            for (frame_index, time) in times.into_iter().enumerate() {
                if cancelled.load(Ordering::Relaxed) {
                    return Ok(());
                }
                if updates
                    .send(LiveViewEvent::FrameStarted {
                        index: frame_index + 1,
                        total: total_frames,
                        time,
                    })
                    .is_err()
                {
                    return Ok(());
                }

                let poses = solver
                    .solve_at_with_progress(time, |progress| {
                        let iteration = match progress {
                            SolverProgress::Residual { iteration, .. }
                            | SolverProgress::StepAccepted { iteration, .. } => iteration,
                            SolverProgress::Converged { iterations, .. } => iterations,
                            SolverProgress::Failed => return,
                        };
                        let _ = updates.send(LiveViewEvent::Progress { iteration });
                    })
                    .map_err(|error| {
                        format!(
                            "failed to solve frame {} at t={time}: {error}",
                            frame_index + 1
                        )
                    })?;
                let frame = ViewFrame::solved(
                    time,
                    Scene::from_body_poses(problem.model(), &poses)
                        .map_err(|error| format!("failed to build solved scene: {error}"))?,
                );
                if updates.send(LiveViewEvent::FrameSolved(frame)).is_err() {
                    return Ok(());
                }
            }

            Ok(())
        })
        .map_err(Into::into);
    }

    let mut frames = Vec::new();
    let mut solver = SequenceSolver::new(&problem);
    let mut stderr = io::stderr().lock();

    if args.progress {
        render_progress_header(&mut stderr);
    }

    for (frame_index, time) in times.iter().copied().enumerate() {
        if args.progress
            && (frame_index == 0
                || (frame_index + 1).is_multiple_of(20)
                || frame_index + 1 == times.len())
        {
            render_progress_step(&mut stderr, frame_index + 1, times.len(), time);
        }

        let poses = solver.solve_at(time)?;
        frames.push((time, poses));
    }

    if args.view == Some(SolveViewMode::After) {
        let view_frames = frames
            .iter()
            .map(|(time, poses)| {
                let scene = Scene::from_body_poses(problem.model(), poses)?;
                Ok(ViewFrame::solved(*time, scene))
            })
            .collect::<Result<Vec<_>>>()?;
        run_viewer(view_frames)?;
    } else {
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
