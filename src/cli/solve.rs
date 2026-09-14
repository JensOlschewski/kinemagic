use std::io::{self, Write};

use anyhow::{Context, Result};
use kinemagic::io::{load_file, text::render_frames};
use kinemagic::problem::prepare;
use kinemagic::solve::solve_at;

use crate::cli::SolveArgs;

pub fn run(args: SolveArgs) -> Result<()> {
    let input = load_file(&args.input)?;

    let problem =
        prepare(input).with_context(|| format!("failed to prepare `{}`", args.input.display()))?;

    let times = problem.solver().times().collect::<Vec<_>>();
    let mut frames = Vec::new();
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

        let poses = solve_at(&problem, time)?;
        frames.push((time, poses));
    }

    let output = render_frames(&frames);

    io::stdout().lock().write_all(output.as_bytes())?;

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
