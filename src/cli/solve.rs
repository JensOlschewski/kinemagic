use std::io::{self, Write};

use crate::cli::SolveArgs;
use kinemagic::io::{load_and_prepare, text::render_frames};
use kinemagic::solve::solve_at;

pub fn run(args: SolveArgs) -> Result<(), Box<dyn std::error::Error>> {
    let problem = load_and_prepare(&args.input)?;
    let frames = problem
        .solver()
        .times()
        .map(|time| solve_at(&problem, time).map(|poses| (time, poses)))
        .collect::<Result<Vec<_>, _>>()?;
    let output = render_frames(&frames);

    io::stdout().lock().write_all(output.as_bytes())?;

    Ok(())
}
