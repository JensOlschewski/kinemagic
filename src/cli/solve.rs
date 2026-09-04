use std::io::{self, Write};

use crate::cli::SolveArgs;
use kinemagic::io::{load_and_prepare, text::render_body_poses};
use kinemagic::solve::solve as solve_problem;

pub fn run(args: SolveArgs) -> Result<(), Box<dyn std::error::Error>> {
    let problem = load_and_prepare(&args.input)?;
    let poses = solve_problem(&problem)?;
    let output = render_body_poses(&poses);

    io::stdout().lock().write_all(output.as_bytes())?;

    Ok(())
}
