use crate::cli::CheckArgs;
use anyhow::{Context, Result};

use kinemagic::io::load_file;
use kinemagic::problem::prepare;
use kinemagic::view::{Scene, ViewFrame, run_viewer};

pub fn run(args: CheckArgs) -> Result<()> {
    let input = load_file(&args.input)?;

    let problem =
        prepare(input).with_context(|| format!("failed to prepare `{}`", args.input.display()))?;

    if args.view {
        let scene = Scene::from_reference(problem.model());
        run_viewer(vec![ViewFrame::reference(scene)])?;
    } else {
        println!("valid: {}", args.input.display());
    }

    Ok(())
}
