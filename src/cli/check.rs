use crate::cli::CheckArgs;
use anyhow::Result;

use kinemagic::io::load_file;
use kinemagic::view::{Scene, ViewFrame, run_viewer};

pub fn run(args: CheckArgs) -> Result<()> {
    let problem = load_file(&args.input)?;

    if args.view {
        let scene = Scene::from_reference(problem.mechanism());
        run_viewer(vec![ViewFrame::reference(scene)])?;
    } else {
        println!("valid: {}", args.input.display());
    }

    Ok(())
}
