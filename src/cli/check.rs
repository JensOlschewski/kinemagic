use crate::cli::CheckArgs;
use anyhow::Result;

use kinemagic::data::coordinates::GeneralizedCoordinates;
use kinemagic::io::load_file;
use kinemagic::report;
use kinemagic::solve::mobility_at;
use kinemagic::view::{Scene, ViewFrame, run_viewer};

pub fn run(args: CheckArgs) -> Result<()> {
    let problem = load_file(&args.input)?;

    if args.view {
        let scene = Scene::from_reference(problem.mechanism());
        run_viewer(vec![ViewFrame::reference(scene)])?;
    } else {
        let report = report::build(&problem);
        let configuration = GeneralizedCoordinates::new(problem.coordinate_layout());
        let mobility = mobility_at(&problem, &configuration, 0.0)?;

        println!("valid: {}", args.input.display());
        println!("{report}");
        println!("mobility: {mobility}");
    }

    Ok(())
}
