pub mod yaml;

use std::{
    fs,
    io::{self, Write},
    path::Path,
};

pub fn read_input(input: &Path) -> io::Result<String> {
    fs::read_to_string(input)
}

pub fn write_output(output: Option<&Path>, rendered: &str) -> io::Result<()> {
    let rendered = with_trailing_newline(rendered);

    match output {
        // If path provided, write to file
        Some(path) if path != Path::new("-") => {
            fs::write(path, rendered)?;
            Ok(())
        }

        // If none or path is "-", write to stdout
        None | Some(_) => {
            io::stdout().write_all(rendered.as_bytes())?;
            Ok(())
        }
    }
}

// Ensure the string ends with a newline character
fn with_trailing_newline(s: &str) -> String {
    if s.ends_with('\n') {
        s.to_string()
    } else {
        format!("{s}\n")
    }
}
