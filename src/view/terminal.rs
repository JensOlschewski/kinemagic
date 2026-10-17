use std::io::{self, IsTerminal};
use std::time::{Duration, Instant};

use crossterm::event::{self, Event, KeyCode, KeyEvent, KeyEventKind};
use ratatui::Frame;
use ratatui::layout::{Constraint, Layout, Rect};
use ratatui::style::Color;
use ratatui::symbols::Marker;
use ratatui::text::Line as TextLine;
use ratatui::widgets::canvas::{Canvas, Line as CanvasLine};
use ratatui::widgets::{Block, Paragraph};
use thiserror::Error;

use super::{ProjectedBounds, Projection, Scene};

const BOUNDS_PADDING: f64 = 0.1;
const DEFAULT_PLAYBACK_FPS: u16 = 10;
const MIN_PLAYBACK_FPS: u16 = 1;
const MAX_PLAYBACK_FPS: u16 = 100;
const PLAYBACK_FPS_STEP: u16 = 10;

#[derive(Clone, Debug, PartialEq)]
pub struct ViewFrame {
    time: Option<f64>,
    scene: Scene,
}

impl ViewFrame {
    pub fn time(&self) -> Option<f64> {
        self.time
    }

    pub fn scene(&self) -> &Scene {
        &self.scene
    }

    pub fn reference(scene: Scene) -> Self {
        Self { time: None, scene }
    }

    pub fn solved(time: f64, scene: Scene) -> Self {
        Self {
            time: Some(time),
            scene,
        }
    }
}

pub fn run_viewer(frames: Vec<ViewFrame>) -> Result<(), ViewerError> {
    let mut app = ViewerApp::new(frames)?;

    if !io::stdin().is_terminal() || !io::stdout().is_terminal() {
        return Err(ViewerError::NotInteractive);
    }

    let mut terminal = ratatui::try_init().map_err(ViewerError::Io)?;
    let result = run_app(&mut terminal, &mut app).map_err(ViewerError::Io);
    let restore_result = ratatui::try_restore().map_err(ViewerError::Io);

    result.and(restore_result)
}

#[derive(Debug, Error)]
pub enum ViewerError {
    #[error("terminal viewer requires interactive stdin and stdout")]
    NotInteractive,
    #[error("terminal viewer requires at least one frame")]
    EmptyFrames,
    #[error("terminal I/O failed")]
    Io(#[source] io::Error),
}

struct ViewerApp {
    frames: Vec<ViewFrame>,
    frame_index: usize,
    projection: Projection,
    viewports: Viewports,
    playing: bool,
    playback_fps: u16,
    loop_mode: LoopMode,
    playback_direction: PlaybackDirection,
    show_labels: bool,
    should_quit: bool,
}

struct Viewports {
    xy: ProjectedBounds,
    xz: ProjectedBounds,
    yz: ProjectedBounds,
    isometric: ProjectedBounds,
}

impl Viewports {
    fn for_frames(frames: &[ViewFrame]) -> Self {
        Self {
            xy: bounds_for_frames(frames, Projection::Xy),
            xz: bounds_for_frames(frames, Projection::Xz),
            yz: bounds_for_frames(frames, Projection::Yz),
            isometric: bounds_for_frames(frames, Projection::Isometric),
        }
    }

    fn get(&self, projection: Projection) -> ProjectedBounds {
        match projection {
            Projection::Xy => self.xy,
            Projection::Xz => self.xz,
            Projection::Yz => self.yz,
            Projection::Isometric => self.isometric,
        }
    }

    fn set(&mut self, projection: Projection, bounds: ProjectedBounds) {
        match projection {
            Projection::Xy => self.xy = bounds,
            Projection::Xz => self.xz = bounds,
            Projection::Yz => self.yz = bounds,
            Projection::Isometric => self.isometric = bounds,
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum LoopMode {
    Restart,
    Bounce,
    Once,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum PlaybackDirection {
    Forward,
    Backward,
}

impl ViewerApp {
    fn new(frames: Vec<ViewFrame>) -> Result<Self, ViewerError> {
        if frames.is_empty() {
            return Err(ViewerError::EmptyFrames);
        }
        let viewports = Viewports::for_frames(&frames);

        Ok(Self {
            playing: frames.len() > 1,
            frames,
            frame_index: 0,
            projection: Projection::Xz,
            viewports,
            playback_fps: DEFAULT_PLAYBACK_FPS,
            loop_mode: LoopMode::Restart,
            playback_direction: PlaybackDirection::Forward,
            show_labels: false,
            should_quit: false,
        })
    }

    fn render(&self, frame: &mut Frame<'_>) {
        let [canvas_area, status_area] =
            Layout::vertical([Constraint::Min(3), Constraint::Length(3)]).areas(frame.area());

        frame.render_widget(self.canvas(canvas_area), canvas_area);
        frame.render_widget(self.status(), status_area);
    }

    fn canvas(&self, area: Rect) -> impl ratatui::widgets::Widget + '_ {
        let scene = self.current_frame().scene();
        let bounds = self.bounds(area);
        let min = bounds.min();
        let max = bounds.max();
        let projection = self.projection;
        let ground_label_offset = label_cell_size(bounds, area);
        let show_labels = self.show_labels;

        Canvas::default()
            .block(Block::bordered().title(" Kinemagic "))
            .marker(Marker::Braille)
            .x_bounds([min.x, max.x])
            .y_bounds([min.y, max.y])
            .paint(move |context| {
                for segment in scene.ground().segments() {
                    draw_segment(context, projection, segment.start(), segment.end());
                }
                if show_labels {
                    let ground_markers = scene.ground().markers();
                    if ground_markers.is_empty() {
                        let point = projection.project(scene.ground().origin());
                        print_label(context, point - ground_label_offset, "G".to_owned());
                    } else {
                        for marker in ground_markers {
                            let point = projection.project(*marker);
                            print_label(context, point - ground_label_offset, "G".to_owned());
                        }
                    }
                }

                for body in scene.bodies() {
                    for segment in body.segments() {
                        draw_segment(context, projection, segment.start(), segment.end());
                    }
                    if show_labels {
                        print_label(
                            context,
                            projection.project(body.origin()),
                            body.label().to_owned(),
                        );
                    }
                }

                for joint in scene.joints() {
                    draw_segment(context, projection, joint.i_position(), joint.j_position());
                    if show_labels {
                        print_label(
                            context,
                            projection.project((joint.i_position() + joint.j_position()) / 2.0),
                            joint.label().to_owned(),
                        );
                    }
                }

                draw_axis_triad(context, bounds, projection);
            })
    }

    fn status(&self) -> Paragraph<'_> {
        let frame = self.current_frame();
        let time = frame
            .time()
            .map_or_else(|| "reference".to_owned(), |time| format!("t={time:.3}"));
        let playback = self.playback_status();
        let (shown_frame, total_frames) = self.frame_counts();
        let status = format!(
            " {}/{}  {}  {}  {} fps  loop={}",
            shown_frame,
            total_frames,
            time,
            playback,
            self.playback_fps,
            loop_mode_name(self.loop_mode),
        );

        Paragraph::new(vec![
            TextLine::from(status),
            TextLine::from(" Space pause   ←/→ step   +/- speed   l loop   v labels"),
            TextLine::from(" f/F fit       1–4 view   q quit"),
        ])
    }

    fn bounds(&self, area: Rect) -> ProjectedBounds {
        self.viewports.get(self.projection).fit_aspect(
            area.width.saturating_sub(2).saturating_mul(2),
            area.height.saturating_sub(2).saturating_mul(4),
        )
    }

    fn current_frame(&self) -> &ViewFrame {
        &self.frames[self.frame_index]
    }

    fn handle_key(&mut self, key: KeyEvent) {
        if key.kind != KeyEventKind::Press {
            return;
        }

        match key.code {
            KeyCode::Char('q') | KeyCode::Esc => self.should_quit = true,

            KeyCode::Char('f') => self.fit_current_frame(),
            KeyCode::Char('F') => self.fit_all_frames(),
            KeyCode::Char('v') => self.show_labels = !self.show_labels,
            KeyCode::Char(' ') if self.frames.len() > 1 => {
                if !self.playing
                    && self.loop_mode == LoopMode::Once
                    && self.frame_index + 1 == self.frames.len()
                {
                    self.frame_index = 0;
                    self.playback_direction = PlaybackDirection::Forward;
                }
                self.playing = !self.playing;
            }
            KeyCode::Left => {
                self.playing = false;
                self.frame_index = self
                    .frame_index
                    .checked_sub(1)
                    .unwrap_or(self.frames.len() - 1);
            }

            KeyCode::Right => {
                self.playing = false;
                self.advance_frame();
            }
            KeyCode::Char('1') => self.projection = Projection::Xy,
            KeyCode::Char('2') => self.projection = Projection::Xz,
            KeyCode::Char('3') => self.projection = Projection::Yz,
            KeyCode::Char('4') => self.projection = Projection::Isometric,
            KeyCode::Char('+') | KeyCode::Char('=') | KeyCode::Up => {
                self.playback_fps = self
                    .playback_fps
                    .saturating_add(PLAYBACK_FPS_STEP)
                    .min(MAX_PLAYBACK_FPS);
            }
            KeyCode::Char('-') | KeyCode::Down => {
                self.playback_fps = self
                    .playback_fps
                    .saturating_sub(PLAYBACK_FPS_STEP)
                    .max(MIN_PLAYBACK_FPS);
            }
            KeyCode::Char('l') => {
                self.loop_mode = match self.loop_mode {
                    LoopMode::Restart => LoopMode::Bounce,
                    LoopMode::Bounce => LoopMode::Once,
                    LoopMode::Once => LoopMode::Restart,
                };
                self.playback_direction = PlaybackDirection::Forward;
            }
            _ => {}
        }
    }

    fn tick(&mut self) {
        if !self.playing {
            return;
        }

        match self.playback_direction {
            PlaybackDirection::Forward if self.frame_index + 1 < self.frames.len() => {
                self.frame_index += 1;
            }
            PlaybackDirection::Forward => match self.loop_mode {
                LoopMode::Restart => self.frame_index = 0,
                LoopMode::Bounce if self.frames.len() > 1 => {
                    self.playback_direction = PlaybackDirection::Backward;
                    self.frame_index -= 1;
                }
                LoopMode::Bounce | LoopMode::Once => self.playing = false,
            },
            PlaybackDirection::Backward if self.frame_index > 0 => {
                self.frame_index -= 1;
            }
            PlaybackDirection::Backward => {
                self.playback_direction = PlaybackDirection::Forward;
                if self.frames.len() > 1 {
                    self.frame_index = 1;
                }
            }
        }
    }

    fn advance_frame(&mut self) {
        self.frame_index = (self.frame_index + 1).min(self.frames.len() - 1);
        self.playback_direction = PlaybackDirection::Forward;
    }

    fn playback_interval(&self) -> Duration {
        Duration::from_secs_f64(1.0 / f64::from(self.playback_fps))
    }

    fn fit_current_frame(&mut self) {
        let bounds = self
            .current_frame()
            .scene()
            .projected_bounds(self.projection)
            .padded(BOUNDS_PADDING);
        self.viewports.set(self.projection, bounds);
    }

    fn fit_all_frames(&mut self) {
        let bounds = bounds_for_frames(&self.frames, self.projection);
        self.viewports.set(self.projection, bounds);
    }

    fn playback_status(&self) -> String {
        if self.playing {
            "playing".to_owned()
        } else {
            "paused".to_owned()
        }
    }

    fn frame_counts(&self) -> (usize, usize) {
        (self.frame_index + 1, self.frames.len())
    }
}

fn bounds_for_frames(frames: &[ViewFrame], projection: Projection) -> ProjectedBounds {
    ProjectedBounds::for_scenes(frames.iter().map(ViewFrame::scene), projection)
        .expect("viewer contains at least one scene")
        .padded(BOUNDS_PADDING)
}

fn run_app(terminal: &mut ratatui::DefaultTerminal, app: &mut ViewerApp) -> io::Result<()> {
    let mut last_tick = Instant::now();

    while !app.should_quit {
        terminal.draw(|frame| app.render(frame))?;
        if app.should_quit {
            break;
        }

        let playback_interval = app.playback_interval();
        let timeout = playback_interval.saturating_sub(last_tick.elapsed());
        if event::poll(timeout)? {
            match event::read()? {
                Event::Key(key) => app.handle_key(key),
                Event::Resize(_, _) => {}
                _ => {}
            }
        }

        if last_tick.elapsed() >= playback_interval {
            app.tick();
            last_tick = Instant::now();
        }
    }

    Ok(())
}

fn draw_segment(
    context: &mut ratatui::widgets::canvas::Context<'_>,
    projection: Projection,
    start: nalgebra::Vector3<f64>,
    end: nalgebra::Vector3<f64>,
) {
    let start = projection.project(start);
    let end = projection.project(end);
    draw_projected_line(context, start, end);
}

fn draw_projected_line(
    context: &mut ratatui::widgets::canvas::Context<'_>,
    start: nalgebra::Vector2<f64>,
    end: nalgebra::Vector2<f64>,
) {
    context.draw(&CanvasLine {
        x1: start.x,
        y1: start.y,
        x2: end.x,
        y2: end.y,
        color: Color::Reset,
    });
}

fn draw_axis_triad(
    context: &mut ratatui::widgets::canvas::Context<'_>,
    bounds: ProjectedBounds,
    projection: Projection,
) {
    let min = bounds.min();
    let axis_length = bounds.width().min(bounds.height()) * 0.08;
    let origin = nalgebra::Vector2::new(
        min.x + bounds.width() * 0.12,
        min.y + bounds.height() * 0.12,
    );

    for (axis, label) in [
        (nalgebra::Vector3::x(), "x"),
        (nalgebra::Vector3::y(), "y"),
        (nalgebra::Vector3::z(), "z"),
    ] {
        let direction = projection.project(axis);
        if direction == nalgebra::Vector2::zeros() {
            continue;
        }
        let direction = direction.normalize();
        let end = origin + direction * axis_length;
        draw_projected_line(context, origin, end);
        context.print(end.x, end.y, label.to_owned());
    }
}

fn print_label(
    context: &mut ratatui::widgets::canvas::Context<'_>,
    point: nalgebra::Vector2<f64>,
    label: String,
) {
    context.print(point.x, point.y, label);
}

fn label_cell_size(bounds: ProjectedBounds, area: Rect) -> nalgebra::Vector2<f64> {
    nalgebra::Vector2::new(
        bounds.width() / f64::from(area.width.saturating_sub(2).max(1)),
        bounds.height() / f64::from(area.height.saturating_sub(2).max(1)),
    )
}

fn loop_mode_name(loop_mode: LoopMode) -> &'static str {
    match loop_mode {
        LoopMode::Restart => "restart",
        LoopMode::Bounce => "bounce",
        LoopMode::Once => "once",
    }
}

#[cfg(test)]
mod tests {
    use nalgebra::Vector3;
    use ratatui::Terminal;
    use ratatui::backend::TestBackend;

    use super::*;
    use crate::io::yaml::parse_yaml_str;

    fn reference_frame() -> ViewFrame {
        let yaml = format!(
            "{}\nmotions:\n  Hold:\n    kind: joint-coordinates\n    joint_id: 1\n    displacement:\n      rot_x: 0\n      rot_y: 0\n      rot_z: 0\n",
            include_str!("../../tests/fixtures/spherical_one_body_parse.yaml")
        );
        let input = parse_yaml_str(&yaml).unwrap().into_model().unwrap();
        ViewFrame::reference(Scene::from_reference(input.mechanism()))
    }

    #[test]
    fn rejects_empty_frame_list() {
        assert!(matches!(
            ViewerApp::new(vec![]),
            Err(ViewerError::EmptyFrames)
        ));
    }

    #[test]
    fn starts_static_reference_view_in_xz_projection() {
        let app = ViewerApp::new(vec![reference_frame()]).unwrap();

        assert_eq!(app.frame_index, 0);
        assert_eq!(app.projection, Projection::Xz);
        assert!(!app.playing);
        assert_eq!(app.playback_fps, DEFAULT_PLAYBACK_FPS);
    }

    #[test]
    fn handles_projection_playback_step_and_quit_keys() {
        let frame = reference_frame();
        let mut app = ViewerApp::new(vec![frame.clone(), frame]).unwrap();

        app.handle_key(KeyEvent::new(
            KeyCode::Char('1'),
            crossterm::event::KeyModifiers::NONE,
        ));
        assert_eq!(app.projection, Projection::Xy);
        app.handle_key(KeyEvent::new(
            KeyCode::Char('4'),
            crossterm::event::KeyModifiers::NONE,
        ));
        assert_eq!(app.projection, Projection::Isometric);

        app.handle_key(KeyEvent::new(
            KeyCode::Char('+'),
            crossterm::event::KeyModifiers::NONE,
        ));
        assert_eq!(app.playback_fps, DEFAULT_PLAYBACK_FPS + PLAYBACK_FPS_STEP);
        app.handle_key(KeyEvent::new(
            KeyCode::Down,
            crossterm::event::KeyModifiers::NONE,
        ));
        assert_eq!(app.playback_fps, DEFAULT_PLAYBACK_FPS);

        app.handle_key(KeyEvent::new(
            KeyCode::Char(' '),
            crossterm::event::KeyModifiers::NONE,
        ));
        assert!(!app.playing);

        app.handle_key(KeyEvent::new(
            KeyCode::Right,
            crossterm::event::KeyModifiers::NONE,
        ));
        assert_eq!(app.frame_index, 1);
        app.handle_key(KeyEvent::new(
            KeyCode::Left,
            crossterm::event::KeyModifiers::NONE,
        ));
        assert_eq!(app.frame_index, 0);

        app.handle_key(KeyEvent::new(
            KeyCode::Esc,
            crossterm::event::KeyModifiers::NONE,
        ));
        assert!(app.should_quit);
    }

    #[test]
    fn advances_and_wraps_playing_frames() {
        let frame = reference_frame();
        let mut app = ViewerApp::new(vec![frame.clone(), frame]).unwrap();

        app.tick();
        assert_eq!(app.frame_index, 1);
        app.tick();
        assert_eq!(app.frame_index, 0);
    }

    #[test]
    fn cycles_restart_bounce_and_once_modes() {
        let mut app = ViewerApp::new(vec![reference_frame()]).unwrap();

        app.handle_key(KeyEvent::new(
            KeyCode::Char('l'),
            crossterm::event::KeyModifiers::NONE,
        ));
        assert_eq!(app.loop_mode, LoopMode::Bounce);
        app.handle_key(KeyEvent::new(
            KeyCode::Char('l'),
            crossterm::event::KeyModifiers::NONE,
        ));
        assert_eq!(app.loop_mode, LoopMode::Once);
        app.handle_key(KeyEvent::new(
            KeyCode::Char('l'),
            crossterm::event::KeyModifiers::NONE,
        ));
        assert_eq!(app.loop_mode, LoopMode::Restart);
    }

    #[test]
    fn toggles_scene_labels() {
        let mut app = ViewerApp::new(vec![reference_frame()]).unwrap();
        // Default is false

        app.handle_key(KeyEvent::new(
            KeyCode::Char('v'),
            crossterm::event::KeyModifiers::NONE,
        ));

        assert!(app.show_labels);

        app.handle_key(KeyEvent::new(
            KeyCode::Char('v'),
            crossterm::event::KeyModifiers::NONE,
        ));

        assert!(!app.show_labels);
    }

    #[test]
    fn bounces_at_both_ends() {
        let frame = reference_frame();
        let mut app = ViewerApp::new(vec![frame.clone(), frame.clone(), frame]).unwrap();
        app.loop_mode = LoopMode::Bounce;

        let indices = (0..5)
            .map(|_| {
                app.tick();
                app.frame_index
            })
            .collect::<Vec<_>>();

        assert_eq!(indices, vec![1, 2, 1, 0, 1]);
        assert!(app.playing);
    }

    #[test]
    fn once_mode_stops_and_can_restart() {
        let frame = reference_frame();
        let mut app = ViewerApp::new(vec![frame.clone(), frame]).unwrap();
        app.loop_mode = LoopMode::Once;

        app.tick();
        app.tick();

        assert_eq!(app.frame_index, 1);
        assert!(!app.playing);

        app.handle_key(KeyEvent::new(
            KeyCode::Char(' '),
            crossterm::event::KeyModifiers::NONE,
        ));
        assert_eq!(app.frame_index, 0);
        assert!(app.playing);
    }

    #[test]
    fn clamps_playback_frame_rate() {
        let mut app = ViewerApp::new(vec![reference_frame()]).unwrap();

        for _ in 0..20 {
            app.handle_key(KeyEvent::new(
                KeyCode::Char('+'),
                crossterm::event::KeyModifiers::NONE,
            ));
        }
        assert_eq!(app.playback_fps, MAX_PLAYBACK_FPS);

        for _ in 0..20 {
            app.handle_key(KeyEvent::new(
                KeyCode::Char('-'),
                crossterm::event::KeyModifiers::NONE,
            ));
        }
        assert_eq!(app.playback_fps, MIN_PLAYBACK_FPS);
        assert_eq!(app.playback_interval(), Duration::from_secs(1));
    }

    #[test]
    fn renders_canvas_and_status_with_test_backend() {
        let backend = TestBackend::new(80, 24);
        let mut terminal = Terminal::new(backend).unwrap();
        let app = ViewerApp::new(vec![reference_frame()]).unwrap();

        terminal.draw(|frame| app.render(frame)).unwrap();

        let rendered = terminal
            .backend()
            .buffer()
            .content()
            .iter()
            .map(|cell| cell.symbol())
            .collect::<String>();
        assert!(rendered.contains("Kinemagic"));
        assert!(rendered.contains("reference"));
        assert!(rendered.contains("10 fps"));
        assert!(rendered.contains("q quit"));
        assert!(!rendered.contains("residual"));
        assert!(
            rendered
                .chars()
                .any(|symbol| ('\u{2800}'..='\u{28ff}').contains(&symbol))
        );
    }

    #[test]
    fn keeps_bounds_stable_across_frames() {
        let mut first = reference_frame();
        let mut second = reference_frame();
        first.scene.bodies[0].origin = Vector3::new(-10.0, 0.0, 0.0);
        second.scene.bodies[0].origin = Vector3::new(20.0, 0.0, 0.0);
        let app = ViewerApp::new(vec![first, second]).unwrap();

        let bounds = app.bounds(Rect::new(0, 0, 80, 20));

        assert!(bounds.min().x <= -10.0);
        assert!(bounds.max().x >= 20.0);
    }
}
