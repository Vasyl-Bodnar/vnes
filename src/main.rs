use std::{
    cell::RefCell,
    fs::File,
    io::Read,
    num::NonZeroU32,
    path::Path,
    rc::Rc,
    time::{Duration, Instant},
};

use env_logger::Builder;
use log::{error, warn};
use softbuffer::{Context, Surface};
use winit::{
    application::ApplicationHandler,
    dpi::PhysicalSize,
    error::EventLoopError,
    event::{KeyEvent, WindowEvent},
    event_loop::{ActiveEventLoop, ControlFlow, EventLoop},
    window::{Window, WindowId},
};

mod controller;
mod cpu;
mod ppu;
mod registers;

const WIDTH: usize = 256;
const HEIGHT: usize = 240;
const SCALE: usize = 3;

const DELTA: f32 = 1. / 60.;
// Currently a magicish number that produces the best result
const CYCLES_PER_FRAME: usize = 29785;

pub static SYS_PALLETE: [u32; 64] = [
    0x808080, 0x003DA6, 0x0012B0, 0x440096, 0xA1005E, 0xC70028, 0xBA0600, 0x8C1700, 0x5C2F00,
    0x104500, 0x054A00, 0x00472E, 0x004166, 0x000000, 0x050505, 0x050505, 0xC7C7C7, 0x0077FF,
    0x2155FF, 0x8237FA, 0xEB2FB5, 0xFF2950, 0xFF2200, 0xD63200, 0xC46200, 0x358000, 0x058F00,
    0x008A55, 0x0099CC, 0x212121, 0x090909, 0x090909, 0xFEFEFE, 0x0FD7FF, 0x69A2FF, 0xD480FF,
    0xFF45F3, 0xFF618B, 0xFF8833, 0xFF9C12, 0xFABC20, 0x9FE30E, 0x2BF035, 0x0CF0A4, 0x05FBFF,
    0x5E5E5E, 0x0D0D0D, 0x0D0D0D, 0xFFFFFF, 0xA6FCFF, 0xB3ECFF, 0xDAABEB, 0xFFA8F9, 0xFFABB3,
    0xFFD2B0, 0xFFEFA6, 0xFFF79C, 0xD7E895, 0xA6EDAF, 0xA2F2DA, 0x99FFFC, 0xDDDDDD, 0x111111,
    0x111111,
];

struct Screen {
    window: Rc<Window>,
    surface: Surface<Rc<Window>, Rc<Window>>,
}

impl Screen {
    fn new(event_loop: &ActiveEventLoop) -> Self {
        let window = Rc::new(
            event_loop
                .create_window(
                    Window::default_attributes()
                        .with_title("vnes 0.1")
                        .with_inner_size(PhysicalSize::new(
                            (WIDTH * SCALE) as u32,
                            (HEIGHT * SCALE) as u32,
                        )),
                )
                .unwrap(),
        );
        let surface = Surface::new(&Context::new(window.clone()).unwrap(), window.clone()).unwrap();
        Screen { window, surface }
    }
}

// Assumes same ratio for x and y
// Simple nearest neighbor interpolation to increase picture size
// Very effective for NES
//
// This function also performs a useful role of deciding whether
// to draw background or sprite pixels
//
// Both are packaged into one for efficiency purposes
fn decide_and_nn_interp(buf: &mut [u32], x: usize) {
    let mut cpy = [0; WIDTH * HEIGHT];
    for i in 0..HEIGHT {
        for j in 0..WIDTH {
            let pixel = buf[i * WIDTH + j];
            let (byt3, byt2, byt1, byt0) = (
                (pixel >> 24) as u8,
                (pixel >> 16) as u8,
                (pixel >> 8) as u8,
                (pixel >> 0) as u8,
            );
            // NOTE: bit packing all the information that we need
            let ext_col = byt0 & 0x3F;
            let bg_col = byt1 & 0x3F;
            let bg_idx = (byt0 >> 6) | ((byt1 >> 4) & 0xC);
            let sp_col = byt2 & 0x3F;
            let sp_idx = byt3 >> 2;
            let sp_att = byt3 & 0x3;

            let col = match (bg_idx, sp_idx & 0xF, sp_att) {
                (0, 0, _) => ext_col,
                (0, _s, _) => sp_col,
                (_b, 0, _) => bg_col,
                (_b, _s, 0 | 1 | 3) => sp_col,
                (_b, _s, 2) => bg_col,
                _ => unreachable!(),
            } as usize;

            cpy[i * WIDTH + j] = SYS_PALLETE[col];
        }
    }

    if x == 0 || x == 1 {
        buf.copy_from_slice(&cpy);
        return;
    }

    let new = (WIDTH * x, HEIGHT * x);

    for i in 0..new.1 {
        for j in 0..new.0 {
            buf[i * new.0 + j] = cpy[(i / x) * WIDTH + j / x];
        }
    }
}

struct NesData {
    ppu: Rc<RefCell<ppu::State>>,
    controllers: Rc<RefCell<(controller::State, controller::State)>>,
    cpu: cpu::State,
    budget: Duration,
    now: Instant,
}

impl Default for NesData {
    fn default() -> Self {
        NesData {
            ppu: Default::default(),
            controllers: Default::default(),
            cpu: Default::default(),
            budget: Duration::new(0, 0),
            now: Instant::now(),
        }
    }
}

impl NesData {
    fn parse_nes(&mut self, buf: Vec<u8>) {
        if self.cpu.ppu_st.is_none() {
            self.cpu.ppu_st = Some(self.ppu.clone());
        }
        if self.cpu.controllers_st.is_none() {
            self.cpu.controllers_st = Some(self.controllers.clone());
        }

        let sig = &buf[0..16];

        if sig[7] == 8 && sig[0x0C] == 8 {
            println!("NES 2.0");
            return;
        } else if sig[7] == 4 && sig[0x0C] == 4 {
            warn!("archaic, trying anyway..");
        } else if sig[7] == 0 && sig[0x0C] == 0 {
            println!("i{}{}{}", sig[0] as char, sig[1] as char, sig[2] as char);
        } else {
            warn!("archaic, possibly 0.7, trying anyway..");
        }

        let prg_size = sig[4] as usize;
        let chr_size = sig[5] as usize; // 0 = RAM
        let nt_arrang = sig[6] & 1 != 0; // 0 = horiz mirror, 1 = vert
        let prg_ram = sig[6] & 2 != 0;
        let trainer = sig[6] & 4 != 0;
        let alt_nt_layout = sig[6] & 8 != 0;
        let mut mapper_num = (sig[6] & (128 + 64 + 32 + 16)) >> 4;
        mapper_num |= sig[7] & (128 + 64 + 32 + 16);
        // These are rarely used in iNES according to nesdev
        let prg_ram_size = sig[8];
        let ntsc = sig[9] & 1 == 0; // ntsc or pal
        println!(
            "PRG: {}*16kb, CHR: {}*8kb, NT_A: {}, PRGRAM: {}, TRN: {}, ALT_NT: {}, MAP: {}, PRGRAM_S: {}, PAL: {}",
            prg_size,
            chr_size,
            nt_arrang,
            prg_ram,
            trainer,
            alt_nt_layout,
            mapper_num,
            prg_ram_size,
            !ntsc
        );

        if trainer {
            error!("This one actually uses trainer");
        }

        let prg = &buf[16..];
        let chr;

        if mapper_num == 0 {
            if prg_size == 2 {
                self.cpu.load_ctg(0x8000..0x10000, &prg[..32 * 1024]);
                chr = &prg[32 * 1024..];
            } else {
                self.cpu.load_ctg(0x8000..0xC000, &prg[..16 * 1024]);
                self.cpu.load_ctg(0xC000..0x10000, &prg[..16 * 1024]);
                chr = &prg[16 * 1024..];
            }
        } else {
            unimplemented!();
        }

        if (chr_size == 0 && chr.len() != 0) || chr_size == 1 {
            self.ppu.borrow_mut().load_chr(0..0x2000, &chr[0..0x2000]);
        } else if chr_size == 0 {
        } else {
            unimplemented!();
        }

        if nt_arrang {
            self.ppu.borrow_mut().mirr = ppu::Mirror::Vertical;
        } else {
            self.ppu.borrow_mut().mirr = ppu::Mirror::Horizontal;
        }

        self.cpu.reset = true;
    }

    pub fn step(&mut self, buf: &mut [u32]) {
        {
            let ppu = &mut self.ppu.borrow_mut();
            ppu.cycle(&mut self.cpu, buf);
            ppu.cycle(&mut self.cpu, buf);
            ppu.cycle(&mut self.cpu, buf);
        }
        self.cpu.cycle().unwrap();
    }

    fn frame(&mut self, buf: &mut [u32]) {
        for _ in 0..CYCLES_PER_FRAME {
            self.step(buf);
        }
    }

    pub fn render(&mut self, screen: &mut Screen) {
        let mut buf = screen.surface.buffer_mut().unwrap();

        decide_and_nn_interp(&mut buf, SCALE);

        buf.present().unwrap();
    }

    pub fn draw(&mut self, screen: &mut Screen) {
        self.budget += self.now.elapsed();
        self.now = Instant::now();

        while self.budget.as_secs_f32() >= DELTA {
            {
                let buf = &mut screen.surface.buffer_mut().unwrap();
                self.frame(buf);
            }

            self.render(screen);
            self.budget -= Duration::from_secs_f32(DELTA);
        }
    }
}

#[derive(Default)]
struct Nes {
    screen: Option<Screen>,
    data: NesData,
}

impl Nes {
    pub fn run(&mut self) -> Result<(), EventLoopError> {
        let event_loop = EventLoop::new()?;
        event_loop.set_control_flow(ControlFlow::Poll);
        event_loop.run_app(self)?;

        Ok(())
    }
}

impl ApplicationHandler for Nes {
    fn resumed(&mut self, event_loop: &ActiveEventLoop) {
        self.screen = Some(Screen::new(event_loop));
    }

    fn window_event(
        &mut self,
        event_loop: &ActiveEventLoop,
        _window_id: WindowId,
        event: WindowEvent,
    ) {
        match event {
            WindowEvent::CloseRequested => {
                println!("My life for Ner'zhul");
                event_loop.exit();
            }
            WindowEvent::KeyboardInput {
                device_id: _,
                event:
                    KeyEvent {
                        physical_key,
                        logical_key: _,
                        text: _,
                        location: _,
                        state,
                        repeat: _,
                        ..
                    },
                is_synthetic: _,
            } => {
                let mut controllers = self.data.controllers.borrow_mut();
                match physical_key {
                    winit::keyboard::PhysicalKey::Code(key_code) => match key_code {
                        winit::keyboard::KeyCode::KeyE => {
                            controllers.0.buttons.up = state.is_pressed()
                        }
                        winit::keyboard::KeyCode::KeyS => {
                            controllers.0.buttons.left = state.is_pressed()
                        }
                        winit::keyboard::KeyCode::KeyD => {
                            controllers.0.buttons.down = state.is_pressed()
                        }
                        winit::keyboard::KeyCode::KeyF => {
                            controllers.0.buttons.right = state.is_pressed()
                        }
                        winit::keyboard::KeyCode::KeyJ => {
                            controllers.0.buttons.a = state.is_pressed()
                        }
                        winit::keyboard::KeyCode::KeyK => {
                            controllers.0.buttons.b = state.is_pressed()
                        }
                        winit::keyboard::KeyCode::KeyL => {
                            controllers.0.buttons.select = state.is_pressed()
                        }
                        winit::keyboard::KeyCode::Semicolon => {
                            controllers.0.buttons.start = state.is_pressed()
                        }
                        winit::keyboard::KeyCode::ArrowUp => {
                            controllers.1.buttons.up = state.is_pressed()
                        }
                        winit::keyboard::KeyCode::ArrowLeft => {
                            controllers.1.buttons.left = state.is_pressed()
                        }
                        winit::keyboard::KeyCode::ArrowDown => {
                            controllers.1.buttons.down = state.is_pressed()
                        }
                        winit::keyboard::KeyCode::ArrowRight => {
                            controllers.1.buttons.right = state.is_pressed()
                        }
                        winit::keyboard::KeyCode::Digit7 => {
                            controllers.1.buttons.a = state.is_pressed()
                        }
                        winit::keyboard::KeyCode::Digit8 => {
                            controllers.1.buttons.b = state.is_pressed()
                        }
                        winit::keyboard::KeyCode::Digit9 => {
                            controllers.1.buttons.select = state.is_pressed()
                        }
                        winit::keyboard::KeyCode::Digit0 => {
                            controllers.1.buttons.start = state.is_pressed()
                        }
                        _ => error!("Unhandled key"),
                    },
                    _ => error!("Unknown keycode"),
                }
            }
            WindowEvent::RedrawRequested => {
                if let Some(screen) = &mut self.screen {
                    let (width, height) = {
                        let size = screen.window.inner_size();
                        (size.width, size.height)
                    };
                    if let Some((w, h)) = NonZeroU32::new(width).zip(NonZeroU32::new(height)) {
                        screen.surface.resize(w, h).unwrap();
                    }

                    self.data.draw(screen)
                }
            }
            _ => (),
        }
    }

    fn about_to_wait(&mut self, _event_loop: &ActiveEventLoop) {
        self.screen.iter().for_each(|s| s.window.request_redraw());
    }
}

fn main() {
    Builder::from_default_env()
        .filter_level(log::LevelFilter::Error)
        .init();

    let mut app = Nes::default();

    let mut buf: Vec<u8> = vec![];
    let mut file = File::open(Path::new("nes/alter.nes")).unwrap();
    file.read_to_end(&mut buf).unwrap();

    app.data.parse_nes(buf);

    let _ = app.run();
}

#[test]
fn vram_write() {
    let mut app = Nes::default();
    let mut ppu = app.data.ppu.borrow_mut();
    ppu.v = 0x2305;
    ppu.write_addr(&mut app.data.cpu, 0x66);
    assert_eq!(ppu.mem[0x0305], 0x66);
    assert_eq!(ppu.mem[ppu.mirror_mem(0x2305) as usize], 0x66);
}
