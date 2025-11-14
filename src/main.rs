use std::{
    cell::RefCell,
    fs::File,
    io::Read,
    num::NonZeroU32,
    path::Path,
    rc::Rc,
    time::{Duration, Instant},
};

use cpal::{
    default_host,
    traits::{DeviceTrait, HostTrait, StreamTrait},
    OutputCallbackInfo, Stream,
};
use env_logger::Builder;
use log::{error, warn};
use ringbuf::{
    storage::Heap,
    traits::{Consumer, Split},
    SharedRb,
};
use softbuffer::{Context, Surface};
use winit::{
    application::ApplicationHandler,
    dpi::PhysicalSize,
    error::EventLoopError,
    event::{KeyEvent, WindowEvent},
    event_loop::{ActiveEventLoop, ControlFlow, EventLoop},
    window::{Window, WindowId},
};

mod apu;
mod controller;
mod cpu;
mod mapper;
mod ppu;
mod registers;

const WIDTH: usize = 256;
const HEIGHT: usize = 240;
const SCALE: usize = 3;

const DELTA: f32 = 1. / 60.0988;
// Currently a magicish number that produces the best result
const PPU_CYCLES_PER_FRAME: usize = 89342;

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
    cpy.copy_from_slice(&buf[0..WIDTH * HEIGHT]);

    if x == 0 || x == 1 {
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
    apu: Rc<RefCell<apu::State>>,
    ppu: Rc<RefCell<ppu::State>>,
    mapper: Rc<RefCell<mapper::State>>,
    controllers: Rc<RefCell<(controller::State, controller::State)>>,
    cpu: cpu::State,
    stream: Option<Stream>,
    budget: Duration,
    now: Instant,
}

impl Default for NesData {
    fn default() -> Self {
        NesData {
            apu: Rc::new(RefCell::new(apu::State::default())),
            ppu: Default::default(),
            mapper: Default::default(),
            controllers: Default::default(),
            cpu: Default::default(),
            stream: Default::default(),
            budget: Duration::new(0, 0),
            now: Instant::now(),
        }
    }
}

impl NesData {
    fn reset(&mut self) {
        if self.cpu.apu_st.is_none() {
            self.cpu.apu_st = Some(self.apu.clone());
        }
        if self.cpu.ppu_st.is_none() {
            self.cpu.ppu_st = Some(self.ppu.clone());
        }
        if self.cpu.controllers_st.is_none() {
            self.cpu.controllers_st = Some(self.controllers.clone());
        }
        self.cpu.reset = true;
    }

    pub fn parse_nes(&mut self, buf: Vec<u8>) {
        self.reset();

        let sig = &buf[0..16];

        if sig[7] == 8 && sig[0x0C] == 8 {
            println!("NES 2.0");
        } else if sig[7] == 4 && sig[0x0C] == 4 {
            println!("WARNING: archaic, trying anyway..");
        } else if sig[7] == 0 && sig[0x0C] == 0 {
            println!("i{}{}{}", sig[0] as char, sig[1] as char, sig[2] as char);
        } else {
            println!("WARNING: archaic, possibly 0.7, trying anyway..");
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

        let cpu = &mut self.cpu;
        let ppu = &mut self.ppu.borrow_mut();

        self.mapper = Rc::new(RefCell::new(mapper::State::new(
            cpu,
            ppu,
            mapper_num,
            prg_size as u8,
            chr_size as u8,
            &buf[16..],
        )));

        if nt_arrang {
            ppu.mirr = ppu::Mirror::Vertical;
        } else {
            ppu.mirr = ppu::Mirror::Horizontal;
        }

        self.cpu.mapper_st = Some(self.mapper.clone());
    }

    pub fn setup_sound(&mut self) {
        let host = default_host();
        let dev = host.default_output_device().expect("No device found");
        let supported = dev
            .supported_output_configs()
            .expect("Can't query configs")
            .filter(|x| x.sample_format() == cpal::SampleFormat::F32)
            .next()
            .expect("No configs")
            .with_max_sample_rate();
        // TODO: making an extra assumption here
        let (prod, mut cons) = SharedRb::<Heap<f32>>::new(7196).split();

        self.apu.borrow_mut().buf = prod;

        error!("{:?}", dev.name().unwrap());
        error!("{:?}", supported);

        self.stream = dev
            .build_output_stream(
                &supported.config(),
                move |data: &mut [f32], _: &OutputCallbackInfo| {
                    for sample in data.iter_mut() {
                        *sample = cons.try_pop().unwrap_or(0.).clamp(-1., 1.);
                    }
                },
                |err| error!("{:?}", err),
                None,
            )
            .ok();
    }

    fn frame(&mut self, buf: &mut [u32]) {
        for i in 0..PPU_CYCLES_PER_FRAME {
            if i % 3 == 0 {
                self.cpu.cycle().unwrap();
                self.apu.borrow_mut().cycle(&mut self.cpu, true);
            }
            if i % 6 == 0 {
                self.apu.borrow_mut().cycle(&mut self.cpu, false);
            }
            self.ppu.borrow_mut().cycle(&mut self.cpu, buf);
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
                        _ => warn!("Unhandled key"),
                    },
                    _ => warn!("Unknown keycode"),
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

    app.data.setup_sound();
    app.data.parse_nes(buf);
    let _ = app.data.stream.as_ref().unwrap().play();

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
