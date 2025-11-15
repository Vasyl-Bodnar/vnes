use std::{f32::consts::PI, sync::Arc};

use ringbuf::{storage::Heap, traits::Producer, wrap::caching::Caching, SharedRb};

use crate::cpu;

const DUTY_SEQ: [u8; 4] = [0b01000000, 0b01100000, 0b01111000, 0b10011111];
const HPASS1: f32 = -2. * PI * 90. / 48000.;
const HPASS2: f32 = -2. * PI * 440. / 48000.;
const LPASS: f32 = -2. * PI * 14000. / 48000.;

trait Channel {
    fn out(&self) -> u8;
    fn clock(&mut self);
    fn qt_clock(&mut self);
    fn hf_clock(&mut self);

    fn qh_clock(&mut self) {
        self.qt_clock();
        self.hf_clock();
    }
}

#[derive(Default)]
pub struct FrameCounter {
    pub mode: bool,
    pub interrupt_inh: bool,
}

#[derive(Default)]
pub struct Timer {
    // 11 bits total
    pub timer_lo: u8, // 8 bits
    pub timer_hi: u8, // 3 bits
}

impl Timer {
    fn inc_time(&mut self) -> u16 {
        let time = (self.time() + 1) % 0x7FF;
        self.set_time(time);
        time
    }

    fn time(&self) -> u16 {
        (self.timer_lo as u16) | (((self.timer_hi & 0x7) as u16) << 8)
    }

    fn set_time(&mut self, timer: u16) {
        self.timer_lo = (timer & 0xFF) as u8;
        self.timer_hi = ((timer & 0x700) >> 8) as u8;
    }
}

#[derive(Default)]
pub struct LengthCounter {
    pub halt: bool,
    pub length_cnt: u8,
}

impl LengthCounter {
    pub fn clock(&mut self) {
        if self.length_cnt > 0 {
            self.length_cnt -= 1;
        }
        if self.halt && self.length_cnt == 0 {
            self.length_cnt = 15;
        }
    }

    pub fn done(&self) -> bool {
        self.length_cnt == 0
    }
}

#[derive(Default)]
pub struct Divider {
    pub period: u16,
    counter: u16,
}

impl Divider {
    pub fn clock(&mut self) -> bool {
        if self.counter == 0 {
            self.counter = self.period;
            return true;
        }
        self.counter -= 1;
        false
    }
}

#[derive(Default)]
pub struct Envelope {
    pub div: Divider,
    pub decay: u8,
    pub vol: u8,
    pub lop: bool,
    pub constn: bool,
    pub start: bool,
}

impl Envelope {
    pub fn clock(&mut self) {
        if self.start {
            self.start = false;
            self.decay = 15;
        } else {
            if self.div.clock() {
                if self.decay == 0 && self.lop {
                    self.decay = 15;
                } else {
                    self.decay -= 1;
                }
            }
        }
    }

    pub fn out(&self) -> u8 {
        if self.constn {
            self.vol
        } else {
            self.decay
        }
    }
}

#[derive(Default)]
pub struct Sweep {
    pub div: Divider,
    pub enable: bool,
    pub negate: bool,
    pub shift: u8,
    pub target: u16,
    pub compliment: bool,
    pub reload: bool,
}

impl Sweep {
    pub fn clock(&mut self, time: u16) -> Option<u16> {
        let mut change = (time >> self.shift) as i16;
        if self.negate {
            if self.compliment {
                change = -change - 1;
            } else {
                change = -change;
            }
            self.target = (time as i16 + change) as u16;
        }

        let div = self.div.clock();

        let res = if change > 0 && self.target < 0x7FF {
            if self.enable && self.shift > 0 && div {
                Some(self.target)
            } else {
                None
            }
        } else {
            Some(0)
        };

        if self.reload {
            self.div.counter = self.div.period;
            self.reload = false;
        }

        res
    }
}

#[derive(Default)]
pub struct Pulse {
    pub env: Envelope,
    pub sweep: Sweep,
    pub timer: Timer,
    pub length_cnt: LengthCounter,
    pub sweep_mute: bool,
    pub seq: u8, // DUTY

    pub reset: bool,
    // 0x4000/4
    pub duty: u8, // 2 bits
}

impl Channel for Pulse {
    fn out(&self) -> u8 {
        if self.timer.time() < 8
            || ((DUTY_SEQ[self.duty as usize] & (1 << self.seq)) != 0)
            || self.length_cnt.done()
            || self.sweep_mute
        {
            0
        } else {
            self.env.out()
        }
    }

    fn clock(&mut self) {
        let time = self.timer.inc_time();
        if time == 0 {
            self.seq = (self.seq + 1) % 8;
        }
    }

    fn qt_clock(&mut self) {
        self.env.clock();
    }

    fn hf_clock(&mut self) {
        self.length_cnt.clock();

        match self.sweep.clock(self.timer.time()) {
            None => self.sweep_mute = false,
            Some(0) => self.sweep_mute = true,
            Some(target) => {
                self.sweep_mute = false;
                self.timer.set_time(target)
            }
        }
    }
}

#[derive(Default)]
pub struct Triangle {
    pub timer: Timer,
    pub length_cnt: LengthCounter,
    pub seq: u8,
    pub linear_rld: bool,
    pub linear_cnt: u8,
    // 0x4008
    pub linear_rld_cnt: u8, // 7 bits
    pub control: bool,
}

impl Channel for Triangle {
    fn out(&self) -> u8 {
        // TODO: Timer can be set to frequencies too high to even hear,
        // we do not consider them while our accuracy overall is not high enough to bother
        if self.timer.time() < 2 || self.length_cnt.done() || self.linear_cnt == 0 {
            0
        } else {
            if self.seq >= 16 {
                16 - (self.seq % 16)
            } else {
                self.seq
            }
        }
    }

    fn clock(&mut self) {
        let time = self.timer.inc_time();
        if time == 0 && self.linear_cnt > 0 && !self.length_cnt.done() {
            self.seq = (self.seq + 1) % 32;
        }
    }

    fn qt_clock(&mut self) {
        self.dec_linear_cnt();
    }

    fn hf_clock(&mut self) {
        self.length_cnt.clock();
    }
}

impl Triangle {
    fn dec_linear_cnt(&mut self) {
        if !self.control {
            self.linear_rld = false;
        }
        if self.linear_rld {
            self.linear_cnt = self.linear_rld_cnt;
        } else {
            self.linear_cnt -= 1;
        }
    }
}

pub struct Noise {
    pub env: Envelope,
    pub length_cnt: LengthCounter,
    pub div: Divider,
    pub mode: bool,
    pub shift: u16,
}

impl Default for Noise {
    fn default() -> Self {
        Self {
            env: Default::default(),
            length_cnt: Default::default(),
            div: Default::default(),
            mode: false,
            shift: 1,
        }
    }
}

impl Channel for Noise {
    fn out(&self) -> u8 {
        if self.length_cnt.done() || self.shift & 0x1 != 0 {
            0
        } else {
            self.env.out()
        }
    }

    fn clock(&mut self) {
        if self.div.clock() {
            let feedback = (self.shift & 0x1)
                ^ if self.mode {
                    (self.shift & 0x40) >> 6
                } else {
                    (self.shift & 0x2) >> 1
                };
            self.shift >>= 1;
            self.shift &= 0x3FFF;
            self.shift |= feedback << 14;
        }
    }

    fn qt_clock(&mut self) {
        self.env.clock();
    }

    fn hf_clock(&mut self) {
        self.length_cnt.clock();
    }
}

#[derive(Default)]
pub struct Dmc {}

#[derive(Default, Clone, Copy)]
pub struct Status {
    dmc_int: bool,
    frame_int: bool,
    _unknown: bool,
    dmc: bool,
    noise: bool,
    tri: bool,
    pulse1: bool,
    pulse0: bool,
}

impl From<u8> for Status {
    fn from(value: u8) -> Self {
        Self {
            dmc_int: value & 0x80 != 0,
            frame_int: value & 0x40 != 0,
            _unknown: value & 0x20 != 0,
            dmc: value & 0x10 != 0,
            noise: value & 0x08 != 0,
            tri: value & 0x04 != 0,
            pulse1: value & 0x02 != 0,
            pulse0: value & 0x01 != 0,
        }
    }
}

impl From<Status> for u8 {
    fn from(value: Status) -> Self {
        let mut byte = 0;
        if value.dmc_int {
            byte |= 1;
        }
        if value.frame_int {
            byte |= 2;
        }
        if value._unknown {
            byte |= 4;
        }
        if value.dmc {
            byte |= 8;
        }
        if value.noise {
            byte |= 16;
        }
        if value.tri {
            byte |= 32;
        }
        if value.pulse1 {
            byte |= 64;
        }
        if value.pulse0 {
            byte |= 128;
        }
        byte
    }
}

#[derive(Default)]
struct Filter {
    pub hp1_prev_unfil: f32,
    pub hp1_prev_fil: f32,
    pub hp2_prev_unfil: f32,
    pub hp2_prev_fil: f32,
    pub lp_prev_fil: f32,
}

impl Filter {
    fn filter(&mut self, cur: f32) -> f32 {
        let s1 = f32::exp(HPASS1) * (self.hp1_prev_fil + cur - self.hp2_prev_unfil);
        (self.hp1_prev_fil, self.hp1_prev_unfil) = (s1, cur);
        let s2 = f32::exp(HPASS2) * (self.hp2_prev_fil + s1 - self.hp2_prev_unfil);
        (self.hp2_prev_fil, self.hp2_prev_unfil) = (s2, s1);
        let s3 = f32::exp(LPASS) * s2 + ((1. - f32::exp(LPASS)) * self.lp_prev_fil);
        self.lp_prev_fil = s3;
        s3.clamp(-1., 1.)
    }
}

pub struct State {
    cycles: u16,
    pub avg: f32,
    filter: Filter,
    pub buf: Caching<Arc<SharedRb<Heap<f32>>>, true, false>,
    pub sample_rate: u32,
    pub framecnt: FrameCounter,
    pub pulse0: Pulse,
    pub pulse1: Pulse,
    pub tri: Triangle,
    pub noise: Noise,
    pub dmc: Dmc,
}

impl Default for State {
    fn default() -> Self {
        let mut pulse0: Pulse = Default::default();
        pulse0.sweep.compliment = true;
        State {
            cycles: 0,
            avg: 0.,
            pulse0,
            pulse1: Default::default(),
            noise: Default::default(),
            tri: Default::default(),
            dmc: Default::default(),
            framecnt: Default::default(),
            filter: Default::default(),
            sample_rate: 0,
            buf: Caching::new(Arc::new(SharedRb::new(1))),
        }
    }
}

impl State {
    fn mix(&self) -> f32 {
        let pulse = (self.pulse0.out() + self.pulse1.out()) as f32;
        let pulse = if pulse == 0. {
            0.
        } else {
            95.88 / ((8128. / pulse) + 100.)
        };

        let tnd = (self.tri.out() as f32) / 8227. + (self.noise.out() as f32) / 12241.; // + self.dmc.out();
        let tnd = if tnd == 0. {
            0.
        } else {
            159.79 / ((1. / tnd) + 100.)
        };

        pulse + tnd
    }

    fn qt_clock(&mut self) {
        self.pulse0.qt_clock();
        self.pulse1.qt_clock();
        self.tri.qt_clock();
    }

    fn qh_clock(&mut self) {
        self.pulse0.qh_clock();
        self.pulse1.qh_clock();
        self.tri.qh_clock();
    }

    pub fn cycle(&mut self, cpu: &mut cpu::State, cpu_cycle: bool) {
        if cpu_cycle {
            // Triangle wave clocks on cpu time
            self.tri.clock();
            return;
        }

        match self.cycles {
            // quarter frame, clock envelopes and triangle linear counter
            3728 | 11185 => {
                self.qt_clock();
            }
            // quarter and half frame, also do sweep units and length counters
            7456 => {
                self.qh_clock();
            }
            // quarter and half frame
            14914 if !self.framecnt.mode => {
                if !self.framecnt.interrupt_inh {
                    cpu.irq = true;
                }
                self.qh_clock();
            }
            // reset
            14915 if !self.framecnt.mode => {
                self.cycles = 0;
                if !self.framecnt.interrupt_inh {
                    cpu.irq = true;
                }
            }
            // quarter and half frame
            18640 if self.framecnt.mode => {
                self.qh_clock();
            }
            // reset
            18641 if self.framecnt.mode => {
                self.cycles = 0;
            }
            _ => {}
        }

        self.pulse0.clock();
        self.pulse1.clock();
        self.cycles = self.cycles.wrapping_add(1);

        let decim = crate::CPU_CYCLES_PER_SEC / self.sample_rate as usize;
        if self.cycles as usize % decim == 0 {
            self.avg = (self.avg + self.mix()) / decim as f32;
            let _ = self.buf.try_push(self.filter.filter(self.avg));
            self.avg = 0.;
        } else {
            self.avg += self.mix();
        }
    }
}
