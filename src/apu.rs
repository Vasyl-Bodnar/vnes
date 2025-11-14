use std::{f32::consts::PI, sync::Arc};

use log::error;
use ringbuf::{storage::Heap, traits::Producer, wrap::caching::Caching, SharedRb};

use crate::cpu;

const DUTY_SEQ: [u8; 4] = [0b01000000, 0b01100000, 0b01111000, 0b10011111];
const HPASS1: f32 = -2. * PI * 90. / 48000.;
const HPASS2: f32 = -2. * PI * 440. / 48000.;
const LPASS: f32 = -2. * PI * 14000. / 48000.;

#[derive(Default)]
pub struct FrameCounter {
    pub mode: bool,
    pub interrupt_inh: bool,
}

#[derive(Default)]
pub struct Divider {
    pub period: u8,
    counter: u8,
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
    pub start: bool,
    pub constn: bool,
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
    pub fn clock(&mut self, timer: u16) -> Option<u16> {
        let mut change = (timer >> self.shift) as i16;
        if self.negate {
            if self.compliment {
                change = -change - 1;
            } else {
                change = -change;
            }
            self.target = (timer as i16 + change) as u16;
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
            self.div.period = self.div.period;
            self.reload = false;
        }

        res
    }
}

#[derive(Default)]
pub struct Pulse {
    pub env: Envelope,
    pub sweep: Sweep,
    pub sweep_mute: bool,
    pub seq: u8, // DUTY

    pub reset: bool,
    // 0x4000/4
    pub duty: u8,         // 2 bits
    pub count_halt: bool, // Also counter's halt
    // 0x4002/6
    pub timer_lo: u8,
    // 0x4003/7
    pub length_cnt: u8, // 5 bits
    pub timer_hi: u8,   // 3 bits
}

impl Pulse {
    pub fn out(&self) -> u8 {
        if self.timer() < 8
            || ((DUTY_SEQ[self.duty as usize] & (1 << self.seq)) != 0)
            || self.length_cnt == 0
            || self.sweep_mute
        {
            0
        } else {
            self.env.out()
        }
    }

    pub fn clock(&mut self) {
        if self.reset {
            self.seq = 0;
            self.env.start = true;
            self.reset = false;
        }

        let timer = (self.timer() + 1) % 0x7FF;
        if timer == 0 {
            self.seq = (self.seq + 1) % 8;
        }
        self.set_timer(timer);
    }

    pub fn qh_clock(&mut self) {
        self.qt_clock();
        self.hf_clock();
    }

    pub fn qt_clock(&mut self) {
        self.env.clock();
    }

    pub fn hf_clock(&mut self) {
        self.dec_length_cnt();

        match self.sweep.clock(self.timer()) {
            None => self.sweep_mute = false,
            Some(0) => self.sweep_mute = true,
            Some(target) => {
                self.sweep_mute = false;
                self.set_timer(target)
            }
        }
    }

    fn dec_length_cnt(&mut self) {
        if self.length_cnt > 0 {
            self.length_cnt -= 1;
        }
        if self.count_halt && self.length_cnt == 0 {
            self.length_cnt = 15;
        }
    }

    fn timer(&self) -> u16 {
        (self.timer_lo as u16) | ((self.timer_hi as u16) << 8)
    }

    fn set_timer(&mut self, timer: u16) {
        self.timer_lo = (timer & 0xFF) as u8;
        self.timer_hi = ((timer & 0x700) >> 8) as u8;
    }
}

#[derive(Default)]
pub struct Triangle {}

#[derive(Default)]
pub struct Noise {}

pub struct State {
    cycles: u16,
    pub avg: f32,
    pub hp1_prev_unfil: f32,
    pub hp1_prev_fil: f32,
    pub hp2_prev_unfil: f32,
    pub hp2_prev_fil: f32,
    pub lp_prev_fil: f32,
    pub buf: Caching<Arc<SharedRb<Heap<f32>>>, true, false>,
    pub framecnt: FrameCounter,
    pub pulse0: Pulse,
    pub pulse1: Pulse,
    pub tri: Triangle,
    pub noise: Noise,
}

impl Default for State {
    fn default() -> Self {
        let mut pulse0: Pulse = Default::default();
        pulse0.sweep.compliment = true;
        State {
            cycles: 0,
            avg: 0.,
            hp1_prev_unfil: 0.,
            hp1_prev_fil: 0.,
            hp2_prev_unfil: 0.,
            hp2_prev_fil: 0.,
            lp_prev_fil: 0.,
            pulse0,
            buf: Caching::new(Arc::new(SharedRb::new(1))),
            pulse1: Default::default(),
            tri: Default::default(),
            noise: Default::default(),
            framecnt: Default::default(),
        }
    }
}

impl State {
    fn mix(&self) -> f32 {
        let pulse = (self.pulse0.out() + self.pulse1.out()) as f32;
        if pulse == 0. {
            0.
        } else {
            95.88 / ((8128. / pulse) + 100.)
        }
    }

    fn filter(&mut self, cur: f32) -> f32 {
        let s1 = f32::exp(HPASS1) * (self.hp1_prev_fil + cur - self.hp2_prev_unfil);
        (self.hp1_prev_fil, self.hp1_prev_unfil) = (s1, cur);
        let s2 = f32::exp(HPASS2) * (self.hp2_prev_fil + s1 - self.hp2_prev_unfil);
        (self.hp2_prev_fil, self.hp2_prev_unfil) = (s2, s1);
        let s3 = f32::exp(LPASS) * s2 + ((1. - f32::exp(LPASS)) * self.lp_prev_fil);
        self.lp_prev_fil = s3;
        s3
    }

    pub fn cycle(&mut self, cpu: &mut cpu::State, cpu_cycle: bool) {
        if cpu_cycle {
            // Triangle wave clocks on cpu time
            //self.tri.clock();
            return;
        }

        match self.cycles {
            // quarter frame, clock envelopes and triangle linear counter
            3728 | 11185 => {
                self.pulse0.qt_clock();
                self.pulse1.qt_clock();
            }
            // quarter and half frame, also do sweep units and length counters
            7456 => {
                self.pulse0.hf_clock();
                self.pulse1.hf_clock();
            }
            // quarter frame
            // quarter and half frame
            14914 if !self.framecnt.mode => {
                if !self.framecnt.interrupt_inh {
                    cpu.irq = true;
                }

                self.pulse0.qh_clock();
                self.pulse1.qh_clock();
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
                self.pulse0.qh_clock();
                self.pulse1.qh_clock();
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

        // TODO: Hardcoded assumption
        if self.cycles as usize % 37 == 0 {
            self.avg = self.filter(self.avg / 37.);
            //error!("{}", self.avg);
            let _ = self.buf.try_push(self.avg);
            self.avg = 0.;
        } else {
            self.avg += self.mix();
        }
    }
}
