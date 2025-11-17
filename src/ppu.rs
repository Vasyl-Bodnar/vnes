use std::{fmt::Debug, ops::Range};

use log::{error, trace, warn};

use crate::cpu;

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

#[allow(dead_code)]
#[derive(Debug)]
pub enum Mirror {
    Horizontal,
    Vertical,
    Single,
    Fourway,
    Special,
}

#[derive(Default)]
struct OamState {
    buf: u8,
    n: u8,
    snd_n: u8,
    m: u8,
    read_n: u8,
    read_sp: [u8; 4],
    task: u8,
}

#[derive(Default)]
struct ShiftState {
    tile_cur_lo: u8,
    tile_cur_hi: u8,
    att_cur_lo: u8,
    att_cur_hi: u8,
    tile_next_lo: u8,
    tile_next_hi: u8,
    att_latch_lo: u8,
    att_latch_hi: u8,
}

#[allow(dead_code)]
pub struct State {
    pub mem: [u8; 0x0800],
    chr: [u8; 0x2000],
    palette: [u8; 32],
    oam: [u8; 256],
    oam_snd: [u8; 32],
    spr_pixels: [u8; 256],
    pub mirr: Mirror,
    pub oam_addr: u8,
    buf: u8,
    oam_st: OamState,
    shift_st: ShiftState,
    pub v: u16, // VRAM addr
    pub t: u16, // tmp VRAM addr
    pub x: u8,  // fine x scroll
    pub w: u8,  // first or second write for ADDR and SCROLL
    pub scanline: u16,
    pub cycles: u16,
}

impl Default for State {
    fn default() -> Self {
        State {
            chr: [0; 0x2000],
            mem: [0; 0x0800],
            palette: [0; 32],
            oam: [0; 256],
            oam_snd: [0; 32],
            spr_pixels: [0; 256],
            mirr: Mirror::Horizontal,
            oam_addr: 0,
            oam_st: OamState::default(),
            shift_st: ShiftState::default(),
            buf: 0,
            v: 0,
            t: 0,
            x: 0,
            w: 0,
            scanline: 0,
            cycles: 0,
        }
    }
}

impl State {
    pub fn cycle(&mut self, cpu: &mut cpu::State, buf: &mut [u32]) {
        match self.cycles {
            341.. => {
                self.cycles = 0;
                self.scanline += 1;

                if self.scanline == 241 {
                    let mut stt = cpu.get_stt();
                    stt.vblank = true;
                    cpu.put_stt(stt);
                    trace!("VBLANK! cyc: {}", cpu.cycles);
                }

                if self.scanline >= 262 {
                    self.scanline = 0;
                    let mut stt = cpu.get_stt();
                    stt.vblank = false;
                    stt.sprite0_hit = false;
                    stt.sprite_overflow = false;
                    cpu.put_stt(stt);
                    trace!("VBLANK OVER! cyc: {}", cpu.cycles);
                }
            }
            // Increment scroll y
            256 if !(240..261).contains(&self.scanline)
                && (cpu.get_mask().bg_rend || cpu.get_mask().spr_rend) =>
            {
                if (self.v & 0x7000) != 0x7000 {
                    self.v += 0x1000;
                } else {
                    self.v &= !0x7000;
                    let mut y = (self.v & 0x03E0) >> 5;
                    if y == 29 {
                        y = 0;
                        self.v ^= 0x0800;
                    } else if y == 31 {
                        y = 0;
                    } else {
                        y = y.wrapping_add(1);
                    }
                    self.v = (self.v & !0x03E0) | (y << 5);
                }
            }
            // Move t to v, first low, then hi multiple times
            257 if !(240..261).contains(&self.scanline)
                && (cpu.get_mask().bg_rend || cpu.get_mask().spr_rend) =>
            {
                self.v &= 0xFBE0;
                self.v |= self.t & !0xFBE0;
            }
            280..=304
                if self.scanline == 261 && (cpu.get_mask().bg_rend || cpu.get_mask().spr_rend) =>
            {
                self.v &= !0xFBE0;
                self.v |= self.t & 0xFBE0;
            }
            _ => {}
        }

        // Increment coarse scroll x on certain cycles, fill next shifts
        match self.cycles {
            328 | 336 | 8..256
                if self.cycles % 8 == 0
                    && !(240..261).contains(&self.scanline)
                    && (cpu.get_mask().bg_rend || cpu.get_mask().spr_rend) =>
            {
                let bg_bank = if cpu.get_ctrl().bp_addr { 0x1000 } else { 0x0 };
                let tile = self.mem[self.mirror_mem(0x2000 | (self.v & 0x0FFF)) as usize] as u16;
                let att = self.mem[self.mirror_mem(
                    0x23C0 | (self.v & 0x0C00) | ((self.v >> 4) & 0x38) | ((self.v >> 2) & 0x7),
                ) as usize];

                let chr_adr = (bg_bank | (tile << 4) | ((self.v >> 12) & 0x7)) as usize;

                self.shift_st.tile_next_lo = self.chr[chr_adr];
                self.shift_st.tile_next_hi = self.chr[chr_adr | 0x8];

                let att_pick = att >> (2 * (((self.v >> 5) & 0x2) | ((self.v >> 1) & 0x1)));

                self.shift_st.att_latch_lo = att_pick & 0x1;
                self.shift_st.att_latch_hi = (att_pick >> 1) & 0x1;

                if (self.v & 0x001F) == 0x001F {
                    self.v &= !0x001F;
                    self.v ^= 0x0400;
                } else {
                    self.v = self.v.wrapping_add(1);
                }
            }
            256 if !(240..261).contains(&self.scanline)
                && (cpu.get_mask().bg_rend || cpu.get_mask().spr_rend) =>
            {
                if (self.v & 0x001F) == 0x001F {
                    self.v &= !0x001F;
                    self.v ^= 0x0400;
                } else {
                    self.v = self.v.wrapping_add(1);
                }
            }
            _ => {}
        }

        match self.cycles {
            257..=320 if !(240..261).contains(&self.scanline) => {
                self.oam_addr = 0;
            }
            _ => {}
        }

        if !(239..261).contains(&self.scanline)
            && (cpu.get_mask().bg_rend || cpu.get_mask().spr_rend)
        {
            self.eval_fetch_sprites(cpu);
        }

        if self.cycles < 256 && !(240..=261).contains(&self.scanline) {
            // Get current value in the shift register
            let att_lo = (self.shift_st.att_cur_lo >> (7 - self.x)) & 0x1;
            let att_hi = (self.shift_st.att_cur_hi >> (7 - self.x)) & 0x1;
            let bg_lo = (self.shift_st.tile_cur_lo >> (7 - self.x)) & 0x1;
            let bg_hi = (self.shift_st.tile_cur_hi >> (7 - self.x)) & 0x1;

            let mut val = (att_hi << 3) | (att_lo << 2) | (bg_hi << 1) | bg_lo;

            if cpu.get_mask().greyscale {
                val &= 0x30;
            }

            if (self.cycles < 8 && !cpu.get_mask().bg_left) || !cpu.get_mask().bg_rend {
                val = 0;
            }

            let addr = (self.scanline * 256 + self.cycles) as usize;

            buf[addr] = self.selector(cpu, val, self.spr_pixels[self.cycles as usize]);
            self.spr_pixels[self.cycles as usize] = 0;
        }

        // Shift everything
        match self.cycles {
            1..256 | 321..=336
                if !(240..261).contains(&self.scanline)
                    && (cpu.get_mask().bg_rend || cpu.get_mask().spr_rend) =>
            {
                self.shift_st.att_cur_lo =
                    (self.shift_st.att_cur_lo << 1) | self.shift_st.att_latch_lo;
                self.shift_st.att_cur_hi =
                    (self.shift_st.att_cur_hi << 1) | self.shift_st.att_latch_hi;
                self.shift_st.tile_cur_lo =
                    (self.shift_st.tile_cur_lo << 1) | self.shift_st.tile_next_lo >> 7;
                self.shift_st.tile_cur_hi =
                    (self.shift_st.tile_cur_hi << 1) | self.shift_st.tile_next_hi >> 7;
                self.shift_st.tile_next_lo = (self.shift_st.tile_next_lo << 1) | 1;
                self.shift_st.tile_next_hi = self.shift_st.tile_next_hi << 1; // | 0
            }
            _ => {}
        }

        self.cycles = self.cycles.wrapping_add(1);
    }

    fn eval_fetch_sprites(&mut self, cpu: &mut cpu::State) {
        match self.cycles {
            1..64 => {
                self.oam_snd[(self.cycles / 2) as usize] = 0xFF;
            }

            64 => {
                self.oam_snd[0] = 0xFF;

                self.oam_st.task = 1;
                self.oam_st.n = 0;
                self.oam_st.snd_n = 0;
                self.oam_st.read_n = 0;
                self.oam_st.m = 0;
            }

            65..257 => match self.oam_st.task {
                0 => {}
                1 => {
                    if self.cycles % 2 == 0 {
                        if self.oam_st.snd_n < 8 {
                            self.oam_snd[(4 * self.oam_st.snd_n + self.oam_st.m) as usize] =
                                self.oam_st.buf;
                        }
                    } else {
                        self.oam_st.buf = self.oam[(4 * self.oam_st.n + self.oam_st.m) as usize];

                        let y = self.oam_st.buf;
                        if y < 0xEF
                            && ((y..y + 8).contains(&(self.scanline as u8))
                                || (cpu.get_ctrl().s_size
                                    && (y..y + 16).contains(&(self.scanline as u8))))
                        {
                            self.oam_st.task = 2;
                        } else {
                            self.oam_st.task = 3;
                        }
                    }
                }
                2 => {
                    if self.cycles % 2 == 0 {
                        if self.oam_st.snd_n < 8 {
                            if self.oam_st.n == 0 && self.oam_st.m == 2 {
                                self.oam_st.buf |= 0x4;
                            }

                            self.oam_snd[(4 * self.oam_st.snd_n + self.oam_st.m) as usize] =
                                self.oam_st.buf;
                            self.oam_st.m += 1;

                            if self.oam_st.m >= 4 {
                                self.oam_st.snd_n += 1;
                                self.oam_st.m = 0;
                                self.oam_st.task = 3;
                            }
                        }
                    } else {
                        self.oam_st.buf = self.oam[(4 * self.oam_st.n + self.oam_st.m) as usize];
                    }
                }
                3 => {
                    self.oam_st.n += 1;
                    if self.oam_st.n >= 64 {
                        self.oam_st.task = 0;
                    } else if self.oam_st.snd_n < 8 {
                        self.oam_st.task = 1;
                    } else if self.oam_st.snd_n == 8 {
                        self.oam_st.task = 4;
                    }
                }

                4 => {
                    let y = self.oam[(4 * self.oam_st.n + self.oam_st.m) as usize];
                    if y < 0xEF
                        && ((y..y + 8).contains(&(self.scanline as u8))
                            || (cpu.get_ctrl().s_size
                                && (y..y + 16).contains(&(self.scanline as u8))))
                    {
                        let mut stt = cpu.get_stt();
                        stt.sprite_overflow = true;
                        cpu.put_stt(stt);

                        self.oam_st.task = 5;
                    } else {
                        self.oam_st.n += 1;
                        // NOTE: This one is a NES bug
                        self.oam_st.m += 1;

                        if self.oam_st.n >= 64 {
                            self.oam_st.task = 0
                        }
                    }
                }
                5 => {
                    self.oam_st.m += 1;
                    if self.oam_st.m >= 4 {
                        self.oam_st.n += 1;
                        self.oam_st.m = 0;
                    }
                    self.oam_st.task = 0
                }
                _ => unreachable!(),
            },
            257..=320 => match self.oam_st.task {
                0..4 if self.oam_st.read_n < self.oam_st.snd_n => {
                    self.oam_st.read_sp[self.oam_st.task as usize] =
                        self.oam_snd[(4 * self.oam_st.read_n + self.oam_st.task) as usize];
                    self.oam_st.task += 1;
                }
                4..8 => {
                    // NOTE: Read X, do nothing with it
                    // Really just draw

                    // NOTE: Scanline accurate technically
                    if self.oam_st.task == 4 {
                        let [y, tile, att, x] = self.oam_st.read_sp;

                        let bank = if (cpu.get_ctrl().s_size && tile & 0x1 != 0)
                            || (!cpu.get_ctrl().s_size && cpu.get_ctrl().sp_addr)
                        {
                            0x1000
                        } else {
                            0x0
                        };

                        let tile = if cpu.get_ctrl().s_size && ((self.scanline as u8 - y) % 16 >= 8)
                        {
                            if att & 0x80 == 0 {
                                (tile & !1) + 1
                            } else {
                                tile & !1
                            }
                        } else if cpu.get_ctrl().s_size {
                            if att & 0x80 == 0 {
                                tile & !1
                            } else {
                                (tile & !1) + 1
                            }
                        } else {
                            tile
                        };

                        let i = (self.scanline as u8 - y) % 8;

                        let chr_addr = bank | ((tile as u16) << 4);

                        self.fetch_tile_scanline(cpu, chr_addr, att, x as u16, i);
                    }

                    self.oam_st.task += 1;
                }
                _ => {
                    self.oam_st.task = 0;
                    self.oam_st.read_n += 1;
                }
            },
            _ => {}
        }
    }

    fn fetch_tile_scanline(
        &mut self,
        cpu: &mut cpu::State,
        chr_addr: u16,
        att: u8,
        sp_x: u16,
        mut i: u8,
    ) {
        if att & 0x80 != 0 {
            i = 7 - i;
        }

        for j in 0..8 {
            let lo = (self.chr[(chr_addr | i as u16) as usize] >> j) & 1;
            let hi = (self.chr[(chr_addr | 0x8 | i as u16) as usize] >> j) & 1;

            let val =
                ((att & 0x4) << 4) | (att & 0x20) | 0x10 | ((att & 0x3) << 2) | (hi << 1) | lo;

            let x = sp_x + if att & 0x40 != 0 { j } else { 7 - j };

            let val = if cpu.get_mask().spr_rend && (x >= 8 || cpu.get_mask().spr_left) {
                val
            } else {
                0
            };

            if x < 256 {
                self.spr_pixels[x as usize] = self.selector_sp(self.spr_pixels[x as usize], val);
            }
        }
    }

    fn sprite0_hit_check(&self, cpu: &mut cpu::State, x: u16) {
        if (!cpu.get_stt().sprite0_hit)
            && cpu.get_mask().spr_rend
            && cpu.get_mask().bg_rend
            && x != 255
        {
            let mut stt = cpu.get_stt();
            stt.sprite0_hit = true;
            cpu.put_stt(stt);
        }
    }

    fn selector(&self, cpu: &mut cpu::State, bg: u8, sp: u8) -> u32 {
        let pal = self.palette[match (bg & 0x3, sp & 0x3, (sp >> 5) & 0x1) {
            (0, 0, _) => {
                if !cpu.get_mask().spr_rend
                    && !cpu.get_mask().bg_rend
                    && (0x3F00..0x4000).contains(&self.v)
                {
                    ((self.v - 0x3F00) & 0x1F) as u8
                } else {
                    0
                }
            }
            (0, _s, _) => sp & 0x1F,
            (_b, 0, _) => bg & 0xF,
            (_b, _s, 0) => {
                if (sp >> 6 & 0x1) != 0 {
                    self.sprite0_hit_check(cpu, self.cycles);
                }
                sp & 0x1F
            }
            (_b, _s, 1) => {
                if (sp >> 6 & 0x1) != 0 {
                    self.sprite0_hit_check(cpu, self.cycles);
                }
                bg & 0xF
            }
            _ => unreachable!(),
        } as usize] as usize;
        if pal >= 64 {
            // TODO: This cannot happen
            // Yet it happens in 240pee.nes on one test on entrance and a couple on exit
            SYS_PALLETE[0]
        } else {
            SYS_PALLETE[pal]
        }
    }

    fn selector_sp(&self, sp0: u8, sp1: u8) -> u8 {
        match (sp0 & 0x3, sp1 & 0x3) {
            (0, 0) => 0,
            (0, _s1) => sp1,
            (_s0, 0) => sp0,
            (_s0, _s1) => sp0,
        }
    }

    pub fn mirror_mem(&self, index: u16) -> u16 {
        match self.mirr {
            Mirror::Horizontal if (0x2000..0x2400).contains(&index) => index - 0x2000,
            Mirror::Horizontal if (0x2400..0x2800).contains(&index) => index - 0x2400,
            Mirror::Horizontal if (0x2800..0x2C00).contains(&index) => index - 0x2400,
            Mirror::Horizontal if (0x2C00..0x3F00).contains(&index) => index - 0x2800,
            Mirror::Vertical if (0x2000..0x2400).contains(&index) => index - 0x2000,
            Mirror::Vertical if (0x2400..0x2800).contains(&index) => index - 0x2000,
            Mirror::Vertical if (0x2800..0x2C00).contains(&index) => index - 0x2800,
            Mirror::Vertical if (0x2C00..0x3F00).contains(&index) => index - 0x2800,
            Mirror::Single => (index - 0x2000) % 0x400,
            Mirror::Fourway => index - 0x2000,
            _ => unimplemented!(),
        }
    }

    pub fn read_addr(&mut self, cpu: &cpu::State) -> u8 {
        let res = match self.v {
            0..0x2000 => {
                let res = self.buf;
                self.buf = self.chr[self.v as usize];
                trace!("Read CHR! {:x} from {:x}", self.buf, self.v);
                res
            }
            0x2000..0x3000 => {
                let res = self.buf;
                self.buf = self.mem[self.mirror_mem(self.v) as usize];
                trace!("Read MEM! {:x} from {:x}", self.buf, self.v);
                res
            }
            0x3000..0x3F00 => {
                let res = self.buf;
                self.buf = self.mem[self.mirror_mem(self.v - 0x1000) as usize];
                trace!("Read MEM MIR! {:x} from {:x}", self.buf, self.v);
                res
            }
            0x3F00..0x4000 => {
                let res = self.palette[((self.v - 0x3F00) & 0x1F) as usize];
                trace!("Read PALT! {:x} from {:x}", res, self.v);
                res
            }
            _ => {
                error!("Bad Read at {:x}", self.v);
                0
            }
        };

        if cpu.get_ctrl().vram_ai {
            self.v = self.v.wrapping_add(32);
        } else {
            self.v = self.v.wrapping_add(1);
        }

        res
    }

    pub fn write_addr(&mut self, cpu: &cpu::State, value: u8) {
        match self.v {
            0..0x2000 => {
                self.chr[self.v as usize] = value;
                warn!("Write CHR! {:x} to {:x}", value, self.v);
            }
            0x2000..0x3000 => {
                self.mem[self.mirror_mem(self.v) as usize] = value;
                trace!("Write MEM! {:x} to {:x}", value, self.v);
            }
            0x3000..0x3F00 => {
                self.mem[self.mirror_mem(self.v - 0x1000) as usize] = value;
                trace!("Write MEM MIR! {:x} to {:x}", value, self.v);
            }
            0x3F00..0x4000 if self.v & 0x1F == 0x00 || self.v & 0x1F == 0x10 => {
                self.palette[0x00] = value;
                self.palette[0x10] = value;
                trace!(
                    "Write PALT0! {:x} to {:x} (0x3F00 and 0x3F10)",
                    value,
                    self.v
                );
            }
            0x3F00..0x4000 => {
                self.palette[(self.v & 0x1F) as usize] = value;
                trace!("Write PALT! {:x} to {:x}", value, self.v);
            }
            _ => {
                warn!("Bad Write at {:x}", self.v);
            }
        }

        if cpu.get_ctrl().vram_ai {
            self.v = self.v.wrapping_add(32);
        } else {
            self.v = self.v.wrapping_add(1);
        }
    }

    pub fn read_oam(&mut self) -> u8 {
        if self.oam_addr % 4 == 2 {
            self.oam[self.oam_addr as usize] & 0xE3
        } else {
            self.oam[self.oam_addr as usize]
        }
    }

    pub fn write_oam(&mut self, value: u8) {
        if self.oam_addr % 4 == 2 {
            self.oam[self.oam_addr as usize] = value & 0xE3;
        } else {
            self.oam[self.oam_addr as usize] = value;
        }
        self.oam_addr = self.oam_addr.wrapping_add(1);
    }

    pub fn load_chr(&mut self, range: Range<usize>, from: &[u8]) {
        self.chr[range].copy_from_slice(from);
    }
}
