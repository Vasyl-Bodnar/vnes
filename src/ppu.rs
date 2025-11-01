use std::{fmt::Debug, ops::Range};

use log::{error, trace, warn};

use crate::cpu;

#[allow(dead_code)]
#[derive(Debug)]
pub enum Mirror {
    Horizontal,
    Vertical,
    Single,
    Fourway,
    Special,
}

// Completely safe
struct FourByte<'a>(&'a mut [(u8, u8, u8, u8)]);

impl<'a> From<&'a mut [u32]> for FourByte<'a> {
    fn from(value: &mut [u32]) -> Self {
        unsafe { FourByte(std::mem::transmute(value)) }
    }
}

impl<'a> From<FourByte<'a>> for &'a mut [u32] {
    fn from(value: FourByte<'a>) -> Self {
        unsafe { std::mem::transmute(value.0) }
    }
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

#[allow(dead_code)]
pub struct State {
    pub mem: [u8; 0x0800],
    chr: [u8; 0x2000],
    palette: [u8; 32],
    oam: [u8; 256],
    oam_snd: [u8; 32],
    pub mirr: Mirror,
    pub oam_addr: u8,
    buf: u8,
    oam_st: OamState,
    pub v: u16, // VRAM addr
    pub t: u16, // tmp VRAM addr
    pub x: u8,  // fine x scroll
    pub w: u8,  // first or second write for ADDR and SCROLL
    pub scanline: u16,
    pub cycles: u16,
}

impl Default for State {
    fn default() -> Self {
        let st = State {
            chr: [0; 0x2000],
            mem: [0; 0x0800],
            palette: [0; 32],
            oam: [0; 256],
            oam_snd: [0; 32],
            mirr: Mirror::Horizontal,
            oam_addr: 0,
            oam_st: OamState::default(),
            buf: 0,
            v: 0,
            t: 0,
            x: 0,
            w: 0,
            scanline: 0,
            cycles: 0,
        };
        st
    }
}

impl State {
    pub fn cycle(&mut self, cpu: &mut cpu::State, buf: &mut [u32]) {
        let fb_buf: FourByte = buf.into();

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
                        y += 1;
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

        // Increment coarse scroll x on certain cycles
        match self.cycles {
            328 | 336 | 8..=256
                if self.cycles % 8 == 0
                    && !(240..261).contains(&self.scanline)
                    && (cpu.get_mask().bg_rend || cpu.get_mask().spr_rend) =>
            {
                if (self.v & 0x001F) == 31 {
                    self.v &= !0x001F;
                    self.v ^= 0x0400;
                } else {
                    self.v += 1;
                }
            }
            _ => {}
        }

        match self.cycles {
            257..=320 if self.scanline == 261 || (0..240).contains(&self.scanline) => {
                self.oam_addr = 0;
            }
            _ => {}
        }

        if !(240..=261).contains(&self.scanline)
            && (cpu.get_mask().bg_rend || cpu.get_mask().spr_rend)
        {
            self.eval_draw_sprites(fb_buf.0, cpu);
        }

        // TODO: This offset is suspicious, but required at the moment for the best result
        let pixel_x = self.cycles;
        let pixel_y = self.scanline;

        // NOTE: While good correctness is currently the goal, there are lot of parts that get
        // recomputed far more than they need to.
        if self.cycles <= 256
            && !(240..=261).contains(&self.scanline)
            && (cpu.get_mask().bg_rend || cpu.get_mask().spr_rend)
        {
            let bg_bank = if cpu.get_ctrl().bp_addr { 0x1000 } else { 0x0 };

            let tile_addr = self.mirror_mem(0x2000 | (self.v & 0x0FFF));
            let att_table_addr = self.mirror_mem(
                0x23C0 | (self.v & 0x0C00) | ((self.v >> 4) & 0x38) | ((self.v >> 2) & 0x7),
            );

            let bg_tile = self.mem[tile_addr as usize] as u16;
            let att_table = self.mem[att_table_addr as usize];
            let att =
                (att_table >> 2 * ((((pixel_y) >> 3) & 0x2) | (((pixel_x + 16) >> 4) & 0x1))) & 0x3;

            let bg_chr_addr = bg_bank | (bg_tile << 4) | ((self.v & 0x7000) >> 12);

            let bg_up = (self.chr[(bg_chr_addr | 0x8) as usize] >> (7 - (pixel_x % 8))) & 1;
            let bg_low = (self.chr[bg_chr_addr as usize] >> (7 - (pixel_x % 8))) & 1;

            let mut bg_val = (att << 2) | (bg_up << 1) | bg_low;

            if cpu.get_mask().greyscale {
                bg_val &= 0x30;
            }

            if pixel_x < 8 && !cpu.get_mask().bg_left {
                bg_val = 0;
            }

            if !cpu.get_mask().bg_rend {
                bg_val = 0;
            }

            fb_buf.0[(pixel_y * 256 + pixel_x + 16) as usize].0 =
                self.palette[0] | ((bg_val & 0x3) << 6);
            fb_buf.0[(pixel_y * 256 + pixel_x + 16) as usize].1 =
                self.palette[bg_val as usize] | ((bg_val & 0xC) << 4);
        }

        self.cycles = self.cycles.wrapping_add(1);
    }

    fn eval_draw_sprites(&mut self, buf: &mut [(u8, u8, u8, u8)], cpu: &mut cpu::State) {
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
                        if (y..y + 8).contains(&((self.scanline + 1) as u8))
                            || (cpu.get_ctrl().s_size
                                && (y..y + 16).contains(&((self.scanline + 1) as u8)))
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
                    if (y..y + 8).contains(&((self.scanline + 1) as u8))
                        || (cpu.get_ctrl().s_size
                            && (y..y + 16).contains(&((self.scanline + 1) as u8)))
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
            257..321 => match self.oam_st.task {
                0..4 if self.oam_st.read_n < self.oam_st.snd_n => {
                    self.oam_st.read_sp[self.oam_st.task as usize] =
                        self.oam_snd[(4 * self.oam_st.read_n + self.oam_st.task) as usize];
                    self.oam_st.task += 1;
                }
                4..8 => {
                    // NOTE: Read X, do nothing with it
                    // Really just draw

                    // NOTE: Scanline accurate technically,
                    // however, this may not be pixel accurate
                    // where it can matter, e.g. sprite 0 hit
                    if self.oam_st.task == 4 {
                        let [y, mut tile, att, x] = self.oam_st.read_sp;

                        let bank =
                            if (cpu.get_ctrl().s_size && tile & 1 != 0) || cpu.get_ctrl().sp_addr {
                                0x1000
                            } else {
                                0x0
                            };

                        tile += if cpu.get_ctrl().s_size
                            && (((self.scanline + 1) as u8 - y) % 16) >= 8
                        {
                            1
                        } else {
                            0
                        };

                        let y_adj = (y + if cpu.get_ctrl().s_size
                            && (((self.scanline + 1) as u8 - y) % 16 >= 8)
                        {
                            8
                        } else {
                            0
                        }) as u16;

                        let i = ((self.scanline + 1) as u8 - y) % 8;

                        let chr_addr = bank | ((tile as u16) << 4);

                        self.draw_tile_scanline(cpu, buf, chr_addr, att, x as u16, y_adj, i);

                        // Doing this might mess up timings
                        if self.oam_st.read_n == 0
                            && !cpu.get_stt().sprite0_hit
                            && buf[(y * 255 + x) as usize].0 > 0
                        {
                            let mut stt = cpu.get_stt();
                            stt.sprite0_hit = true;
                            cpu.put_stt(stt);
                        }
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

    fn draw_tile_scanline(
        &mut self,
        cpu: &mut cpu::State,
        buf: &mut [(u8, u8, u8, u8)],
        chr_addr: u16,
        att: u8,
        x: u16,
        y: u16,
        i: u8,
    ) {
        for j in (0..8).rev() {
            let up = (self.chr[(chr_addr | 0x8 | (i % 8) as u16) as usize] >> j) & 1;
            let low = (self.chr[(chr_addr | (i % 8) as u16) as usize] >> j) & 1;

            let val = 0x10 | ((att & 0x3) << 2) | (up << 1) | low;

            if cpu.get_mask().spr_rend && (x >= 8 || cpu.get_mask().spr_left) {
                let att_p_val = (val << 1) | ((att >> 5) & 0x1);

                let (x, y) = match (att & 0x40 != 0, att & 0x80 != 0) {
                    (false, true) => (x + 7 - j, y + 7 - i as u16),
                    (false, false) => (x + 7 - j, y + i as u16),
                    (true, true) => (x + j, y + 7 - i as u16),
                    (true, false) => (x + j, y + i as u16),
                };
                if buf[(y * 256 + x) as usize].2 != 0 {
                    match (buf[(y * 256 + x) as usize].3 >> 1) & 0x3 {
                        0 => {
                            buf[(y * 256 + x) as usize].2 = self.palette[val as usize];
                            buf[(y * 256 + x) as usize].3 = att_p_val;
                        }
                        _ => {}
                    }
                } else {
                    buf[(y * 256 + x) as usize].2 = self.palette[val as usize];
                    buf[(y * 256 + x) as usize].3 = att_p_val;
                }
            }
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
            0x3000..0x3f00 => {
                let res = self.buf;
                self.buf = self.mem[self.mirror_mem(self.v - 0x1000) as usize];
                trace!("Read MEM MIR! {:x} from {:x}", self.buf, self.v);
                res
            }
            0x3f00..0x4000 => {
                let res = self.palette[((self.v - 0x3f00) % 32) as usize];
                trace!("Read PALT! {:x} from {:x}", res, self.v);
                res
            }
            _ => {
                error!("Bad Read at {:x}", self.v);
                unimplemented!()
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
            0x3000..0x3f00 => {
                self.mem[self.mirror_mem(self.v - 0x1000) as usize] = value;
                trace!("Write MEM MIR! {:x} to {:x}", value, self.v);
            }
            0x3f00..0x4000 => {
                self.palette[((self.v - 0x3f00) % 32) as usize] = value;
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
