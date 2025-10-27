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

        // TODO: Rework to be more accurate
        if self.scanline == 0 {
            if !(240..=261).contains(&self.scanline)
                && (cpu.get_mask().bg_rend || cpu.get_mask().spr_rend)
            {
                match self.cycles {
                    1..64 => {
                        self.oam_snd[(self.cycles / 2) as usize] = 0xFF;
                    }

                    64 => {
                        self.oam_snd[0] = 0xFF;

                        self.oam_st.task = 0;
                        self.oam_st.n = 0;
                        self.oam_st.snd_n = 0;
                        self.oam_st.m = 0;
                    }

                    65..257 => match self.oam_st.task {
                        0 => {
                            if self.cycles % 2 == 0 {
                                if self.oam_st.snd_n < 8 {
                                    self.oam_snd
                                        [(4 * self.oam_st.snd_n + self.oam_st.m) as usize] =
                                        self.oam_st.buf;
                                    self.oam_st.m += 1;
                                }
                            } else {
                                self.oam_st.buf =
                                    self.oam[(4 * self.oam_st.n + self.oam_st.m) as usize];
                                if (0..241).contains(&self.oam_st.buf) {
                                    self.oam_st.task = 1;
                                }
                            }
                        }
                        1 => {
                            if self.cycles % 2 == 0 {
                                if self.oam_st.snd_n < 8 {
                                    self.oam_snd
                                        [(4 * self.oam_st.snd_n + self.oam_st.m) as usize] =
                                        self.oam_st.buf;
                                    self.oam_st.m += 1;

                                    if self.oam_st.m >= 4 {
                                        self.oam_st.m = 0;
                                        self.oam_st.task = 2;
                                    }
                                }
                            } else {
                                self.oam_st.buf =
                                    self.oam[(4 * self.oam_st.n + self.oam_st.m) as usize];

                                // Use unimplemented bit to recognize this as "snd_oam" sprite
                                if self.oam_st.m == 2 {
                                    self.oam[(4 * self.oam_st.n + self.oam_st.m) as usize] |= 0x10;
                                }
                            }
                        }
                        2 => {
                            self.oam_st.n += 1;
                            if self.oam_st.n >= 64 {
                                self.oam_st.task = 4;
                            } else if self.oam_st.snd_n < 8 {
                                self.oam_st.task = 0;
                            } else {
                                self.oam_st.task = 3;
                            }
                        }
                        3 => {
                            if (0..241)
                                .contains(&self.oam[(4 * self.oam_st.n + self.oam_st.m) as usize])
                            {
                                let mut stt = cpu.get_stt();
                                stt.sprite_overflow = true;
                                cpu.put_stt(stt);
                                // NOTE: Technically, this is a state where we need to read more entries, but
                                // this does not concern us at this point
                                self.oam_st.task = 4;
                            } else {
                                self.oam_st.n += 1;
                                // NOTE: This one is a NES bug
                                self.oam_st.m += 1;

                                if self.oam_st.n >= 64 {
                                    self.oam_st.task = 4
                                }
                            }
                        }
                        4 => {}
                        _ => unreachable!(),
                    },
                    // NOTE: Fetches and prefetches occur after
                    _ => {}
                }
            }
        }

        // TODO: This offset is suspicious, but required at the moment for the best result
        let pixel_x = (self.cycles + 16) % 256;
        let pixel_y = self.scanline;

        // NOTE: While good correctness is currently the goal, there are lot of parts that get
        // recomputed far more than they need to.
        if self.cycles < 256
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
            let att = (att_table >> 2 * ((((pixel_y) >> 3) & 0x2) | ((pixel_x >> 4) & 0x1))) & 0x3;

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

            fb_buf.0[(pixel_y * 256 + pixel_x) as usize].0 =
                self.palette[0] | ((bg_val & 0x3) << 6);
            fb_buf.0[(pixel_y * 256 + pixel_x) as usize].1 =
                self.palette[bg_val as usize] | ((bg_val & 0xC) << 4);

            // For now, we will do sprite updates in some specific time
            // instead of proper NES-accurate method
            if pixel_x == 240 && (1..65).contains(&pixel_y) {
                let sp = (63 - (pixel_y % 64)) * 4;
                let sp_y = self.oam[sp as usize] as u16;
                if (0..240).contains(&sp_y) {
                    let sp_tile = self.oam[(sp + 1) as usize] as u16;
                    let sp_att = self.oam[(sp + 2) as usize];
                    let sp_x = self.oam[(sp + 3) as usize] as u16;

                    if cpu.get_ctrl().s_size {
                        let sp_bank = if sp_tile & 1 != 0 { 0x1000 } else { 0x0 };

                        self.draw_tile(
                            cpu,
                            fb_buf.0,
                            sp,
                            sp_bank | (sp_tile << 4),
                            sp_att,
                            sp_x,
                            sp_y + if sp_att & 0x80 != 0 { 8 } else { 0 },
                        );
                        self.draw_tile(
                            cpu,
                            fb_buf.0,
                            sp,
                            sp_bank | ((sp_tile + 1) << 4),
                            sp_att,
                            sp_x,
                            sp_y + if sp_att & 0x80 == 0 { 8 } else { 0 },
                        );
                    } else {
                        let sp_bank = if cpu.get_ctrl().sp_addr { 0x1000 } else { 0x0 };

                        let sp_chr_addr = sp_bank | (sp_tile << 4);

                        self.draw_tile(cpu, fb_buf.0, sp, sp_chr_addr, sp_att, sp_x, sp_y);
                    }
                }
            }
        }

        self.cycles = self.cycles.wrapping_add(1);
    }

    fn draw_tile(
        &mut self,
        cpu: &mut cpu::State,
        buf: &mut [(u8, u8, u8, u8)],
        sp: u16,
        sp_chr_addr: u16,
        sp_att: u8,
        sp_x: u16,
        sp_y: u16,
    ) {
        for i in 0..8 {
            for j in (0..8).rev() {
                let sp_up = (self.chr[(sp_chr_addr | 0x8 | i) as usize] >> j) & 1;
                let sp_low = (self.chr[(sp_chr_addr | i) as usize] >> j) & 1;

                let sp_val = 0x10 | ((sp_att & 0x3) << 2) | (sp_up << 1) | sp_low;

                if sp_val & 0x3 != 0
                    && cpu.get_mask().spr_rend
                    && (sp_x >= 8 || cpu.get_mask().spr_left)
                {
                    let att_p_val = (sp_val << 2) | ((sp_att >> 4) & 0x3);

                    let (x, y) = match (sp_att & 0x40 != 0, sp_att & 0x80 != 0) {
                        (false, true) => ((sp_x + 7 - j) % 256, (sp_y + 7 - i) % 241),
                        (false, false) => ((sp_x + 7 - j) % 256, (sp_y + i) % 241),
                        (true, true) => ((sp_x + j) % 256, (sp_y + 7 - i) % 241),
                        (true, false) => ((sp_x + j) % 256, (sp_y + i) % 241),
                    };

                    if buf[(y * 256 + x) as usize].2 != 0 {
                        match buf[(y * 256 + x) as usize].3 & 0x3 {
                            0b00 => {
                                buf[(y * 256 + x) as usize].2 = self.palette[sp_val as usize];
                                buf[(y * 256 + x) as usize].3 = att_p_val;
                            }
                            0b01 => {
                                if att_p_val & 1 != 0 {
                                    buf[(y * 256 + x) as usize].2 = self.palette[sp_val as usize];
                                    buf[(y * 256 + x) as usize].3 = att_p_val;
                                }
                            }
                            0b10 | 0b11 => {
                                if att_p_val & 1 != 0 {
                                    buf[(y * 256 + x) as usize].2 = self.palette[sp_val as usize];
                                    buf[(y * 256 + x) as usize].3 = att_p_val;
                                } else if att_p_val & 2 != 0 {
                                    buf[(y * 256 + x) as usize].2 = self.palette[sp_val as usize];
                                    buf[(y * 256 + x) as usize].3 = att_p_val;
                                }
                            }
                            _ => unreachable!(),
                        }
                    } else {
                        buf[(y * 256 + x) as usize].2 = self.palette[sp_val as usize];
                        buf[(y * 256 + x) as usize].3 = att_p_val;
                    }
                    if sp == 0 && !cpu.get_stt().sprite0_hit && buf[(y * 256 + x) as usize].0 > 0 {
                        let mut stt = cpu.get_stt();
                        stt.sprite0_hit = true;
                        cpu.put_stt(stt);
                    }
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
