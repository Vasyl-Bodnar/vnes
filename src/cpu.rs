use std::{cell::RefCell, fmt::Debug, ops::Range, rc::Rc};

use log::trace;

use crate::apu;
use crate::controller;
use crate::mapper;
use crate::ppu;
use crate::registers;

#[derive(Default, Clone, Copy)]
struct Status {
    carry: bool,
    iszero: bool,
    int_dis: bool,
    decimal: bool,
    b_flag: bool,
    one: bool,
    overflow: bool,
    negative: bool,
}

impl Debug for Status {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "{{ c: {}, z: {}, i: {}, d: {}, b: {}, o: {}, v: {}, n: {}, byte: {:x} }}",
            self.carry as u8,
            self.iszero as u8,
            self.int_dis as u8,
            self.decimal as u8,
            self.b_flag as u8,
            self.one as u8,
            self.overflow as u8,
            self.negative as u8,
            u8::from(*self),
        )
    }
}

impl From<Status> for u8 {
    fn from(p: Status) -> Self {
        let mut res = 0u8;
        if p.carry {
            res |= 1;
        }
        if p.iszero {
            res |= 2;
        }
        if p.int_dis {
            res |= 4;
        }
        if p.decimal {
            res |= 8;
        }
        if p.b_flag {
            res |= 16;
        }
        if p.one {
            res |= 32;
        }
        if p.overflow {
            res |= 64;
        }
        if p.negative {
            res |= 128;
        }
        res
    }
}

#[derive(Debug, Clone, Copy)]
pub enum Value {
    Impl,
    Acc,
    Imm(u8),
    ZP(u8),
    ZPX(u8),
    ZPY(u8),
    Abs(u16),
    AbsX(u16),
    AbsY(u16),
    Rel(u8),
    Ind(u16),
    IdxInd(u8), // X
    IndIdx(u8), // Y
}

#[allow(dead_code)]
#[derive(Debug, Clone, Copy)]
pub enum Instr {
    Adc(Value),
    And(Value),
    Asl(Value),
    Bcc(Value),
    Bcs(Value),
    Beq(Value),
    Bit(Value),
    Bmi(Value),
    Bne(Value),
    Bpl(Value),
    Brk(Value),
    Bvc(Value),
    Bvs(Value),
    Clc,
    Cld,
    Cli,
    Clv,
    Cmp(Value),
    Cpx(Value),
    Cpy(Value),
    Dec(Value),
    Dex,
    Dey,
    Eor(Value),
    Inc(Value),
    Inx,
    Iny,
    Jmp(Value),
    Jsr(Value),
    Lda(Value),
    Ldx(Value),
    Ldy(Value),
    Lsr(Value),
    Nop(Value),
    Ora(Value),
    Pha,
    Php,
    Pla,
    Plp,
    Rol(Value),
    Ror(Value),
    Rti,
    Rts,
    Sbc(Value),
    Sec,
    Sed,
    Sei,
    Sta(Value),
    Stx(Value),
    Sty(Value),
    Tax,
    Tay,
    Tsx,
    Txa,
    Txs,
    Tya,
    // Unofficial
    Kil,
    Slo(Value),
    Rla(Value),
    Sre(Value),
    Rra(Value),
    Sax(Value),
    Lax(Value),
    Dcp(Value),
    Isc(Value),
    Anc(Value),
    Alr(Value),
    Arr(Value),
    Xaa(Value), // highly unstable
    Axs(Value),
    Ahx(Value), // potentially unstable
    Shy(Value), // potentially unstable
    Shx(Value), // potentially unstable
    Tas(Value), // potentially unstable
    Las(Value), // potentially unstable
}

#[allow(dead_code)]
#[derive(Debug)]
pub enum InstrErr {
    ImpossibleCycle(Instr, u8),
    ImpossibleInstr(Instr),
    UnhandledByte(u8),
    Halt(u64),
    Bubble,
}

// This practically handles step by step creation of an instr
type Partial = Result<Instr, Box<dyn Fn(u16) -> Instr>>;
type Unrefined = Result<Partial, u8>;

struct CurrInstr {
    instr: Option<Unrefined>,
    cycle: u8,
}

impl Debug for CurrInstr {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self.instr {
            Some(Ok(Err(_))) => {
                write!(f, "{{ instr: Partial, cycle: {} }}", self.cycle)
            }
            Some(Ok(Ok(i))) => write!(f, "{{ instr: {:?}, cycle: {} }}", i, self.cycle),
            Some(Err(b)) => write!(f, "{{ instr: Incomplete({:x}), cycle: {} }}", b, self.cycle),
            None => write!(f, "{{ instr: None, cycle: {} }}", self.cycle),
        }
    }
}

#[derive(Default)]
struct CurrOam {
    addr: u16,
    val: u8,
    halt: bool,
    dma: bool,
    fix: bool,
}

#[derive(Default)]
struct CurrDmc {
    time: u8, // These are scheduled 3-4 cycles after
    halt: bool,
    dma: bool,
    fix: bool,
}

#[derive(Default)]
struct CurrWork {
    prev_addr: u16,
    hold8: u8,
    hold16: u16,
    oam: CurrOam,
    dmc: CurrDmc,
}

type APURef = Rc<RefCell<apu::State>>;
type PPURef = Rc<RefCell<ppu::State>>;
type CntrsRef = Rc<RefCell<(controller::State, controller::State)>>;
type MapperRef = Rc<RefCell<mapper::State>>;

// NOTE: Fields are needlessly public for now
pub struct State {
    a: u8,
    x: u8,
    y: u8,
    pub pc: u16,
    s: u8,
    p: Status,
    curr_instr: CurrInstr,
    curr_work: CurrWork,
    buf: u8,
    pub cycles: u64,
    pub reset: bool,
    pub apu_st: Option<APURef>,
    pub ppu_st: Option<PPURef>,
    pub controllers_st: Option<CntrsRef>,
    pub mapper_st: Option<MapperRef>,
    pub mem: [u8; 0x2000],
    pub ppu: [u8; 8],
    pub misc: [u8; 32],
    pub prg: [u8; 0xBFE0],
}

impl Debug for State {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "CPU: a: {:#x}, x: {:#x}, y: {:#x}, pc: {:#x}, s: {:#x}, cyc: {}, \n  cur: {:?} p: {:?}",
            self.a, self.x, self.y, self.pc, self.s, self.cycles, self.curr_instr, self.p
        )
    }
}

impl Default for State {
    fn default() -> Self {
        State {
            a: 0,
            x: 0,
            y: 0,
            pc: 0,
            s: 0xFF,
            p: Status {
                carry: false,
                iszero: false,
                int_dis: true,
                decimal: false,
                b_flag: false,
                one: true,
                overflow: false,
                negative: false,
            },
            curr_instr: CurrInstr {
                instr: None,
                cycle: 0,
            },
            curr_work: CurrWork::default(),
            buf: 0,
            cycles: 0,
            reset: true,
            apu_st: None,
            ppu_st: None,
            controllers_st: None,
            mapper_st: None,
            mem: [0; 0x2000],
            ppu: [0; 8],
            misc: [0; 32],
            prg: [0; 0xBFE0],
        }
    }
}

impl State {
    fn status_fromu8(&mut self, op: u8) {
        self.p.carry = (op & 1) != 0;
        self.p.iszero = (op & 2) != 0;
        self.p.int_dis = (op & 4) != 0;
        self.p.decimal = (op & 8) != 0;
        self.p.overflow = (op & 64) != 0;
        self.p.negative = (op & 128) != 0;
    }

    pub fn get_ctrl(&self) -> registers::Control {
        self.at_nc(0x2000).into()
    }

    #[allow(dead_code)]
    pub fn put_ctrl(&mut self, ctrl: registers::Control) {
        self.assign_nc(0x2000, ctrl.into());
    }

    pub fn get_mask(&self) -> registers::Mask {
        self.at_nc(0x2001).into()
    }

    #[allow(dead_code)]
    pub fn put_mask(&mut self, mask: registers::Mask) {
        self.assign_nc(0x2001, mask.into());
    }

    pub fn get_stt(&self) -> registers::Status {
        self.at_nc(0x2002).into()
    }

    pub fn put_stt(&mut self, stt: registers::Status) {
        self.assign_nc(0x2002, stt.into());
    }

    fn push8_stack(&mut self, val: u8) {
        let s = self.s as u16;
        self.assign(0x0100 + s, val);
        self.s = self.s.wrapping_sub(1);
    }

    fn push16_stack(&mut self, val: u16) {
        self.push8_stack((val >> 8) as u8);
        self.push8_stack(val as u8);
    }

    fn do_oam_dma(&mut self, ppu: &mut ppu::State) {
        if (self.curr_work.oam.addr & 0xFF) == 0xFF {
            self.curr_work.oam.dma = false;
        }

        if self.curr_work.oam.fix {
            self.curr_work.oam.fix = false;
            self.cycles += 1;
            return;
        }

        // NOTE: Whether DMA put cycle is even or odd, is random at power
        // As such, decided by a fair dice roll, it shall be even
        if self.cycles % 2 == 0 {
            ppu.write_oam(self.curr_work.oam.val);
            self.curr_work.oam.addr = self.curr_work.oam.addr.wrapping_add(1);
        } else {
            self.curr_work.oam.val = self.at_nc(self.curr_work.oam.addr);
        }
        self.cycles += 1;
    }

    pub fn reload_dmc_reader(&mut self) {
        if !self.curr_work.dmc.dma {
            self.curr_work.dmc.dma = true;
            self.curr_work.dmc.halt = true;
            self.curr_work.dmc.time = if self.cycles % 2 == 0 { 3 } else { 4 };
        }
    }

    // TODO: There are potentially bugs to add
    fn do_dmc_dma(&mut self, apu: &mut apu::State) {
        if self.curr_work.oam.fix {
            self.curr_work.oam.fix = false;
            self.cycles += 1;
        }

        apu.dmc.sample = Some(self.at(apu.dmc.reader_addr));

        apu.dmc.reader_addr = if apu.dmc.reader_addr == 0xFFFF {
            0x8000
        } else {
            apu.dmc.reader_addr + 1
        };

        apu.dmc.bytes_remain -= 1;
        if apu.dmc.bytes_remain == 0 {
            if apu.dmc.lop {
                apu.dmc.reader_addr = apu.dmc.loaded_addr;
                apu.dmc.bytes_remain = apu.dmc.loaded_bytes_remain;
            } else if apu.dmc.int_enable {
                apu.dmc.int = true;
            }
        }

        self.curr_work.dmc.dma = false;
        self.cycles += 1;
    }

    fn proc_ppu_write(&mut self, ppu_n: u8) {
        let Some(ppu) = self.ppu_st.clone() else {
            return;
        };
        let mut ppu = ppu.borrow_mut();
        match ppu_n {
            0 => {
                ppu.t &= 0xF3FF;
                ppu.t |= if self.get_ctrl().nt1 { 0x400 } else { 0 };
                ppu.t |= if self.get_ctrl().nt2 { 0x800 } else { 0 };
            }
            3 => {
                ppu.oam_addr = self.at_nc(0x2003);
            }
            4 => {
                let val = self.at_nc(0x2004);
                ppu.write_oam(val);
            }
            5 => {
                if ppu.w == 0 {
                    let val = self.at_nc(0x2005);
                    ppu.t &= 0x7FE0;
                    ppu.t |= (val >> 3) as u16;
                    ppu.x = val & 0x7;
                } else {
                    let val = self.at_nc(0x2005);
                    ppu.t &= 0x0C1F;
                    ppu.t |= ((val >> 3) as u16) << 5;
                    ppu.t |= ((val & 0x7) as u16) << 12;
                }
                ppu.w ^= 1;
            }
            6 => {
                if ppu.w == 0 {
                    ppu.t &= 0x00FF;
                    ppu.t |= ((self.at_nc(0x2006) & 0x3F) as u16) << 8;
                    trace!("SET HI! from {:x} to {:x}", self.at_nc(0x2006), ppu.t);
                } else {
                    ppu.t &= 0xFF00;
                    ppu.t |= self.at_nc(0x2006) as u16;
                    ppu.v = ppu.t;
                    trace!("SET LOW! from {:x} to {:x}", self.at_nc(0x2006), ppu.t);
                }
                ppu.w ^= 1;
            }
            7 => {
                let val = self.at_nc(0x2007);
                ppu.write_addr(self, val);
            }

            _ => (),
        }
    }

    fn proc_ppu_read(&mut self, ppu_n: u8) -> u8 {
        let Some(ppu) = self.ppu_st.clone() else {
            return self.ppu[ppu_n as usize];
        };
        let mut ppu = ppu.borrow_mut();
        match ppu_n {
            2 => {
                ppu.w = 0;
                let res = self.buf;
                self.buf = self.get_stt().into();
                let mut new = self.get_stt();
                new.vblank = false;
                self.put_stt(new);
                res
            }
            4 => {
                let res = ppu.read_oam();
                self.assign_nc(0x2004, res);
                res
            }
            7 => {
                let res = ppu.read_addr(self);
                self.assign_nc(0x2007, res);
                res
            }
            i => self.ppu[i as usize],
        }
    }

    // TODO: APU and Controllers are needlessly coupled
    fn proc_misc_write(&mut self, misc_n: u8) {
        let (Some(controllers), Some(apu)) = (self.controllers_st.clone(), self.apu_st.clone())
        else {
            return;
        };
        let mut controllers = controllers.borrow_mut();
        let mut apu = apu.borrow_mut();

        let val = self.at_nc(0x4000 + misc_n as u16);

        let pulse = if misc_n == 0x00 {
            &mut apu.pulse0
        } else {
            &mut apu.pulse1
        };
        // TODO: This should probably be handled in the APU
        match misc_n {
            0x00 | 0x04 => {
                pulse.duty = (val & 0xC0) >> 6;

                pulse.length_cnt.halt = val & 0x20 == 0;
                pulse.env.lop = val & 0x20 != 0;

                pulse.env.constn = val & 0x10 != 0;

                pulse.env.vol = val & 0x0F;
                pulse.env.div.period = (val & 0x0F) as u16;
            }
            0x01 | 0x05 => {
                pulse.sweep.enable = val & 0x80 != 0;
                pulse.sweep.div.period = ((val & 0x70) >> 4) as u16;
                pulse.sweep.negate = val & 0x08 != 0;
                pulse.sweep.shift = val & 0x07;
                pulse.sweep.reload = true;
            }
            0x02 | 0x06 => {
                pulse.timer.timer_lo = val;
            }
            0x03 | 0x07 => {
                pulse.length_cnt.length_cnt = (val & 0xF8) >> 3;
                pulse.timer.timer_hi = val & 0x07;
                pulse.reset = true;
            }
            0x08 => {
                apu.tri.control = val & 0x80 != 0;
                apu.tri.length_cnt.halt = val & 0x80 == 0;
                apu.tri.linear_rld_cnt = val & 0x7F;
            }
            0x0A => {
                apu.tri.timer.timer_lo = val;
            }
            0x0B => {
                apu.tri.length_cnt.length_cnt = (val & 0xF8) >> 3;
                apu.tri.timer.timer_hi = val & 0x07;
                apu.tri.linear_rld = true;
            }
            0x0C => {
                apu.noise.length_cnt.halt = val & 0x20 == 0;
                apu.noise.env.lop = val & 0x20 != 0;

                apu.noise.env.constn = val & 0x10 != 0;

                apu.noise.env.vol = val & 0x0F;
                apu.noise.env.div.period = (val & 0x0F) as u16;
            }
            0x0E => {
                const NOISE_PERIOD_LUT: [u16; 16] = [
                    2, 4, 8, 16, 32, 48, 64, 80, 101, 129, 190, 254, 381, 508, 1017, 2034,
                ];
                apu.noise.mode = val & 0x80 != 0;
                apu.noise.div.period = NOISE_PERIOD_LUT[(val & 0x0F) as usize];
            }
            0x0F => {}
            0x10 => {
                const DMC_RATE_LUT: [u16; 16] = [
                    214, 190, 170, 160, 143, 127, 113, 107, 95, 80, 71, 64, 53, 42, 36, 27,
                ];
                apu.dmc.int_enable = if val & 0x80 == 0 { false } else { apu.dmc.int };
                apu.dmc.lop = val & 0x40 != 0;
                apu.dmc.div.period = DMC_RATE_LUT[(val & 0x0F) as usize];
            }
            0x11 => {
                apu.dmc.output = val & 0x7F;
            }
            0x12 => {
                apu.dmc.loaded_addr = ((val as u16) << 6) | 0xC000;
            }
            0x13 => {
                apu.dmc.loaded_bytes_remain = ((val as u16) << 4) | 0x1;
            }
            0x14 => {
                let addr = val;
                self.curr_work.oam.halt = true;
                self.curr_work.oam.dma = true;
                self.curr_work.oam.addr = (addr as u16) << 8;
            }
            0x15 => {
                if val & 0x10 != 0 {
                    self.curr_work.dmc.dma = true;
                    self.curr_work.dmc.halt = true;
                    self.curr_work.dmc.time = if self.cycles % 2 == 0 { 3 } else { 4 };
                } else {
                    apu.dmc.bytes_remain = 0;
                }
                apu.noise.length_cnt.halt = val & 0x08 == 0;
                apu.tri.length_cnt.halt = val & 0x04 == 0;
                apu.pulse1.length_cnt.halt = val & 0x02 == 0;
                apu.pulse0.length_cnt.halt = val & 0x01 == 0;
                apu.dmc.int = false;
            }
            0x16 => {
                controllers.0.write_strobe(val);
                controllers.1.write_strobe(val);
            }
            0x17 => {
                apu.framecnt.mode = val & 0x80 != 0;
                if apu.framecnt.mode {
                    apu.qh_clock();
                }
                apu.framecnt.int_inh = val & 0x40 != 0;
                if apu.framecnt.int_inh {
                    apu.framecnt.int = false;
                }
                apu.framecnt.reset_time = if self.cycles % 6 == 0 { 6 } else { 8 };
            }
            _ => (),
        }
    }

    fn proc_misc_read(&mut self, misc_n: u8) -> u8 {
        let (Some(controllers), Some(apu)) = (self.controllers_st.clone(), self.apu_st.clone())
        else {
            return self.misc[misc_n as usize];
        };
        let mut controllers = controllers.borrow_mut();
        let mut apu = apu.borrow_mut();
        match misc_n {
            0x15 => {
                let mut byte = 0;
                if apu.dmc.int {
                    byte |= 0x80;
                }
                if apu.framecnt.int {
                    byte |= 0x40;
                }
                // 0x20 = 0
                if apu.dmc.bytes_remain > 0 {
                    byte |= 0x10;
                }
                if apu.noise.length_cnt.length_cnt > 0 {
                    byte |= 0x08;
                }
                if apu.tri.length_cnt.length_cnt > 0 {
                    byte |= 0x04;
                }
                if apu.pulse1.length_cnt.length_cnt > 0 {
                    byte |= 0x02;
                }
                if apu.pulse0.length_cnt.length_cnt > 0 {
                    byte |= 0x01;
                }

                apu.framecnt.int = false;
                byte
            }
            0x16 => controllers.0.read() | (0x40 << 3),
            0x17 => controllers.1.read() | (0x41 << 3),
            i => self.misc[i as usize],
        }
    }

    fn proc_mapper_write(&mut self, index: u16, value: u8) {
        let Some(mapper) = self.mapper_st.clone() else {
            return;
        };
        let mut mapper = mapper.borrow_mut();
        mapper.bank_switch(self, index, value);
    }

    fn assign(&mut self, mut index: u16, value: u8) {
        match index {
            0..0x2000 => {
                self.mem[index as usize] = value;
            }
            0x2000..0x4000 => {
                index %= 8; // PPU
                trace!("assign, writing to PPU {}", index);
                self.ppu[index as usize] = value;
                self.proc_ppu_write(index as u8);
            }
            0x4000..0x4020 => {
                index -= 0x4000;
                trace!("assign, writing to MISC {}", index);
                self.misc[index as usize] = value;
                self.proc_misc_write(index as u8);
            }
            0x4020..=0xFFFF => {
                self.proc_mapper_write(index, value);
                index -= 0x4020; // RAM, ROM, catridge stuff
                self.prg[index as usize] = value;
            }
        }
    }

    // NOTE: Despite involving mutation, this should be used for reading
    fn at(&mut self, mut index: u16) -> u8 {
        match index {
            0..0x2000 => self.mem[index as usize],
            0x2000..0x4000 => {
                index %= 8; // PPU
                trace!("at, reading PPU {}", index);
                self.proc_ppu_read(index as u8)
            }
            0x4000..0x4020 => {
                index -= 0x4000;
                trace!("at, reading MISC {}", index);
                self.proc_misc_read(index as u8)
            }
            0x4020..=0xFFFF => {
                index -= 0x4020; // RAM, ROM, catridge stuff
                self.prg[index as usize]
            }
        }
    }

    pub fn assign_nc(&mut self, mut index: u16, value: u8) {
        match index {
            0..0x2000 => {
                self.mem[index as usize] = value;
            }
            0x2000..0x4000 => {
                index %= 8; // PPU
                self.ppu[index as usize] = value;
            }
            0x4000..0x4020 => {
                index -= 0x4000;
                self.misc[index as usize] = value;
            }
            0x4020..=0xFFFF => {
                index -= 0x4020; // RAM, ROM, catridge stuff
                self.prg[index as usize] = value;
            }
        }
    }

    pub fn at_nc(&self, mut index: u16) -> u8 {
        match index {
            0..0x2000 => self.mem[index as usize],
            0x2000..0x4000 => {
                index %= 8; // PPU
                self.ppu[index as usize]
            }
            0x4000..0x4020 => {
                index -= 0x4000;
                self.misc[index as usize]
            }
            0x4020..=0xFFFF => {
                index -= 0x4020; // RAM, ROM, catridge stuff
                self.prg[index as usize]
            }
        }
    }

    pub fn at16(&mut self, idx: u16) -> u16 {
        self.at(idx) as u16 + ((self.at(idx.wrapping_add(1)) as u16) << 8)
    }

    // NOTE: Intended bug with page crossing
    fn at16_p(&mut self, idx: u16) -> u16 {
        if (idx & 0xFF) == 0xFF {
            self.at(idx) as u16 + ((self.at(idx ^ 0xFF) as u16) << 8)
        } else {
            self.at16(idx)
        }
    }

    fn pop8_stack(&mut self) -> u8 {
        self.s = self.s.wrapping_add(1);
        self.at(0x0100 + self.s as u16)
    }

    fn pop16_stack(&mut self) -> u16 {
        let mut res = self.pop8_stack() as u16;
        res |= (self.pop8_stack() as u16) << 8;
        res
    }

    pub fn load_prg(&mut self, mut range: Range<usize>, from: &[u8]) {
        range = range.start - 0x4020..range.end - 0x4020;
        self.prg[range].copy_from_slice(from);
    }

    fn addrmode_in8(&mut self, val: Value) -> u8 {
        use Value::*;
        match val {
            Acc => self.a,
            Impl => 0,
            Imm(val) => val,
            ZP(addr) => self.at(addr.into()),
            ZPX(addr) => self.at(addr.wrapping_add(self.x).into()),
            ZPY(addr) => self.at(addr.wrapping_add(self.y).into()),
            Abs(addr) => self.at(addr),
            AbsX(addr) => self.at(addr.wrapping_add(self.x as u16)),
            AbsY(addr) => self.at(addr.wrapping_add(self.y as u16)),
            Rel(..) => unreachable!(),
            Ind(..) => unreachable!(),
            IdxInd(addr) => {
                let zp = addr.wrapping_add(self.x) as u16;
                let ptr = self.at16_p(zp);
                self.at(ptr)
            }
            IndIdx(addr) => {
                let zp = addr as u16;
                let ptr = self.at16_p(zp);
                self.at(ptr.wrapping_add(self.y as u16))
            }
        }
    }

    fn addrmode_in16(&mut self, val: Value) -> u16 {
        use Value::*;
        match val {
            Acc => self.a as u16,
            Impl => 0,
            Imm(val) => val as u16,
            ZP(addr) => addr as u16,
            ZPX(addr) => addr.wrapping_add(self.x) as u16,
            ZPY(addr) => addr.wrapping_add(self.y) as u16,
            Abs(addr) => addr,
            AbsX(addr) => addr.wrapping_add(self.x as u16),
            AbsY(addr) => addr.wrapping_add(self.y as u16),
            Rel(addr) => (self.pc as i16).wrapping_add((addr as i8) as i16) as u16,
            Ind(addr) => self.at16_p(addr),
            IdxInd(addr) => {
                let zp = addr.wrapping_add(self.x) as u16;
                let ptr = self.at16_p(zp);
                self.at(ptr) as u16
            }
            IndIdx(addr) => {
                let zp = addr as u16;
                let ptr = self.at16_p(zp);
                self.at(ptr.wrapping_add(self.y as u16)) as u16
            }
        }
    }

    fn addrmode_out(&mut self, val: Value, value: u8) {
        use Value::*;
        let x = self.x;
        let y = self.y;
        match val {
            Acc => self.a = value,
            Impl => unreachable!(),
            Imm(_) => unreachable!(),
            ZP(addr) => self.assign(addr.into(), value),
            ZPX(addr) => self.assign(addr.wrapping_add(x).into(), value),
            ZPY(addr) => self.assign(addr.wrapping_add(y).into(), value),
            Abs(addr) => self.assign(addr, value),
            AbsX(addr) => self.assign(addr.wrapping_add(x as u16), value),
            AbsY(addr) => self.assign(addr.wrapping_add(y as u16), value),
            Rel(_) => unreachable!(),
            Ind(_) => unreachable!(),
            IdxInd(addr) => {
                let zp = addr.wrapping_add(x) as u16;
                let ptr = self.at16_p(zp);
                self.assign(ptr, value)
            }
            IndIdx(addr) => {
                let zp = addr as u16;
                let ptr = self.at16_p(zp);
                self.assign(ptr.wrapping_add(y as u16), value)
            }
        }
    }

    // Waste a cycle
    fn plus_zero(&mut self) {
        self.curr_instr.cycle += 1;
        self.at(self.pc);
    }

    fn plus_one(&mut self) -> u8 {
        self.curr_instr.cycle += 1;
        let res = self.at(self.pc);
        self.pc = self.pc.wrapping_add(1);
        res
    }

    fn plus_two0(&mut self) {
        self.curr_instr.cycle += 1;
        self.pc = self.pc.wrapping_add(1);
    }

    fn plus_two1(&mut self) -> Instr {
        let res = self.at16(self.pc.wrapping_sub(1));
        self.curr_instr.cycle += 1;
        self.pc = self.pc.wrapping_add(1);
        if let Some(Ok(Err(f))) = &self.curr_instr.instr {
            f(res)
        } else {
            unreachable!()
        }
    }

    fn int_irq(&mut self) -> Result<(), InstrErr> {
        match self.curr_instr.cycle {
            0..3 => (),
            3 => self.push8_stack((self.pc >> 8) as u8),
            4 => self.push8_stack(self.pc as u8),
            5 => {
                self.p.b_flag = false;
                self.push8_stack(Into::<u8>::into(self.p));
            }
            6 => {
                self.pc = 0x00FE;
                // Technically int_dis is here, but we want to finish this
            }
            7 => {
                self.p.int_dis = true;
                trace!("IRQ!");
                self.pc |= 0xFF00;
                self.pc = self.at16(self.pc);
                return self.finish_instr();
            }
            _ => {
                return Err(InstrErr::ImpossibleCycle(
                    Instr::Brk(Value::Impl),
                    self.curr_instr.cycle,
                ))
            }
        }
        self.curr_instr.cycle += 1;
        Ok(())
    }

    fn int_nmi(&mut self) -> Result<(), InstrErr> {
        match self.curr_instr.cycle {
            0..3 => (),
            3 => self.push8_stack((self.pc >> 8) as u8),
            4 => self.push8_stack(self.pc as u8),
            5 => {
                self.p.b_flag = false;
                self.push8_stack(Into::<u8>::into(self.p));
            }
            6 => {
                self.pc = 0x00FA;
                self.p.int_dis = true;
            }
            7 => {
                trace!("NMI!");
                self.pc |= 0xFF00;
                self.pc = self.at16(self.pc);
                let mut stt = self.get_stt();
                stt.vblank = false;
                self.put_stt(stt);
                return self.finish_instr();
            }
            _ => {
                return Err(InstrErr::ImpossibleCycle(
                    Instr::Brk(Value::Impl),
                    self.curr_instr.cycle,
                ))
            }
        }
        self.curr_instr.cycle += 1;
        Ok(())
    }

    pub fn int_reset(&mut self) -> Result<(), InstrErr> {
        match self.curr_instr.cycle {
            0..3 => (),
            3 => self.s = self.s.wrapping_sub(1),
            4 => self.s = self.s.wrapping_sub(1),
            5 => {
                self.assign(0x4015, 0);
                self.s = self.s.wrapping_sub(1)
            }
            6 => {
                self.pc = 0x00FC;
                self.p.int_dis = true;
            }
            7 => {
                trace!("RESET!");
                self.pc |= 0xFF00;
                self.pc = self.at16(self.pc);
                self.reset = false;
                return self.finish_instr();
            }
            _ => {
                return Err(InstrErr::ImpossibleCycle(
                    Instr::Brk(Value::Impl),
                    self.curr_instr.cycle,
                ))
            }
        }
        self.curr_instr.cycle += 1;
        Ok(())
    }

    fn fetch_instr(&mut self) -> Result<(), InstrErr> {
        use Instr::*;
        use Value::*;

        let instr: Unrefined = match self.curr_instr.instr {
            None => {
                if self.reset {
                    return self.int_reset();
                }
                if self.get_ctrl().gen_nmi && self.get_stt().vblank {
                    return self.int_nmi();
                }
                if let Some(apu) = self.apu_st.clone() {
                    if !self.p.int_dis && apu.borrow().dmc.int {
                        return self.int_irq();
                    }
                    if !self.p.int_dis && apu.borrow().framecnt.int {
                        return self.int_irq();
                    }
                }
                self.curr_instr.cycle = 1;
                let res = Err(self.at(self.pc));
                self.pc = self.pc.wrapping_add(1);
                res
            }
            Some(Ok(Ok(i))) => Ok(Ok(i)),
            Some(Ok(Err(_))) => Ok(Ok(self.plus_two1())),
            Some(Err(byte)) => {
                Ok(match byte {
                    0x69 => Ok(Adc(Imm(self.plus_one()))),
                    0x65 => Ok(Adc(ZP(self.plus_one()))),
                    0x75 => Ok(Adc(ZPX(self.plus_one()))),
                    0x61 => Ok(Adc(IdxInd(self.plus_one()))),
                    0x71 => Ok(Adc(IndIdx(self.plus_one()))),
                    0x29 => Ok(And(Imm(self.plus_one()))),
                    0x25 => Ok(And(ZP(self.plus_one()))),
                    0x35 => Ok(And(ZPX(self.plus_one()))),
                    0x21 => Ok(And(IdxInd(self.plus_one()))),
                    0x31 => Ok(And(IndIdx(self.plus_one()))),
                    0x0A => {
                        self.plus_zero();
                        Ok(Asl(Acc))
                    }
                    0x06 => Ok(Asl(ZP(self.plus_one()))),
                    0x16 => Ok(Asl(ZPX(self.plus_one()))),
                    0x90 => Ok(Bcc(Rel(self.plus_one()))),
                    0xB0 => Ok(Bcs(Rel(self.plus_one()))),
                    0xF0 => Ok(Beq(Rel(self.plus_one()))),
                    0x24 => Ok(Bit(ZP(self.plus_one()))),
                    0x30 => Ok(Bmi(Rel(self.plus_one()))),
                    0xD0 => Ok(Bne(Rel(self.plus_one()))),
                    0x10 => Ok(Bpl(Rel(self.plus_one()))),
                    0x00 => {
                        let (pc, car) = self.pc.overflowing_add(1);
                        if car {
                            self.pc = pc;
                            self.curr_instr.cycle += 1;
                            Ok(Instr::Brk(Value::Imm(self.at(self.pc))))
                        } else {
                            Ok(Instr::Brk(Value::Impl))
                        }
                    }
                    0x50 => Ok(Bvc(Rel(self.plus_one()))),
                    0x70 => Ok(Bvs(Rel(self.plus_one()))),
                    0x18 => {
                        self.plus_zero();
                        Ok(Clc)
                    }
                    0xD8 => {
                        self.plus_zero();
                        Ok(Cld)
                    }
                    0x58 => {
                        self.plus_zero();
                        Ok(Cli)
                    }
                    0xB8 => {
                        self.plus_zero();
                        Ok(Clv)
                    }
                    0xC9 => Ok(Cmp(Imm(self.plus_one()))),
                    0xC5 => Ok(Cmp(ZP(self.plus_one()))),
                    0xD5 => Ok(Cmp(ZPX(self.plus_one()))),
                    0xC1 => Ok(Cmp(IdxInd(self.plus_one()))),
                    0xD1 => Ok(Cmp(IndIdx(self.plus_one()))),
                    0xE0 => Ok(Cpx(Imm(self.plus_one()))),
                    0xE4 => Ok(Cpx(ZP(self.plus_one()))),
                    0xC0 => Ok(Cpy(Imm(self.plus_one()))),
                    0xC4 => Ok(Cpy(ZP(self.plus_one()))),
                    0xC6 => Ok(Dec(ZP(self.plus_one()))),
                    0xD6 => Ok(Dec(ZPX(self.plus_one()))),
                    0xCA => {
                        self.plus_zero();
                        Ok(Dex)
                    }
                    0x88 => {
                        self.plus_zero();
                        Ok(Dey)
                    }
                    0x49 => Ok(Eor(Imm(self.plus_one()))),
                    0x45 => Ok(Eor(ZP(self.plus_one()))),
                    0x55 => Ok(Eor(ZPX(self.plus_one()))),
                    0x41 => Ok(Eor(IdxInd(self.plus_one()))),
                    0x51 => Ok(Eor(IndIdx(self.plus_one()))),
                    0xE6 => Ok(Inc(ZP(self.plus_one()))),
                    0xF6 => Ok(Inc(ZPX(self.plus_one()))),
                    0xE8 => {
                        self.plus_zero();
                        Ok(Inx)
                    }
                    0xC8 => {
                        self.plus_zero();
                        Ok(Iny)
                    }
                    0xA9 => Ok(Lda(Imm(self.plus_one()))),
                    0xA5 => Ok(Lda(ZP(self.plus_one()))),
                    0xB5 => Ok(Lda(ZPX(self.plus_one()))),
                    0xA1 => Ok(Lda(IdxInd(self.plus_one()))),
                    0xB1 => Ok(Lda(IndIdx(self.plus_one()))),
                    0xA2 => Ok(Ldx(Imm(self.plus_one()))),
                    0xA6 => Ok(Ldx(ZP(self.plus_one()))),
                    0xB6 => Ok(Ldx(ZPY(self.plus_one()))),
                    0xA0 => Ok(Ldy(Imm(self.plus_one()))),
                    0xA4 => Ok(Ldy(ZP(self.plus_one()))),
                    0xB4 => Ok(Ldy(ZPX(self.plus_one()))),
                    0x4A => {
                        self.plus_zero();
                        Ok(Lsr(Acc))
                    }
                    0x46 => Ok(Lsr(ZP(self.plus_one()))),
                    0x56 => Ok(Lsr(ZPX(self.plus_one()))),
                    0xEA => {
                        self.plus_zero();
                        Ok(Nop(Impl))
                    }
                    0x09 => Ok(Ora(Imm(self.plus_one()))),
                    0x05 => Ok(Ora(ZP(self.plus_one()))),
                    0x15 => Ok(Ora(ZPX(self.plus_one()))),
                    0x01 => Ok(Ora(IdxInd(self.plus_one()))),
                    0x11 => Ok(Ora(IndIdx(self.plus_one()))),
                    0x48 => {
                        self.plus_zero();
                        Ok(Pha)
                    }
                    0x08 => {
                        self.plus_zero();
                        Ok(Php)
                    }
                    0x68 => {
                        self.plus_zero();
                        Ok(Pla)
                    }
                    0x28 => {
                        self.plus_zero();
                        Ok(Plp)
                    }
                    0x2A => {
                        self.plus_zero();
                        Ok(Rol(Acc))
                    }
                    0x26 => Ok(Rol(ZP(self.plus_one()))),
                    0x36 => Ok(Rol(ZPX(self.plus_one()))),
                    0x6A => {
                        self.plus_zero();
                        Ok(Ror(Acc))
                    }
                    0x66 => Ok(Ror(ZP(self.plus_one()))),
                    0x76 => Ok(Ror(ZPX(self.plus_one()))),
                    0x40 => {
                        self.plus_zero();
                        Ok(Rti)
                    }
                    0x60 => {
                        self.plus_zero();
                        Ok(Rts)
                    }
                    0xE9 => Ok(Sbc(Imm(self.plus_one()))),
                    0xE5 => Ok(Sbc(ZP(self.plus_one()))),
                    0xF5 => Ok(Sbc(ZPX(self.plus_one()))),
                    0xE1 => Ok(Sbc(IdxInd(self.plus_one()))),
                    0xF1 => Ok(Sbc(IndIdx(self.plus_one()))),
                    0x38 => {
                        self.plus_zero();
                        Ok(Sec)
                    }
                    0xF8 => {
                        self.plus_zero();
                        Ok(Sed)
                    }
                    0x78 => {
                        self.plus_zero();
                        Ok(Sei)
                    }
                    0x85 => Ok(Sta(ZP(self.plus_one()))),
                    0x95 => Ok(Sta(ZPX(self.plus_one()))),
                    0x81 => Ok(Sta(IdxInd(self.plus_one()))),
                    0x91 => Ok(Sta(IndIdx(self.plus_one()))),
                    0x86 => Ok(Stx(ZP(self.plus_one()))),
                    0x96 => Ok(Stx(ZPY(self.plus_one()))),
                    0x84 => Ok(Sty(ZP(self.plus_one()))),
                    0x94 => Ok(Sty(ZPX(self.plus_one()))),
                    0xAA => {
                        self.plus_zero();
                        Ok(Tax)
                    }
                    0xA8 => {
                        self.plus_zero();
                        Ok(Tay)
                    }
                    0xBA => {
                        self.plus_zero();
                        Ok(Tsx)
                    }
                    0x8A => {
                        self.plus_zero();
                        Ok(Txa)
                    }
                    0x9A => {
                        self.plus_zero();
                        Ok(Txs)
                    }
                    0x98 => {
                        self.plus_zero();
                        Ok(Tya)
                    }
                    0x8C => {
                        self.plus_two0();
                        Err(Box::new(|wrd| Sty(Abs(wrd))))
                    }
                    0x8E => {
                        self.plus_two0();
                        Err(Box::new(|wrd| Stx(Abs(wrd))))
                    }
                    0x8D => {
                        self.plus_two0();
                        Err(Box::new(|wrd| Sta(Abs(wrd))))
                    }
                    0x9D => {
                        self.plus_two0();
                        Err(Box::new(|wrd| Sta(AbsX(wrd))))
                    }
                    0x99 => {
                        self.plus_two0();
                        Err(Box::new(|wrd| Sta(AbsY(wrd))))
                    }
                    0xED => {
                        self.plus_two0();
                        Err(Box::new(|wrd| Sbc(Abs(wrd))))
                    }
                    0xFD => {
                        self.plus_two0();
                        Err(Box::new(|wrd| Sbc(AbsX(wrd))))
                    }
                    0xF9 => {
                        self.plus_two0();
                        Err(Box::new(|wrd| Sbc(AbsY(wrd))))
                    }
                    0x6E => {
                        self.plus_two0();
                        Err(Box::new(|wrd| Ror(Abs(wrd))))
                    }
                    0x7E => {
                        self.plus_two0();
                        Err(Box::new(|wrd| Ror(AbsX(wrd))))
                    }
                    0x2E => {
                        self.plus_two0();
                        Err(Box::new(|wrd| Rol(Abs(wrd))))
                    }
                    0x3E => {
                        self.plus_two0();
                        Err(Box::new(|wrd| Rol(AbsX(wrd))))
                    }
                    0x0D => {
                        self.plus_two0();
                        Err(Box::new(|wrd| Ora(Abs(wrd))))
                    }
                    0x1D => {
                        self.plus_two0();
                        Err(Box::new(|wrd| Ora(AbsX(wrd))))
                    }
                    0x19 => {
                        self.plus_two0();
                        Err(Box::new(|wrd| Ora(AbsY(wrd))))
                    }
                    0x4E => {
                        self.plus_two0();
                        Err(Box::new(|wrd| Lsr(Abs(wrd))))
                    }
                    0x5E => {
                        self.plus_two0();
                        Err(Box::new(|wrd| Lsr(AbsX(wrd))))
                    }
                    0xAC => {
                        self.plus_two0();
                        Err(Box::new(|wrd| Ldy(Abs(wrd))))
                    }
                    0xBC => {
                        self.plus_two0();
                        Err(Box::new(|wrd| Ldy(AbsX(wrd))))
                    }
                    0xAE => {
                        self.plus_two0();
                        Err(Box::new(|wrd| Ldx(Abs(wrd))))
                    }
                    0xBE => {
                        self.plus_two0();
                        Err(Box::new(|wrd| Ldx(AbsY(wrd))))
                    }
                    0xAD => {
                        self.plus_two0();
                        Err(Box::new(|wrd| Lda(Abs(wrd))))
                    }
                    0xBD => {
                        self.plus_two0();
                        Err(Box::new(|wrd| Lda(AbsX(wrd))))
                    }
                    0xB9 => {
                        self.plus_two0();
                        Err(Box::new(|wrd| Lda(AbsY(wrd))))
                    }
                    0x4C => {
                        self.plus_two0();
                        Err(Box::new(|wrd| Jmp(Abs(wrd))))
                    }
                    0x6C => {
                        self.plus_two0();
                        Err(Box::new(|wrd| Jmp(Ind(wrd))))
                    }
                    0x20 => {
                        self.plus_two0();
                        Err(Box::new(|wrd| Jsr(Abs(wrd))))
                    }
                    0xEE => {
                        self.plus_two0();
                        Err(Box::new(|wrd| Inc(Abs(wrd))))
                    }
                    0xFE => {
                        self.plus_two0();
                        Err(Box::new(|wrd| Inc(AbsX(wrd))))
                    }
                    0x4D => {
                        self.plus_two0();
                        Err(Box::new(|wrd| Eor(Abs(wrd))))
                    }
                    0x5D => {
                        self.plus_two0();
                        Err(Box::new(|wrd| Eor(AbsX(wrd))))
                    }
                    0x59 => {
                        self.plus_two0();
                        Err(Box::new(|wrd| Eor(AbsY(wrd))))
                    }
                    0xCE => {
                        self.plus_two0();
                        Err(Box::new(|wrd| Dec(Abs(wrd))))
                    }
                    0xDE => {
                        self.plus_two0();
                        Err(Box::new(|wrd| Dec(AbsX(wrd))))
                    }
                    0xCC => {
                        self.plus_two0();
                        Err(Box::new(|wrd| Cpy(Abs(wrd))))
                    }
                    0xEC => {
                        self.plus_two0();
                        Err(Box::new(|wrd| Cpx(Abs(wrd))))
                    }
                    0xCD => {
                        self.plus_two0();
                        Err(Box::new(|wrd| Cmp(Abs(wrd))))
                    }
                    0xDD => {
                        self.plus_two0();
                        Err(Box::new(|wrd| Cmp(AbsX(wrd))))
                    }
                    0xD9 => {
                        self.plus_two0();
                        Err(Box::new(|wrd| Cmp(AbsY(wrd))))
                    }
                    0x2C => {
                        self.plus_two0();
                        Err(Box::new(|wrd| Bit(Abs(wrd))))
                    }
                    0x0E => {
                        self.plus_two0();
                        Err(Box::new(|wrd| Asl(Abs(wrd))))
                    }
                    0x1E => {
                        self.plus_two0();
                        Err(Box::new(|wrd| Asl(AbsX(wrd))))
                    }
                    0x2D => {
                        self.plus_two0();
                        Err(Box::new(|wrd| And(Abs(wrd))))
                    }
                    0x3D => {
                        self.plus_two0();
                        Err(Box::new(|wrd| And(AbsX(wrd))))
                    }
                    0x39 => {
                        self.plus_two0();
                        Err(Box::new(|wrd| And(AbsY(wrd))))
                    }
                    0x6D => {
                        self.plus_two0();
                        Err(Box::new(|wrd| Adc(Abs(wrd))))
                    }
                    0x7D => {
                        self.plus_two0();
                        Err(Box::new(|wrd| Adc(AbsX(wrd))))
                    }
                    0x79 => {
                        self.plus_two0();
                        Err(Box::new(|wrd| Adc(AbsY(wrd))))
                    }
                    // Unofficial maps
                    0x80 | 0x82 | 0xC2 | 0xE2 => Ok(Nop(Imm(self.plus_one()))),
                    0x02 | 0x12 | 0x22 | 0x32 | 0x42 | 0x52 | 0x62 | 0x72 | 0x92 | 0xB2 | 0xD2
                    | 0xF2 => {
                        self.plus_zero();
                        Ok(Kil)
                    }
                    0x03 => Ok(Slo(IdxInd(self.plus_one()))),
                    0x13 => Ok(Slo(IndIdx(self.plus_one()))),
                    0x23 => Ok(Rla(IdxInd(self.plus_one()))),
                    0x33 => Ok(Rla(IndIdx(self.plus_one()))),
                    0x43 => Ok(Sre(IdxInd(self.plus_one()))),
                    0x53 => Ok(Sre(IndIdx(self.plus_one()))),
                    0x63 => Ok(Rra(IdxInd(self.plus_one()))),
                    0x73 => Ok(Rra(IndIdx(self.plus_one()))),
                    0x83 => Ok(Sax(IdxInd(self.plus_one()))),
                    0x93 => Ok(Ahx(IndIdx(self.plus_one()))), // unstable
                    0xA3 => Ok(Lax(IdxInd(self.plus_one()))),
                    0xB3 => Ok(Lax(IndIdx(self.plus_one()))),
                    0xC3 => Ok(Dcp(IdxInd(self.plus_one()))),
                    0xD3 => Ok(Dcp(IndIdx(self.plus_one()))),
                    0xE3 => Ok(Isc(IdxInd(self.plus_one()))),
                    0xF3 => Ok(Isc(IndIdx(self.plus_one()))),
                    0x04 => Ok(Nop(ZP(self.plus_one()))),
                    0x14 => Ok(Nop(ZPX(self.plus_one()))),
                    0x34 => Ok(Nop(ZPX(self.plus_one()))),
                    0x44 => Ok(Nop(ZP(self.plus_one()))),
                    0x54 => Ok(Nop(ZPX(self.plus_one()))),
                    0x64 => Ok(Nop(ZP(self.plus_one()))),
                    0x74 => Ok(Nop(ZPX(self.plus_one()))),
                    0xD4 => Ok(Nop(ZPX(self.plus_one()))),
                    0xF4 => Ok(Nop(ZPX(self.plus_one()))),
                    0x07 => Ok(Slo(ZP(self.plus_one()))),
                    0x17 => Ok(Slo(ZPX(self.plus_one()))),
                    0x27 => Ok(Rla(ZP(self.plus_one()))),
                    0x37 => Ok(Rla(ZPX(self.plus_one()))),
                    0x47 => Ok(Sre(ZP(self.plus_one()))),
                    0x57 => Ok(Sre(ZPX(self.plus_one()))),
                    0x67 => Ok(Rra(ZP(self.plus_one()))),
                    0x77 => Ok(Rra(ZPX(self.plus_one()))),
                    0x87 => Ok(Sax(ZP(self.plus_one()))),
                    0x97 => Ok(Sax(ZPY(self.plus_one()))),
                    0xA7 => Ok(Lax(ZP(self.plus_one()))),
                    0xB7 => Ok(Lax(ZPY(self.plus_one()))),
                    0xC7 => Ok(Dcp(ZP(self.plus_one()))),
                    0xD7 => Ok(Dcp(ZPX(self.plus_one()))),
                    0xE7 => Ok(Isc(ZP(self.plus_one()))),
                    0xF7 => Ok(Isc(ZPX(self.plus_one()))),
                    0x89 => Ok(Nop(Imm(self.plus_one()))),
                    0x1A | 0x3A | 0x5A | 0x7A | 0xDA | 0xFA => {
                        self.plus_zero();
                        Ok(Nop(Impl))
                    }
                    0x0B => Ok(Anc(Imm(self.plus_one()))),
                    0x2B => Ok(Anc(Imm(self.plus_one()))),
                    0x4B => Ok(Alr(Imm(self.plus_one()))),
                    0x6B => Ok(Arr(Imm(self.plus_one()))),
                    0x8B => Ok(Xaa(Imm(self.plus_one()))), // Highly unstable
                    0xAB => Ok(Lax(Imm(self.plus_one()))), // Highly unstable
                    0xCB => Ok(Axs(Imm(self.plus_one()))),
                    0xEB => Ok(Sbc(Imm(self.plus_one()))),
                    0x1B => {
                        self.plus_two0();
                        Err(Box::new(|wrd| Slo(AbsY(wrd))))
                    }
                    0x3B => {
                        self.plus_two0();
                        Err(Box::new(|wrd| Rla(AbsY(wrd))))
                    }
                    0x5B => {
                        self.plus_two0();
                        Err(Box::new(|wrd| Sre(AbsY(wrd))))
                    }
                    0x7B => {
                        self.plus_two0();
                        Err(Box::new(|wrd| Rra(AbsY(wrd))))
                    }
                    0x9B => {
                        self.plus_two0();
                        Err(Box::new(|wrd| Tas(AbsY(wrd))))
                    } // unstable
                    0xBB => {
                        self.plus_two0();
                        Err(Box::new(|wrd| Las(AbsY(wrd))))
                    }
                    0xDB => {
                        self.plus_two0();
                        Err(Box::new(|wrd| Dcp(AbsY(wrd))))
                    }
                    0xFB => {
                        self.plus_two0();
                        Err(Box::new(|wrd| Isc(AbsY(wrd))))
                    }
                    0x0C => {
                        self.plus_two0();
                        Err(Box::new(|wrd| Nop(Abs(wrd))))
                    }
                    0x1C | 0x3C | 0x5C | 0x7C | 0xDC | 0xFC => {
                        self.plus_two0();
                        Err(Box::new(|wrd| Nop(AbsX(wrd))))
                    }
                    0x9C => {
                        self.plus_two0();
                        Err(Box::new(|wrd| Shy(AbsX(wrd))))
                    } // unstable
                    0x9E => {
                        self.plus_two0();
                        Err(Box::new(|wrd| Shx(AbsY(wrd))))
                    } // unstable
                    0x0F => {
                        self.plus_two0();
                        Err(Box::new(|wrd| Slo(Abs(wrd))))
                    }
                    0x1F => {
                        self.plus_two0();
                        Err(Box::new(|wrd| Slo(AbsX(wrd))))
                    }
                    0x2F => {
                        self.plus_two0();
                        Err(Box::new(|wrd| Rla(Abs(wrd))))
                    }
                    0x3F => {
                        self.plus_two0();
                        Err(Box::new(|wrd| Rla(AbsX(wrd))))
                    }
                    0x4F => {
                        self.plus_two0();
                        Err(Box::new(|wrd| Sre(Abs(wrd))))
                    }
                    0x5F => {
                        self.plus_two0();
                        Err(Box::new(|wrd| Sre(AbsX(wrd))))
                    }
                    0x6F => {
                        self.plus_two0();
                        Err(Box::new(|wrd| Rra(Abs(wrd))))
                    }
                    0x7F => {
                        self.plus_two0();
                        Err(Box::new(|wrd| Rra(AbsX(wrd))))
                    }
                    0x8F => {
                        self.plus_two0();
                        Err(Box::new(|wrd| Sax(Abs(wrd))))
                    }
                    0x9F => {
                        self.plus_two0();
                        Err(Box::new(|wrd| Ahx(AbsY(wrd))))
                    } // unstable
                    0xAF => {
                        self.plus_two0();
                        Err(Box::new(|wrd| Lax(Abs(wrd))))
                    }
                    0xBF => {
                        self.plus_two0();
                        Err(Box::new(|wrd| Lax(AbsY(wrd))))
                    }
                    0xCF => {
                        self.plus_two0();
                        Err(Box::new(|wrd| Dcp(Abs(wrd))))
                    }
                    0xDF => {
                        self.plus_two0();
                        Err(Box::new(|wrd| Dcp(AbsX(wrd))))
                    }
                    0xEF => {
                        self.plus_two0();
                        Err(Box::new(|wrd| Isc(Abs(wrd))))
                    }
                    0xFF => {
                        self.plus_two0();
                        Err(Box::new(|wrd| Isc(AbsX(wrd))))
                    }
                })
            }
        };

        self.curr_instr.instr = Some(instr);
        Ok(())
    }

    fn finish_instr(&mut self) -> Result<(), InstrErr> {
        if let Some(ppu) = &self.ppu_st {
            trace!(
                "V: {} H: {} v: {:x} t: {:x} ctrl: {:b} mask: {:b} stt: {:b}",
                ppu.borrow().scanline,
                ppu.borrow().cycles,
                ppu.borrow().v,
                ppu.borrow().t,
                Into::<u8>::into(self.get_ctrl()),
                Into::<u8>::into(self.get_mask()),
                Into::<u8>::into(self.get_stt()),
            );
        }

        trace!("{:?}", self);

        self.cycles += self.curr_instr.cycle as u64;

        self.curr_instr = CurrInstr {
            instr: None,
            cycle: 0,
        };
        self.fetch_instr()?;

        Err(InstrErr::Bubble)
    }

    fn zn_check(&mut self, res: u8) {
        self.p.iszero = res == 0;
        self.p.negative = (res & 128) != 0;
    }

    fn an_incantation(ingr: &Option<Unrefined>) -> Instr {
        *ingr
            .as_ref()
            .unwrap()
            .as_ref()
            .unwrap()
            .as_ref()
            .unwrap_or(&Instr::Kil)
    }

    fn rel_branch(&mut self, val: Value, check: bool) -> Result<(), InstrErr> {
        match self.curr_instr.cycle {
            2 => {
                if check {
                    self.curr_work.prev_addr = self.pc;
                    self.pc = self.addrmode_in16(val);
                    Ok(())
                } else {
                    self.finish_instr()
                }
            }
            3 if (self.curr_work.prev_addr & 0xFF00) == (self.pc & 0xFF00) => self.finish_instr(),
            3 => Ok(()),
            4 => self.finish_instr(),
            _ => Err(InstrErr::ImpossibleCycle(
                Self::an_incantation(&self.curr_instr.instr),
                self.curr_instr.cycle,
            )),
        }
    }

    fn impld<F: FnOnce(&mut State)>(&mut self, f: F) -> Result<(), InstrErr> {
        match self.curr_instr.cycle {
            2 => {
                f(self);
                self.finish_instr()
            }
            _ => Err(InstrErr::ImpossibleCycle(
                Self::an_incantation(&self.curr_instr.instr),
                self.curr_instr.cycle,
            )),
        }
    }

    fn acc_rmw<F: FnOnce(&mut State, u8) -> u8>(&mut self, f: F) -> Result<(), InstrErr> {
        match self.curr_instr.cycle {
            2 => {
                self.a = f(self, self.a);
                self.finish_instr()
            }
            _ => Err(InstrErr::ImpossibleCycle(
                Self::an_incantation(&self.curr_instr.instr),
                self.curr_instr.cycle,
            )),
        }
    }

    fn imm<F: FnOnce(&mut State, u8)>(&mut self, val: Value, f: F) -> Result<(), InstrErr> {
        match self.curr_instr.cycle {
            2 => {
                let in8 = self.addrmode_in8(val);
                f(self, in8);
                self.finish_instr()
            }
            _ => Err(InstrErr::ImpossibleCycle(
                Self::an_incantation(&self.curr_instr.instr),
                self.curr_instr.cycle,
            )),
        }
    }

    fn zp_w<F: FnOnce(&mut State) -> u8>(&mut self, val: Value, f: F) -> Result<(), InstrErr> {
        match self.curr_instr.cycle {
            2 => Ok(()),
            3 => {
                let value = f(self);
                self.addrmode_out(val, value);
                self.finish_instr()
            }
            _ => Err(InstrErr::ImpossibleCycle(
                Self::an_incantation(&self.curr_instr.instr),
                self.curr_instr.cycle,
            )),
        }
    }

    fn zp_r<F: FnOnce(&mut State, u8)>(&mut self, val: Value, f: F) -> Result<(), InstrErr> {
        match self.curr_instr.cycle {
            2 => Ok(()),
            3 => {
                let in8 = self.addrmode_in8(val);
                f(self, in8);
                self.finish_instr()
            }
            _ => Err(InstrErr::ImpossibleCycle(
                Self::an_incantation(&self.curr_instr.instr),
                self.curr_instr.cycle,
            )),
        }
    }

    fn zp_rmw<F: FnOnce(&mut State, u8) -> u8>(
        &mut self,
        val: Value,
        f: F,
    ) -> Result<(), InstrErr> {
        match self.curr_instr.cycle {
            2 => Ok(()),
            3 => {
                self.curr_work.hold8 = self.addrmode_in8(val);
                Ok(())
            }
            4 => {
                self.addrmode_out(val, self.curr_work.hold8);
                self.curr_work.hold8 = f(self, self.curr_work.hold8);
                Ok(())
            }
            5 => {
                self.addrmode_out(val, self.curr_work.hold8);
                self.finish_instr()
            }
            _ => Err(InstrErr::ImpossibleCycle(
                Self::an_incantation(&self.curr_instr.instr),
                self.curr_instr.cycle,
            )),
        }
    }

    fn zpxy_w<F: FnOnce(&mut State) -> u8>(&mut self, val: Value, f: F) -> Result<(), InstrErr> {
        match self.curr_instr.cycle {
            2 => Ok(()),
            3 => match val {
                Value::ZPX(addr) => {
                    self.curr_work.hold16 = addr.wrapping_add(self.x) as u16;
                    self.curr_work.hold8 = self.at(self.curr_work.hold16);
                    Ok(())
                }
                Value::ZPY(addr) => {
                    self.curr_work.hold16 = addr.wrapping_add(self.y) as u16;
                    self.curr_work.hold8 = self.at(self.curr_work.hold16);
                    Ok(())
                }
                _ => Err(InstrErr::ImpossibleCycle(
                    Self::an_incantation(&self.curr_instr.instr),
                    self.curr_instr.cycle,
                )),
            },
            4 => {
                let hold = self.curr_work.hold16;
                let val = f(self);
                self.assign(hold, val);
                self.finish_instr()
            }

            _ => Err(InstrErr::ImpossibleCycle(
                Self::an_incantation(&self.curr_instr.instr),
                self.curr_instr.cycle,
            )),
        }
    }

    fn zpxy_r<F: FnOnce(&mut State, u8)>(&mut self, val: Value, f: F) -> Result<(), InstrErr> {
        match self.curr_instr.cycle {
            2 => Ok(()),
            3 => match val {
                Value::ZPX(addr) => {
                    self.curr_work.hold16 = addr.wrapping_add(self.x) as u16;
                    Ok(())
                }
                Value::ZPY(addr) => {
                    self.curr_work.hold16 = addr.wrapping_add(self.y) as u16;
                    Ok(())
                }
                _ => Err(InstrErr::ImpossibleCycle(
                    Self::an_incantation(&self.curr_instr.instr),
                    self.curr_instr.cycle,
                )),
            },
            4 => {
                self.curr_work.hold8 = self.at(self.curr_work.hold16);
                f(self, self.curr_work.hold8);
                self.finish_instr()
            }

            _ => Err(InstrErr::ImpossibleCycle(
                Self::an_incantation(&self.curr_instr.instr),
                self.curr_instr.cycle,
            )),
        }
    }

    fn zpxy_rmw<F: FnOnce(&mut State, u8) -> u8>(
        &mut self,
        val: Value,
        f: F,
    ) -> Result<(), InstrErr> {
        match self.curr_instr.cycle {
            2 => Ok(()),
            3 => match val {
                Value::ZPX(addr) => {
                    self.curr_work.hold16 = addr.wrapping_add(self.x) as u16;
                    Ok(())
                }
                Value::ZPY(addr) => {
                    self.curr_work.hold16 = addr.wrapping_add(self.y) as u16;
                    Ok(())
                }
                _ => Err(InstrErr::ImpossibleCycle(
                    Self::an_incantation(&self.curr_instr.instr),
                    self.curr_instr.cycle,
                )),
            },
            4 => {
                self.curr_work.hold8 = self.at(self.curr_work.hold16);
                Ok(())
            }
            5 => {
                let hold = self.curr_work.hold16;
                self.assign(hold, self.curr_work.hold8);
                self.curr_work.hold8 = f(self, self.curr_work.hold8);
                Ok(())
            }
            6 => {
                let hold = self.curr_work.hold16;
                self.assign(hold, self.curr_work.hold8);
                self.finish_instr()
            }
            _ => Err(InstrErr::ImpossibleCycle(
                Self::an_incantation(&self.curr_instr.instr),
                self.curr_instr.cycle,
            )),
        }
    }

    fn abs_w<F: FnOnce(&mut State) -> u8>(&mut self, val: Value, f: F) -> Result<(), InstrErr> {
        match self.curr_instr.cycle {
            3 => Ok(()),
            4 => {
                let value = f(self);
                self.addrmode_out(val, value);
                self.finish_instr()
            }

            _ => Err(InstrErr::ImpossibleCycle(
                Self::an_incantation(&self.curr_instr.instr),
                self.curr_instr.cycle,
            )),
        }
    }

    fn abs_r<F: FnOnce(&mut State, u8)>(&mut self, val: Value, f: F) -> Result<(), InstrErr> {
        match self.curr_instr.cycle {
            3 => Ok(()),
            4 => {
                let in8 = self.addrmode_in8(val);
                f(self, in8);
                self.finish_instr()
            }

            _ => Err(InstrErr::ImpossibleCycle(
                Self::an_incantation(&self.curr_instr.instr),
                self.curr_instr.cycle,
            )),
        }
    }

    fn abs_rmw<F: FnOnce(&mut State, u8) -> u8>(
        &mut self,
        val: Value,
        f: F,
    ) -> Result<(), InstrErr> {
        match self.curr_instr.cycle {
            3 => Ok(()),
            4 => {
                self.curr_work.hold8 = self.addrmode_in8(val);
                Ok(())
            }
            5 => {
                self.addrmode_out(val, self.curr_work.hold8);
                self.curr_work.hold8 = f(self, self.curr_work.hold8);
                Ok(())
            }
            6 => {
                self.addrmode_out(val, self.curr_work.hold8);
                self.finish_instr()
            }
            _ => Err(InstrErr::ImpossibleCycle(
                Self::an_incantation(&self.curr_instr.instr),
                self.curr_instr.cycle,
            )),
        }
    }

    fn absxy_w<F: FnOnce(&mut State) -> u8>(&mut self, val: Value, f: F) -> Result<(), InstrErr> {
        match self.curr_instr.cycle {
            3 => Ok(()),
            4 => match val {
                Value::AbsX(addr) => {
                    let low = (addr & 0xFF).wrapping_add(self.x as u16);
                    self.curr_work.hold16 = (addr & 0xFF00) + low;
                    self.curr_work.hold8 = self.at(self.curr_work.hold16);
                    Ok(())
                }
                Value::AbsY(addr) => {
                    let low = (addr & 0xFF).wrapping_add(self.y as u16);
                    self.curr_work.hold16 = (addr & 0xFF00) + low;
                    self.curr_work.hold8 = self.at(self.curr_work.hold16);
                    Ok(())
                }
                _ => Err(InstrErr::ImpossibleCycle(
                    Self::an_incantation(&self.curr_instr.instr),
                    self.curr_instr.cycle,
                )),
            },
            5 => {
                let hold = self.curr_work.hold16;
                self.curr_work.hold8 = f(self);
                self.assign(hold, self.curr_work.hold8);
                self.finish_instr()
            }
            _ => Err(InstrErr::ImpossibleCycle(
                Self::an_incantation(&self.curr_instr.instr),
                self.curr_instr.cycle,
            )),
        }
    }

    fn absxy_r<F: FnOnce(&mut State, u8)>(&mut self, val: Value, f: F) -> Result<(), InstrErr> {
        match self.curr_instr.cycle {
            3 => Ok(()),
            4 => match val {
                Value::AbsX(addr) => {
                    if !(addr as u8).overflowing_add(self.x).1 {
                        let in8 = self.addrmode_in8(val);
                        f(self, in8);
                        self.finish_instr()
                    } else {
                        Ok(())
                    }
                }
                Value::AbsY(addr) => {
                    if !(addr as u8).overflowing_add(self.y).1 {
                        let in8 = self.addrmode_in8(val);
                        f(self, in8);
                        self.finish_instr()
                    } else {
                        Ok(())
                    }
                }
                _ => Err(InstrErr::ImpossibleCycle(
                    Self::an_incantation(&self.curr_instr.instr),
                    self.curr_instr.cycle,
                )),
            },
            5 => {
                let in8 = self.addrmode_in8(val);
                f(self, in8);
                self.finish_instr()
            }

            _ => Err(InstrErr::ImpossibleCycle(
                Self::an_incantation(&self.curr_instr.instr),
                self.curr_instr.cycle,
            )),
        }
    }

    fn absxy_rmw<F: FnOnce(&mut State, u8) -> u8>(
        &mut self,
        val: Value,
        f: F,
    ) -> Result<(), InstrErr> {
        match self.curr_instr.cycle {
            3 => Ok(()),
            4 => match val {
                Value::AbsX(addr) => {
                    let low = (addr & 0xFF).wrapping_add(self.x as u16);
                    self.curr_work.hold16 = (addr & 0xFF00) + low;
                    Ok(())
                }
                Value::AbsY(addr) => {
                    let low = (addr & 0xFF).wrapping_add(self.y as u16);
                    self.curr_work.hold16 = (addr & 0xFF00) + low;
                    Ok(())
                }
                _ => Err(InstrErr::ImpossibleCycle(
                    Self::an_incantation(&self.curr_instr.instr),
                    self.curr_instr.cycle,
                )),
            },
            5 => {
                self.curr_work.hold8 = self.at(self.curr_work.hold16);
                Ok(())
            }
            6 => {
                let hold = self.curr_work.hold16;
                self.assign(hold, self.curr_work.hold8);
                self.curr_work.hold8 = f(self, self.curr_work.hold8);
                Ok(())
            }
            7 => {
                let hold = self.curr_work.hold16;
                self.assign(hold, self.curr_work.hold8);
                self.finish_instr()
            }
            _ => Err(InstrErr::ImpossibleCycle(
                Self::an_incantation(&self.curr_instr.instr),
                self.curr_instr.cycle,
            )),
        }
    }

    fn idxind_w<F: FnOnce(&mut State) -> u8>(&mut self, val: Value, f: F) -> Result<(), InstrErr> {
        match self.curr_instr.cycle {
            2 => (),
            3 => match val {
                Value::IdxInd(addr) => {
                    self.curr_work.hold8 = addr.wrapping_add(self.x);
                }
                _ => {
                    return Err(InstrErr::ImpossibleCycle(
                        Self::an_incantation(&self.curr_instr.instr),
                        self.curr_instr.cycle,
                    ))
                }
            },
            4 => {
                self.curr_work.hold16 = self.at(self.curr_work.hold8 as u16) as u16;
            }
            5 => {
                self.curr_work.hold16 = self.at16_p(self.curr_work.hold8 as u16);
            }
            6 => {
                let hold = self.curr_work.hold16;
                let val = f(self);
                self.assign(hold, val);
                return self.finish_instr();
            }
            _ => {
                return Err(InstrErr::ImpossibleCycle(
                    Self::an_incantation(&self.curr_instr.instr),
                    self.curr_instr.cycle,
                ))
            }
        }
        Ok(())
    }

    fn idxind_r<F: FnOnce(&mut State, u8)>(&mut self, val: Value, f: F) -> Result<(), InstrErr> {
        match self.curr_instr.cycle {
            2 => (),
            3 => match val {
                Value::IdxInd(addr) => {
                    self.curr_work.hold8 = addr.wrapping_add(self.x);
                }
                _ => {
                    return Err(InstrErr::ImpossibleCycle(
                        Self::an_incantation(&self.curr_instr.instr),
                        self.curr_instr.cycle,
                    ))
                }
            },
            4 => {
                self.curr_work.hold16 = self.at(self.curr_work.hold8 as u16) as u16;
            }
            5 => {
                self.curr_work.hold16 = self.at16_p(self.curr_work.hold8 as u16);
            }
            6 => {
                let hold = self.curr_work.hold16;
                self.curr_work.hold8 = self.at(hold);
                f(self, self.curr_work.hold8);
                return self.finish_instr();
            }
            _ => {
                return Err(InstrErr::ImpossibleCycle(
                    Self::an_incantation(&self.curr_instr.instr),
                    self.curr_instr.cycle,
                ))
            }
        }
        Ok(())
    }

    fn idxind_rmw<F: FnOnce(&mut State, u8) -> u8>(
        &mut self,
        val: Value,
        f: F,
    ) -> Result<(), InstrErr> {
        match self.curr_instr.cycle {
            2 => (),
            3 => match val {
                Value::IdxInd(addr) => {
                    self.curr_work.hold8 = addr.wrapping_add(self.x);
                }
                _ => {
                    return Err(InstrErr::ImpossibleCycle(
                        Self::an_incantation(&self.curr_instr.instr),
                        self.curr_instr.cycle,
                    ))
                }
            },
            4 => {
                self.curr_work.hold16 = self.at(self.curr_work.hold8 as u16) as u16;
            }
            5 => {
                self.curr_work.hold16 = self.at16_p(self.curr_work.hold8 as u16);
            }
            6 => {
                self.curr_work.hold8 = self.at(self.curr_work.hold16);
            }
            7 => {
                let hold = self.curr_work.hold16;
                self.assign(hold, self.curr_work.hold8);
                self.curr_work.hold8 = f(self, self.curr_work.hold8);
            }
            8 => {
                let hold = self.curr_work.hold16;
                self.assign(hold, self.curr_work.hold8);
                return self.finish_instr();
            }
            _ => {
                return Err(InstrErr::ImpossibleCycle(
                    Self::an_incantation(&self.curr_instr.instr),
                    self.curr_instr.cycle,
                ))
            }
        }
        Ok(())
    }

    fn indidx_w<F: FnOnce(&mut State) -> u8>(&mut self, val: Value, f: F) -> Result<(), InstrErr> {
        match self.curr_instr.cycle {
            2 => (),
            3 => match val {
                Value::IndIdx(addr) => {
                    self.curr_work.hold8 = addr;
                }
                _ => {
                    return Err(InstrErr::ImpossibleCycle(
                        Self::an_incantation(&self.curr_instr.instr),
                        self.curr_instr.cycle,
                    ))
                }
            },
            4 => {
                self.curr_work.hold16 = self
                    .at16_p(self.curr_work.hold8 as u16)
                    .wrapping_add(self.y as u16);
            }
            5 => {
                self.curr_work.hold8 = self.at(self.curr_work.hold16);
            }
            6 => {
                let hold = self.curr_work.hold16;
                let val = f(self);
                self.assign(hold, val);
                return self.finish_instr();
            }
            _ => {
                return Err(InstrErr::ImpossibleCycle(
                    Self::an_incantation(&self.curr_instr.instr),
                    self.curr_instr.cycle,
                ))
            }
        }
        Ok(())
    }

    fn indidx_r<F: FnOnce(&mut State, u8)>(&mut self, val: Value, f: F) -> Result<(), InstrErr> {
        match self.curr_instr.cycle {
            2 => (),
            3 => match val {
                Value::IndIdx(addr) => {
                    self.curr_work.hold8 = addr;
                }
                _ => {
                    return Err(InstrErr::ImpossibleCycle(
                        Self::an_incantation(&self.curr_instr.instr),
                        self.curr_instr.cycle,
                    ))
                }
            },
            4 => {
                let hold = self.at16_p(self.curr_work.hold8 as u16);
                let (_, car) = (hold as u8).overflowing_add(self.y);
                self.curr_work.hold16 = hold.wrapping_add(self.y as u16);
                self.curr_work.hold8 = car as u8;
            }
            5 => {
                if self.curr_work.hold8 == 0 {
                    self.curr_work.hold8 = self.at(self.curr_work.hold16);
                    f(self, self.curr_work.hold8);
                    return self.finish_instr();
                } else {
                    self.curr_work.hold8 = self.at(self.curr_work.hold16);
                }
            }
            6 => {
                self.curr_work.hold8 = self.at(self.curr_work.hold16);
                f(self, self.curr_work.hold8);
                return self.finish_instr();
            }
            _ => {
                return Err(InstrErr::ImpossibleCycle(
                    Self::an_incantation(&self.curr_instr.instr),
                    self.curr_instr.cycle,
                ))
            }
        }
        Ok(())
    }

    fn indidx_rmw<F: FnOnce(&mut State, u8) -> u8>(
        &mut self,
        val: Value,
        f: F,
    ) -> Result<(), InstrErr> {
        match self.curr_instr.cycle {
            2 => (),
            3 => match val {
                Value::IndIdx(addr) => {
                    self.curr_work.hold8 = addr;
                }
                _ => {
                    return Err(InstrErr::ImpossibleCycle(
                        Self::an_incantation(&self.curr_instr.instr),
                        self.curr_instr.cycle,
                    ))
                }
            },
            4 => {
                self.curr_work.hold16 = self
                    .at16_p(self.curr_work.hold8 as u16)
                    .wrapping_add(self.y as u16);
            }
            5 => {
                self.curr_work.hold8 = self.at(self.curr_work.hold16);
            }
            6 => {
                self.curr_work.hold8 = self.at(self.curr_work.hold16);
            }
            7 => {
                let hold = self.curr_work.hold16;
                self.assign(hold, self.curr_work.hold8);
                self.curr_work.hold8 = f(self, self.curr_work.hold8);
            }
            8 => {
                let hold = self.curr_work.hold16;
                self.assign(hold, self.curr_work.hold8);
                return self.finish_instr();
            }
            _ => {
                return Err(InstrErr::ImpossibleCycle(
                    Self::an_incantation(&self.curr_instr.instr),
                    self.curr_instr.cycle,
                ))
            }
        }
        Ok(())
    }

    fn delegate_w<F: FnOnce(&mut State) -> u8>(
        &mut self,
        val: Value,
        f: F,
    ) -> Result<(), InstrErr> {
        use Value::*;
        match val {
            ZP(..) => self.zp_w(val, f),
            Abs(..) => self.abs_w(val, f),
            AbsX(..) | AbsY(..) => self.absxy_w(val, f),
            ZPX(..) | ZPY(..) => self.zpxy_w(val, f),
            IdxInd(..) => self.idxind_w(val, f),
            IndIdx(..) => self.indidx_w(val, f),
            _ => Err(InstrErr::ImpossibleCycle(
                Self::an_incantation(&self.curr_instr.instr),
                self.curr_instr.cycle,
            )),
        }
    }

    fn delegate_r<F: FnOnce(&mut State, u8)>(&mut self, val: Value, f: F) -> Result<(), InstrErr> {
        use Value::*;
        match val {
            Imm(..) => self.imm(val, f),
            ZP(..) => self.zp_r(val, f),
            Abs(..) => self.abs_r(val, f),
            AbsX(..) | AbsY(..) => self.absxy_r(val, f),
            ZPX(..) | ZPY(..) => self.zpxy_r(val, f),
            IdxInd(..) => self.idxind_r(val, f),
            IndIdx(..) => self.indidx_r(val, f),
            _ => Err(InstrErr::ImpossibleCycle(
                Self::an_incantation(&self.curr_instr.instr),
                self.curr_instr.cycle,
            )),
        }
    }

    fn delegate_rmw<F: FnOnce(&mut State, u8) -> u8>(
        &mut self,
        val: Value,
        f: F,
    ) -> Result<(), InstrErr> {
        use Value::*;
        match val {
            Acc => self.acc_rmw(f),
            ZP(..) => self.zp_rmw(val, f),
            ZPX(..) | ZPY(..) => self.zpxy_rmw(val, f),
            Abs(..) => self.abs_rmw(val, f),
            AbsX(..) | AbsY(..) => self.absxy_rmw(val, f),
            IdxInd(..) => self.idxind_rmw(val, f),
            IndIdx(..) => self.indidx_rmw(val, f),
            _ => Err(InstrErr::ImpossibleCycle(
                Self::an_incantation(&self.curr_instr.instr),
                self.curr_instr.cycle,
            )),
        }
    }

    fn handle_instr(&mut self, i: Instr) -> Result<(), InstrErr> {
        use Instr::*;
        use Value::*;

        let curr = &self.curr_instr;

        match i {
            Brk(Impl | Imm(..)) => {
                if (self.get_ctrl().gen_nmi) && (self.get_stt().vblank) && (curr.cycle < 5) {
                    return self.int_nmi();
                }
                match curr.cycle {
                    1..3 => (),
                    3 => self.push8_stack((self.pc >> 8) as u8),
                    4 => self.push8_stack(self.pc as u8),
                    5 => {
                        self.p.int_dis = true;
                        self.push8_stack(Into::<u8>::into(self.p) | 16u8);
                    }
                    6 => self.pc = 0x00FE,
                    7 => {
                        self.pc |= 0xFF00;
                        self.pc = self.at16(self.pc);
                        return self.finish_instr();
                    }
                    _ => return Err(InstrErr::ImpossibleCycle(i, curr.cycle)),
                }
                Ok(())
            }

            Rti => {
                match curr.cycle {
                    2..4 => (),
                    4 => {
                        let op = self.pop8_stack();
                        self.status_fromu8(op)
                    }
                    5 => (),
                    6 => {
                        self.pc = self.pop16_stack();
                        return self.finish_instr();
                    }
                    _ => return Err(InstrErr::ImpossibleCycle(i, curr.cycle)),
                }
                Ok(())
            }

            Rts => match curr.cycle {
                1..6 => Ok(()),
                6 => {
                    self.pc = self.pop16_stack().wrapping_add(1);
                    self.finish_instr()
                }
                _ => Err(InstrErr::ImpossibleCycle(i, curr.cycle)),
            },

            Jmp(val @ Abs(..)) => match curr.cycle {
                3 => {
                    self.pc = self.addrmode_in16(val);
                    self.finish_instr()
                }
                _ => Err(InstrErr::ImpossibleCycle(i, curr.cycle)),
            },
            Jmp(val @ Ind(..)) => match curr.cycle {
                3..5 => Ok(()),
                5 => {
                    self.pc = self.addrmode_in16(val);
                    self.finish_instr()
                }
                _ => Err(InstrErr::ImpossibleCycle(i, curr.cycle)),
            },

            Jsr(val @ Abs(..)) => {
                match curr.cycle {
                    3 => (),
                    4 => (),
                    5 => self.push16_stack(self.pc.wrapping_sub(1)),
                    6 => {
                        self.pc = self.addrmode_in16(val);
                        return self.finish_instr();
                    }
                    7 => return self.finish_instr(),
                    _ => return Err(InstrErr::ImpossibleCycle(i, curr.cycle)),
                }
                Ok(())
            }

            Bcs(val @ Rel(..)) => self.rel_branch(val, self.p.carry),
            Bcc(val @ Rel(..)) => self.rel_branch(val, !self.p.carry),

            Beq(val @ Rel(..)) => self.rel_branch(val, self.p.iszero),
            Bne(val @ Rel(..)) => self.rel_branch(val, !self.p.iszero),

            Bmi(val @ Rel(..)) => self.rel_branch(val, self.p.negative),
            Bpl(val @ Rel(..)) => self.rel_branch(val, !self.p.negative),

            Bvs(val @ Rel(..)) => self.rel_branch(val, self.p.overflow),
            Bvc(val @ Rel(..)) => self.rel_branch(val, !self.p.overflow),

            Nop(Impl) => self.impld(|_| ()),
            Nop(val) => self.delegate_r(val, |_, _| ()),

            Sec => self.impld(|st| st.p.carry = true),
            Sed => self.impld(|st| st.p.decimal = true),
            Sei => self.impld(|st| st.p.int_dis = true),

            Clc => self.impld(|st| st.p.carry = false),
            Cld => self.impld(|st| st.p.decimal = false),
            Cli => self.impld(|st| st.p.int_dis = false),
            Clv => self.impld(|st| st.p.overflow = false),

            Pha => match curr.cycle {
                2 => {
                    self.push8_stack(self.a);
                    Ok(())
                }
                3 => self.finish_instr(),
                _ => Err(InstrErr::ImpossibleCycle(i, curr.cycle)),
            },
            Php => match curr.cycle {
                2 => {
                    self.push8_stack(self.p.into());
                    Ok(())
                }
                3 => self.finish_instr(),
                _ => Err(InstrErr::ImpossibleCycle(i, curr.cycle)),
            },

            Pla => match curr.cycle {
                2 => Ok(()),
                3 => {
                    self.a = self.pop8_stack();
                    self.zn_check(self.a);
                    Ok(())
                }
                4 => self.finish_instr(),
                _ => Err(InstrErr::ImpossibleCycle(i, curr.cycle)),
            },
            Plp => match curr.cycle {
                2 => Ok(()),
                3 => {
                    let p = self.pop8_stack();
                    self.status_fromu8(p);
                    Ok(())
                }
                4 => self.finish_instr(),
                _ => Err(InstrErr::ImpossibleCycle(i, curr.cycle)),
            },

            Adc(
                val @ (Imm(..) | ZP(..) | ZPX(..) | Abs(..) | AbsX(..) | AbsY(..) | IdxInd(..)
                | IndIdx(..)),
            ) => self.delegate_r(val, |st, val| {
                let (res0, car0) = st.a.overflowing_add(val);
                let (res, car1) = res0.overflowing_add(st.p.carry as u8);
                st.p.carry = car0 | car1;
                st.p.overflow = (st.a < 128 && val < 128 && res >= 128)
                    || (st.a >= 128 && val >= 128 && res < 128);
                st.zn_check(res);
                st.a = res;
            }),

            Sbc(
                val @ (Imm(..) | ZP(..) | ZPX(..) | Abs(..) | AbsX(..) | AbsY(..) | IdxInd(..)
                | IndIdx(..)),
            ) => self.delegate_r(val, |st, val| {
                let val = !val;
                let (res0, car0) = st.a.overflowing_add(val);
                let (res, car1) = res0.overflowing_add(st.p.carry as u8);
                st.p.carry = car0 | car1;
                st.p.overflow = (st.a < 128 && val < 128 && res >= 128)
                    || (st.a >= 128 && val >= 128 && res < 128);
                st.zn_check(res);
                st.a = res;
            }),

            Inc(val @ (ZP(..) | ZPX(..) | Abs(..) | AbsX(..))) => {
                self.delegate_rmw(val, |st, val| {
                    let res = val.wrapping_add(1);
                    st.zn_check(res);
                    res
                })
            }

            Inx => self.impld(|st| {
                st.zn_check(st.x.wrapping_add(1));
                st.x = st.x.wrapping_add(1);
            }),

            Iny => self.impld(|st| {
                st.zn_check(st.y.wrapping_add(1));
                st.y = st.y.wrapping_add(1);
            }),

            Dec(val @ (ZP(..) | ZPX(..) | Abs(..) | AbsX(..))) => {
                self.delegate_rmw(val, |st, val| {
                    let res = val.wrapping_sub(1);
                    st.zn_check(res);
                    res
                })
            }

            Dex => self.impld(|st| {
                st.zn_check(st.x.wrapping_sub(1));
                st.x = st.x.wrapping_sub(1);
            }),

            Dey => self.impld(|st| {
                st.zn_check(st.y.wrapping_sub(1));
                st.y = st.y.wrapping_sub(1);
            }),

            And(
                val @ (Imm(..) | ZP(..) | ZPX(..) | Abs(..) | AbsX(..) | AbsY(..) | IdxInd(..)
                | IndIdx(..)),
            ) => self.delegate_r(val, |st, val| {
                let res = st.a & val;
                st.zn_check(res);
                st.a = res;
            }),

            Ora(
                val @ (Imm(..) | ZP(..) | ZPX(..) | Abs(..) | AbsX(..) | AbsY(..) | IdxInd(..)
                | IndIdx(..)),
            ) => self.delegate_r(val, |st, val| {
                let res = st.a | val;
                st.zn_check(res);
                st.a = res;
            }),

            Eor(
                val @ (Imm(..) | ZP(..) | ZPX(..) | Abs(..) | AbsX(..) | AbsY(..) | IdxInd(..)
                | IndIdx(..)),
            ) => self.delegate_r(val, |st, val| {
                let res = st.a ^ val;
                st.zn_check(res);
                st.a = res;
            }),

            Asl(val @ (Acc | ZP(..) | ZPX(..) | Abs(..) | AbsX(..))) => {
                self.delegate_rmw(val, |st, val| {
                    let res = val << 1;
                    st.p.carry = (val & 128) != 0;
                    st.zn_check(res);
                    res
                })
            }

            Lsr(val @ (Acc | ZP(..) | ZPX(..) | Abs(..) | AbsX(..))) => {
                self.delegate_rmw(val, |st, val| {
                    let res = val >> 1;
                    st.p.carry = (val & 1) != 0;
                    st.p.iszero = res == 0;
                    st.p.negative = false;
                    res
                })
            }

            Rol(val @ (Acc | ZP(..) | ZPX(..) | Abs(..) | AbsX(..))) => {
                self.delegate_rmw(val, |st, val| {
                    let mut res = val.rotate_left(1);
                    res = if st.p.carry { res | 1 } else { res & 0xFE };
                    st.p.carry = (val & 128) != 0;
                    st.zn_check(res);
                    res
                })
            }

            Ror(val @ (Acc | ZP(..) | ZPX(..) | Abs(..) | AbsX(..))) => {
                self.delegate_rmw(val, |st, val| {
                    let mut res = val.rotate_right(1);
                    res = if st.p.carry { res | 128 } else { res & 0x7F };
                    st.p.carry = (val & 1) != 0;
                    st.zn_check(res);
                    res
                })
            }

            Bit(val @ (ZP(..) | Abs(..))) => self.delegate_r(val, |st, val| {
                let res = st.a & val;
                st.p.iszero = res == 0;
                st.p.overflow = (val & 64) != 0;
                st.p.negative = (val & 128) != 0;
            }),

            Cmp(
                val @ (Imm(..) | ZP(..) | ZPX(..) | Abs(..) | AbsX(..) | AbsY(..) | IdxInd(..)
                | IndIdx(..)),
            ) => self.delegate_r(val, |st, val| {
                st.p.carry = st.a >= val;
                st.p.iszero = st.a == val;
                st.p.negative = ((st.a.wrapping_sub(val)) & 128) != 0;
            }),

            Cpx(val @ (Imm(..) | ZP(..) | Abs(..))) => self.delegate_r(val, |st, val| {
                st.p.carry = st.x >= val;
                st.p.iszero = st.x == val;
                st.p.negative = ((st.x.wrapping_sub(val)) & 128) != 0;
            }),

            Cpy(val @ (Imm(..) | ZP(..) | Abs(..))) => self.delegate_r(val, |st, val| {
                st.p.carry = st.y >= val;
                st.p.iszero = st.y == val;
                st.p.negative = ((st.y.wrapping_sub(val)) & 128) != 0;
            }),

            Lda(
                val @ (Imm(..) | ZP(..) | ZPX(..) | Abs(..) | AbsX(..) | AbsY(..) | IdxInd(..)
                | IndIdx(..)),
            ) => self.delegate_r(val, |st, val| {
                st.zn_check(val);
                st.a = val;
            }),

            Ldx(val @ (Imm(..) | ZP(..) | ZPY(..) | Abs(..) | AbsY(..))) => {
                self.delegate_r(val, |st, val| {
                    st.zn_check(val);
                    st.x = val;
                })
            }

            Ldy(val @ (Imm(..) | ZP(..) | ZPX(..) | Abs(..) | AbsX(..))) => {
                self.delegate_r(val, |st, val| {
                    st.zn_check(val);
                    st.y = val;
                })
            }

            Sta(
                val @ (ZP(..) | ZPX(..) | Abs(..) | AbsX(..) | AbsY(..) | IdxInd(..) | IndIdx(..)),
            ) => self.delegate_w(val, |st| st.a),

            Stx(val @ (ZP(..) | ZPY(..) | Abs(..))) => self.delegate_w(val, |st| st.x),

            Sty(val @ (ZP(..) | ZPX(..) | Abs(..))) => self.delegate_w(val, |st| st.y),

            Tay => self.impld(|st| {
                st.y = st.a;
                st.zn_check(st.a);
            }),
            Tax => self.impld(|st| {
                st.x = st.a;
                st.zn_check(st.a);
            }),
            Tsx => self.impld(|st| {
                st.x = st.s;
                st.zn_check(st.s);
            }),
            Txa => self.impld(|st| {
                st.a = st.x;
                st.zn_check(st.x);
            }),
            Txs => self.impld(|st| st.s = st.x),
            Tya => self.impld(|st| {
                st.a = st.y;
                st.zn_check(st.y);
            }),

            Kil => match curr.cycle {
                2 => Err(InstrErr::Halt(self.cycles)),
                _ => Err(InstrErr::ImpossibleCycle(i, curr.cycle)),
            },

            Lax(val @ (ZP(..) | ZPY(..) | Abs(..) | AbsY(..) | IdxInd(..) | IndIdx(..))) => self
                .delegate_r(val, |st, val| {
                    st.a = val;
                    st.x = val;
                    st.zn_check(val);
                }),

            Sax(val @ (ZP(..) | ZPY(..) | Abs(..) | IdxInd(..))) => {
                self.delegate_w(val, |st| st.a & st.x)
            }

            Dcp(
                val @ (ZP(..) | ZPX(..) | Abs(..) | AbsX(..) | AbsY(..) | IdxInd(..) | IndIdx(..)),
            ) => self.delegate_rmw(val, |st, val| {
                let res = val.wrapping_sub(1);
                st.zn_check(res);
                st.p.carry = st.a >= res;
                st.p.iszero = st.a == res;
                st.p.negative = ((st.a.wrapping_sub(res)) & 128) != 0;
                res
            }),

            Isc(
                val @ (ZP(..) | ZPX(..) | Abs(..) | AbsX(..) | AbsY(..) | IdxInd(..) | IndIdx(..)),
            ) => self.delegate_rmw(val, |st, val| {
                let res = val.wrapping_add(1);
                st.zn_check(res);
                let val = !res;
                let (res0, car0) = st.a.overflowing_add(val);
                let (res1, car1) = res0.overflowing_add(st.p.carry as u8);
                st.p.carry = car0 | car1;
                st.p.overflow = (st.a < 128 && val < 128 && res1 >= 128)
                    || (st.a >= 128 && val >= 128 && res1 < 128);
                st.zn_check(res1);
                st.a = res1;
                res
            }),

            Slo(
                val @ (ZP(..) | ZPX(..) | Abs(..) | AbsX(..) | AbsY(..) | IdxInd(..) | IndIdx(..)),
            ) => self.delegate_rmw(val, |st, val| {
                let res = val << 1;
                st.p.carry = (val & 128) != 0;
                st.zn_check(res);
                let res1 = st.a | res;
                st.zn_check(res1);
                st.a = res1;
                res
            }),

            Rla(
                val @ (ZP(..) | ZPX(..) | Abs(..) | AbsX(..) | AbsY(..) | IdxInd(..) | IndIdx(..)),
            ) => self.delegate_rmw(val, |st, val| {
                let mut res = val.rotate_left(1);
                res = if st.p.carry { res | 1 } else { res & 0xFE };
                st.p.carry = (val & 128) != 0;
                st.zn_check(res);
                let res1 = st.a & res;
                st.zn_check(res1);
                st.a = res1;
                res
            }),
            Sre(
                val @ (ZP(..) | ZPX(..) | Abs(..) | AbsX(..) | AbsY(..) | IdxInd(..) | IndIdx(..)),
            ) => self.delegate_rmw(val, |st, val| {
                let res = val >> 1;
                st.p.carry = (val & 1) != 0;
                st.p.iszero = res == 0;
                st.p.negative = false;
                let res1 = st.a ^ res;
                st.zn_check(res1);
                st.a = res1;
                res
            }),
            Rra(
                val @ (ZP(..) | ZPX(..) | Abs(..) | AbsX(..) | AbsY(..) | IdxInd(..) | IndIdx(..)),
            ) => self.delegate_rmw(val, |st, val| {
                let mut res = val.rotate_right(1);
                res = if st.p.carry { res | 128 } else { res & 0x7F };
                st.p.carry = (val & 1) != 0;
                st.zn_check(res);
                let (res0, car0) = st.a.overflowing_add(res);
                let (res1, car1) = res0.overflowing_add(st.p.carry as u8);
                st.p.carry = car0 | car1;
                st.p.overflow = (st.a < 128 && res < 128 && res1 >= 128)
                    || (st.a >= 128 && res >= 128 && res1 < 128);
                st.zn_check(res1);
                st.a = res1;
                res
            }),

            // NOTE: These might work, might not, more testing required
            Anc(val @ Imm(..)) => self.delegate_r(val, |st, val| {
                let res = st.a & val;
                st.zn_check(res);
                st.a = res;
                st.p.carry = st.a & 128 != 0;
            }),
            Alr(val @ Imm(..)) => self.delegate_r(val, |st, val| {
                let res = st.a & val;
                st.zn_check(res);
                st.a = res;
                let res = st.a >> 1;
                st.p.carry = (st.a & 1) != 0;
                st.p.iszero = res == 0;
                st.p.negative = false;
                st.a = res;
            }),
            Arr(val @ Imm(..)) => self.delegate_r(val, |st, val| {
                let res = st.a & val;
                st.zn_check(res);
                st.a = res;
                let mut res = st.a.rotate_right(1);
                res = if st.p.carry { res | 128 } else { res & 0x7F };
                st.p.carry = (st.a & 1) != 0;
                st.zn_check(res);
                st.a = res;
            }),
            Xaa(val @ Imm(..)) => self.delegate_r(val, |st, val| {
                st.a = st.x & val;
                st.zn_check(st.x & val);
            }),
            Axs(val @ Imm(..)) => self.delegate_r(val, |st, val| {
                st.x &= st.a;
                st.p.carry = st.x >= val;
                st.p.iszero = st.x == val;
                st.p.negative = ((st.x.wrapping_sub(val)) & 128) != 0;
                st.x = st.x.wrapping_sub(val);
            }),
            Ahx(val @ (AbsY(..) | IndIdx(..))) => self.delegate_w(val, |st| st.a & st.x),
            Shy(val @ AbsX(..)) => self.delegate_w(val, |st| st.y),
            Shx(val @ AbsY(..)) => self.delegate_w(val, |st| st.x),
            Tas(val @ AbsY(..)) => self.delegate_w(val, |st| {
                st.s = st.a & st.x;
                st.s
            }),
            // This one is potentially more reliable than the ones above
            Las(val @ AbsY(..)) => self.delegate_r(val, |st, val| {
                let res = val & st.s;
                st.zn_check(res);
                st.a = res;
                st.x = res;
                st.s = res;
            }),
            _ => Err(InstrErr::ImpossibleInstr(i)),
        }
    }

    pub fn cycle(&mut self) -> Result<(), InstrErr> {
        // Should probably handle these in a separate pseudo-instructions
        // But it works well for now
        if self.curr_work.dmc.dma {
            if self.curr_work.dmc.time != 0 {
                self.curr_work.dmc.time -= 1;
                self.cycles += 1;
                return Ok(());
            }
            if self.curr_work.dmc.halt {
                self.curr_work.dmc.fix = self.cycles % 2 == 1;
                self.curr_work.dmc.halt = false;
                self.cycles += 1;
                return Ok(());
            }
            if let Some(apu) = self.apu_st.clone() {
                self.do_dmc_dma(&mut apu.borrow_mut());
                return Ok(());
            }
        }

        if self.curr_work.oam.dma {
            if self.curr_work.oam.halt {
                self.curr_work.oam.fix = self.cycles % 2 == 1;
                self.curr_work.oam.halt = false;
                self.cycles += 1;
                return Ok(());
            }
            if let Some(ppu) = self.ppu_st.clone() {
                self.do_oam_dma(&mut ppu.borrow_mut());
                return Ok(());
            }
        }

        match self.curr_instr.instr {
            Some(Ok(Ok(i))) => {
                match self.handle_instr(i) {
                    Ok(()) => (),
                    Err(InstrErr::Bubble) => return Ok(()),
                    Err(e) => return Err(e),
                }
                self.curr_instr.cycle += 1;
                Ok(())
            }
            _ => match self.fetch_instr() {
                Ok(()) => Ok(()),
                Err(InstrErr::Bubble) => Ok(()),
                Err(e) => Err(e),
            },
        }
    }
}
