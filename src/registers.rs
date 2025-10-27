#[derive(Default, Clone, Copy)]
pub struct Control {
    pub nt1: bool,
    pub nt2: bool,
    pub vram_ai: bool,
    pub sp_addr: bool,
    pub bp_addr: bool,
    pub s_size: bool,
    pub ms_select: bool,
    pub gen_nmi: bool,
}

impl From<u8> for Control {
    fn from(value: u8) -> Self {
        let mut ctrl = Control::default();
        ctrl.nt1 = value & 1 != 0;
        ctrl.nt2 = value & 2 != 0;
        ctrl.vram_ai = value & 4 != 0;
        ctrl.sp_addr = value & 8 != 0;
        ctrl.bp_addr = value & 16 != 0;
        ctrl.s_size = value & 32 != 0;
        ctrl.ms_select = value & 64 != 0;
        ctrl.gen_nmi = value & 128 != 0;
        ctrl
    }
}

impl From<Control> for u8 {
    fn from(value: Control) -> Self {
        let mut byte = 0;
        if value.nt1 {
            byte |= 1;
        }
        if value.nt2 {
            byte |= 2;
        }
        if value.vram_ai {
            byte |= 4;
        }
        if value.sp_addr {
            byte |= 8;
        }
        if value.bp_addr {
            byte |= 16;
        }
        if value.s_size {
            byte |= 32;
        }
        if value.ms_select {
            byte |= 64;
        }
        if value.gen_nmi {
            byte |= 128;
        }
        byte
    }
}

#[derive(Default, Clone, Copy)]
pub struct Mask {
    pub greyscale: bool,
    pub bg_left: bool,
    pub spr_left: bool,
    pub bg_rend: bool,
    pub spr_rend: bool,
    pub emp_red: bool,
    pub emp_green: bool,
    pub emp_blue: bool,
}

impl From<u8> for Mask {
    fn from(value: u8) -> Self {
        let mut mask = Mask::default();
        mask.greyscale = value & 1 != 0;
        mask.bg_left = value & 2 != 0;
        mask.spr_left = value & 4 != 0;
        mask.bg_rend = value & 8 != 0;
        mask.spr_rend = value & 16 != 0;
        mask.emp_red = value & 32 != 0;
        mask.emp_green = value & 64 != 0;
        mask.emp_blue = value & 128 != 0;
        mask
    }
}

impl From<Mask> for u8 {
    fn from(value: Mask) -> Self {
        let mut byte = 0;
        if value.greyscale {
            byte |= 1;
        }
        if value.bg_left {
            byte |= 2;
        }
        if value.spr_left {
            byte |= 4;
        }
        if value.bg_rend {
            byte |= 8;
        }
        if value.spr_rend {
            byte |= 16;
        }
        if value.emp_red {
            byte |= 32;
        }
        if value.emp_green {
            byte |= 64;
        }
        if value.emp_blue {
            byte |= 128;
        }
        byte
    }
}

#[derive(Default, Clone, Copy)]
pub struct Status {
    pub open_bus: u8,
    pub sprite_overflow: bool,
    pub sprite0_hit: bool,
    pub vblank: bool,
}

impl From<u8> for Status {
    fn from(value: u8) -> Self {
        let mut stt = Status::default();
        stt.open_bus = value & 0x1F;
        stt.sprite_overflow = value & 32 != 0;
        stt.sprite0_hit = value & 64 != 0;
        stt.vblank = value & 128 != 0;
        stt
    }
}

impl From<Status> for u8 {
    fn from(value: Status) -> Self {
        let mut byte = value.open_bus & 0x1F;
        if value.sprite_overflow {
            byte |= 32;
        }
        if value.sprite0_hit {
            byte |= 64;
        }
        if value.vblank {
            byte |= 128;
        }
        byte
    }
}
