#[derive(Debug, Default, Clone, Copy)]
pub struct Buttons {
    pub a: bool,
    pub b: bool,
    pub select: bool,
    pub start: bool,
    pub up: bool,
    pub down: bool,
    pub left: bool,
    pub right: bool,
}

impl From<u8> for Buttons {
    fn from(value: u8) -> Self {
        let mut buts = Buttons::default();
        buts.a = value & 1 != 0;
        buts.b = value & 2 != 0;
        buts.select = value & 4 != 0;
        buts.start = value & 8 != 0;
        buts.up = value & 16 != 0;
        buts.down = value & 32 != 0;
        buts.left = value & 64 != 0;
        buts.right = value & 128 != 0;
        buts
    }
}

impl From<Buttons> for u8 {
    fn from(value: Buttons) -> Self {
        let mut byte = 0;
        if value.a {
            byte |= 1;
        }
        if value.b {
            byte |= 2;
        }
        if value.select {
            byte |= 4;
        }
        if value.start {
            byte |= 8;
        }
        if value.up {
            byte |= 16;
        }
        if value.down {
            byte |= 32;
        }
        if value.left {
            byte |= 64;
        }
        if value.right {
            byte |= 128;
        }
        byte
    }
}

#[derive(Default, Debug)]
pub struct State {
    strobe: bool,
    shift: u8,
    pub buttons: Buttons,
}

impl State {
    pub fn write_strobe(&mut self, value: u8) {
        if value == 0 {
            self.shift = 1;
        }
        self.strobe = value != 0;
    }

    pub fn read(&mut self) -> u8 {
        if self.strobe {
            self.buttons.a as u8
        } else {
            if self.shift == 0 {
                1
            } else {
                let res = u8::from(self.buttons) & self.shift != 0;
                self.shift = self.shift.wrapping_shl(1);
                res as u8
            }
        }
    }
}
