use log::error;

use crate::cpu;
use crate::ppu;

#[derive(Default)]
pub struct State {
    map: u8,
    prg: Vec<u8>,
    chr: Vec<u8>,
    bank_prg: u8,
    bank_n_prg: u8,
}

impl State {
    pub fn new(
        cpu: &mut cpu::State,
        ppu: &mut ppu::State,
        map: u8,
        prg_s: u8,
        chr_s: u8,
        buf: &[u8],
    ) -> Self {
        let mut mapper = State {
            map,
            prg: vec![],
            chr: vec![],
            bank_prg: 0,
            bank_n_prg: 0,
        };

        match mapper.map {
            0 => {
                if prg_s == 2 {
                    mapper.prg = buf[..0x8000].to_vec();
                    cpu.load_prg(0x8000..0x10000, &mapper.prg[..0x8000]);
                    mapper.chr = buf[32 * 1024..].to_vec();
                } else if prg_s == 1 {
                    mapper.prg = buf[..0x4000].to_vec();
                    cpu.load_prg(0x8000..0xC000, &mapper.prg[..0x4000]);
                    cpu.load_prg(0xC000..0x10000, &mapper.prg[..0x4000]);
                    mapper.chr = buf[16 * 1024..].to_vec();
                } else {
                    error!("Failed to load PRG");
                }

                if (chr_s == 0 && mapper.chr.len() != 0) || chr_s == 1 {
                    ppu.load_chr(0..0x2000, &mapper.chr[0..0x2000]);
                } else if chr_s == 0 {
                } else {
                    error!("Failed to load CHR");
                }
            }
            2 => {
                mapper.bank_n_prg = prg_s - 1;
                mapper.prg = buf[..0x4000 * prg_s as usize].to_vec();

                cpu.load_prg(0x8000..0xC000, &mapper.prg[..0x4000]);
                cpu.load_prg(
                    0xC000..0x10000,
                    &mapper.prg[0x4000 * (prg_s as usize - 1)..0x4000 * prg_s as usize],
                );

                if chr_s != 0 {
                    error!("Did not expect CHR");
                }
            }

            _ => unimplemented!(),
        }

        mapper
    }

    pub fn bank_switch(&mut self, cpu: &mut cpu::State, addr: u16, value: u8) {
        match self.map {
            2 if (0x8000..0xFFFF).contains(&addr) => {
                self.bank_prg = value % self.bank_n_prg;
                cpu.load_prg(
                    0x8000..0xC000,
                    &self.prg
                        [0x4000 * self.bank_prg as usize..0x4000 * (self.bank_prg as usize + 1)],
                );
            }
            _ => {}
        }
    }
}
