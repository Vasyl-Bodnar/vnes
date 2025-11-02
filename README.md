# vnes
Personal NES emulator with cycle-accurate 6502, getting-there PPU, 
and keyboard input support.

Utilizes [softbuffer](https://github.com/rust-windowing/softbuffer) for graphics, 
which means this is entirely CPU driven.

Includes a nearest-neighbor interpolation for screen scaling purposes, 
though the value is currently hard-coded.

Currently supports mappers 0 and 2 only.

## Build
You can build it with the usual `cargo build --release`. 

Note that platforms other than Windows are untested. 
However X11, Wayland, and Mac should be supported according to 
`softbuffer`.

Theoretically even mobile devices and the web are supported, 
but these likely require a little bit more work.

## Run
Default rom included here is `alter.nes`, 
which is ["Alter Ego"](https://www.nesworld.com/article.php?system=nes&data=neshomebrew), 
a homebrew port of an older ZX Spectrum game. 

Running the binary, or through `cargo run`, should be enough to enjoy it. 
Note that debug version may be too slow, in which case you would need to use the release version
through `cargo run --release`.

Otherwise, you may feel free to add your own roms and change the hardcoded file value.

Current hard-coded controls for player 1 are ESDF for movement, 'J'=A 'K'=B, 'L'=Select, ';'=Start. 
Player 2 is arrow keys, and 7890 in the same order as player 1.

## Tests
A few ["nesdev tests"](https://www.nesdev.org/wiki/Emulator_tests) are provided as well for testing
purposes. These are then compared to an existing accurate emulator, like Mesen.

## Plans
- Currently PPU needs finalization and higher accuracy
- Need to add an APU and support for actual gamepads (likely an extra library)
- Need to add a CLI/config for things like scaling, game selection, framerate, and keyboard input
- More mappers
- Refactoring of older parts
