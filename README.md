# vnes
Personal NES emulator with cycle-accurate 6502 CPU, PPU, getting-there APU
and keyboard input support.

Utilizes [softbuffer](https://github.com/rust-windowing/softbuffer) for graphics, 
which means this is entirely CPU driven and does not require a GPU.

Includes a nearest-neighbor interpolation for screen scaling purposes.

Currently supports mappers 0 and 2 only.

## Build
You can build it with the usual `cargo build --release`. 
For cross-platform builds use e.g. `cargo build --release --target x86_64-pc-windows-gnu`.

Note that platforms other than Windows are untested. 
However X11, Wayland, and Mac should be supported according to 
`softbuffer`.

Theoretically even mobile devices and the web are supported, 
but these likely require a little bit more work.

## Run
One rom included here is `alter.nes`, 
which is [Alter Ego](https://www.nesworld.com/article.php?system=nes&data=neshomebrew), 
a homebrew port of an older ZX Spectrum game. 

Running the binary with a path to it, e.g. `vnes nes/alter.nes`, should be enough to enjoy it. 
You may choose to build and run it yourself with `cargo run -- nes/alter.nes`.
Note that debug version may be too slow, in which case you would need to use the release version
through `cargo run --release -- nes/alter.nes`.

Otherwise, you may feel free to use your own roms

Current hard-coded controls for player 1 are ESDF for movement, 'J'=A 'K'=B, 'L'=Select, ';'=Start. 
Player 2 is arrow keys, and 7890 in the same order as player 1.

## Tests
A few [nesdev tests](https://www.nesdev.org/wiki/Emulator_tests) are provided as well for testing
purposes. These are then compared to an existing accurate emulator, like Mesen.

Currently tests like 240p are almost completely identical to Mesen 
bar a bug due to sprite rendering and WIP APU.

## Plans
- Currently APU needs work
- Fix sprite rendering bug
- Support for actual gamepads (likely an extra library)
- Need to add a CLI/config for things like scaling, framerate, and keyboard input
- More mappers
