# Super Mario World Recomp

A native PC port of *Super Mario World* (SNES, 1990) made by **static
recompilation**: the game's 65816 machine code is translated to C at build
time and compiled into a regular executable. It runs on a C reimplementation
of the SNES hardware it talks to (PPU, DMA/HDMA, timers and interrupts,
joypads, and the SPC700/S-DSP sound system), with SDL2 for the window, audio
and input.

> **No game data is included.** This repository contains only the
> recompiler, the runtime and the frontend. You need your own copy of
> Super Mario World (USA): the build reads it to generate the game code, and
> the game reads it at run time for graphics, levels and music.

## Features

- Plays the whole game natively: no emulated CPU for the game code
- Keyboard and game controllers (XInput, DualShock/DualSense, Switch Pro and
  anything in SDL's controller database), hot-pluggable, two players
- Window scaling, fullscreen, 4:3 or square pixels, optional linear filtering
- Battery saves, save states, fast-forward, screenshots
- Remappable keys and settings in a simple `smw.ini`

## Getting started

### 1. Bring your own ROM

You need **Super Mario World (USA)**, headered (`.smc`) or unheadered (`.sfc`).
The 512 KB image without a copier header must have this SHA-1:

```
6b47bb75d16514b6a476aa0c73a683a2a4c18765
```

Copy it into the repository root as **`smw.sfc`** (or `smw.smc`), or pass its
path to the build. Other revisions, regions and ROM hacks are not supported.

### 2. Build on Windows

Requirements:

- Visual Studio 2022 or newer with **Desktop development with C++** (it
  includes CMake and Ninja; tested with Visual Studio 2026)
- Python 3 on `PATH` (tested with 3.14)
- An internet connection the first time: CMake downloads the official SDL2
  2.32.10 development package and checks its SHA-256

```bat
git clone https://github.com/GabrielBigardi/SuperMarioWorld-Recomp.git
cd SuperMarioWorld-Recomp
copy "path\to\Super Mario World (USA).sfc" smw.sfc
build.bat
```

`build.bat path\to\rom.sfc` works too. A full build takes about a minute and
produces `build\smw.exe` and `build\SDL2.dll`; the C runtime is linked
statically, so nothing else is needed.

Or with CMake directly, from a Developer Command Prompt:

```
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release -DSMW_ROM=path/to/rom.sfc
cmake --build build
```

### Other platforms

The code is portable C17 + SDL2 and the CMake script has non-MSVC paths, so
Linux and macOS should build with the system SDL2 development package
(`libsdl2-dev`, `brew install sdl2`). Only Windows with MSVC has been built and
tested so far; reports and fixes are welcome.

## Playing

Run `build\smw.exe`. The ROM is looked up in this order: command-line argument,
`Rom=` in `smw.ini`, the ROM the game was built from, then `smw.sfc`/`smw.smc`
next to the exe or in its parent folder.

### Controls

| SNES | Keyboard (default) | Game controller |
|---|---|---|
| D-pad | Arrow keys | D-pad / left stick |
| B (jump) | Z | South (A / Cross) |
| A (spin jump) | X | East (B / Circle) |
| Y (run / fire) | A | West (X / Square) |
| X (run) | S | North (Y / Triangle) |
| L / R | Q / W | LB / RB (or triggers) |
| Start / Select | Enter / Right Shift | Start / Back |

| Hotkey | |
|---|---|
| Esc | quit |
| F11 / Alt+Enter | fullscreen |
| P | pause |
| Tab (hold) | fast-forward |
| F5 / F9 | save / load state |
| F12 | screenshot (BMP) |
| F1 | reset |

The second controller is player 2. For unusual controllers, put a
[`gamecontrollerdb.txt`](https://github.com/mdqinc/SDL_GameControllerDB) next
to the exe.

### Configuration and files

`smw.ini` is created next to the exe on first run. It holds the ROM path,
window scale, fullscreen, 4:3 aspect, filtering, volume, audio latency and the
keyboard mapping (SDL key names such as `Left Shift` or `Return`).

| File | Contents |
|---|---|
| `smw.srm` | battery save (the game's three save files) |
| `smw.state` | save state (F5 / F9) |
| `screenshot_NNN.bmp` | screenshots (F12) |
| `smw_misses.txt` | written only if code outside the recompiled set ran (see below) |

## How it works

```
tools/recomp/      65816 static recompiler (Python)
  w65816.py        instruction set and decoder (aware of the M/X width flags)
  analysis.py      control-flow discovery
  codegen.py       C emitter: recompiled code and the fallback interpreter
  recomp.py        driver: ROM -> build/generated/*.c
src/
  cpu.h bus.h      CPU state, instruction semantics, LoROM memory map
  snes.c           I/O registers, DMA/HDMA, scanline scheduler, NMI/IRQ, dispatcher, save states
  ppu.c            PPU: BG modes 0-7, sprites, windows, color math, mosaic, mode 7
  apu*.c           SPC700 + S-DSP (runs the sound driver the game uploads)
  main.c           SDL2 frontend
tests/             headless runner and CPU/PPU/APU test programs (see tests/README.md)
```

1. **Discovery.** Starting at the reset, NMI and IRQ vectors, the analyzer
   follows branches, calls and jump tables while tracking the 65816's M/X
   register-width flags; code is compiled once per flag state it can run in.
   It understands the idioms SMW relies on: `JSL ExecutePtr` / `ExecutePtrLong`
   inline pointer tables (including empty entries), hand-rolled calls
   (`PHK : PER : JML`), flag restoration through `PHP`/`PLP` (a dataflow over
   pushed flag contexts) and per-routine return-flag summaries. A final pass
   compiles plausible code in the gaps between known code. The result is
   about 27,000 blocks and 131,000 instructions, emitted as about 2,750 C
   functions.
2. **Execution.** Each block is ordinary C. Direct branches inside a function
   are `goto`s; everything else returns to a dispatcher keyed by program
   counter and flag state. Every instruction accounts for its master-clock
   cost, so raster effects (the status-bar IRQ split, HDMA windows and
   gradients) land on the right scanlines.
3. **Fallback.** If execution reaches code that was not recompiled, a generated
   interpreter (built from the same semantics as the recompiled code) runs it.
   In practice that is only the small OAM-clearing routine SMW copies into RAM
   at `$7F8000`. ROM addresses that ever need the fallback are written to
   `smw_misses.txt` on exit; add them to `tools/recomp/seeds.txt` and rebuild
   to compile them natively.

The generated code lives in `build/generated/` and is derived from your ROM,
so it stays out of the repository.

## Verification

- **CPU semantics:** 143,452 / 143,452 native-mode cases of the
  [SingleStepTests 65816](https://github.com/SingleStepTests/65816) suite pass
  (all 256 opcodes with random M/X/D and flags).
- **Recompiled code vs. interpreter:** WRAM and cycle counts are bit-identical
  on every frame over 4,000 frames of scripted gameplay.
- **Coverage:** boot, title screen and attract demo, file select, intro,
  overworld, Yoshi's House and Yoshi's Island 1 played through, plus all 95
  levels (translevels 01-5F) warped into and played for about 20 seconds each.
  None of it needed the fallback interpreter for ROM code.
- **Save states:** save, run, load and replay is bit-identical over 600 frames.
- **APU:** 256,000 / 256,000
  [SingleStepTests SPC700](https://github.com/SingleStepTests/spc700) cases;
  IPL boot, ADSR/GAIN, BRR, echo/FIR and noise unit tests; title music
  captured to WAV and checked.
- **PPU:** 103 register and rendering checks plus visual tests.

See [tests/README.md](tests/README.md) to run them yourself.

## Known limitations

- **Untested paths:** boss rooms entered through doors and the ending/credits
  were not reached by the automated runs. If they hit code that was not
  recompiled, it still runs correctly through the fallback and is logged to
  `smw_misses.txt`.
- **Sound driver:** the SPC700 sound program is code the game uploads to the
  audio CPU, so it runs on an SPC700 interpreter rather than being recompiled.
  The S-DSP is sample-accurate, not cycle-accurate.
- **Timing:** CPU timing is per instruction (8 master clocks per memory cycle,
  6 per internal cycle), so slowdown in busy scenes is close to, but not
  cycle-exact with, real hardware.
- **PPU:** hires, pseudo-hires and interlace modes render 256 pixels wide (SMW
  does not use them); sprite evaluation happens on the rendered line rather
  than one line earlier as on hardware.
- **Platforms:** only Windows/MSVC is tested (see above).

## Legal

This project is not affiliated with, endorsed by or sponsored by Nintendo.
*Super Mario World* is a trademark of Nintendo.

The repository contains no ROM data, game code or game assets. The recompiler
reads the ROM you supply at build time; the generated C code and the
executables built from it are derived from that ROM, so don't distribute them.
Like other SNES emulators, `src/apu.c` embeds the 64-byte SPC700 IPL boot ROM
that the hardware needs to start the sound CPU.

The source code is released under the [MIT License](LICENSE). SDL2 (zlib
license) is downloaded at build time; the SingleStepTests suites are
downloaded only when you run those tests.

## Acknowledgements

- [SDL2](https://www.libsdl.org/) for windowing, audio and input
- [SingleStepTests](https://github.com/SingleStepTests) for the 65816 and
  SPC700 test suites
- The SNES documentation and reference behaviour of fullsnes, anomie's
  documents, bsnes/higan and blargg's SPC_DSP notes
