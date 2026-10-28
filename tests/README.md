# Tests and tools

Configure with `-DSMW_BUILD_TESTS=ON` (needs the ROM like the normal build) to
build these programs into `build/tests/`:

```
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release -DSMW_BUILD_TESTS=ON
cmake --build build
ctest --test-dir build          # self-contained tests: APU unit, DSP end-to-end, PPU
```

## `smw_headless` — the game without a window

Runs the recompiled game for N frames with optional scripted input and writes
hashes, screenshots, audio or coverage information.

```
build/tests/smw_headless smw.sfc --frames 4000 --script tests/headless/play3.txt --hash-log rec.txt
build/tests/smw_headless smw.sfc --frames 4000 --script tests/headless/play3.txt --hash-log int.txt --interp
```

Identical `rec.txt` / `int.txt` (WRAM hash + cycle count per frame) means the
recompiled code and the interpreter behave the same.

| Option | |
|---|---|
| `--frames N` | number of frames to run (default 600) |
| `--script file` | input script, lines `frame buttons_hex` (held from that frame on) |
| `--interp` | run everything through the fallback interpreter instead of the recompiled code |
| `--hash-log file` | per-frame WRAM hash and master-clock count |
| `--shots dir N` | dump a raw 256x224 XRGB frame every N frames (`tools/sheet.py` makes a PNG contact sheet) |
| `--wav file` | record the audio output |
| `--warp XX` | enter translevel `XX` (hex) instead of the one selected on the overworld; use with `sweep.txt` |
| `--state-test F` | save state at frame F, keep running, reload and check that the replay is identical |
| `--misses file` | write ROM blocks that needed the fallback interpreter (seed format for `tools/recomp/seeds.txt`) |

Button bits: B `8000`, Y `4000`, Select `2000`, Start `1000`, Up `0800`,
Down `0400`, Left `0200`, Right `0100`, A `0080`, X `0040`, L `0020`, R `0010`.

## `cpu_sst` — 65816 instruction semantics

Runs the native-mode cases of the [SingleStepTests 65816](https://github.com/SingleStepTests/65816)
suite through the generated interpreter (the same semantics generator that
produces the recompiled code), on a flat 16 MB bus.

```
python -I tests/cpu_sst/fetch_tests.py tests/cpu_sst/data       # ~77 MB download
python -I tests/cpu_sst/convert.py tests/cpu_sst/data tests/cpu_sst/tests.txt
build/tests/cpu_sst tests/cpu_sst/tests.txt
```

## APU — SPC700 and S-DSP

```
python -I tests/apu_test/fetch_spc700_tests.py tests/apu_test/data   # ~80 MB download
python -I tests/apu_test/convert_spc700_tests.py tests/apu_test/data tests/apu_test/data/spc700_tests.bin
build/tests/spc700_test tests/apu_test/data/spc700_tests.bin
build/tests/apu_unit_test
build/tests/dsp_test build/tests/out && python -I tests/apu_test/analyze_dsp.py build/tests/out   # needs numpy
```

`dsp_test` uploads a small program through the IPL boot protocol and records
two WAVs (with and without echo); `analyze_dsp.py` checks pitch, envelope,
stereo levels and echo timing.

## `ppu_test` — rendering

Builds synthetic scenes (BG modes 1 and 7, sprites, windows, color math,
mosaic, fades) and checks registers and pixels. Pass an output directory to
get PPM dumps; `tests/ppu_test/convert.py` turns them into PNGs (needs Pillow).
