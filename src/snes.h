// SNES system glue for the SMW native port: memory map, CPU-side registers,
// DMA/HDMA, the scanline scheduler and the recompiled-code dispatcher.
#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

// SNES joypad bits as returned by $4218/$4219 (16-bit, $4219 = high byte)
enum {
  JOY_B = 0x8000, JOY_Y = 0x4000, JOY_SELECT = 0x2000, JOY_START = 0x1000,
  JOY_UP = 0x0800, JOY_DOWN = 0x0400, JOY_LEFT = 0x0200, JOY_RIGHT = 0x0100,
  JOY_A = 0x0080, JOY_X = 0x0040, JOY_L = 0x0020, JOY_R = 0x0010,
};

typedef struct SnesStats {
  uint64_t recomp_dispatches;   // entries into recompiled regions
  uint64_t interp_blocks;       // blocks executed by the fallback interpreter
  uint64_t interp_rom_blocks;   // ... of which were ROM code (= coverage misses)
  uint64_t frames;
} SnesStats;

bool snes_load_rom(const uint8_t *data, size_t size);   // strips copier header, checks hash
const char *snes_rom_sha1_expected(void);
void snes_reset(void);
void snes_run_frame(void);                 // run until line 225 (frame complete)
void snes_set_joypad(int port, uint16_t buttons);

uint8_t *snes_sram(void);
size_t snes_sram_size(void);
bool snes_sram_take_dirty(void);           // returns true once after SRAM changed

void snes_set_force_interpreter(bool on);  // debug: run everything through the interpreter
const SnesStats *snes_stats(void);
// write the list of ROM code entry points that had no recompiled variant
// (feed back into tools/recomp/recomp.py as a seed file). Returns count.
int snes_write_misses(const char *path);

size_t snes_save_state(uint8_t *buf);      // buf == NULL -> required size
bool snes_load_state(const uint8_t *buf, size_t size);
