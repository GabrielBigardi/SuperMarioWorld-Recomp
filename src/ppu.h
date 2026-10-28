// SNES PPU (S-PPU1/S-PPU2) — scanline renderer used by the SMW native port.
// Pure C17, no dependencies besides libc.
#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#define PPU_WIDTH  256
#define PPU_HEIGHT 224   // visible lines 1..224 -> framebuffer rows 0..223

void ppu_init(void);
void ppu_reset(void);

// CPU write to $2100-$2133.  reg = (address & 0xFF), i.e. 0x00..0x33.
void ppu_write(uint8_t reg, uint8_t val);

// CPU read from $2134-$213F. reg = (address & 0xFF), i.e. 0x34..0x3F.
// For write-only registers / undefined bits return open_bus (the CPU MDR) as hardware does.
uint8_t ppu_read(uint8_t reg, uint8_t open_bus);

// Latch the H/V counters (called by the bus on a $2137 read, or $4201 bit7 1->0).
void ppu_latch_hv(uint16_t h, uint16_t v);

// Called by the scheduler at V=0 (start of a new frame).
void ppu_frame_start(void);

// Called by the scheduler at the first line of vblank (V=225).
// Hardware: if not in forced blank, OAM address is reloaded from the OAMADD register.
void ppu_vblank_start(void);

// Render visible line y (1..224) using the current register state into
// framebuffer row (y-1). Called by the scheduler once per line, at the start of
// H-blank of that line (i.e. after any mid-line register writes for the line,
// before HDMA for the next line runs).
void ppu_render_line(int y);

// 256x224 pixels, 0x00RRGGBB, row-major, stride PPU_WIDTH.
uint32_t *ppu_framebuffer(void);

// Raw memories (for debugging / save states).
uint8_t  *ppu_vram(void);   // 64 KiB
uint8_t  *ppu_cgram(void);  // 512 bytes
uint8_t  *ppu_oam(void);    // 544 bytes

// Serialize full PPU state (registers, latches, memories). Returns bytes written.
// If buf == NULL, returns required size.
size_t ppu_save_state(uint8_t *buf);
void   ppu_load_state(const uint8_t *buf);
