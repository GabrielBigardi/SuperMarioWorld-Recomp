// SNES memory bus (LoROM, as used by Super Mario World).
#pragma once
#include <stdint.h>
#include <stdbool.h>

extern uint8_t g_wram[0x20000];
extern uint8_t *g_rom;
extern uint32_t g_rom_mask;
extern uint8_t g_mdr;   // last value on the data bus (open bus)

uint8_t bus_read_slow(uint32_t addr);
void bus_write_slow(uint32_t addr, uint8_t v);

#ifdef SST_FLAT_BUS
// CPU conformance tests (tests/cpu_sst): flat 16 MiB address space
extern uint8_t g_flat_mem[0x1000000];
static inline uint8_t rd8(uint32_t a) { return g_mdr = g_flat_mem[a & 0xFFFFFF]; }
static inline void wr8(uint32_t a, uint8_t v) { g_mdr = v; g_flat_mem[a & 0xFFFFFF] = v; }
#else

static inline uint8_t rd8(uint32_t a) {
  uint32_t bank = (a >> 16) & 0xFF;
  uint32_t off = a & 0xFFFF;
  if (!(bank & 0x40)) {                      // $00-$3F / $80-$BF
    if (off < 0x2000) return g_mdr = g_wram[off];
    if (off & 0x8000) return g_mdr = g_rom[(((bank & 0x7F) << 15) | (off & 0x7FFF)) & g_rom_mask];
  } else if ((bank & 0xFE) == 0x7E) {
    return g_mdr = g_wram[a & 0x1FFFF];
  }
  return bus_read_slow(a & 0xFFFFFF);
}

static inline void wr8(uint32_t a, uint8_t v) {
  uint32_t bank = (a >> 16) & 0xFF;
  uint32_t off = a & 0xFFFF;
  g_mdr = v;
  if (!(bank & 0x40)) {
    if (off < 0x2000) { g_wram[off] = v; return; }
  } else if ((bank & 0xFE) == 0x7E) {
    g_wram[a & 0x1FFFF] = v; return;
  }
  bus_write_slow(a & 0xFFFFFF, v);
}
#endif
