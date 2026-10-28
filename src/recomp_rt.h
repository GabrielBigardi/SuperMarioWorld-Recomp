// Runtime interface used by the generated (recompiled) code.
#pragma once
#include "cpu.h"

typedef void (*RegionFn)(uint32_t entry);
typedef struct RecompEntry {
  uint32_t key;      // (canonical 24-bit pc << 3) | flag state
  RegionFn fn;
  uint32_t idx;      // entry index inside the region function
} RecompEntry;

extern const RecompEntry g_recomp_entries[];
extern const uint32_t g_recomp_entry_count;
extern const char g_recomp_rom_sha1[];

// fallback interpreter (generated/interp.c); returns 1 at a block boundary
int interp_step(void);

// instruction fetch: does not disturb the open-bus latch, so interpreter and
// recompiled code observe identical MDR values
static inline uint8_t fetch8(uint32_t a) {
  uint8_t m = g_mdr;
  uint8_t v = rd8(a);
  g_mdr = m;
  return v;
}
