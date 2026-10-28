// SNES system glue for the SMW native port.
//
// The game's 65816 code runs as statically recompiled C (generated/recomp_*.c).
// This file provides everything around it: the LoROM memory map, CPU-side
// I/O registers ($4200-$421F), DMA/HDMA, joypads, and a scanline scheduler that
// renders lines, raises NMI/IRQ and keeps the APU in sync.  The dispatcher maps
// the CPU's program counter (+ M/X/E flag state) to recompiled code, falling back
// to the generated interpreter for code that was not discovered statically
// (e.g. the small routine SMW copies to RAM at $7F8000).
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "snes.h"
#include "recomp_rt.h"
#include "ppu.h"
#include "apu.h"

// ------------------------------------------------------------------ timing
enum {
  MCLK_PER_LINE = 1364,
  LINES_PER_FRAME = 262,
  VBLANK_LINE = 225,
  HBLANK_MCLK = 1096,          // dot 274: render point + HDMA
  REFRESH_MCLK = 40,           // DRAM refresh steals ~40 clocks per line
  AUTOJOY_START = 132,
  AUTOJOY_LEN = 4224,
};

// ------------------------------------------------------------------ state
Cpu cpu;
uint64_t g_next_event;
uint8_t g_wram[0x20000];
uint8_t *g_rom;
uint32_t g_rom_mask;
uint8_t g_mdr;

static uint8_t *rom_buf;
static size_t rom_size;

#define SRAM_SIZE 0x800

typedef struct DmaChannel {
  uint8_t dmap, bbad, a1b, dasb, ntlr, unused;
  uint16_t a1t, das, a2a;
  uint8_t do_transfer, terminated;
} DmaChannel;

typedef struct Regs {
  uint8_t nmitimen, wrio, wrmpya, wrmpyb, hdmaen, memsel;
  uint16_t wrdiv, rddiv, rdmpy, htime, vtime;
  uint8_t rdnmi_flag, timeup_flag;
  uint16_t joy[4];         // auto-read latched values
  uint16_t joy_input[4];   // current host input
  uint8_t joy_strobe;
  uint32_t joy_shift[2];
  uint32_t wmadd;
  // scheduler
  uint64_t frame_start;    // mclk at V=0 of the current frame
  int line;                // current line
  uint64_t line_start;
  uint64_t hblank_time, irq_time, line_end;
  uint64_t vblank_time;
  uint8_t in_vblank, nmi_pending, irq_line, allow_vblank, frame_done;
  uint8_t sram_dirty;
} Regs;

static Regs r;
static DmaChannel dma[8];
static uint8_t sram[SRAM_SIZE];
static bool force_interp;
static SnesStats stats;

static const uint64_t NEVER = ~0ull;

// ------------------------------------------------------------------ SHA-1
typedef struct { uint32_t h[5]; uint64_t len; uint8_t buf[64]; size_t n; } Sha1;
static uint32_t rol32(uint32_t v, int s) { return v << s | v >> (32 - s); }
static void sha1_block(Sha1 *c, const uint8_t *p) {
  uint32_t w[80], a, b, d, e, f, k, t, cc;
  for (int i = 0; i < 16; i++) w[i] = (uint32_t)p[i * 4] << 24 | p[i * 4 + 1] << 16 | p[i * 4 + 2] << 8 | p[i * 4 + 3];
  for (int i = 16; i < 80; i++) w[i] = rol32(w[i - 3] ^ w[i - 8] ^ w[i - 14] ^ w[i - 16], 1);
  a = c->h[0]; b = c->h[1]; cc = c->h[2]; d = c->h[3]; e = c->h[4];
  for (int i = 0; i < 80; i++) {
    if (i < 20) { f = (b & cc) | (~b & d); k = 0x5A827999; }
    else if (i < 40) { f = b ^ cc ^ d; k = 0x6ED9EBA1; }
    else if (i < 60) { f = (b & cc) | (b & d) | (cc & d); k = 0x8F1BBCDC; }
    else { f = b ^ cc ^ d; k = 0xCA62C1D6; }
    t = rol32(a, 5) + f + e + k + w[i]; e = d; d = cc; cc = rol32(b, 30); b = a; a = t;
  }
  c->h[0] += a; c->h[1] += b; c->h[2] += cc; c->h[3] += d; c->h[4] += e;
}
static void sha1_hex(const uint8_t *data, size_t len, char out[41]) {
  Sha1 c = {{0x67452301, 0xEFCDAB89, 0x98BADCFE, 0x10325476, 0xC3D2E1F0}, 0, {0}, 0};
  size_t i = 0;
  for (; i + 64 <= len; i += 64) sha1_block(&c, data + i);
  uint8_t tail[128] = {0};
  size_t rem = len - i;
  memcpy(tail, data + i, rem);
  tail[rem] = 0x80;
  size_t tl = rem < 56 ? 64 : 128;
  uint64_t bits = (uint64_t)len * 8;
  for (int j = 0; j < 8; j++) tail[tl - 1 - j] = (uint8_t)(bits >> (8 * j));
  sha1_block(&c, tail);
  if (tl == 128) sha1_block(&c, tail + 64);
  for (int j = 0; j < 5; j++) sprintf(out + j * 8, "%08x", c.h[j]);
}

// ------------------------------------------------------------------ dispatcher table
#define HASH_BITS 17
#define HASH_SIZE (1u << HASH_BITS)
static const RecompEntry *dispatch_hash[HASH_SIZE];

static inline uint32_t hash_key(uint32_t k) { return (k * 2654435761u) >> (32 - HASH_BITS); }

static void build_dispatch(void) {
  memset(dispatch_hash, 0, sizeof(dispatch_hash));
  for (uint32_t i = 0; i < g_recomp_entry_count; i++) {
    const RecompEntry *e = &g_recomp_entries[i];
    uint32_t h = hash_key(e->key);
    while (dispatch_hash[h]) h = (h + 1) & (HASH_SIZE - 1);
    dispatch_hash[h] = e;
  }
}

static inline const RecompEntry *lookup(uint32_t key) {
  uint32_t h = hash_key(key);
  for (;;) {
    const RecompEntry *e = dispatch_hash[h];
    if (!e) return NULL;
    if (e->key == key) return e;
    h = (h + 1) & (HASH_SIZE - 1);
  }
}

// coverage misses (ROM code executed by the interpreter)
#define MISS_MAX 4096
static uint32_t misses[MISS_MAX], miss_from[MISS_MAX];
static int miss_count;
static uint32_t last_dispatch_pc;
static void record_miss(uint32_t key) {
  for (int i = 0; i < miss_count; i++) if (misses[i] == key) return;
  if (miss_count < MISS_MAX) { miss_from[miss_count] = last_dispatch_pc; misses[miss_count++] = key; }
}

int snes_write_misses(const char *path) {
  FILE *f = fopen(path, "w");
  if (!f) return -1;
  static const char *fsn[] = {"m0x0", "m0x1", "m1x0", "m1x1", "e"};
  for (int i = 0; i < miss_count; i++)
    fprintf(f, "%06X %s  # from %06X\n", misses[i] >> 3, fsn[misses[i] & 7], miss_from[i]);
  fclose(f);
  return miss_count;
}

// ------------------------------------------------------------------ ROM
const char *snes_rom_sha1_expected(void) { return g_recomp_rom_sha1; }

bool snes_load_rom(const uint8_t *data, size_t size) {
  if (size % 1024 == 512) { data += 512; size -= 512; }
  char hex[41];
  sha1_hex(data, size, hex);
  if (strcmp(hex, g_recomp_rom_sha1) != 0) {
    fprintf(stderr, "ROM SHA-1 %s does not match the ROM this port was recompiled from (%s)\n", hex, g_recomp_rom_sha1);
    return false;
  }
  uint32_t pow2 = 1;
  while (pow2 < size) pow2 <<= 1;
  free(rom_buf);
  rom_buf = (uint8_t *)malloc(pow2);
  if (!rom_buf) return false;
  for (uint32_t i = 0; i < pow2; i++) rom_buf[i] = data[i % size];   // mirror to power of two
  rom_size = pow2;
  g_rom = rom_buf;
  g_rom_mask = pow2 - 1;
  build_dispatch();
  return true;
}

// ------------------------------------------------------------------ helpers
static inline uint16_t cur_hpos(void) {
  uint64_t d = cpu.mclk - r.line_start;
  if (d >= MCLK_PER_LINE) d = MCLK_PER_LINE - 1;
  return (uint16_t)(d >> 2);
}

static void update_next_event(void) {
  uint64_t t = r.line_end;
  if (r.hblank_time < t) t = r.hblank_time;
  if (r.irq_time < t) t = r.irq_time;
  if (r.nmi_pending || (r.irq_line && !cpu.fi)) t = 0;
  g_next_event = t;
}

// compute the H/V IRQ trigger time within the current line.  At line start the
// IRQ is always armed (even if the CPU overshot it slightly); when the timer
// registers change mid-line it only fires if the trigger point is still ahead.
static void schedule_irq_ex(bool at_line_start) {
  r.irq_time = NEVER;
  int mode = (r.nmitimen >> 4) & 3;
  if (!mode) return;
  uint64_t t;
  if (mode == 2) {                       // V-IRQ: start of line VTIME
    if (r.line != r.vtime) return;
    t = r.line_start + 10;
  } else {                               // H-IRQ (every line) or HV-IRQ
    if (mode == 3 && r.line != r.vtime) return;
    if (r.htime > 339) return;
    t = r.line_start + r.htime * 4u + 14;
  }
  if (at_line_start || t >= cpu.mclk) r.irq_time = t;
}
static void schedule_irq(void) { schedule_irq_ex(false); }

// ------------------------------------------------------------------ B-bus
static uint8_t bbus_read(uint8_t reg) {
  if (reg >= 0x34 && reg <= 0x3F) {
    if (reg == 0x37) {
      ppu_latch_hv(cur_hpos(), (uint16_t)r.line);
      return g_mdr;
    }
    return ppu_read(reg, g_mdr);
  }
  if (reg >= 0x40 && reg <= 0x7F) {
    apu_run_to(cpu.mclk);
    return apu_read_port(reg & 3);
  }
  if (reg == 0x80) {
    uint8_t v = g_wram[r.wmadd];
    r.wmadd = (r.wmadd + 1) & 0x1FFFF;
    return v;
  }
  return g_mdr;
}

static void bbus_write(uint8_t reg, uint8_t v) {
  if (reg <= 0x33) { ppu_write(reg, v); return; }
  if (reg >= 0x40 && reg <= 0x7F) {
    apu_run_to(cpu.mclk);
    apu_write_port(reg & 3, v);
    return;
  }
  switch (reg) {
  case 0x80: g_wram[r.wmadd] = v; r.wmadd = (r.wmadd + 1) & 0x1FFFF; break;
  case 0x81: r.wmadd = (r.wmadd & 0x1FF00) | v; break;
  case 0x82: r.wmadd = (r.wmadd & 0x100FF) | (uint32_t)v << 8; break;
  case 0x83: r.wmadd = (r.wmadd & 0x0FFFF) | (uint32_t)(v & 1) << 16; break;
  }
}

// ------------------------------------------------------------------ DMA
static const uint8_t dma_pattern[8][4] = {
  {0, 0, 0, 0}, {0, 1, 0, 1}, {0, 0, 0, 0}, {0, 0, 1, 1},
  {0, 1, 2, 3}, {0, 1, 0, 1}, {0, 0, 0, 0}, {0, 0, 1, 1},
};
static const uint8_t dma_unit[8] = {1, 2, 2, 4, 4, 4, 2, 4};

// A-bus accesses by the DMA controller cannot reach B-bus / DMA registers
static uint8_t abus_read(uint32_t a) {
  uint16_t off = (uint16_t)a;
  if (!(a & 0x400000) && ((off & 0xFF00) == 0x2100 || (off >= 0x4300 && off < 0x4380) || off == 0x420B || off == 0x420C))
    return g_mdr;
  return rd8(a);
}
static void abus_write(uint32_t a, uint8_t v) {
  uint16_t off = (uint16_t)a;
  if (!(a & 0x400000) && ((off & 0xFF00) == 0x2100 || (off >= 0x4300 && off < 0x4380) || off == 0x420B || off == 0x420C))
    return;
  wr8(a, v);
}

static void run_dma(uint8_t mask) {
  cpu.mclk += 12;
  for (int ch = 0; ch < 8; ch++) {
    if (!(mask & (1 << ch))) continue;
    DmaChannel *c = &dma[ch];
    int mode = c->dmap & 7;
    bool to_a = c->dmap & 0x80;
    int step = (c->dmap & 0x08) ? 0 : (c->dmap & 0x10) ? -1 : 1;
    uint32_t k = 0;
    cpu.mclk += 8;
    do {
      uint8_t breg = (uint8_t)(c->bbad + dma_pattern[mode][k & 3]);
      uint32_t aaddr = (uint32_t)c->a1b << 16 | c->a1t;
      if (to_a) abus_write(aaddr, bbus_read(breg));
      else bbus_write(breg, abus_read(aaddr));
      c->a1t = (uint16_t)(c->a1t + step);
      c->das--;
      k++;
      cpu.mclk += 8;
    } while (c->das != 0);
  }
}

static void hdma_reload(DmaChannel *c) {
  if ((c->ntlr & 0x7F) == 0) {
    c->ntlr = rd8((uint32_t)c->a1b << 16 | c->a2a);
    c->a2a++;
    c->terminated = c->ntlr == 0;
    c->do_transfer = !c->terminated;
    if (c->dmap & 0x40) {
      uint8_t lo = rd8((uint32_t)c->a1b << 16 | c->a2a); c->a2a++;
      uint8_t hi = rd8((uint32_t)c->a1b << 16 | c->a2a); c->a2a++;
      c->das = (uint16_t)(lo | hi << 8);
      cpu.mclk += 16;
    }
    cpu.mclk += 8;
  }
}

static void hdma_init(void) {
  for (int ch = 0; ch < 8; ch++) {
    DmaChannel *c = &dma[ch];
    if (!(r.hdmaen & (1 << ch))) continue;
    c->a2a = c->a1t;
    c->ntlr = 0;
    c->terminated = 0;
    hdma_reload(c);
  }
}

static void hdma_run(void) {
  if (!r.hdmaen) return;
  cpu.mclk += 18;
  for (int ch = 0; ch < 8; ch++) {
    DmaChannel *c = &dma[ch];
    if (!(r.hdmaen & (1 << ch)) || c->terminated) continue;
    cpu.mclk += 8;
    if (c->do_transfer) {
      int mode = c->dmap & 7;
      bool to_a = c->dmap & 0x80;
      for (int k = 0; k < dma_unit[mode]; k++) {
        uint32_t aaddr;
        if (c->dmap & 0x40) { aaddr = (uint32_t)c->dasb << 16 | c->das; c->das++; }
        else { aaddr = (uint32_t)c->a1b << 16 | c->a2a; c->a2a++; }
        uint8_t breg = (uint8_t)(c->bbad + dma_pattern[mode][k]);
        if (to_a) abus_write(aaddr, bbus_read(breg));
        else bbus_write(breg, abus_read(aaddr));
        cpu.mclk += 8;
      }
    }
  }
  for (int ch = 0; ch < 8; ch++) {
    DmaChannel *c = &dma[ch];
    if (!(r.hdmaen & (1 << ch)) || c->terminated) continue;
    c->ntlr--;
    c->do_transfer = (c->ntlr & 0x80) != 0;
    hdma_reload(c);
  }
}

// ------------------------------------------------------------------ CPU I/O
static void latch_autojoy(void) {
  for (int i = 0; i < 4; i++) r.joy[i] = r.joy_input[i];
}

static uint8_t cpu_reg_read(uint16_t off) {
  switch (off) {
  case 0x4210: {
    uint8_t v = (uint8_t)((r.rdnmi_flag << 7) | (g_mdr & 0x70) | 0x02);
    r.rdnmi_flag = 0;
    return v;
  }
  case 0x4211: {
    uint8_t v = (uint8_t)((r.timeup_flag << 7) | (g_mdr & 0x7F));
    r.timeup_flag = 0;
    r.irq_line = 0;
    return v;
  }
  case 0x4212: {
    uint16_t h = cur_hpos();
    uint8_t v = g_mdr & 0x3E;
    if (r.in_vblank) v |= 0x80;
    if (h >= 274 || h < 1) v |= 0x40;
    if (r.in_vblank && (r.nmitimen & 1)) {
      uint64_t d = cpu.mclk - r.vblank_time;
      if (d >= AUTOJOY_START && d < AUTOJOY_START + AUTOJOY_LEN) v |= 0x01;
    }
    return v;
  }
  case 0x4213: return r.wrio;
  case 0x4214: return (uint8_t)r.rddiv;
  case 0x4215: return (uint8_t)(r.rddiv >> 8);
  case 0x4216: return (uint8_t)r.rdmpy;
  case 0x4217: return (uint8_t)(r.rdmpy >> 8);
  case 0x4218: case 0x421A: case 0x421C: case 0x421E:
    return (uint8_t)r.joy[(off - 0x4218) >> 1];
  case 0x4219: case 0x421B: case 0x421D: case 0x421F:
    return (uint8_t)(r.joy[(off - 0x4219) >> 1] >> 8);
  }
  return g_mdr;
}

static void cpu_reg_write(uint16_t off, uint8_t v) {
  switch (off) {
  case 0x4200: {
    uint8_t old = r.nmitimen;
    r.nmitimen = v;
    if (!((v >> 4) & 3)) { r.irq_line = 0; r.timeup_flag = 0; }
    if (!(old & 0x80) && (v & 0x80) && r.rdnmi_flag) r.nmi_pending = 1;
    schedule_irq();
    update_next_event();
    break;
  }
  case 0x4201:
    if ((r.wrio & 0x80) && !(v & 0x80)) ppu_latch_hv(cur_hpos(), (uint16_t)r.line);
    r.wrio = v;
    break;
  case 0x4202: r.wrmpya = v; break;
  case 0x4203:
    r.wrmpyb = v;
    r.rdmpy = (uint16_t)(r.wrmpya * v);
    r.rddiv = v;
    break;
  case 0x4204: r.wrdiv = (r.wrdiv & 0xFF00) | v; break;
  case 0x4205: r.wrdiv = (uint16_t)((r.wrdiv & 0x00FF) | v << 8); break;
  case 0x4206:
    if (v == 0) { r.rddiv = 0xFFFF; r.rdmpy = r.wrdiv; }
    else { r.rddiv = r.wrdiv / v; r.rdmpy = r.wrdiv % v; }
    break;
  case 0x4207: r.htime = (r.htime & 0x100) | v; schedule_irq(); update_next_event(); break;
  case 0x4208: r.htime = (r.htime & 0xFF) | (uint16_t)(v & 1) << 8; schedule_irq(); update_next_event(); break;
  case 0x4209: r.vtime = (r.vtime & 0x100) | v; schedule_irq(); update_next_event(); break;
  case 0x420A: r.vtime = (r.vtime & 0xFF) | (uint16_t)(v & 1) << 8; schedule_irq(); update_next_event(); break;
  case 0x420B: if (v) run_dma(v); break;
  case 0x420C: r.hdmaen = v; break;
  case 0x420D: r.memsel = v; break;
  }
}

static uint8_t dma_reg_read(uint16_t off) {
  DmaChannel *c = &dma[(off >> 4) & 7];
  switch (off & 0xF) {
  case 0x0: return c->dmap;
  case 0x1: return c->bbad;
  case 0x2: return (uint8_t)c->a1t;
  case 0x3: return (uint8_t)(c->a1t >> 8);
  case 0x4: return c->a1b;
  case 0x5: return (uint8_t)c->das;
  case 0x6: return (uint8_t)(c->das >> 8);
  case 0x7: return c->dasb;
  case 0x8: return (uint8_t)c->a2a;
  case 0x9: return (uint8_t)(c->a2a >> 8);
  case 0xA: return c->ntlr;
  case 0xB: case 0xF: return c->unused;
  }
  return g_mdr;
}

static void dma_reg_write(uint16_t off, uint8_t v) {
  DmaChannel *c = &dma[(off >> 4) & 7];
  switch (off & 0xF) {
  case 0x0: c->dmap = v; break;
  case 0x1: c->bbad = v; break;
  case 0x2: c->a1t = (c->a1t & 0xFF00) | v; break;
  case 0x3: c->a1t = (uint16_t)((c->a1t & 0x00FF) | v << 8); break;
  case 0x4: c->a1b = v; break;
  case 0x5: c->das = (c->das & 0xFF00) | v; break;
  case 0x6: c->das = (uint16_t)((c->das & 0x00FF) | v << 8); break;
  case 0x7: c->dasb = v; break;
  case 0x8: c->a2a = (c->a2a & 0xFF00) | v; break;
  case 0x9: c->a2a = (uint16_t)((c->a2a & 0x00FF) | v << 8); break;
  case 0xA: c->ntlr = v; break;
  case 0xB: case 0xF: c->unused = v; break;
  }
}

static uint8_t joyser_read(int port) {
  // manual serial read ($4016/$4017): bits shift out B,Y,Sel,St,U,D,L,R,A,X,L,R,0,0,0,0 then 1s
  uint8_t bit;
  if (r.joy_strobe) bit = (r.joy_input[port] >> 15) & 1;
  else { bit = (r.joy_shift[port] >> 31) & 1; r.joy_shift[port] = r.joy_shift[port] << 1 | 1; }
  return (uint8_t)((g_mdr & (port ? 0xE0 : 0xFC)) | bit | (port ? 0x1C : 0));
}

uint8_t bus_read_slow(uint32_t a) {
  uint32_t bank = a >> 16, off = a & 0xFFFF;
  uint32_t b7 = bank & 0x7F;
  if (b7 < 0x40) {
    g_mdr = (uint8_t)(off >> 8);   // open bus: last byte fetched was the address high byte
    if ((off & 0xFF00) == 0x2100) return g_mdr = bbus_read((uint8_t)off);
    if (off == 0x4016 || off == 0x4017) return g_mdr = joyser_read(off & 1);
    if (off >= 0x4200 && off < 0x4220) return g_mdr = cpu_reg_read((uint16_t)off);
    if (off >= 0x4300 && off < 0x4380) return g_mdr = dma_reg_read((uint16_t)off);
    return g_mdr;
  }
  if (b7 >= 0x70 && off < 0x8000) return g_mdr = sram[off & (SRAM_SIZE - 1)];
  if (off >= 0x8000) return g_mdr = g_rom[((b7 << 15) | (off & 0x7FFF)) & g_rom_mask];
  return g_mdr;
}

void bus_write_slow(uint32_t a, uint8_t v) {
  uint32_t bank = a >> 16, off = a & 0xFFFF;
  uint32_t b7 = bank & 0x7F;
  if (b7 < 0x40) {
    if ((off & 0xFF00) == 0x2100) { bbus_write((uint8_t)off, v); return; }
    if (off == 0x4016) {
      uint8_t s = v & 1;
      if (r.joy_strobe && !s) { r.joy_shift[0] = (uint32_t)r.joy_input[0] << 16; r.joy_shift[1] = (uint32_t)r.joy_input[1] << 16; }
      r.joy_strobe = s;
      if (s) { r.joy_shift[0] = (uint32_t)r.joy_input[0] << 16; r.joy_shift[1] = (uint32_t)r.joy_input[1] << 16; }
      return;
    }
    if (off >= 0x4200 && off < 0x4220) { cpu_reg_write((uint16_t)off, v); return; }
    if (off >= 0x4300 && off < 0x4380) { dma_reg_write((uint16_t)off, v); return; }
    return;
  }
  if (b7 >= 0x70 && off < 0x8000) {
    uint8_t *p = &sram[off & (SRAM_SIZE - 1)];
    if (*p != v) { *p = v; r.sram_dirty = 1; }
  }
}

// ------------------------------------------------------------------ scheduler
static void start_line(int line) {
  r.line = line;
  r.line_start = r.frame_start + (uint64_t)line * MCLK_PER_LINE;
  r.line_end = r.line_start + MCLK_PER_LINE;
  r.hblank_time = (line <= VBLANK_LINE - 1) ? r.line_start + HBLANK_MCLK : NEVER;
  cpu.mclk += REFRESH_MCLK;
  if (line == 0) {
    r.in_vblank = 0;
    r.rdnmi_flag = 0;
    ppu_frame_start();
    hdma_init();
  } else if (line == VBLANK_LINE) {
    r.in_vblank = 1;
    r.vblank_time = r.line_start;
    r.rdnmi_flag = 1;
    if (r.nmitimen & 0x80) r.nmi_pending = 1;
    if (r.nmitimen & 1) latch_autojoy();
    ppu_vblank_start();
  }
  schedule_irq_ex(true);
}

static void process_events(void) {
  for (;;) {
    uint64_t t = r.line_end;
    int which = 2;
    if (r.hblank_time <= t) { t = r.hblank_time; which = 1; }
    if (r.irq_time <= t) { t = r.irq_time; which = 0; }
    if (cpu.mclk < t) break;
    if (which == 0) {
      r.irq_time = NEVER;
      r.timeup_flag = 1;
      r.irq_line = 1;
    } else if (which == 1) {
      r.hblank_time = NEVER;
      if (r.line >= 1 && r.line <= 224) ppu_render_line(r.line);
      hdma_run();
    } else {
      int next = r.line + 1;
      if (next == LINES_PER_FRAME) {
        r.frame_start += (uint64_t)LINES_PER_FRAME * MCLK_PER_LINE;
        next = 0;
      }
      if (next == VBLANK_LINE && !r.allow_vblank) {
        r.frame_done = 1;
        break;
      }
      if (next == VBLANK_LINE) r.allow_vblank = 0;
      start_line(next);
    }
  }
  update_next_event();
}

void cpu_interrupt(uint16_t native_vec, uint16_t emu_vec, bool brk) {
  cpu.waiting = 0;
  if (!cpu.fe) {
    push8(cpu.pb);
    push16(cpu.pc);
    push8(cpu_get_p());
  } else {
    push16(cpu.pc);
    push8((uint8_t)(brk ? cpu_get_p() | 0x10 : cpu_get_p() & ~0x10));
  }
  cpu.fi = 1;
  cpu.fd = 0;
  cpu.pb = 0;
  cpu.pc = rd16(cpu.fe ? emu_vec : native_vec);
  cpu.mclk += 8 * 8;
}

static inline void dispatch(void) {
  uint32_t pc24 = (uint32_t)cpu.pb << 16 | cpu.pc;
  uint32_t canon = (pc24 >= 0x800000 && pc24 < 0xFE0000) ? pc24 & 0x7FFFFF : pc24;
  uint32_t key = canon << 3 | (uint32_t)cpu_fs();
  uint32_t from = last_dispatch_pc;
  last_dispatch_pc = canon;
  (void)from;
  if (!force_interp) {
    const RecompEntry *e = lookup(key);
    if (e) {
      stats.recomp_dispatches++;
      e->fn(e->idx);
      return;
    }
  }
  stats.interp_blocks++;
  uint32_t bank = canon >> 16;
  if ((canon & 0x8000) && bank < 0x7E) {
    stats.interp_rom_blocks++;
    if (!force_interp) { last_dispatch_pc = from; record_miss(key); last_dispatch_pc = canon; }
  }
  while (!interp_step()) {}
}

void snes_run_frame(void) {
  r.allow_vblank = 1;
  r.frame_done = 0;
  update_next_event();
  for (;;) {
    if (cpu.mclk >= g_next_event) {
      process_events();
      if (r.frame_done) break;
    }
    if (r.nmi_pending) {
      r.nmi_pending = 0;
      cpu_interrupt(0xFFEA, 0xFFFA, false);
      update_next_event();
    } else if (r.irq_line && !cpu.fi) {
      cpu_interrupt(0xFFEE, 0xFFFE, false);
      update_next_event();
    }
    if (cpu.stopped) { cpu.mclk = g_next_event; continue; }
    if (cpu.waiting) {
      if (r.irq_line) cpu.waiting = 0;   // WAI with I set resumes without servicing
      else { if (cpu.mclk < g_next_event) cpu.mclk = g_next_event; continue; }
    }
    dispatch();
    if (r.nmi_pending || (r.irq_line && !cpu.fi)) g_next_event = 0;
  }
  apu_run_to(cpu.mclk);
  stats.frames++;
}

// ------------------------------------------------------------------ public
void cpu_reset(void) {
  memset(&cpu, 0, sizeof(cpu));
  cpu.fe = 1; cpu.fm = 1; cpu.fx = 1; cpu.fi = 1;
  cpu.s = 0x01FF;
  cpu.pc = rd16(0xFFFC);
}

void snes_reset(void) {
  memset(g_wram, 0, sizeof(g_wram));
  memset(&r, 0, sizeof(r));
  memset(dma, 0xFF, sizeof(dma));
  for (int i = 0; i < 8; i++) { dma[i].do_transfer = 0; dma[i].terminated = 1; }
  g_mdr = 0;
  ppu_reset();
  apu_reset();
  cpu_reset();
  r.frame_start = cpu.mclk;
  r.line = 0;
  r.vtime = r.htime = 0x1FF;
  r.irq_time = NEVER;
  r.allow_vblank = 0;
  start_line(0);
  update_next_event();
}

void snes_set_joypad(int port, uint16_t buttons) {
  if (port >= 0 && port < 4) r.joy_input[port] = buttons;
}

uint8_t *snes_sram(void) { return sram; }
size_t snes_sram_size(void) { return SRAM_SIZE; }
bool snes_sram_take_dirty(void) { bool d = r.sram_dirty; r.sram_dirty = 0; return d; }
void snes_set_force_interpreter(bool on) { force_interp = on; }
const SnesStats *snes_stats(void) { return &stats; }

// ------------------------------------------------------------------ save states
#define STATE_MAGIC 0x31534D53u  // "SMS1"

size_t snes_save_state(uint8_t *buf) {
  size_t n = 4 + sizeof(cpu) + sizeof(g_wram) + sizeof(sram) + sizeof(r) + sizeof(dma) + 1;
  size_t ppu_n = ppu_save_state(NULL), apu_n = apu_save_state(NULL);
  n += 8 + ppu_n + apu_n;
  if (!buf) return n;
  uint8_t *p = buf;
  uint32_t magic = STATE_MAGIC;
  memcpy(p, &magic, 4); p += 4;
  memcpy(p, &cpu, sizeof(cpu)); p += sizeof(cpu);
  memcpy(p, g_wram, sizeof(g_wram)); p += sizeof(g_wram);
  memcpy(p, sram, sizeof(sram)); p += sizeof(sram);
  memcpy(p, &r, sizeof(r)); p += sizeof(r);
  memcpy(p, dma, sizeof(dma)); p += sizeof(dma);
  *p++ = g_mdr;
  uint32_t a = (uint32_t)ppu_n, b = (uint32_t)apu_n;
  memcpy(p, &a, 4); p += 4;
  memcpy(p, &b, 4); p += 4;
  p += ppu_save_state(p);
  p += apu_save_state(p);
  return (size_t)(p - buf);
}

bool snes_load_state(const uint8_t *buf, size_t size) {
  if (size != snes_save_state(NULL)) return false;
  const uint8_t *p = buf;
  uint32_t magic;
  memcpy(&magic, p, 4); p += 4;
  if (magic != STATE_MAGIC) return false;
  uint64_t keep_mclk_base = 0; (void)keep_mclk_base;
  memcpy(&cpu, p, sizeof(cpu)); p += sizeof(cpu);
  memcpy(g_wram, p, sizeof(g_wram)); p += sizeof(g_wram);
  memcpy(sram, p, sizeof(sram)); p += sizeof(sram);
  uint16_t joy_in[4];
  memcpy(joy_in, r.joy_input, sizeof(joy_in));
  memcpy(&r, p, sizeof(r)); p += sizeof(r);
  memcpy(r.joy_input, joy_in, sizeof(joy_in));
  memcpy(dma, p, sizeof(dma)); p += sizeof(dma);
  g_mdr = *p++;
  uint32_t a, b;
  memcpy(&a, p, 4); p += 4;
  memcpy(&b, p, 4); p += 4;
  ppu_load_state(p); p += a;
  apu_load_state(p); p += b;
  update_next_event();
  return true;
}
