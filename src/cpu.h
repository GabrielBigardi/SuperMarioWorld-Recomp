// 65816 CPU state + instruction semantics shared by the statically recompiled
// game code (generated/*.c) and the fallback interpreter (generated/interp.c).
#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "bus.h"

typedef struct Cpu {
  uint16_t a, x, y, s, dp, pc;
  uint8_t db, pb;
  // flags, one byte each (0/1)
  uint8_t fn, fv, fm, fx, fd, fi, fz, fc, fe;
  uint8_t waiting, stopped;
  uint64_t mclk;       // master clock (21.477 MHz) consumed so far
} Cpu;

extern Cpu cpu;
extern uint64_t g_next_event;   // dispatcher must service events when mclk >= this

#define CYC(n) (cpu.mclk += (n))
#define EVENT_DUE() (cpu.mclk >= g_next_event)

// flag state index used to select compiled variants: 0..3 = (m<<1)|x, 4 = emulation
static inline int cpu_fs(void) { return cpu.fe ? 4 : (cpu.fm << 1) | cpu.fx; }

static inline uint8_t cpu_get_p(void) {
  return (uint8_t)(cpu.fn << 7 | cpu.fv << 6 | cpu.fm << 5 | cpu.fx << 4 |
                   cpu.fd << 3 | cpu.fi << 2 | cpu.fz << 1 | cpu.fc);
}
static inline void cpu_set_p(uint8_t p) {
  cpu.fn = (p >> 7) & 1; cpu.fv = (p >> 6) & 1; cpu.fm = (p >> 5) & 1; cpu.fx = (p >> 4) & 1;
  cpu.fd = (p >> 3) & 1; cpu.fi = (p >> 2) & 1; cpu.fz = (p >> 1) & 1; cpu.fc = p & 1;
  if (cpu.fe) { cpu.fm = 1; cpu.fx = 1; }
  if (cpu.fx) { cpu.x &= 0xFF; cpu.y &= 0xFF; }
}

// ---- register helpers
#define A8 ((uint8_t)cpu.a)
#define X8 ((uint8_t)cpu.x)
#define Y8 ((uint8_t)cpu.y)
#define SET_A8(v) (cpu.a = (uint16_t)((cpu.a & 0xFF00) | (uint8_t)(v)))
#define NZ8(v)  do { uint8_t _t = (uint8_t)(v); cpu.fz = _t == 0; cpu.fn = _t >> 7; } while (0)
#define NZ16(v) do { uint16_t _t = (uint16_t)(v); cpu.fz = _t == 0; cpu.fn = (uint8_t)(_t >> 15); } while (0)
#define DBADDR(a) (((uint32_t)cpu.db << 16) | (uint16_t)(a))

// ---- 16/24-bit memory helpers
static inline uint16_t rd16(uint32_t ea) {          // linear (crosses banks)
  uint8_t lo = rd8(ea);
  return (uint16_t)(lo | rd8((ea + 1) & 0xFFFFFF) << 8);
}
static inline uint16_t rd16_b0(uint16_t ea) {       // bank 0, wraps at $FFFF
  uint8_t lo = rd8(ea);
  return (uint16_t)(lo | rd8((uint16_t)(ea + 1)) << 8);
}
static inline uint32_t rd24_b0(uint16_t ea) {
  uint16_t lo = rd16_b0(ea);
  return lo | (uint32_t)rd8((uint16_t)(ea + 2)) << 16;
}
static inline uint32_t rd24(uint32_t ea) {
  uint16_t lo = rd16(ea);
  return lo | (uint32_t)rd8((ea + 2) & 0xFFFFFF) << 16;
}
static inline void wr16(uint32_t ea, uint16_t v) {
  wr8(ea, (uint8_t)v); wr8((ea + 1) & 0xFFFFFF, (uint8_t)(v >> 8));
}
static inline void wr16_b0(uint16_t ea, uint16_t v) {
  wr8(ea, (uint8_t)v); wr8((uint16_t)(ea + 1), (uint8_t)(v >> 8));
}
// read-modify-write order: high byte first
static inline void wr16_rmw(uint32_t ea, uint16_t v) {
  wr8((ea + 1) & 0xFFFFFF, (uint8_t)(v >> 8)); wr8(ea, (uint8_t)v);
}
static inline void wr16_rmw_b0(uint16_t ea, uint16_t v) {
  wr8((uint16_t)(ea + 1), (uint8_t)(v >> 8)); wr8(ea, (uint8_t)v);
}

// ---- stack
static inline void push8(uint8_t v) {
  wr8(cpu.s, v);
  cpu.s--;
  if (cpu.fe) cpu.s = 0x100 | (cpu.s & 0xFF);
}
static inline uint8_t pull8(void) {
  cpu.s++;
  if (cpu.fe) cpu.s = 0x100 | (cpu.s & 0xFF);
  return rd8(cpu.s);
}
static inline void push16(uint16_t v) { push8((uint8_t)(v >> 8)); push8((uint8_t)v); }
static inline uint16_t pull16(void) { uint8_t lo = pull8(); return (uint16_t)(lo | pull8() << 8); }

// ---- ALU
static inline void op_adc8(uint8_t data) {
  int a = (uint8_t)cpu.a, r;
  if (!cpu.fd) {
    r = a + data + cpu.fc;
  } else {
    r = (a & 0x0f) + (data & 0x0f) + cpu.fc;
    if (r > 0x09) r += 0x06;
    cpu.fc = r > 0x0f;
    r = (a & 0xf0) + (data & 0xf0) + (cpu.fc << 4) + (r & 0x0f);
  }
  cpu.fv = (~(a ^ data) & (a ^ r) & 0x80) != 0;
  if (cpu.fd && r > 0x9f) r += 0x60;
  cpu.fc = r > 0xff;
  SET_A8(r); NZ8(r);
}
static inline void op_adc16(uint16_t data) {
  int a = cpu.a, r;
  if (!cpu.fd) {
    r = a + data + cpu.fc;
  } else {
    r = (a & 0x000f) + (data & 0x000f) + cpu.fc;
    if (r > 0x0009) r += 0x0006;
    cpu.fc = r > 0x000f;
    r = (a & 0x00f0) + (data & 0x00f0) + (cpu.fc << 4) + (r & 0x000f);
    if (r > 0x009f) r += 0x0060;
    cpu.fc = r > 0x00ff;
    r = (a & 0x0f00) + (data & 0x0f00) + (cpu.fc << 8) + (r & 0x00ff);
    if (r > 0x09ff) r += 0x0600;
    cpu.fc = r > 0x0fff;
    r = (a & 0xf000) + (data & 0xf000) + (cpu.fc << 12) + (r & 0x0fff);
  }
  cpu.fv = (~(a ^ data) & (a ^ r) & 0x8000) != 0;
  if (cpu.fd && r > 0x9fff) r += 0x6000;
  cpu.fc = r > 0xffff;
  cpu.a = (uint16_t)r; NZ16(r);
}
static inline void op_sbc8(uint8_t data) {
  int a = (uint8_t)cpu.a, r;
  data = (uint8_t)~data;
  if (!cpu.fd) {
    r = a + data + cpu.fc;
  } else {
    r = (a & 0x0f) + (data & 0x0f) + cpu.fc;
    if (r <= 0x0f) r -= 0x06;
    cpu.fc = r > 0x0f;
    r = (a & 0xf0) + (data & 0xf0) + (cpu.fc << 4) + (r & 0x0f);
  }
  cpu.fv = (~(a ^ data) & (a ^ r) & 0x80) != 0;
  if (cpu.fd && r <= 0xff) r -= 0x60;
  cpu.fc = r > 0xff;
  SET_A8(r); NZ8(r);
}
static inline void op_sbc16(uint16_t data) {
  int a = cpu.a, r;
  data = (uint16_t)~data;
  if (!cpu.fd) {
    r = a + data + cpu.fc;
  } else {
    r = (a & 0x000f) + (data & 0x000f) + cpu.fc;
    if (r <= 0x000f) r -= 0x0006;
    cpu.fc = r > 0x000f;
    r = (a & 0x00f0) + (data & 0x00f0) + (cpu.fc << 4) + (r & 0x000f);
    if (r <= 0x00ff) r -= 0x0060;
    cpu.fc = r > 0x00ff;
    r = (a & 0x0f00) + (data & 0x0f00) + (cpu.fc << 8) + (r & 0x00ff);
    if (r <= 0x0fff) r -= 0x0600;
    cpu.fc = r > 0x0fff;
    r = (a & 0xf000) + (data & 0xf000) + (cpu.fc << 12) + (r & 0x0fff);
  }
  cpu.fv = (~(a ^ data) & (a ^ r) & 0x8000) != 0;
  if (cpu.fd && r <= 0xffff) r -= 0x6000;
  cpu.fc = r > 0xffff;
  cpu.a = (uint16_t)r; NZ16(r);
}
static inline void op_cmp8(uint8_t r, uint8_t v) {
  int t = r - v; cpu.fc = t >= 0; NZ8(t);
}
static inline void op_cmp16(uint16_t r, uint16_t v) {
  int t = r - v; cpu.fc = t >= 0; NZ16(t);
}
static inline void op_bit8(uint8_t v) {
  cpu.fn = v >> 7; cpu.fv = (v >> 6) & 1; cpu.fz = (v & (uint8_t)cpu.a) == 0;
}
static inline void op_bit16(uint16_t v) {
  cpu.fn = (uint8_t)(v >> 15); cpu.fv = (v >> 14) & 1; cpu.fz = (v & cpu.a) == 0;
}
static inline uint8_t op_asl8(uint8_t v) { cpu.fc = v >> 7; v = (uint8_t)(v << 1); NZ8(v); return v; }
static inline uint16_t op_asl16(uint16_t v) { cpu.fc = v >> 15; v = (uint16_t)(v << 1); NZ16(v); return v; }
static inline uint8_t op_lsr8(uint8_t v) { cpu.fc = v & 1; v >>= 1; NZ8(v); return v; }
static inline uint16_t op_lsr16(uint16_t v) { cpu.fc = v & 1; v >>= 1; NZ16(v); return v; }
static inline uint8_t op_rol8(uint8_t v) { uint8_t c = cpu.fc; cpu.fc = v >> 7; v = (uint8_t)(v << 1 | c); NZ8(v); return v; }
static inline uint16_t op_rol16(uint16_t v) { uint16_t c = cpu.fc; cpu.fc = v >> 15; v = (uint16_t)(v << 1 | c); NZ16(v); return v; }
static inline uint8_t op_ror8(uint8_t v) { uint8_t c = cpu.fc; cpu.fc = v & 1; v = (uint8_t)(v >> 1 | c << 7); NZ8(v); return v; }
static inline uint16_t op_ror16(uint16_t v) { uint16_t c = cpu.fc; cpu.fc = v & 1; v = (uint16_t)(v >> 1 | c << 15); NZ16(v); return v; }
static inline uint8_t op_inc8(uint8_t v) { v++; NZ8(v); return v; }
static inline uint16_t op_inc16(uint16_t v) { v++; NZ16(v); return v; }
static inline uint8_t op_dec8(uint8_t v) { v--; NZ8(v); return v; }
static inline uint16_t op_dec16(uint16_t v) { v--; NZ16(v); return v; }
static inline uint8_t op_tsb8(uint8_t v) { cpu.fz = (v & (uint8_t)cpu.a) == 0; return v | (uint8_t)cpu.a; }
static inline uint16_t op_tsb16(uint16_t v) { cpu.fz = (v & cpu.a) == 0; return v | cpu.a; }
static inline uint8_t op_trb8(uint8_t v) { cpu.fz = (v & (uint8_t)cpu.a) == 0; return v & (uint8_t)~cpu.a; }
static inline uint16_t op_trb16(uint16_t v) { cpu.fz = (v & cpu.a) == 0; return v & (uint16_t)~cpu.a; }

static inline void op_xce(void) {
  uint8_t t = cpu.fc; cpu.fc = cpu.fe; cpu.fe = t;
  if (cpu.fe) {
    cpu.fm = 1; cpu.fx = 1; cpu.x &= 0xFF; cpu.y &= 0xFF;
    cpu.s = 0x100 | (cpu.s & 0xFF);
  }
}

// One step of MVN/MVP (moves one byte). Returns true when the move is finished.
static inline bool op_mvn_step(uint8_t dst, uint8_t src) {
  cpu.db = dst;
  wr8((uint32_t)dst << 16 | cpu.y, rd8((uint32_t)src << 16 | cpu.x));
  if (cpu.fx) { cpu.x = (cpu.x + 1) & 0xFF; cpu.y = (cpu.y + 1) & 0xFF; }
  else { cpu.x++; cpu.y++; }
  return cpu.a-- == 0;
}
static inline bool op_mvp_step(uint8_t dst, uint8_t src) {
  cpu.db = dst;
  wr8((uint32_t)dst << 16 | cpu.y, rd8((uint32_t)src << 16 | cpu.x));
  if (cpu.fx) { cpu.x = (cpu.x - 1) & 0xFF; cpu.y = (cpu.y - 1) & 0xFF; }
  else { cpu.x--; cpu.y--; }
  return cpu.a-- == 0;
}

// Interrupt / exception entry (vectors are 16-bit addresses in bank 0)
void cpu_interrupt(uint16_t native_vec, uint16_t emu_vec, bool brk);
void cpu_reset(void);
