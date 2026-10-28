// SNES APU: glue between the SPC700 core (apu_spc700.c), the S-DSP (apu_dsp.c),
// the SPC700 I/O registers ($F0-$FF), the three timers, the IPL boot ROM, the
// main-CPU communication ports and the output sample FIFO.
//
// Integration notes:
//  - Timing: apu_run_to(master) runs whole SPC700 instructions until
//    spc_cycles >= master * 5632 / 118125 (exact integer conversion, no drift;
//    the SPC may overshoot by one instruction). Timers and the DSP are advanced
//    after each instruction (DSP: one stereo frame every 32 SPC cycles).
//    If the master clock goes backwards (host reset its counter) or jumps
//    forward by more than 10 s of SPC time, the APU resynchronises instead of
//    stalling / catching up.
//  - apu_reset() keeps ARAM and the clock sync point (like the console's reset
//    line); apu_init() clears everything.
//  - Save states contain all emulation state (CPU, ARAM, I/O, timers, DSP, clock
//    sync point) but not the host-side output FIFO.
//  - Not thread-safe: if an audio callback thread drains apu_read_samples(),
//    guard it and apu_run_to() with the same lock.
#include <string.h>
#include "apu.h"
#include "apu_internal.h"

ApuState apu_state;
#define S apu_state

// Standard 64-byte IPL boot ROM, mapped at $FFC0-$FFFF while CONTROL bit 7 is set.
static const uint8_t ipl_rom[64] = {
  0xCD, 0xEF, 0xBD, 0xE8, 0x00, 0xC6, 0x1D, 0xD0, 0xFC, 0x8F, 0xAA, 0xF4, 0x8F, 0xBB, 0xF5, 0x78,
  0xCC, 0xF4, 0xD0, 0xFB, 0x2F, 0x19, 0xEB, 0xF4, 0xD0, 0xFC, 0x7E, 0xF4, 0xD0, 0x0B, 0xE4, 0xF5,
  0xCB, 0xF4, 0xD7, 0x00, 0xFC, 0xD0, 0xF3, 0xAB, 0x01, 0x10, 0xEF, 0x7E, 0xF4, 0x10, 0xEB, 0xBA,
  0xF6, 0xDA, 0x00, 0xBA, 0xF4, 0xC4, 0xF4, 0xDD, 0x5D, 0xD0, 0xDB, 0x1F, 0x00, 0x00, 0xC0, 0xFF,
};

// ---------------------------------------------------------------------------
// Output FIFO (host-side queue; not part of the saved emulation state)
// ---------------------------------------------------------------------------
#define FIFO_FRAMES 16384
static int16_t fifo[FIFO_FRAMES * 2];
static int fifo_read_pos;
static int fifo_count;

static void fifo_push(const int16_t s[2]) {
  if (fifo_count == FIFO_FRAMES) {          // overflow: drop the oldest frame
    fifo_read_pos = (fifo_read_pos + 1) & (FIFO_FRAMES - 1);
    fifo_count--;
  }
  int w = (fifo_read_pos + fifo_count) & (FIFO_FRAMES - 1);
  fifo[w * 2 + 0] = s[0];
  fifo[w * 2 + 1] = s[1];
  fifo_count++;
}

int apu_read_samples(int16_t *out, int max_frames) {
  int n = max_frames < fifo_count ? max_frames : fifo_count;
  if (n <= 0)
    return 0;
  int first = FIFO_FRAMES - fifo_read_pos;
  if (first > n) first = n;
  memcpy(out, &fifo[fifo_read_pos * 2], (size_t)first * 2 * sizeof(int16_t));
  if (n > first)
    memcpy(out + first * 2, &fifo[0], (size_t)(n - first) * 2 * sizeof(int16_t));
  fifo_read_pos = (fifo_read_pos + n) & (FIFO_FRAMES - 1);
  fifo_count -= n;
  return n;
}

int apu_samples_available(void) {
  return fifo_count;
}

// ---------------------------------------------------------------------------
// Test hooks
// ---------------------------------------------------------------------------
static int flat_memory;
static ApuBusTraceFn bus_trace;

void apu_test_set_flat_memory(int enable, ApuBusTraceFn trace) {
  flat_memory = enable;
  bus_trace = trace;
}

// ---------------------------------------------------------------------------
// SPC700 bus
// ---------------------------------------------------------------------------
static uint8_t io_read(uint16_t addr) {
  switch (addr) {
  case 0xF2: return S.dspaddr;
  case 0xF3: return apu_dsp_read(S.dspaddr & 0x7F);
  case 0xF4: case 0xF5: case 0xF6: case 0xF7: return S.port_in[addr - 0xF4];
  case 0xF8: case 0xF9: return S.ram[addr];          // AUXIO: plain RAM-like registers
  case 0xFD: case 0xFE: case 0xFF: {                 // TnOUT: 4-bit counter, cleared on read
    ApuTimer *t = &S.timer[addr - 0xFD];
    uint8_t v = t->counter;
    t->counter = 0;
    return v;
  }
  default: return 0;                                 // TEST, CONTROL, TnTARGET are write-only
  }
}

static void io_write(uint16_t addr, uint8_t val) {
  switch (addr) {
  case 0xF0: S.test = val; break;
  case 0xF1:
    if (val & 0x10) S.port_in[0] = S.port_in[1] = 0;
    if (val & 0x20) S.port_in[2] = S.port_in[3] = 0;
    for (int i = 0; i < 3; i++) {
      int en = (val >> i) & 1;
      if (en && !S.timer[i].enabled) {               // 0 -> 1 resets the timer
        S.timer[i].divider = 0;
        S.timer[i].counter = 0;
      }
      S.timer[i].enabled = (uint8_t)en;
    }
    S.control = val;
    break;
  case 0xF2: S.dspaddr = val; break;
  case 0xF3:
    if (S.dspaddr < 0x80)                            // $80-$FF are read-only mirrors
      apu_dsp_write(S.dspaddr, val);
    break;
  case 0xF4: case 0xF5: case 0xF6: case 0xF7: S.port_out[addr - 0xF4] = val; break;
  case 0xFA: case 0xFB: case 0xFC: S.timer[addr - 0xFA].target = val; break;
  default: break;                                    // $F8/$F9 RAM-like, $FD-$FF read-only
  }
}

uint8_t apu_bus_read(uint16_t addr) {
  if (flat_memory) {
    uint8_t v = S.ram[addr];
    if (bus_trace) bus_trace(addr, v, 0);
    return v;
  }
  if ((addr & 0xFFF0) == 0x00F0)
    return io_read(addr);
  if (addr >= 0xFFC0 && (S.control & 0x80))
    return ipl_rom[addr - 0xFFC0];
  return S.ram[addr];
}

void apu_bus_write(uint16_t addr, uint8_t val) {
  if (flat_memory) {
    S.ram[addr] = val;
    if (bus_trace) bus_trace(addr, val, 1);
    return;
  }
  S.ram[addr] = val;   // RAM underneath I/O and the IPL ROM is always written
  if ((addr & 0xFFF0) == 0x00F0)
    io_write(addr, val);
}

// ---------------------------------------------------------------------------
// Timing
// ---------------------------------------------------------------------------
static inline void timer_tick(ApuTimer *t) {
  if (!t->enabled)
    return;
  t->divider++;                      // 8-bit wrap; target 0 means 256
  if (t->divider == t->target) {
    t->divider = 0;
    t->counter = (t->counter + 1) & 0x0F;
  }
}

static void advance(int cycles) {
  S.cycles += (uint64_t)cycles;
  // Timers: T2 ticks at 64 kHz (every 16 SPC cycles), T0/T1 at 8 kHz.
  S.timer_phase += (uint32_t)cycles;
  while (S.timer_phase >= 16) {
    S.timer_phase -= 16;
    timer_tick(&S.timer[2]);
    if (++S.timer_div8 == 8) {
      S.timer_div8 = 0;
      timer_tick(&S.timer[0]);
      timer_tick(&S.timer[1]);
    }
  }
  // DSP: one stereo sample every 32 SPC cycles.
  S.dsp_phase += (uint32_t)cycles;
  while (S.dsp_phase >= 32) {
    S.dsp_phase -= 32;
    int16_t s[2];
    apu_dsp_run_sample(s);
    fifo_push(s);
  }
}

static inline uint64_t master_to_spc(uint64_t master) {
  // spc = master * 5632 / 118125, computed without overflow and without drift.
  return (master / 118125u) * 5632u + ((master % 118125u) * 5632u) / 118125u;
}

void apu_run_to(uint64_t master_clock) {
  uint64_t target = master_to_spc(master_clock);
  // A clock that went backwards (host reset its master clock) or a gap of more
  // than 10 s: resynchronise instead of stalling / running for ages.
  if (target + 64 < S.cycles || (target > S.cycles && target - S.cycles > 10240000u))
    S.cycles = target;
  while (S.cycles < target) {
    int c = apu_spc_step();
    if (c == 0) {                         // halted (SLEEP/STOP): time still passes
      uint64_t gap = target - S.cycles;
      c = gap > 256 ? 256 : (int)gap;
    }
    advance(c);
  }
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------
void apu_init(void) {
  memset(&S, 0, sizeof(S));
  fifo_read_pos = 0;
  fifo_count = 0;
  apu_reset();
}

void apu_reset(void) {
  // RAM and the master-clock sync point survive a reset, everything else is reinitialised.
  S.test = 0x0A;
  S.control = 0xB0;          // IPL ROM on, input ports cleared, timers off
  S.dspaddr = 0;
  memset(S.port_in, 0, sizeof(S.port_in));
  memset(S.port_out, 0, sizeof(S.port_out));
  memset(S.timer, 0, sizeof(S.timer));
  S.timer_phase = 0;
  S.timer_div8 = 0;
  S.dsp_phase = 0;
  apu_dsp_reset();
  apu_spc_reset();
}

uint8_t apu_read_port(int port) {
  return S.port_out[port & 3];
}

void apu_write_port(int port, uint8_t val) {
  S.port_in[port & 3] = val;
}

uint8_t *apu_ram(void) {
  return S.ram;
}

#define STATE_MAGIC 0x31555041u  // "APU1"

size_t apu_save_state(uint8_t *buf) {
  const uint32_t hdr[2] = { STATE_MAGIC, (uint32_t)sizeof(ApuState) };
  size_t size = sizeof(hdr) + sizeof(ApuState);
  if (buf) {
    memcpy(buf, hdr, sizeof(hdr));
    memcpy(buf + sizeof(hdr), &S, sizeof(ApuState));
  }
  return size;
}

void apu_load_state(const uint8_t *buf) {
  uint32_t hdr[2];
  memcpy(hdr, buf, sizeof(hdr));
  if (hdr[0] != STATE_MAGIC || hdr[1] != (uint32_t)sizeof(ApuState))
    return;  // incompatible state blob: ignore
  memcpy(&S, buf + sizeof(hdr), sizeof(ApuState));
}
