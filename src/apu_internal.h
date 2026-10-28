// Private definitions shared by the APU implementation files
// (apu.c, apu_spc700.c, apu_dsp.c). Not part of the public interface.
//
// The complete emulated APU state lives in one plain struct (apu_state) without
// pointers, so save/load state is a straight memcpy.
#pragma once
#include <stdint.h>
#include <stddef.h>

// ---------------------------------------------------------------------------
// S-DSP
// ---------------------------------------------------------------------------
enum { APU_ENV_RELEASE = 0, APU_ENV_ATTACK, APU_ENV_DECAY, APU_ENV_SUSTAIN };

typedef struct {
  int32_t buf[24];      // decoded BRR samples: 12-entry ring, stored twice to avoid wrap checks
  int32_t buf_pos;      // where the next 4 decoded samples go (0, 4 or 8)
  int32_t interp_pos;   // 3.12 fixed point position relative to buf_pos (0..0x7FFF)
  int32_t brr_addr;     // address of the current BRR block header
  int32_t brr_offset;   // offset of the next data byte pair in the block (1,3,5,7)
  int32_t kon_delay;    // KON start-up countdown (5..0)
  int32_t env_mode;     // APU_ENV_*
  int32_t env;          // current envelope (0..0x7FF)
  int32_t hidden_env;   // envelope value computed even when the rate counter did not fire
} ApuDspVoice;

typedef struct {
  uint8_t regs[128];
  ApuDspVoice voice[8];
  int32_t echo_hist[16][2];  // FIR history (8 entries, stored twice)
  int32_t echo_hist_pos;     // 0..7
  int32_t every_other_sample;
  int32_t kon;               // KON bits latched for processing
  int32_t new_kon;           // KON bits written by the SPC and not yet latched
  int32_t t_koff;            // latched KOFF
  int32_t counter;           // global rate counter (0..30719)
  int32_t noise;             // 15-bit LFSR
  int32_t echo_offset;       // byte offset within the echo buffer
  int32_t echo_length;       // echo buffer length in bytes
  int32_t t_esa;             // latched echo start address (page)
} ApuDspState;

// ---------------------------------------------------------------------------
// SPC700 + I/O
// ---------------------------------------------------------------------------
typedef struct {
  uint8_t enabled;
  uint8_t target;    // TnTARGET (0 = 256)
  uint8_t divider;   // internal stage-2 up-counter
  uint8_t counter;   // 4-bit TnOUT
} ApuTimer;

typedef struct {
  uint16_t pc;
  uint8_t a, x, y, sp, psw;
  uint8_t halted;    // SLEEP / STOP executed
} ApuSpcRegs;

typedef struct {
  ApuSpcRegs cpu;
  uint8_t control;      // $F1
  uint8_t dspaddr;      // $F2
  uint8_t test;         // $F0
  uint8_t pad0;
  uint8_t port_in[4];   // written by the main CPU ($2140-$2143), read by the SPC at $F4-$F7
  uint8_t port_out[4];  // written by the SPC at $F4-$F7, read by the main CPU
  ApuTimer timer[3];
  uint32_t timer_phase; // SPC cycles towards the next 64 kHz timer tick (0..15)
  uint32_t timer_div8;  // 64 kHz ticks towards the next 8 kHz tick (0..7)
  uint32_t dsp_phase;   // SPC cycles towards the next 32 kHz DSP sample (0..31)
  uint32_t pad1;
  uint64_t cycles;      // SPC cycles executed since apu_init (master-clock domain sync)
  ApuDspState dsp;
  uint8_t ram[0x10000];
} ApuState;

extern ApuState apu_state;

// Bus as seen by the SPC700 (handles $F0-$FF I/O and the IPL ROM overlay).
uint8_t apu_bus_read(uint16_t addr);
void    apu_bus_write(uint16_t addr, uint8_t val);

// SPC700 core (apu_spc700.c)
void apu_spc_reset(void);
// Executes one instruction and returns the number of SPC700 cycles it took.
// Returns 0 (and does nothing) when the CPU is halted by SLEEP/STOP.
int  apu_spc_step(void);

// S-DSP (apu_dsp.c)
void    apu_dsp_reset(void);
uint8_t apu_dsp_read(int addr);
void    apu_dsp_write(int addr, uint8_t val);
void    apu_dsp_run_sample(int16_t out[2]);  // generate one 32 kHz stereo frame

// Test hooks (used by tests/apu_test only).
// Flat-memory mode: the SPC700 sees a plain 64 KiB RAM (no I/O regs, no IPL ROM),
// and every bus access is reported to the callback (type 0 = read, 1 = write).
typedef void (*ApuBusTraceFn)(uint16_t addr, uint8_t val, int type);
void apu_test_set_flat_memory(int enable, ApuBusTraceFn trace);
