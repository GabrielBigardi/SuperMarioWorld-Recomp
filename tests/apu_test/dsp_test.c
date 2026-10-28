// End-to-end APU test driven only through the public main-CPU interface
// (apu_write_port / apu_read_port / apu_run_to), the way the 65816 code does it:
//
//  1. Boots the APU, waits for the IPL ROM's $AA/$BB signature and uploads an
//     SPC700 program + sample directory + BRR sample through the IPL protocol
//     ($CC handshake, per-byte index echo, multiple blocks, final jump).
//  2. The SPC700 program configures the S-DSP (voice 0: 1 kHz looped BRR sine,
//     ADSR with decaying sustain; echo 64 ms, feedback, FIR), keys the voice on,
//     waits 0.6 s using timer 0, reports ENDX/ENVX through the ports, keys off.
//  3. Captures the audio, writes WAV files (with and without echo) plus a marker
//     file for analyze_dsp.py, and checks: exact sample-count accounting, timer
//     timing, ENDX/ENVX readback, save/load-state determinism, performance.
//
// usage: dsp_test <out_dir>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include "../../src/apu.h"

#define MASTER_HZ 21477272.0
#define FRAME_MCLK 357368u        // one NTSC frame (1364 * 262 master clocks)

static int failures;
#define CHECK(cond, ...) do { if (cond) { printf("  ok   : "); } else { printf("  FAIL : "); failures++; } \
                              printf(__VA_ARGS__); printf("\n"); } while (0)

// ---------------------------------------------------------------------------
// Main-CPU side helpers (simulated 65816 timeline)
// ---------------------------------------------------------------------------
static uint64_t mclk = 1000;   // master clock (arbitrary non-zero start)

static uint8_t port_rd(int p) { mclk += 8; apu_run_to(mclk); return apu_read_port(p); }
static void port_wr(int p, uint8_t v) { mclk += 8; apu_run_to(mclk); apu_write_port(p, v); }

static int wait_port0(uint8_t v) {
  uint64_t deadline = mclk + (uint64_t)MASTER_HZ;   // 1 s timeout
  while (port_rd(0) != v) {
    mclk += 40;
    if (mclk > deadline) return 0;
  }
  return 1;
}

typedef struct { uint16_t addr; const uint8_t *data; int len; } Block;

// Standard IPL upload protocol, as used by SMW and most SNES games.
static int ipl_upload(const Block *blocks, int nblocks, uint16_t entry) {
  uint64_t deadline = mclk + (uint64_t)MASTER_HZ;
  while (!(port_rd(0) == 0xAA && port_rd(1) == 0xBB)) {
    mclk += 40;
    if (mclk > deadline) { printf("  timeout waiting for $AA/$BB\n"); return 0; }
  }
  uint8_t kick = 0xCC;
  for (int b = 0; b < nblocks; b++) {
    port_wr(2, (uint8_t)blocks[b].addr);
    port_wr(3, (uint8_t)(blocks[b].addr >> 8));
    port_wr(1, 1);                      // non-zero: transfer follows
    port_wr(0, kick);
    if (!wait_port0(kick)) { printf("  timeout on block %d kick\n", b); return 0; }
    for (int i = 0; i < blocks[b].len; i++) {
      port_wr(1, blocks[b].data[i]);
      port_wr(0, (uint8_t)i);
      if (!wait_port0((uint8_t)i)) { printf("  timeout on block %d byte %d\n", b, i); return 0; }
    }
    kick = (uint8_t)(blocks[b].len + 1);  // last index + 2
    if (!kick) kick = 1;
  }
  port_wr(2, (uint8_t)entry);
  port_wr(3, (uint8_t)(entry >> 8));
  port_wr(1, 0);                        // zero: jump to entry
  port_wr(0, kick);
  if (!wait_port0(kick)) { printf("  timeout on final jump\n"); return 0; }
  return 1;
}

// ---------------------------------------------------------------------------
// SPC700 program builder
// ---------------------------------------------------------------------------
static uint8_t prog[1024];
static int plen;
static void e(int b) { prog[plen++] = (uint8_t)b; }
static void dsp(int reg, int val) { e(0x8F); e(reg); e(0xF2); e(0x8F); e(val); e(0xF3); } // MOV $F2,#reg / MOV $F3,#val

#define PROG_ORG 0x0400

typedef struct { int eon; int nvoices; int vol_l, vol_r; } ProgCfg;

static void build_program(ProgCfg c) {
  plen = 0;
  dsp(0x6C, 0x20);                      // FLG: no reset, unmuted, echo writes off while configuring
  dsp(0x5D, 0x02);                      // DIR = $0200
  // Move the echo buffer first, then let the 8-entry FIR history flush before
  // enabling FIR/EVOL/EFB (otherwise the stale data the echo unit was reading at
  // ESA=$00 - the IPL's pointers at $0000 - would be fed back, as on hardware).
  dsp(0x6D, 0x80);                      // ESA = $8000
  dsp(0x7D, 0x04);                      // EDL = 4 -> 8 KB = 64 ms
  e(0x8D); e(0x00); e(0xFE); e(0xFE);   // MOV Y,#0 / DBNZ Y,* (~1500 cycles = 48 samples)
  for (int v = 0; v < c.nvoices; v++) {
    int b = v << 4;
    dsp(b + 0, c.vol_l);                // VOL L
    dsp(b + 1, c.vol_r);                // VOL R
    dsp(b + 2, 0x00);                   // PITCH = $1000 (32 kHz per sample step)
    dsp(b + 3, 0x10 + v / 4);           // (voices 4-7 slightly higher pitch)
    dsp(b + 4, 0x00);                   // SRCN 0
    dsp(b + 5, 0x8F);                   // ADSR on, AR=15 (instant), DR=0
    dsp(b + 6, 0xF0);                   // SL=7, SR=$10 (exponential decay, 64 samples/step)
    dsp(b + 7, 0x00);
  }
  dsp(0x0C, 0x7F); dsp(0x1C, 0x7F);     // MVOL
  dsp(0x2C, 0x50); dsp(0x3C, 0x50);     // EVOL
  dsp(0x0D, 0x50);                      // EFB = 0.625
  dsp(0x0F, 0x7F);                     // FIR C0 = $7F, C1..C7 = 0
  for (int i = 1; i < 8; i++) dsp(0x0F + i * 0x10, 0x00);
  dsp(0x2D, 0x00);                      // PMON off
  dsp(0x3D, 0x00);                      // NON off
  dsp(0x4D, c.eon);                     // EON
  dsp(0x5C, 0x00);                      // KOFF = 0
  dsp(0x6C, 0x00);                      // FLG = 0: echo writes on
  dsp(0x4C, (1 << c.nvoices) - 1);      // KON
  e(0x8F); e(0x55); e(0xF4);            // MOV $F4,#$55  (marker: keyed on)
  e(0x8F); e(80); e(0xFA);              // MOV $FA,#80   T0 target: 8000/80 = 100 Hz
  e(0x8F); e(0x01); e(0xF1);            // MOV $F1,#$01  enable T0 (also unmaps IPL ROM)
  e(0x8D); e(60);                       // MOV Y,#60     wait 60 ticks = 0.6 s
  int loop = plen;
  e(0xE4); e(0xFD);                     // loop: MOV A,$FD
  e(0xF0); e((uint8_t)(loop - (plen + 1)));          // BEQ loop
  e(0xDC);                              // DEC Y
  e(0xD0); e((uint8_t)(loop - (plen + 1)));          // BNE loop
  e(0x8F); e(0x7C); e(0xF2); e(0xE4); e(0xF3); e(0xC4); e(0xF6);   // port2 = ENDX
  e(0x8F); e(0x08); e(0xF2); e(0xE4); e(0xF3); e(0xC4); e(0xF7);   // port3 = V0 ENVX
  dsp(0x5C, (1 << c.nvoices) - 1);      // KOFF
  e(0x8F); e(0x66); e(0xF4);            // MOV $F4,#$66  (marker: keyed off)
  e(0x2F); e(0xFE);                     // BRA *
}

// Sample directory and a 32-sample sine as 2 BRR blocks (filter 0, shift 11, looping)
static uint8_t dir_data[4] = { 0x00, 0x03, 0x00, 0x03 };   // start $0300, loop $0300
static uint8_t brr_data[18];
static void build_brr(void) {
  for (int blk = 0; blk < 2; blk++) {
    uint8_t *b = &brr_data[blk * 9];
    b[0] = (uint8_t)(0xB0 | (blk == 1 ? 0x03 : 0x00));     // shift 11, filter 0, [loop+end]
    for (int i = 0; i < 16; i++) {
      int k = blk * 16 + i;
      int n = (int)lround(7.0 * sin(2.0 * 3.14159265358979 * k / 32.0));
      uint8_t nib = (uint8_t)(n & 0x0F);
      if (i & 1) b[1 + i / 2] |= nib; else b[1 + i / 2] = (uint8_t)(nib << 4);
    }
  }
}

// ---------------------------------------------------------------------------
// Audio capture / WAV
// ---------------------------------------------------------------------------
#define MAX_FRAMES (32000 * 20)
static int16_t audio[MAX_FRAMES * 2];
static int nframes;

static void drain(void) {
  while (apu_samples_available() > 0 && nframes < MAX_FRAMES) {
    int n = apu_read_samples(&audio[nframes * 2], MAX_FRAMES - nframes);
    if (n <= 0) break;
    nframes += n;
  }
}

static void write_wav(const char *path, const int16_t *s, int frames) {
  FILE *f = fopen(path, "wb");
  if (!f) { perror(path); failures++; return; }
  uint32_t data_bytes = (uint32_t)frames * 4;
  uint8_t h[44];
  memcpy(h, "RIFF", 4);
  uint32_t v = 36 + data_bytes; memcpy(h + 4, &v, 4);
  memcpy(h + 8, "WAVEfmt ", 8);
  v = 16; memcpy(h + 16, &v, 4);
  uint16_t w = 1; memcpy(h + 20, &w, 2);        // PCM
  w = 2; memcpy(h + 22, &w, 2);                 // stereo
  v = 32000; memcpy(h + 24, &v, 4);
  v = 32000 * 4; memcpy(h + 28, &v, 4);
  w = 4; memcpy(h + 32, &w, 2);
  w = 16; memcpy(h + 34, &w, 2);
  memcpy(h + 36, "data", 4);
  memcpy(h + 40, &data_bytes, 4);
  fwrite(h, 1, 44, f);
  fwrite(s, 4, (size_t)frames, f);
  fclose(f);
}

static double now_sec(void) {
  struct timespec ts;
  timespec_get(&ts, TIME_UTC);
  return (double)ts.tv_sec + ts.tv_nsec * 1e-9;
}

typedef struct { int kon_sample, koff_sample; int endx, envx; uint64_t mclk_at_jump; } RunInfo;

// Boot + upload + run for `seconds` after the jump. Audio goes to audio[]/nframes.
static int run_scenario(ProgCfg cfg, double seconds, RunInfo *info, int do_state_test) {
  apu_init();
  mclk = 1000;
  nframes = 0;
  build_program(cfg);
  Block blocks[3] = {
    { 0x0200, dir_data, (int)sizeof(dir_data) },
    { 0x0300, brr_data, (int)sizeof(brr_data) },
    { PROG_ORG, prog, plen },
  };
  if (!ipl_upload(blocks, 3, PROG_ORG)) return 0;
  // verify the upload landed in ARAM
  int same = !memcmp(apu_ram() + PROG_ORG, prog, (size_t)plen) &&
             !memcmp(apu_ram() + 0x0300, brr_data, sizeof(brr_data)) &&
             !memcmp(apu_ram() + 0x0200, dir_data, sizeof(dir_data));
  if (!same) { printf("  uploaded data mismatch in ARAM\n"); return 0; }
  info->mclk_at_jump = mclk;
  info->kon_sample = info->koff_sample = -1;
  drain();
  uint64_t end = mclk + (uint64_t)(seconds * MASTER_HZ);
  int state_done = 0;
  while (mclk < end) {
    mclk += FRAME_MCLK / 8;   // poll ports 8x per frame
    apu_run_to(mclk);
    drain();
    uint8_t p0 = apu_read_port(0);
    if (p0 == 0x55 && info->kon_sample < 0) info->kon_sample = nframes;
    if (p0 == 0x66 && info->koff_sample < 0) {
      info->koff_sample = nframes;
      info->endx = apu_read_port(2);
      info->envx = apu_read_port(3);
    }
    if (do_state_test && !state_done && info->kon_sample >= 0 && nframes > info->kon_sample + 3000) {
      // Save state, run 1/3 s, reload, run the same 1/3 s again: audio must be identical.
      state_done = 1;
      size_t sz = apu_save_state(NULL);
      uint8_t *st = malloc(sz);
      apu_save_state(st);
      uint64_t mclk0 = mclk;
      int f0 = nframes;
      for (int i = 0; i < 20; i++) { mclk += FRAME_MCLK; apu_run_to(mclk); drain(); }
      int fa = nframes - f0;
      int16_t *a = malloc((size_t)fa * 4);
      memcpy(a, &audio[f0 * 2], (size_t)fa * 4);
      uint8_t ram_a[0x10000];
      memcpy(ram_a, apu_ram(), sizeof(ram_a));
      apu_load_state(st);
      mclk = mclk0;
      nframes = f0;
      for (int i = 0; i < 20; i++) { mclk += FRAME_MCLK; apu_run_to(mclk); drain(); }
      int fb = nframes - f0;
      int identical = fa == fb && !memcmp(a, &audio[f0 * 2], (size_t)fa * 4) &&
                      !memcmp(ram_a, apu_ram(), sizeof(ram_a));
      CHECK(identical, "save/load state: %d frames re-generated after reload are bit-identical (state %zu bytes)", fa, sz);
      free(a);
      free(st);
    }
  }
  drain();
  return 1;
}

int main(int argc, char **argv) {
  const char *out_dir = argc > 1 ? argv[1] : ".";
  char path[1024];
  build_brr();

  printf("[1] IPL upload + voice with ADSR and echo\n");
  RunInfo echo_info, noecho_info;
  ProgCfg echo_cfg = { 0x01, 1, 0x60, 0x40 };
  int ok = run_scenario(echo_cfg, 1.6, &echo_info, 1);
  CHECK(ok, "IPL boot protocol upload (3 blocks, %d program bytes) and jump", plen);
  if (!ok) return 1;
  CHECK(echo_info.kon_sample >= 0 && echo_info.koff_sample > echo_info.kon_sample,
        "program ran: KON marker at frame %d, KOFF marker at frame %d", echo_info.kon_sample, echo_info.koff_sample);
  double interval = (echo_info.koff_sample - echo_info.kon_sample) / 32000.0;
  CHECK(fabs(interval - 0.6) < 0.01, "timer 0 (8 kHz, target 80) x60 ticks = %.4f s (expected 0.600)", interval);
  CHECK(echo_info.endx & 1, "ENDX bit 0 set after the looped sample wrapped (ENDX=$%02X)", echo_info.endx);
  CHECK(echo_info.envx > 0 && echo_info.envx < 0x7F, "ENVX readback shows decayed sustain (ENVX=$%02X)", echo_info.envx);

  // exact clock accounting: frames generated == spc cycles / 32
  uint64_t spc = (mclk / 118125u) * 5632u + (mclk % 118125u) * 5632u / 118125u;
  long long expected = (long long)(spc / 32);
  CHECK(llabs((long long)nframes - expected) <= 1, "sample accounting: %d frames generated, %lld expected from master clock",
        nframes, expected);
  snprintf(path, sizeof(path), "%s/dsp_echo.wav", out_dir);
  write_wav(path, audio, nframes);
  int echo_frames = nframes;

  printf("[2] same program with echo disabled for the voice (EON=0)\n");
  ProgCfg noecho_cfg = { 0x00, 1, 0x60, 0x40 };
  ok = run_scenario(noecho_cfg, 1.6, &noecho_info, 0);
  CHECK(ok, "IPL upload (EON=0 variant)");
  snprintf(path, sizeof(path), "%s/dsp_noecho.wav", out_dir);
  write_wav(path, audio, nframes);

  snprintf(path, sizeof(path), "%s/dsp_markers.json", out_dir);
  FILE *f = fopen(path, "w");
  if (f) {
    fprintf(f, "{\"echo\": {\"kon\": %d, \"koff\": %d, \"frames\": %d},\n", echo_info.kon_sample, echo_info.koff_sample, echo_frames);
    fprintf(f, " \"noecho\": {\"kon\": %d, \"koff\": %d, \"frames\": %d}}\n", noecho_info.kon_sample, noecho_info.koff_sample, nframes);
    fclose(f);
  }

  printf("[3] performance: 8 voices + echo, 10 s of emulated time\n");
  {
    ProgCfg perf_cfg = { 0xFF, 8, 0x10, 0x10 };
    RunInfo pi;
    double t0 = now_sec();
    ok = run_scenario(perf_cfg, 10.0, &pi, 0);
    double t1 = now_sec();
    double emulated = (double)nframes / 32000.0;
    CHECK(ok && nframes > 32000 * 9, "ran %.2f s of audio in %.3f s wall time -> %.2f%% of one core",
          emulated, t1 - t0, 100.0 * (t1 - t0) / emulated);
  }

  printf("\n%s (%d failure%s)\n", failures ? "DSP TEST FAILED" : "DSP TEST PASSED", failures, failures == 1 ? "" : "s");
  return failures ? 1 : 0;
}
