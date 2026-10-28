// APU unit tests:
//  - SPC700 I/O registers, timers, IPL ROM mapping, ports and master-clock
//    accounting, exercised through the SPC700 bus with the CPU frozen;
//  - S-DSP driven directly through its register interface. Timings are checked
//    against the documented hardware tables (fullsnes / Anomie's S-DSP doc);
//    BRR decoding is checked against an independent implementation of the
//    documented filter formulas.
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "../../src/apu.h"
#include "../../src/apu_internal.h"

static int failures;
#define CHECK(cond, ...) do { if (cond) { printf("  ok   : "); } else { printf("  FAIL : "); failures++; } \
                              printf(__VA_ARGS__); printf("\n"); } while (0)

static void w(int r, int v) { apu_dsp_write(r, (uint8_t)v); }
static int16_t last[2];
static void run(int n) { while (n-- > 0) apu_dsp_run_sample(last); }
static int envx(int v) { return apu_dsp_read((v << 4) | 8); }
static uint8_t *ram;

// 32-sample looping sine at $0300 (dir entry 0), non-looping one-block sample at $0400 (entry 1)
static void setup(void) {
  memset(apu_state.ram, 0, 0x10000);
  ram = apu_state.ram;
  apu_dsp_reset();
  ram[0x200] = 0x00; ram[0x201] = 0x03; ram[0x202] = 0x00; ram[0x203] = 0x03;
  ram[0x204] = 0x00; ram[0x205] = 0x04; ram[0x206] = 0x00; ram[0x207] = 0x04;
  for (int blk = 0; blk < 2; blk++) {
    uint8_t *b = &ram[0x300 + blk * 9];
    b[0] = (uint8_t)(0xB0 | (blk ? 3 : 0));
    for (int i = 0; i < 16; i++) {
      int n = (int)lround(7.0 * sin(2 * 3.14159265358979 * (blk * 16 + i) / 32.0)) & 15;
      if (i & 1) b[1 + i / 2] |= (uint8_t)n; else b[1 + i / 2] = (uint8_t)(n << 4);
    }
  }
  for (int b = 0; b < 3; b++) {          // 3 blocks, last one: end without loop
    ram[0x400 + b * 9] = (uint8_t)(b == 2 ? 0xB1 : 0xB0);
    memset(&ram[0x401 + b * 9], 0x77, 8);
  }
  w(0x6C, 0x00);                         // FLG: run, unmuted, echo writes on
  w(0x5D, 0x02);                         // DIR
  w(0x0C, 0x7F); w(0x1C, 0x7F);          // MVOL
  w(0x00, 0x7F); w(0x01, 0x7F);          // V0 VOL
  w(0x02, 0x00); w(0x03, 0x10);          // V0 pitch $1000
  w(0x04, 0x00);
  w(0x6D, 0xF0); w(0x7D, 0x00);          // echo buffer at $F000, EDL 0
}

// Samples from KON until cond(env) holds (max n)
static int samples_until(int v, int target_envx, int above, int max) {
  for (int t = 0; t < max; t++) {
    run(1);
    int e = envx(v);
    if (above ? e >= target_envx : e <= target_envx) return t + 1;
  }
  return -1;
}

static const int rate_period[32] = {
  0, 2048, 1536, 1280, 1024, 768, 640, 512, 384, 320, 256, 192, 160, 128, 96, 80,
  64, 48, 40, 32, 24, 20, 16, 12, 10, 8, 6, 5, 4, 3, 2, 1,
};

// ---- independent BRR decoder (fullsnes formulas, 15-bit sample domain) ----
static int clamp16(int x) { return x < -32768 ? -32768 : x > 32767 ? 32767 : x; }
static void ref_brr_block(const uint8_t *blk, int *old, int *older, int out[16]) {
  int shift = blk[0] >> 4, filter = (blk[0] >> 2) & 3;
  for (int i = 0; i < 16; i++) {
    int nib = (blk[1 + i / 2] >> ((i & 1) ? 0 : 4)) & 15;
    if (nib >= 8) nib -= 16;
    int s = shift <= 12 ? (nib * (1 << shift)) >> 1 : (nib < 0 ? -2048 : 0);
    int o = *old, oo = *older;            // 15-bit values
    switch (filter) {
    case 1: s += o + ((-o) >> 4); break;
    case 2: s += 2 * o + ((-3 * o) >> 5) - oo + (oo >> 4); break;
    case 3: s += 2 * o + ((-13 * o) >> 6) - oo + ((3 * oo) >> 4); break;
    }
    s = clamp16(s);
    s = (int16_t)(s * 2) >> 1;            // keep 15 bits (wraps like hardware)
    *older = *old;
    *old = s;
    out[i] = s * 2;                       // stored value as the DSP keeps it
  }
}


// ---------------------------------------------------------------------------
// I/O + timers (CPU frozen with the halt flag, time advanced via apu_run_to)
// ---------------------------------------------------------------------------
static uint64_t io_spc;   // SPC cycle target
static void adv_spc(uint64_t n) {
  io_spc += n;
  uint64_t mc = (io_spc * 118125u + 5631u) / 5632u;   // smallest master clock reaching io_spc
  apu_run_to(mc);
}
static uint8_t br(uint16_t a) { return apu_bus_read(a); }
static void bw(uint16_t a, uint8_t v) { apu_bus_write(a, v); }

static void io_tests(void) {
  printf("[SPC700 I/O registers and timers]\n");
  apu_init();
  apu_state.cpu.halted = 1;   // freeze the CPU; time still advances
  io_spc = 0;

  CHECK(apu_state.cpu.pc == 0xFFC0 && br(0xFFC0) == 0xCD && br(0xFFFE) == 0xC0 && br(0xFFFF) == 0xFF,
        "reset: PC=$%04X from the IPL ROM vector, IPL mapped at $FFC0", apu_state.cpu.pc);
  bw(0xFFC0, 0x12);
  CHECK(br(0xFFC0) == 0xCD && apu_ram()[0xFFC0] == 0x12, "writes under the IPL ROM go to RAM");
  bw(0xF1, 0x00);
  CHECK(br(0xFFC0) == 0x12, "CONTROL bit 7 = 0 unmaps the IPL ROM");
  bw(0xF1, 0x80);
  CHECK(br(0xFFC0) == 0xCD, "CONTROL bit 7 = 1 maps it again");

  // ports
  for (int i = 0; i < 4; i++) apu_write_port(i, (uint8_t)(0x11 * (i + 1)));
  CHECK(br(0xF4) == 0x11 && br(0xF5) == 0x22 && br(0xF6) == 0x33 && br(0xF7) == 0x44, "SPC reads main-CPU port writes at $F4-$F7");
  bw(0xF4, 0xA5); bw(0xF7, 0x5A);
  CHECK(apu_read_port(0) == 0xA5 && apu_read_port(3) == 0x5A && br(0xF4) == 0x11,
        "SPC writes to $F4-$F7 go to the main CPU; reads still return the input side");
  bw(0xF1, 0x90);
  CHECK(br(0xF4) == 0 && br(0xF5) == 0 && br(0xF6) == 0x33 && br(0xF7) == 0x44, "CONTROL bit 4 clears input ports 0-1");
  bw(0xF1, 0xA0);
  CHECK(br(0xF6) == 0 && br(0xF7) == 0, "CONTROL bit 5 clears input ports 2-3");

  // AUXIO, write-only registers, DSP address/data
  bw(0xF8, 0x3C); bw(0xF9, 0xC3);
  CHECK(br(0xF8) == 0x3C && br(0xF9) == 0xC3, "$F8/$F9 behave like RAM");
  bw(0xFA, 0x10);
  CHECK(br(0xFA) == 0 && br(0xF1) == 0 && br(0xF0) == 0, "TnTARGET / CONTROL / TEST read back as 0");
  bw(0xF2, 0x0C); bw(0xF3, 0x55);
  CHECK(br(0xF2) == 0x0C && apu_dsp_read(0x0C) == 0x55 && br(0xF3) == 0x55, "DSPADDR/DSPDATA write + read");
  bw(0xF2, 0x8C);
  CHECK(br(0xF3) == 0x55, "DSPADDR $80-$FF reads mirror $00-$7F");
  bw(0xF3, 0x66);
  CHECK(apu_dsp_read(0x0C) == 0x55, "DSPADDR $80-$FF writes are ignored");

  // timers
  bw(0xFC, 0x00);               // T2 target 0 = 256
  bw(0xF1, 0x04);               // enable T2 only (resets it)
  adv_spc(16 * 256 * 5);
  int t2 = br(0xFF);
  CHECK(t2 == 5 && br(0xFF) == 0, "T2 (64 kHz, target 0=256): %d ticks after 5*4096 cycles, cleared on read", t2);
  bw(0xFA, 100);
  bw(0xF1, 0x05);               // T0 0->1: reset; T2 stays running
  adv_spc(128 * 100 * 3);
  int t0 = br(0xFD);
  CHECK(t0 == 3, "T0 (8 kHz, target 100): %d ticks after 38400 cycles", t0);
  bw(0xFC, 1);
  bw(0xF1, 0x01); bw(0xF1, 0x05);   // restart T2 (a divider already past the new target would wrap first)
  adv_spc(16 * 20);
  int w = br(0xFF);
  CHECK(w == 4, "TnOUT is a 4-bit counter: 20 ticks read as %d", w);
  bw(0xFB, 50);
  bw(0xF1, 0x02);
  adv_spc(128 * 30);
  bw(0xF1, 0x00); bw(0xF1, 0x02);   // re-enable: internal divider reset
  adv_spc(128 * 49);
  int a = br(0xFE);
  adv_spc(128 * 1);
  int b = br(0xFE);
  CHECK(a == 0 && b == 1, "re-enabling a timer resets its divider (%d then %d)", a, b);
  bw(0xF1, 0x00);
  adv_spc(128 * 1000);
  CHECK(br(0xFD) == 0 && br(0xFE) == 0 && br(0xFF) == 0, "disabled timers do not count");
  bw(0xFA, 4); bw(0xF1, 0x01);
  adv_spc(128 * 4 * 3);
  bw(0xFD, 0x00);              // writing TnOUT has no effect (MOV dp,A's dummy read is what clears)
  CHECK(br(0xFD) == 3, "write to TnOUT does not clear it by itself");

  // master clock accounting: many small steps == exact conversion, no drift
  apu_init();
  apu_state.cpu.halted = 1;
  uint64_t mc = 0;
  uint32_t r = 1;
  for (int i = 0; i < 200000; i++) {
    r = r * 1103515245u + 12345u;
    mc += (r >> 16) % 2000;
    apu_run_to(mc);
  }
  uint64_t exp = (mc / 118125u) * 5632u + (mc % 118125u) * 5632u / 118125u;
  CHECK(apu_state.cycles == exp, "200000 irregular apu_run_to steps: %llu SPC cycles == %llu exact",
        (unsigned long long)apu_state.cycles, (unsigned long long)exp);
  int frames = apu_samples_available();
  CHECK(frames == 16384, "FIFO keeps the newest 16384 frames on overflow (%d)", frames);
  int16_t buf[100 * 2];
  CHECK(apu_read_samples(buf, 100) == 100 && apu_samples_available() == 16284, "apu_read_samples pops frames");
  // 1 s of master clock = 32000 samples (+-1)
  apu_init();
  apu_state.cpu.halted = 1;
  int total = 0;
  for (int f = 0; f < 60; f++) {
    apu_run_to((uint64_t)((f + 1) * (21477272.0 / 60.0)));
    total += apu_read_samples(buf, 100) ;
    while (apu_samples_available()) total += apu_read_samples(buf, 100);
  }
  CHECK(total >= 31999 && total <= 32001, "1 s of master clock produced %d frames at 32 kHz", total);
}

int main(void) {
  io_tests();
  apu_init();

  printf("[ADSR attack times vs documented table]\n");
  {
    static const double doc_ms[16] = { 4100, 2600, 1500, 1000, 640, 380, 260, 160, 96, 64, 40, 24, 16, 10, 6, 0 };
    int ok = 1;
    for (int ar = 0; ar < 16; ar++) {
      setup();
      w(0x05, 0x80 | ar); w(0x06, 0xE0);  // SL=7, SR=0 -> holds at max
      w(0x4C, 1);
      // AR<15: the step that overflows $7FF switches to decay without being stored,
      // so the envelope tops out at $7E0 (ENVX $7E) when SR=0 (hardware behaviour).
      int t = samples_until(0, 0x7E, 1, 32000 * 5);
      double ms = t / 32.0;
      int good = ar == 15 ? (t > 0 && t <= 12) : fabs(ms - doc_ms[ar]) <= doc_ms[ar] * 0.06 + 0.5;
      if (!good) { ok = 0; printf("    AR=%X: %.1f ms (doc %.0f)\n", ar, ms, doc_ms[ar]); }
    }
    CHECK(ok, "AR 0..F attack times within 6%% of 4.1s/2.6s/.../6ms/0ms");
  }

  printf("[sustain (SR) exponential decay times vs documented table]\n");
  {
    static const struct { int sr; double ms; } tbl[] = {
      { 0x1F, 18 }, { 0x1E, 37 }, { 0x1C, 74 }, { 0x1A, 110 }, { 0x16, 290 }, { 0x14, 440 }, { 0x10, 1200 }, { 0x0C, 2900 },
    };
    int ok = 1;
    for (size_t i = 0; i < sizeof(tbl) / sizeof(tbl[0]); i++) {
      setup();
      w(0x05, 0x8F); w(0x06, 0xE0 | tbl[i].sr);   // instant attack, SL=7 -> straight into sustain
      w(0x4C, 1);
      int t0 = samples_until(0, 0x7F, 1, 100);
      int t = samples_until(0, 7, 0, 32000 * 10);   // documented times ~ decay to 1/16 (ENVX <= 7)
      double ms = (t + t0) / 32.0;
      if (!(t > 0 && fabs(ms - tbl[i].ms) <= tbl[i].ms * 0.1)) { ok = 0; printf("    SR=%02X: %.1f ms (doc %.0f)\n", tbl[i].sr, ms, tbl[i].ms); }
    }
    CHECK(ok, "SR exponential decay times (to ENVX<=7) within 10%% of the documented values");
  }

  printf("[decay to sustain level]\n");
  {
    setup();
    w(0x05, 0x8F | (7 << 4)); w(0x06, 0x40);  // DR=7 (fastest decay), SL=2 -> 3/8, SR=0 (hold)
    w(0x4C, 1);
    run(32000 / 10);
    int e = envx(0);
    CHECK(e >= 0x2E && e <= 0x30, "decay stops at sustain level SL=2: ENVX=$%02X (expected ~$2F = 3/8)", e);
  }

  printf("[release]\n");
  {
    setup();
    w(0x05, 0x8F); w(0x06, 0xE0);
    w(0x4C, 1);
    run(100);
    int before = envx(0);
    w(0x5C, 1);                                // KOFF
    int t = samples_until(0, 0, 0, 1000);
    CHECK(before == 0x7F && t >= 255 && t <= 260, "KOFF: ENVX $7F -> 0 in %d samples (8/sample -> 256, ~8 ms)", t);
  }

  printf("[GAIN modes]\n");
  {
    setup();
    w(0x05, 0x00); w(0x07, 0x40);              // direct gain $40
    w(0x4C, 1);
    run(12);
    CHECK(envx(0) == 0x40, "direct GAIN $40 -> ENVX $%02X", envx(0));

    w(0x07, 0x80 | 0x1F);                      // linear decrease, every sample
    int t = samples_until(0, 0, 0, 1000);
    CHECK(t >= 31 && t <= 33, "linear decrease from $400 at rate $1F: %d samples (expected 32 = $400/$20)", t);

    setup();
    w(0x05, 0x00); w(0x07, 0xC0 | 0x1F);       // linear increase, every sample
    w(0x4C, 1);
    run(7);
    t = samples_until(0, 0x7F, 1, 1000);
    CHECK(t >= 62 && t <= 66, "linear increase 0 -> max at rate $1F: %d samples (expected ~64)", t);

    setup();
    w(0x05, 0x00); w(0x07, 0xE0 | 0x1F);       // bent line increase
    w(0x4C, 1);
    run(7);
    int t1 = samples_until(0, 0x60, 1, 1000);
    int t2 = samples_until(0, 0x7F, 1, 1000);
    CHECK(t1 >= 46 && t1 <= 50 && t2 >= 60 && t2 <= 66,
          "bent line: to $600 in %d (exp ~48 @ +$20), then to max in %d more (exp ~63 @ +8)", t1, t2);

    setup();
    w(0x05, 0x00); w(0x07, 0x7F);              // direct max, then exponential decrease
    w(0x4C, 1);
    run(12);
    w(0x07, 0xA0 | 0x1F);
    t = samples_until(0, 0, 0, 5000);
    CHECK(t > 400 && t < 700, "exponential decrease at rate $1F reaches 0 in %d samples (~18 ms)", t);
  }

  printf("[KON delay, ENDX, end of non-looping sample]\n");
  {
    setup();
    w(0x05, 0x8F); w(0x06, 0xE0);
    w(0x04, 0x01);                             // SRCN 1: 3 blocks, end without loop
    w(0x7C, 0);
    w(0x4C, 1);
    int t = samples_until(0, 1, 1, 100);
    CHECK(t >= 5 && t <= 9, "KON -> envelope starts after %d samples (KON delay)", t);
    run(64);
    CHECK((apu_dsp_read(0x7C) & 1) && envx(0) == 0 && apu_dsp_read(0x09) == 0,
          "after end block: ENDX=$%02X, ENVX=$%02X, OUTX=$%02X (voice silenced)", apu_dsp_read(0x7C), envx(0), apu_dsp_read(0x09));
    w(0x7C, 0x55);
    CHECK(apu_dsp_read(0x7C) == 0, "writing ENDX clears it");
    w(0x4C, 1);
    run(1);
    run(1);
    CHECK((apu_dsp_read(0x7C) & 1) == 0, "KON clears the voice's ENDX bit");
  }

  printf("[looping sample, pitch]\n");
  {
    setup();
    w(0x05, 0x8F); w(0x06, 0xE0);
    w(0x4C, 1);
    run(200);
    // count zero crossings of OUTX over 3200 samples: 1000 Hz -> 200 crossings
    int prev = (int8_t)apu_dsp_read(0x09), zc = 0;
    for (int i = 0; i < 3200; i++) {
      run(1);
      int o = (int8_t)apu_dsp_read(0x09);
      if ((prev < 0) != (o < 0)) zc++;
      prev = o;
    }
    CHECK(zc >= 198 && zc <= 202, "pitch $1000 on a 32-sample loop: %d zero crossings / 0.1 s (1000 Hz)", zc);
    w(0x03, 0x08);                              // pitch $0800 -> 500 Hz
    run(100);
    zc = 0;
    for (int i = 0; i < 3200; i++) {
      run(1);
      int o = (int8_t)apu_dsp_read(0x09);
      if ((prev < 0) != (o < 0)) zc++;
      prev = o;
    }
    CHECK(zc >= 98 && zc <= 102, "pitch $0800: %d zero crossings / 0.1 s (500 Hz)", zc);
    w(0x03, 0x50);                              // only 14 bits are used: $5000 & $3FFF = $1000
    run(100);
    zc = 0;
    for (int i = 0; i < 3200; i++) {
      run(1);
      int o = (int8_t)apu_dsp_read(0x09);
      if ((prev < 0) != (o < 0)) zc++;
      prev = o;
    }
    CHECK(zc >= 198 && zc <= 202, "PITCHH bits 6-7 ignored: %d zero crossings", zc);
  }

  printf("[BRR decoding vs independent decoder]\n");
  {
    srand(1234);
    int ok = 1, blocks_checked = 0;
    for (int trial = 0; trial < 200 && ok; trial++) {
      setup();
      // 12 random blocks at $1000, looping to itself at the end
      for (int b = 0; b < 12; b++) {
        uint8_t *p = &ram[0x1000 + b * 9];
        p[0] = (uint8_t)(((rand() % 16) << 4) | ((rand() % 4) << 2) | (b == 11 ? 3 : 0));
        for (int k = 1; k < 9; k++) p[k] = (uint8_t)rand();
      }
      ram[0x208] = 0x00; ram[0x209] = 0x10; ram[0x20A] = 0x00; ram[0x20B] = 0x10;
      w(0x04, 2);
      w(0x05, 0x8F); w(0x06, 0xE0);
      w(0x02, 0x00); w(0x03, 0x10);
      w(0x4C, 1);
      // reference decode of the whole sample
      int old = 0, older = 0, refout[12 * 16];
      for (int b = 0; b < 12; b++) ref_brr_block(&ram[0x1000 + b * 9], &old, &older, &refout[b * 16]);
      // run and snapshot the voice ring buffer every 4 samples
      ApuDspVoice *v = &apu_state.dsp.voice[0];
      int decoded = 0;
      for (int t = 0; t < 400 && decoded < 12 * 16; t++) {
        int pos_before = v->buf_pos;
        run(1);
        if (v->buf_pos != pos_before) {     // 4 new samples written at pos_before
          for (int k = 0; k < 4 && decoded < 12 * 16; k++, decoded++)
            if (v->buf[pos_before + k] != refout[decoded]) {
              ok = 0;
              printf("    trial %d sample %d: got %d expected %d (header $%02X)\n", trial, decoded,
                     v->buf[pos_before + k], refout[decoded], ram[0x1000 + (decoded / 16) * 9]);
              break;
            }
        }
      }
      if (decoded < 12 * 16) { ok = 0; printf("    trial %d: only %d samples decoded\n", trial, decoded); }
      blocks_checked += 12;
    }
    CHECK(ok, "%d random BRR blocks (all shifts incl. invalid 13-15, filters 0-3) decode identically", blocks_checked);
  }

  printf("[noise]\n");
  {
    setup();
    w(0x05, 0x00); w(0x07, 0x7F);              // direct max envelope
    w(0x3D, 0x01);                             // NON voice 0
    w(0x6C, 0x1F);                             // noise clock: every sample
    w(0x4C, 1);
    run(20);
    // our output: OUTX = ((noise*2 as int16) * env >> 11) >> 8. Track the LFSR instead.
    int n = apu_state.dsp.noise, ok = 1;
    for (int i = 0; i < 2000; i++) {
      run(1);
      int fb = (n << 13) ^ (n << 14);
      n = (fb & 0x4000) ^ (n >> 1);
      if (apu_state.dsp.noise != n) { ok = 0; break; }
    }
    int start = n, period = 0;
    do { int fb = (n << 13) ^ (n << 14); n = (fb & 0x4000) ^ (n >> 1); period++; } while (n != start && period < 70000);
    CHECK(ok && period == 32767, "noise LFSR steps every sample at rate $1F, period %d", period);
    setup();
    w(0x6C, 0x10);                             // rate $10: every 64 samples
    run(1);
    int changes = 0, prevn = apu_state.dsp.noise;
    for (int i = 0; i < 6400; i++) { run(1); if (apu_state.dsp.noise != prevn) { changes++; prevn = apu_state.dsp.noise; } }
    CHECK(changes == 100, "noise rate $10 -> %d LFSR steps in 6400 samples (expected 100)", changes);
  }

  printf("[pitch modulation]\n");
  {
    setup();
    w(0x05, 0x8F); w(0x06, 0xE0);
    w(0x10, 0x7F); w(0x11, 0x7F); w(0x12, 0x00); w(0x13, 0x10); w(0x14, 0x00); w(0x15, 0x8F); w(0x16, 0xE0);
    w(0x03, 0x01);                             // voice 0: slow modulator (31 Hz)
    w(0x4C, 3);
    run(32000 / 4);
    int pos0 = apu_state.dsp.voice[1].interp_pos;
    (void)pos0;
    long steps_unmod = 0, steps_mod = 0;
    // without PMON voice 1 advances exactly $1000 per sample
    for (int i = 0; i < 1000; i++) { int a = apu_state.dsp.voice[1].interp_pos & 0x3FFF; run(1); int b = apu_state.dsp.voice[1].interp_pos; steps_unmod += b - a; }
    w(0x2D, 0x02);                             // PMON voice 1 by voice 0
    int mn = 1 << 30, mx = 0;
    for (int i = 0; i < 2000; i++) {
      int a = apu_state.dsp.voice[1].interp_pos & 0x3FFF; run(1);
      int d = apu_state.dsp.voice[1].interp_pos - a;
      steps_mod += d; if (d < mn) mn = d; if (d > mx) mx = d;
    }
    CHECK(steps_unmod == 1000L * 0x1000 && mn < 0x1000 - 0x400 && mx > 0x1000 + 0x400,
          "PMON: voice 1 step $1000 unmodulated, $%X..$%X modulated by voice 0", mn, mx);
  }

  printf("[echo]\n");
  {
    setup();
    w(0x05, 0x8F); w(0x06, 0xE0);
    w(0x6D, 0x20); w(0x7D, 0x00);             // ESA $2000, EDL 0 -> 4-byte buffer
    w(0x4D, 0x01); w(0x0D, 0x40); w(0x0F, 0x7F);
    memset(&ram[0x2000], 0xAA, 0x100);
    w(0x4C, 1);
    run(3000);
    int untouched = 1;
    for (int i = 4; i < 0x100; i++) if (ram[0x2000 + i] != 0xAA) untouched = 0;
    CHECK(untouched && memcmp(&ram[0x2000], "\xAA\xAA\xAA\xAA", 4) != 0, "EDL=0: only the 4 bytes at ESA are used");

    w(0x6C, 0x20);                             // echo write disable
    uint8_t snap[4];
    memcpy(snap, &ram[0x2000], 4);
    run(500);
    CHECK(!memcmp(snap, &ram[0x2000], 4), "FLG bit 5 disables echo buffer writes");

    // FIR overflow: 7 taps at $7F on a full-scale buffer wrap to 16 bits before tap 7 + clamp
    setup();
    w(0x6D, 0x20); w(0x7D, 0x00);
    for (int i = 0; i < 8; i++) w(0x0F + i * 0x10, 0x7F);
    w(0x2C, 0x7F); w(0x3C, 0x7F); w(0x0C, 0x00); w(0x1C, 0x00); w(0x6C, 0x20);
    ram[0x2000] = 0xFF; ram[0x2001] = 0x7F; ram[0x2002] = 0x00; ram[0x2003] = 0x80;   // L=32767, R=-32768
    run(16);
    // L: each tap (16383*127)>>6 = 32510; 7 taps = 227570 -> int16 = 30962... + 32510 -> clamp 32767 &~1 = 32766
    // output = (32766*127)>>7 = 32510.  R: (-16384*127)>>6 = -32512; *7 = -227584 -> int16 = -30976 (+65536k)
    int sumL = (int16_t)(7 * ((16383 * 127) >> 6)) + (int16_t)((16383 * 127) >> 6);
    if (sumL > 32767) sumL = 32767;
    sumL &= ~1;
    int sumR = (int16_t)(7 * ((-16384 * 127) >> 6)) + (int16_t)((-16384 * 127) >> 6);
    if (sumR < -32768) sumR = -32768;
    sumR &= ~1;
    int expL = (int16_t)((sumL * 127) >> 7), expR = (int16_t)((sumR * 127) >> 7);
    CHECK(last[0] == expL && last[1] == expR, "FIR overflow/clamp: output (%d,%d), expected (%d,%d)", last[0], last[1], expL, expR);
  }

  printf("[FLG mute / soft reset, negative volume]\n");
  {
    setup();
    w(0x05, 0x8F); w(0x06, 0xE0);
    w(0x4C, 1);
    run(500);
    int16_t a = last[0];
    w(0x6C, 0x40);
    run(1);
    CHECK(a != 0 && last[0] == 0 && last[1] == 0 && envx(0) != 0, "FLG mute silences output, voice keeps running");
    w(0x6C, 0x00);
    w(0x0C, 0x80);                             // MVOLL = -128
    run(1);
    int16_t neg = last[0];
    w(0x0C, 0x7F);
    run(32);                                   // same phase one period later
    CHECK((neg < 0) != (last[0] < 0) || (neg == 0 && last[0] == 0), "negative MVOL inverts output (%d vs %d)", neg, last[0]);
    w(0x6C, 0x80);
    run(2);
    CHECK(envx(0) == 0, "FLG soft reset silences voices immediately");
  }

  printf("\n%s (%d failure%s)\n", failures ? "APU UNIT TESTS FAILED" : "APU UNIT TESTS PASSED", failures, failures == 1 ? "" : "s");
  return failures ? 1 : 0;
}
