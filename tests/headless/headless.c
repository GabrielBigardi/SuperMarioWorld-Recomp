// Headless runner: boots the recompiled game, runs N frames with optional
// scripted input, logs per-frame WRAM hashes (for recomp-vs-interpreter
// lockstep comparison) and coverage statistics.
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "../../src/snes.h"
#include "../../src/cpu.h"
#include "../../src/ppu.h"
#include "../../src/apu.h"

static uint64_t fnv(const uint8_t *p, size_t n) {
  uint64_t h = 1469598103934665603ull;
  for (size_t i = 0; i < n; i++) { h ^= p[i]; h *= 1099511628211ull; }
  return h;
}

// input script: lines "frame buttons_hex" (buttons held from that frame on)
typedef struct { int frame; unsigned buttons; } Cue;

int main(int argc, char **argv) {
  const char *rom_path = NULL, *hash_log = NULL, *script = NULL, *misses = NULL, *shot_dir = NULL, *wav = NULL;
  int frames = 600, interp = 0, shot_every = 0, warp = -1, state_at = -1;
  for (int i = 1; i < argc; i++) {
    if (!strcmp(argv[i], "--frames")) frames = atoi(argv[++i]);
    else if (!strcmp(argv[i], "--interp")) interp = 1;
    else if (!strcmp(argv[i], "--hash-log")) hash_log = argv[++i];
    else if (!strcmp(argv[i], "--script")) script = argv[++i];
    else if (!strcmp(argv[i], "--misses")) misses = argv[++i];
    else if (!strcmp(argv[i], "--wav")) wav = argv[++i];
    else if (!strcmp(argv[i], "--state-test")) state_at = atoi(argv[++i]);
    else if (!strcmp(argv[i], "--warp")) warp = (int)strtol(argv[++i], NULL, 16);
    else if (!strcmp(argv[i], "--shots")) { shot_dir = argv[++i]; shot_every = atoi(argv[++i]); }
    else rom_path = argv[i];
  }
  FILE *f = fopen(rom_path, "rb");
  if (!f) { fprintf(stderr, "cannot open rom\n"); return 1; }
  fseek(f, 0, SEEK_END); long sz = ftell(f); fseek(f, 0, SEEK_SET);
  uint8_t *data = malloc(sz); fread(data, 1, sz, f); fclose(f);
  if (!snes_load_rom(data, sz)) return 1;
  Cue cues[512]; int ncue = 0;
  if (script) {
    FILE *s = fopen(script, "r");
    while (s && ncue < 512 && fscanf(s, "%d %x", &cues[ncue].frame, &cues[ncue].buttons) == 2) ncue++;
    if (s) fclose(s);
  }
  apu_init();
  ppu_init();
  snes_set_force_interpreter(interp);
  snes_reset();
  FILE *hl = hash_log ? fopen(hash_log, "w") : NULL;
  FILE *wf = wav ? fopen(wav, "wb") : NULL;
  uint32_t wav_frames = 0;
  if (wf) { uint8_t hdr[44] = {0}; fwrite(hdr, 1, 44, wf); }
  extern uint8_t g_wram[0x20000];
  unsigned buttons = 0; int ci = 0;
  uint8_t *state = NULL; size_t state_n = 0; int replayed = 0;
  unsigned state_buttons = 0; int state_ci = 0;
  uint64_t hashes[2][4096];
  for (int fr = 0; fr < frames; fr++) {
    if (fr == state_at) {
      if (!replayed) {
        state_n = snes_save_state(NULL); state = malloc(state_n); snes_save_state(state);
        state_buttons = buttons; state_ci = ci;
      }
    }
    while (ci < ncue && cues[ci].frame <= fr) buttons = cues[ci++].buttons;
    snes_set_joypad(0, (uint16_t)buttons);
    // level warp: while the game is in "enter level" mode, override the translevel number
    if (warp >= 0 && fr >= 1800 && g_wram[0x100] >= 0x0F && g_wram[0x100] <= 0x11) memset(g_wram + 0xD000, warp, 0x800);  // $7ED000 OW level table
    snes_run_frame();
    { int16_t ab[4096 * 2]; int n = apu_read_samples(ab, 4096); if (wf) { fwrite(ab, 4, n, wf); wav_frames += n; } }
    if (state_at >= 0 && fr >= state_at && fr - state_at < 4096)
      hashes[replayed][fr - state_at] = fnv(g_wram, sizeof(g_wram)) ^ cpu.mclk;
    if (state_at >= 0 && fr == frames - 1 && !replayed) {
      if (!snes_load_state(state, state_n)) { printf("load_state failed\n"); return 1; }
      replayed = 1; buttons = state_buttons; ci = state_ci; fr = state_at - 1;
      continue;
    }
    if (hl) fprintf(hl, "%d %016llx %02x %llu\n", fr, (unsigned long long)fnv(g_wram, sizeof(g_wram)), g_wram[0x100], (unsigned long long)cpu.mclk);
    if (shot_dir && shot_every && (fr % shot_every) == 0) {
      char p[512]; snprintf(p, sizeof p, "%s/f%05d.raw", shot_dir, fr);
      FILE *o = fopen(p, "wb"); if (o) { fwrite(ppu_framebuffer(), 4, 256 * 224, o); fclose(o); }
    }
    if (fr % 60 == 0 || fr == frames - 1)
      printf("frame %4d mode=%02X level=%03X pc=%02X:%04X mclk=%llu\n", fr, g_wram[0x100], g_wram[0x13BF], cpu.pb, cpu.pc, (unsigned long long)cpu.mclk);
  }
  if (state_at >= 0) {
    int n = frames - state_at, bad = 0;
    for (int i = 0; i < n && i < 4096; i++) if (hashes[0][i] != hashes[1][i]) { if (!bad) printf("state replay diverges at frame %d\n", state_at + i); bad++; }
    printf("save-state replay: %d frames compared, %d mismatches (state %zu bytes)\n", n, bad, state_n);
  }
  if (hl) fclose(hl);
  if (wf) {
    uint32_t data = wav_frames * 4, riff = 36 + data, rate = 32000, br = rate * 4, fmt = 16;
    uint16_t pcm = 1, ch = 2, ba = 4, bits = 16;
    fseek(wf, 0, SEEK_SET);
    fwrite("RIFF", 1, 4, wf); fwrite(&riff, 4, 1, wf); fwrite("WAVEfmt ", 1, 8, wf); fwrite(&fmt, 4, 1, wf);
    fwrite(&pcm, 2, 1, wf); fwrite(&ch, 2, 1, wf); fwrite(&rate, 4, 1, wf); fwrite(&br, 4, 1, wf);
    fwrite(&ba, 2, 1, wf); fwrite(&bits, 2, 1, wf); fwrite("data", 1, 4, wf); fwrite(&data, 4, 1, wf);
    fclose(wf);
  }
  const SnesStats *st = snes_stats();
  printf("recomp dispatches %llu, interp blocks %llu (rom %llu)\n", (unsigned long long)st->recomp_dispatches,
         (unsigned long long)st->interp_blocks, (unsigned long long)st->interp_rom_blocks);
  if (misses) printf("misses written: %d\n", snes_write_misses(misses));
  return 0;
}
