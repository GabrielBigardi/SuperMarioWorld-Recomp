// Runs the SingleStepTests SPC700 suite (converted by convert_spc700_tests.py)
// against the APU's SPC700 core in flat-memory mode.
//
// Checked per test: final A/X/Y/SP/PSW/PC, final RAM contents, no stray writes,
// and the cycle count (except SLEEP/STOP whose reference trace is a truncated
// snapshot of an endless halt). As an extra (informational) metric the sequence
// of data-carrying bus accesses (reads with a value, writes) is compared too.
//
// usage: spc700_test <spc700_tests.bin> [-v]
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "../../src/apu.h"
#include "../../src/apu_internal.h"

typedef struct { uint16_t addr; uint8_t val; uint8_t type; } Access;

#define MAX_ACC 64
static Access log_acc[MAX_ACC];
static int log_n;
static int log_overflow;

static void trace(uint16_t addr, uint8_t val, int type) {
  if (log_n < MAX_ACC) {
    log_acc[log_n].addr = addr;
    log_acc[log_n].val = val;
    log_acc[log_n].type = (uint8_t)type;
    log_n++;
  } else {
    log_overflow = 1;
  }
}

static const uint8_t *p, *end;
static uint8_t u8(void) { return *p++; }
static uint16_t u16(void) { uint16_t v = (uint16_t)(p[0] | (p[1] << 8)); p += 2; return v; }

typedef struct {
  uint16_t pc; uint8_t a, x, y, sp, psw;
  int n; uint16_t addr[64]; uint8_t val[64];
} TState;

static void read_state(TState *s) {
  s->pc = u16(); s->a = u8(); s->x = u8(); s->y = u8(); s->sp = u8(); s->psw = u8();
  s->n = u16();
  if (s->n > 64) { fprintf(stderr, "too many ram entries\n"); exit(2); }
  for (int i = 0; i < s->n; i++) { s->addr[i] = u16(); s->val[i] = u8(); }
}

int main(int argc, char **argv) {
  if (argc < 2) { fprintf(stderr, "usage: %s spc700_tests.bin [-v]\n", argv[0]); return 2; }
  int verbose = argc > 2 && !strcmp(argv[2], "-v");
  FILE *f = fopen(argv[1], "rb");
  if (!f) { perror(argv[1]); return 2; }
  fseek(f, 0, SEEK_END);
  long size = ftell(f);
  fseek(f, 0, SEEK_SET);
  uint8_t *data = malloc((size_t)size);
  if (fread(data, 1, (size_t)size, f) != (size_t)size) { fprintf(stderr, "read error\n"); return 2; }
  fclose(f);
  if (memcmp(data, "SPCT", 4)) { fprintf(stderr, "bad file\n"); return 2; }
  p = data + 4;
  end = data + size;
  uint32_t count = (uint32_t)(p[0] | (p[1] << 8) | (p[2] << 16) | ((uint32_t)p[3] << 24));
  p += 4;

  apu_init();
  apu_test_set_flat_memory(1, trace);
  uint8_t *ram = apu_ram();

  static int fail_state[256], fail_cycles[256], fail_bus[256], total[256];
  int shown = 0;
  long pass_all = 0, pass_state = 0, pass_cycles = 0, pass_bus = 0;

  for (uint32_t t = 0; t < count; t++) {
    TState in, out;
    int op = u8();
    read_state(&in);
    read_state(&out);
    int ncyc = u8();
    int nacc = u16();
    Access exp[MAX_ACC];
    for (int i = 0; i < nacc; i++) {
      Access a; a.addr = u16(); a.val = u8(); a.type = u8();
      if (i < MAX_ACC) exp[i] = a;
    }

    for (int i = 0; i < in.n; i++) ram[in.addr[i]] = in.val[i];
    apu_state.cpu.pc = in.pc; apu_state.cpu.a = in.a; apu_state.cpu.x = in.x;
    apu_state.cpu.y = in.y; apu_state.cpu.sp = in.sp; apu_state.cpu.psw = in.psw;
    apu_state.cpu.halted = 0;
    log_n = 0; log_overflow = 0;

    int cyc = apu_spc_step();

    // --- compare state ---
    int ok_state = apu_state.cpu.pc == out.pc && apu_state.cpu.a == out.a &&
                   apu_state.cpu.x == out.x && apu_state.cpu.y == out.y &&
                   apu_state.cpu.sp == out.sp && apu_state.cpu.psw == out.psw;
    for (int i = 0; i < out.n; i++)
      if (ram[out.addr[i]] != out.val[i]) ok_state = 0;
    // stray writes: every address we wrote must be listed in the final RAM
    for (int i = 0; i < log_n; i++) {
      if (log_acc[i].type != 1) continue;
      int listed = 0;
      for (int j = 0; j < out.n; j++) if (out.addr[j] == log_acc[i].addr) listed = 1;
      if (!listed) ok_state = 0;
    }
    int halt_op = (op == 0xEF || op == 0xFF);
    int ok_cycles = halt_op || cyc == ncyc;
    int ok_bus = !log_overflow && log_n == nacc;
    for (int i = 0; ok_bus && i < nacc && i < MAX_ACC; i++)
      if (log_acc[i].addr != exp[i].addr || log_acc[i].val != exp[i].val || log_acc[i].type != exp[i].type)
        ok_bus = 0;

    total[op]++;
    if (!ok_state) fail_state[op]++;
    if (!ok_cycles) fail_cycles[op]++;
    if (!ok_bus) fail_bus[op]++;
    pass_state += ok_state;
    pass_cycles += ok_cycles;
    pass_bus += ok_bus;
    pass_all += ok_state && ok_cycles;

    if ((!ok_state || !ok_cycles || (verbose && !ok_bus)) && shown < 20) {
      shown++;
      printf("FAIL test %u op %02X%s%s%s\n", t, op, ok_state ? "" : " [state]",
             ok_cycles ? "" : " [cycles]", ok_bus ? "" : " [bus]");
      printf("  in : pc=%04X a=%02X x=%02X y=%02X sp=%02X psw=%02X\n", in.pc, in.a, in.x, in.y, in.sp, in.psw);
      printf("  exp: pc=%04X a=%02X x=%02X y=%02X sp=%02X psw=%02X cyc=%d\n", out.pc, out.a, out.x, out.y, out.sp, out.psw, ncyc);
      printf("  got: pc=%04X a=%02X x=%02X y=%02X sp=%02X psw=%02X cyc=%d\n", apu_state.cpu.pc, apu_state.cpu.a,
             apu_state.cpu.x, apu_state.cpu.y, apu_state.cpu.sp, apu_state.cpu.psw, cyc);
      for (int i = 0; i < out.n; i++)
        if (ram[out.addr[i]] != out.val[i])
          printf("  ram[%04X] exp %02X got %02X\n", out.addr[i], out.val[i], ram[out.addr[i]]);
      if (!ok_bus || !ok_state) {
        printf("  exp bus:");
        for (int i = 0; i < nacc && i < MAX_ACC; i++) printf(" %c%04X=%02X", exp[i].type ? 'W' : 'R', exp[i].addr, exp[i].val);
        printf("\n  got bus:");
        for (int i = 0; i < log_n; i++) printf(" %c%04X=%02X", log_acc[i].type ? 'W' : 'R', log_acc[i].addr, log_acc[i].val);
        printf("\n");
      }
    }

    // restore RAM to zero for the next test
    for (int i = 0; i < in.n; i++) ram[in.addr[i]] = 0;
    for (int i = 0; i < out.n; i++) ram[out.addr[i]] = 0;
    for (int i = 0; i < log_n; i++) ram[log_acc[i].addr] = 0;
  }

  int ops_fail = 0;
  for (int op = 0; op < 256; op++) {
    if (fail_state[op] || fail_cycles[op]) {
      ops_fail++;
      printf("opcode %02X: %d/%d state failures, %d cycle failures\n", op, fail_state[op], total[op], fail_cycles[op]);
    }
  }
  int ops_bus_fail = 0;
  for (int op = 0; op < 256; op++) if (fail_bus[op]) ops_bus_fail++;
  printf("\nSPC700 SingleStepTests: %u tests, 256 opcodes\n", count);
  printf("  registers+RAM : %ld/%u passed (%.3f%%)\n", pass_state, count, 100.0 * pass_state / count);
  printf("  cycle counts  : %ld/%u passed (%.3f%%)\n", pass_cycles, count, 100.0 * pass_cycles / count);
  printf("  both          : %ld/%u passed (%.3f%%), %d opcodes with failures\n", pass_all, count, 100.0 * pass_all / count, ops_fail);
  printf("  [info] data bus access sequence identical: %ld/%u (%.3f%%), %d opcodes differ\n",
         pass_bus, count, 100.0 * pass_bus / count, ops_bus_fail);
  if (verbose)
    for (int op = 0; op < 256; op++) if (fail_bus[op]) printf("  bus diff op %02X: %d\n", op, fail_bus[op]);
  free(data);
  return pass_all == (long)count ? 0 : 1;
}
