// Runs SingleStepTests 65816 (native mode subset) against the generated interpreter,
// which shares its instruction-semantics generator with the recompiled code.
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "../../src/recomp_rt.h"

uint8_t g_flat_mem[0x1000000];
Cpu cpu;
uint64_t g_next_event;
uint8_t g_mdr, g_wram[0x20000], *g_rom;
uint32_t g_rom_mask;
uint8_t bus_read_slow(uint32_t a) { (void)a; return 0; }
void bus_write_slow(uint32_t a, uint8_t v) { (void)a; (void)v; }
void cpu_interrupt(uint16_t native_vec, uint16_t emu_vec, bool brk) {
  if (!cpu.fe) { push8(cpu.pb); push16(cpu.pc); push8(cpu_get_p()); }
  else { push16(cpu.pc); push8((uint8_t)(brk ? cpu_get_p() | 0x10 : cpu_get_p() & ~0x10)); }
  cpu.fi = 1; cpu.fd = 0; cpu.pb = 0;
  cpu.pc = rd16(cpu.fe ? emu_vec : native_vec);
}

typedef struct { int pc, s, p, a, x, y, dbr, d, pbr, e, n; int ram[4096][2]; } St;
static int read_st(FILE *f, St *s) {
  if (fscanf(f, "%d %d %d %d %d %d %d %d %d %d %d", &s->pc, &s->s, &s->p, &s->a, &s->x, &s->y, &s->dbr, &s->d, &s->pbr, &s->e, &s->n) != 11) return 0;
  for (int i = 0; i < s->n; i++) if (fscanf(f, "%d %d", &s->ram[i][0], &s->ram[i][1]) != 2) return 0;
  return 1;
}

int main(int argc, char **argv) {
  FILE *f = fopen(argc > 1 ? argv[1] : "tests.txt", "r");
  if (!f) return 1;
  char op[8];
  static St in, fin;
  int total = 0, fail = 0;
  static int fail_by_op[256], total_by_op[256];
  static char first_fail[256][256];
  while (fscanf(f, "%7s", op) == 1) {
    if (!read_st(f, &in) || !read_st(f, &fin)) break;
    int opc = (int)strtol(op, NULL, 16);
    for (int i = 0; i < in.n; i++) g_flat_mem[in.ram[i][0]] = (uint8_t)in.ram[i][1];
    memset(&cpu, 0, sizeof cpu);
    cpu.fe = (uint8_t)in.e;
    cpu.a = (uint16_t)in.a; cpu.x = (uint16_t)in.x; cpu.y = (uint16_t)in.y;
    cpu.s = (uint16_t)in.s; cpu.dp = (uint16_t)in.d; cpu.db = (uint8_t)in.dbr; cpu.pb = (uint8_t)in.pbr;
    cpu.pc = (uint16_t)in.pc;
    cpu_set_p((uint8_t)in.p);
    interp_step();
    // MVN/MVP move one byte per step (interruptible, as on hardware); the
    // reference tests treat the whole block move as one instruction.
    if (opc == 0x44 || opc == 0x54) while (cpu.pc == (uint16_t)in.pc) interp_step();
    char why[200] = "";
    if (cpu.pc != fin.pc) snprintf(why, sizeof why, "pc %04X want %04X", cpu.pc, fin.pc);
    else if (cpu_get_p() != fin.p) snprintf(why, sizeof why, "p %02X want %02X", cpu_get_p(), fin.p);
    else if (cpu.a != fin.a) snprintf(why, sizeof why, "a %04X want %04X", cpu.a, fin.a);
    else if (cpu.x != fin.x) snprintf(why, sizeof why, "x %04X want %04X", cpu.x, fin.x);
    else if (cpu.y != fin.y) snprintf(why, sizeof why, "y %04X want %04X", cpu.y, fin.y);
    else if (cpu.s != fin.s) snprintf(why, sizeof why, "s %04X want %04X", cpu.s, fin.s);
    else if (cpu.dp != fin.d) snprintf(why, sizeof why, "d %04X want %04X", cpu.dp, fin.d);
    else if (cpu.db != fin.dbr) snprintf(why, sizeof why, "dbr %02X want %02X", cpu.db, fin.dbr);
    else if (cpu.pb != fin.pbr) snprintf(why, sizeof why, "pbr %02X want %02X", cpu.pb, fin.pbr);
    else if (cpu.fe != fin.e) snprintf(why, sizeof why, "e %d want %d", cpu.fe, fin.e);
    else for (int i = 0; i < fin.n; i++)
      if (g_flat_mem[fin.ram[i][0]] != fin.ram[i][1]) {
        snprintf(why, sizeof why, "mem[%06X]=%02X want %02X", fin.ram[i][0], g_flat_mem[fin.ram[i][0]], fin.ram[i][1]);
        break;
      }
    total++; total_by_op[opc]++;
    if (why[0]) {
      fail++;
      if (!fail_by_op[opc]++)
        snprintf(first_fail[opc], sizeof first_fail[opc], "p_in=%02X a=%04X x=%04X y=%04X d=%04X: %s", in.p, in.a, in.x, in.y, in.d, why);
    }
    for (int i = 0; i < in.n; i++) g_flat_mem[in.ram[i][0]] = 0;
    for (int i = 0; i < fin.n; i++) g_flat_mem[fin.ram[i][0]] = 0;
  }
  for (int i = 0; i < 256; i++)
    if (fail_by_op[i]) printf("op %02X: %d/%d fail  first: %s\n", i, fail_by_op[i], total_by_op[i], first_fail[i]);
  printf("TOTAL %d tests, %d failed (%.2f%% pass)\n", total, fail, 100.0 * (total - fail) / total);
  return 0;
}
