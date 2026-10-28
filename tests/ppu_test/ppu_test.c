// Standalone PPU test: drives the PPU purely through ppu_write/ppu_read
// (VRAM/CGRAM/OAM uploads via $2118/$2119, $2122, $2104), renders frames and
// dumps them as PPM files (converted to PNG by convert.py).
//
// usage: ppu_test.exe <output dir>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "../../src/ppu.h"

// Extra debug accessors exported by ppu.c (not part of ppu.h).
uint16_t ppu_debug_bg_hofs(int bg);
uint16_t ppu_debug_bg_vofs(int bg);
int      ppu_debug_m7_hofs(void);
int      ppu_debug_m7_vofs(void);
uint16_t ppu_debug_vram_addr(void);
uint16_t ppu_debug_oam_addr(void);
uint8_t  ppu_debug_first_sprite(void);

static int g_fail = 0, g_checks = 0;
static const char *g_out = ".";

#define CHECK_EQ(a, b)                                                                     \
  do {                                                                                     \
    long _a = (long)(a), _b = (long)(b);                                                   \
    g_checks++;                                                                            \
    if (_a != _b) {                                                                        \
      printf("FAIL %s:%d: %s == %s (got 0x%lX, want 0x%lX)\n", __FILE__, __LINE__, #a, #b, \
             _a, _b);                                                                      \
      g_fail++;                                                                            \
    }                                                                                      \
  } while (0)

/* ------------------------------------------------------------------------ */
/* CPU-like register helpers                                                 */
/* ------------------------------------------------------------------------ */

static void W(uint8_t r, unsigned v) { ppu_write(r, (uint8_t)v); }
static uint8_t R(uint8_t r) { return ppu_read(r, 0xEE); }
static void W16(uint8_t r, int v) { W(r, (unsigned)v & 0xff); W(r, ((unsigned)v >> 8) & 0xff); }
static void set_vaddr(unsigned a) { W(0x16, a & 0xff); W(0x17, (a >> 8) & 0xff); }
static void vwrite(unsigned w) { W(0x18, w & 0xff); W(0x19, (w >> 8) & 0xff); } // VMAIN = 0x80
static uint16_t rgb(int r, int g, int b) { return (uint16_t)(r | (g << 5) | (b << 10)); }
static void set_color(int i, uint16_t c) { W(0x21, (unsigned)i); W(0x22, c & 0xff); W(0x22, c >> 8); }
static uint16_t vram_word(unsigned w) { const uint8_t *v = ppu_vram(); return (uint16_t)(v[w * 2] | (v[w * 2 + 1] << 8)); }
static uint16_t cgram_word(unsigned i) { const uint8_t *c = ppu_cgram(); return (uint16_t)(c[i * 2] | (c[i * 2 + 1] << 8)); }

static uint8_t g_oam[544];
static void oam_clear(void) {
  memset(g_oam, 0, sizeof g_oam);
  for (int i = 0; i < 128; i++) g_oam[i * 4 + 1] = 0xF0; // y = 240: off screen
}
// attr: vhoopppN layout built by ATTR(); tile may be 0..511 (bit 8 = name table)
#define ATTR(pal, prio, h, v) (((v) << 7) | ((h) << 6) | ((prio) << 4) | ((pal) << 1))
static void oam_set(int i, int x, int y, int tile, int attr, int large) {
  g_oam[i * 4 + 0] = (uint8_t)(x & 0xff);
  g_oam[i * 4 + 1] = (uint8_t)y;
  g_oam[i * 4 + 2] = (uint8_t)(tile & 0xff);
  g_oam[i * 4 + 3] = (uint8_t)(attr | ((tile >> 8) & 1));
  const int sh = (i & 3) * 2;
  g_oam[512 + i / 4] = (uint8_t)((g_oam[512 + i / 4] & ~(3 << sh)) | ((((x >> 8) & 1) | (large << 1)) << sh));
}
static void oam_upload(void) {
  W(0x02, 0);
  W(0x03, 0);
  for (int i = 0; i < 544; i++) W(0x04, g_oam[i]);
}

static void upload_tile(unsigned waddr, const uint8_t pix[64], int bpp) {
  W(0x15, 0x80);
  set_vaddr(waddr);
  for (int pl = 0; pl < bpp; pl += 2) {
    for (int r = 0; r < 8; r++) {
      unsigned lo = 0, hi = 0;
      for (int c = 0; c < 8; c++) {
        lo |= (unsigned)((pix[r * 8 + c] >> pl) & 1) << (7 - c);
        hi |= (unsigned)((pix[r * 8 + c] >> (pl + 1)) & 1) << (7 - c);
      }
      vwrite(lo | (hi << 8));
    }
  }
}
// Upload a 16x16 picture as 4 4bpp chars n, n+1, n+16, n+17 of a 16-char-wide name table.
static void upload_obj16(unsigned base, int n, const uint8_t pix[256]) {
  for (int q = 0; q < 4; q++) {
    uint8_t t[64];
    const int ox = (q & 1) * 8, oy = (q >> 1) * 8;
    for (int r = 0; r < 8; r++)
      for (int c = 0; c < 8; c++) t[r * 8 + c] = pix[(oy + r) * 16 + ox + c];
    upload_tile(base + (unsigned)(n + (q & 1) + (q >> 1) * 16) * 16, t, 4);
  }
}

static void render_frame(void (*per_line)(int y)) {
  ppu_frame_start();
  for (int y = 1; y <= 224; y++) {
    if (per_line) per_line(y);
    ppu_render_line(y);
  }
  ppu_vblank_start();
}

static void save_ppm(const char *name) {
  char path[512];
  snprintf(path, sizeof path, "%s/%s.ppm", g_out, name);
  FILE *f = fopen(path, "wb");
  if (!f) { printf("cannot write %s\n", path); g_fail++; return; }
  fprintf(f, "P6\n256 224\n255\n");
  const uint32_t *fb = ppu_framebuffer();
  for (int i = 0; i < 256 * 224; i++) {
    uint8_t px[3] = {(uint8_t)(fb[i] >> 16), (uint8_t)(fb[i] >> 8), (uint8_t)fb[i]};
    fwrite(px, 1, 3, f);
  }
  fclose(f);
  printf("wrote %s\n", path);
}

static uint32_t fb_px(int x, int row) { return ppu_framebuffer()[row * 256 + x]; }
static uint32_t c15to32(uint16_t c) {
  unsigned r = c & 31, g = (c >> 5) & 31, b = (c >> 10) & 31;
  r = (r << 3) | (r >> 2); g = (g << 3) | (g >> 2); b = (b << 3) | (b >> 2);
  return (r << 16) | (g << 8) | b;
}

/* ------------------------------------------------------------------------ */
/* Tiny 8x8 font                                                             */
/* ------------------------------------------------------------------------ */

typedef struct { char ch; const char *r[8]; } Glyph;
static const Glyph k_font[] = {
  {'0', {"..###...", ".#...#..", ".#..##..", ".#.#.#..", ".##..#..", ".#...#..", "..###...", "........"}},
  {'1', {"...#....", "..##....", "...#....", "...#....", "...#....", "...#....", "..###...", "........"}},
  {'2', {"..###...", ".#...#..", ".....#..", "....#...", "...#....", "..#.....", ".#####..", "........"}},
  {'3', {".#####..", "....#...", "...#....", "....#...", ".....#..", ".#...#..", "..###...", "........"}},
  {'4', {"....#...", "...##...", "..#.#...", ".#..#...", ".#####..", "....#...", "....#...", "........"}},
  {'5', {".#####..", ".#......", ".####...", ".....#..", ".....#..", ".#...#..", "..###...", "........"}},
  {'6', {"...##...", "..#.....", ".#......", ".####...", ".#...#..", ".#...#..", "..###...", "........"}},
  {'7', {".#####..", ".....#..", "....#...", "...#....", "..#.....", "..#.....", "..#.....", "........"}},
  {'8', {"..###...", ".#...#..", ".#...#..", "..###...", ".#...#..", ".#...#..", "..###...", "........"}},
  {'9', {"..###...", ".#...#..", ".#...#..", "..####..", ".....#..", "....#...", "..##....", "........"}},
  {'A', {"..###...", ".#...#..", ".#...#..", ".#####..", ".#...#..", ".#...#..", ".#...#..", "........"}},
  {'C', {"..###...", ".#...#..", ".#......", ".#......", ".#......", ".#...#..", "..###...", "........"}},
  {'E', {".#####..", ".#......", ".#......", ".####...", ".#......", ".#......", ".#####..", "........"}},
  {'F', {".#####..", ".#......", ".#......", ".####...", ".#......", ".#......", ".#......", "........"}},
  {'I', {"..###...", "...#....", "...#....", "...#....", "...#....", "...#....", "..###...", "........"}},
  {'M', {".#...#..", ".##.##..", ".#.#.#..", ".#.#.#..", ".#...#..", ".#...#..", ".#...#..", "........"}},
  {'N', {".#...#..", ".#...#..", ".##..#..", ".#.#.#..", ".#..##..", ".#...#..", ".#...#..", "........"}},
  {'O', {"..###...", ".#...#..", ".#...#..", ".#...#..", ".#...#..", ".#...#..", "..###...", "........"}},
  {'R', {".####...", ".#...#..", ".#...#..", ".####...", ".#.#....", ".#..#...", ".#...#..", "........"}},
  {'S', {"..####..", ".#......", ".#......", "..###...", ".....#..", ".....#..", ".####...", "........"}},
  {'T', {".#####..", "...#....", "...#....", "...#....", "...#....", "...#....", "...#....", "........"}},
  {'X', {".#...#..", ".#...#..", "..#.#...", "...#....", "..#.#...", ".#...#..", ".#...#..", "........"}},
};
#define FONT_N ((int)(sizeof k_font / sizeof k_font[0]))

static int glyph_index(char ch) {
  for (int i = 0; i < FONT_N; i++) if (k_font[i].ch == ch) return i;
  return -1;
}
static int glyph_bit(char ch, int r, int c) {
  const int i = glyph_index(ch);
  if (i < 0 || r < 0 || r > 7 || c < 0 || c > 7) return 0;
  return k_font[i].r[r][c] == '#';
}
static void glyph_pixels(char ch, uint8_t fg, uint8_t bg, uint8_t pix[64]) {
  for (int r = 0; r < 8; r++)
    for (int c = 0; c < 8; c++) pix[r * 8 + c] = glyph_bit(ch, r, c) ? fg : bg;
}

/* ------------------------------------------------------------------------ */
/* Register semantics checks                                                 */
/* ------------------------------------------------------------------------ */

static void test_registers(void) {
  ppu_init();

  // --- VRAM writes, increment on high byte, by 1 ---
  W(0x15, 0x80);
  set_vaddr(0x1000);
  W(0x18, 0x34);
  CHECK_EQ(ppu_debug_vram_addr(), 0x1000);
  W(0x19, 0x12);
  CHECK_EQ(ppu_debug_vram_addr(), 0x1001);
  vwrite(0x5678);
  CHECK_EQ(vram_word(0x1000), 0x1234);
  CHECK_EQ(vram_word(0x1001), 0x5678);
  // increment on low byte
  W(0x15, 0x00);
  set_vaddr(0x1100);
  W(0x18, 0xAA);
  CHECK_EQ(ppu_debug_vram_addr(), 0x1101);
  W(0x19, 0xBB);
  CHECK_EQ(ppu_debug_vram_addr(), 0x1101);
  CHECK_EQ(vram_word(0x1100), 0x00AA);
  CHECK_EQ(vram_word(0x1101), 0xBB00);
  // increment 32 / 128 / 128
  W(0x15, 0x81); set_vaddr(0x2000); vwrite(0x1111); CHECK_EQ(ppu_debug_vram_addr(), 0x2020);
  W(0x15, 0x82); set_vaddr(0x2000); vwrite(0x1111); CHECK_EQ(ppu_debug_vram_addr(), 0x2080);
  W(0x15, 0x83); set_vaddr(0x2000); vwrite(0x1111); CHECK_EQ(ppu_debug_vram_addr(), 0x2080);
  // address remapping
  W(0x15, 0x84); set_vaddr(0x0001); vwrite(0xBEEF); CHECK_EQ(vram_word(0x0008), 0xBEEF);
  set_vaddr(0x0020); vwrite(0xCAFE); CHECK_EQ(vram_word(0x0001), 0xCAFE);
  W(0x15, 0x88); set_vaddr(0x0041); vwrite(0x4242); CHECK_EQ(vram_word(0x0009), 0x4242);
  W(0x15, 0x8C); set_vaddr(0x0082); vwrite(0x4343); CHECK_EQ(vram_word(0x0011), 0x4343);
  // 32K-word wrap
  W(0x15, 0x80); set_vaddr(0xFFFF); vwrite(0x7777); CHECK_EQ(vram_word(0x7FFF), 0x7777);
  CHECK_EQ(ppu_debug_vram_addr(), 0x0000);

  // --- VRAM read prefetch ($2139/$213A), bsnes/Mesen semantics:
  // the latch is loaded when VMADD is written and, on the incrementing read,
  // reloaded from the (pre-increment) address before the address increments.
  W(0x15, 0x80);
  set_vaddr(0x3000); vwrite(0x1234); vwrite(0x5678); vwrite(0x9ABC);
  set_vaddr(0x3000);
  CHECK_EQ(R(0x39), 0x34);
  CHECK_EQ(ppu_debug_vram_addr(), 0x3000); // low read doesn't increment in mode 0x80
  CHECK_EQ(R(0x3A), 0x12);
  CHECK_EQ(ppu_debug_vram_addr(), 0x3001);
  CHECK_EQ(R(0x39), 0x34); // latch was refilled from 0x3000
  CHECK_EQ(R(0x3A), 0x12);
  CHECK_EQ(R(0x39), 0x78);
  CHECK_EQ(R(0x3A), 0x56);
  set_vaddr(0x3002); // prefetch on address write
  CHECK_EQ(R(0x3A), 0x9A);
  W(0x15, 0x00); // increment on low byte read
  set_vaddr(0x3001);
  CHECK_EQ(R(0x39), 0x78);
  CHECK_EQ(ppu_debug_vram_addr(), 0x3002);
  CHECK_EQ(R(0x3A), 0x56);
  CHECK_EQ(ppu_debug_vram_addr(), 0x3002);

  // --- OAM low table write latch ---
  const uint8_t *oam = ppu_oam();
  W(0x02, 0); W(0x03, 0);
  W(0x04, 0x11);
  CHECK_EQ(oam[0], 0x00); // buffered
  W(0x04, 0x22);
  CHECK_EQ(oam[0], 0x11);
  CHECK_EQ(oam[1], 0x22);
  W(0x04, 0x33);     // even byte -> latch only
  W(0x02, 0x00);     // address reload drops the half-written pair
  CHECK_EQ(oam[2], 0x00);
  CHECK_EQ(ppu_debug_oam_addr(), 0);
  // high table: written directly, mirrored every 32 bytes
  W(0x02, 0x00); W(0x03, 0x01);
  CHECK_EQ(ppu_debug_oam_addr(), 0x200);
  W(0x04, 0x55);
  CHECK_EQ(oam[0x200], 0x55);
  W(0x04, 0x66);
  CHECK_EQ(oam[0x201], 0x66);
  W(0x02, 0x10); W(0x03, 0x01); // byte 0x220 -> mirrors 0x200
  W(0x04, 0x77);
  CHECK_EQ(oam[0x200], 0x77);
  // OAM read with increment
  W(0x02, 0); W(0x03, 0);
  CHECK_EQ(R(0x38), 0x11);
  CHECK_EQ(R(0x38), 0x22);
  CHECK_EQ(ppu_debug_oam_addr(), 2);
  // priority rotation: first sprite = (OAMADD word >> 1) & 127
  W(0x02, 0x0A); W(0x03, 0x80);
  CHECK_EQ(ppu_debug_first_sprite(), 5);
  W(0x03, 0x00);
  CHECK_EQ(ppu_debug_first_sprite(), 0);
  // reload at vblank (not in forced blank)
  W(0x00, 0x0F);
  W(0x02, 0x04); W(0x03, 0x00);
  W(0x04, 1); W(0x04, 2); W(0x04, 3);
  CHECK_EQ(ppu_debug_oam_addr(), 11);
  ppu_vblank_start();
  CHECK_EQ(ppu_debug_oam_addr(), 8);
  W(0x00, 0x80); // forced blank: no reload
  W(0x04, 1);
  ppu_vblank_start();
  CHECK_EQ(ppu_debug_oam_addr(), 9);

  // --- CGRAM latch ---
  W(0x21, 5); W(0x22, 0xFF); W(0x22, 0xFF);
  CHECK_EQ(cgram_word(5), 0x7FFF);
  W(0x21, 6); W(0x22, 0x34);
  CHECK_EQ(cgram_word(6), 0x0000); // buffered
  W(0x22, 0x12);
  CHECK_EQ(cgram_word(6), 0x1234);
  W(0x22, 0xCD); W(0x22, 0x0B); // auto-increment to 7
  CHECK_EQ(cgram_word(7), 0x0BCD);
  W(0x21, 6);
  CHECK_EQ(R(0x3B), 0x34);
  CHECK_EQ(R(0x3B), 0x12); // bit 7 = PPU2 open bus (was 0x34 -> 0)
  CHECK_EQ(R(0x3B), 0xCD);
  CHECK_EQ(R(0x3B), 0x8B); // bit 7 from previous 0xCD
  W(0x21, 5);
  CHECK_EQ(R(0x3B), 0xFF);
  CHECK_EQ(R(0x3B), 0xFF);

  // --- BGnHOFS / BGnVOFS latch formula ---
  // HOFS = (val<<8) | (prev & ~7) | ((HOFS>>8) & 7), applied on every write
  W(0x0D, 0x05);
  CHECK_EQ(ppu_debug_bg_hofs(0), 0x100); // 0x500 & 0x3FF
  W(0x0D, 0x01);
  CHECK_EQ(ppu_debug_bg_hofs(0), 0x105); // (1<<8) | (5&~7) | ((0x500>>8)&7)
  W(0x0D, 0xFF); W(0x0D, 0x02);
  CHECK_EQ(ppu_debug_bg_hofs(0), 0x2FF); // lo/hi pair gives the plain value
  CHECK_EQ(ppu_debug_m7_hofs(), 0x2FF);  // mode-7 latch: plain 16-bit
  W(0x0D, 0x0B);                         // lone high write: (0xB<<8)|(0x02&~7)|((0x2FF>>8)&7)
  CHECK_EQ(ppu_debug_bg_hofs(0), 0x302);
  W(0x0E, 0x34); W(0x0E, 0x01);
  CHECK_EQ(ppu_debug_bg_vofs(0), 0x134);
  W(0x0F, 0xAB); W(0x10, 0x00); // shared "previous byte" between registers
  CHECK_EQ(ppu_debug_bg_vofs(1), 0x0AB);
  W(0x13, 0x07); W(0x13, 0x03);
  CHECK_EQ(ppu_debug_bg_hofs(3), 0x307);
  W(0x0D, 0x00); W(0x0D, 0x1F); // M7HOFS = 0x1F00 -> 13-bit signed -256
  CHECK_EQ(ppu_debug_m7_hofs(), -256);
  W(0x0E, 0x00); W(0x0E, 0x10); // 0x1000 -> -4096
  CHECK_EQ(ppu_debug_m7_vofs(), -4096);

  // --- MPY ---
  W16(0x1B, 0x1234);
  W(0x1C, 0x00); W(0x1C, 0xFE); // M7B high byte = -2
  CHECK_EQ(R(0x34), 0x98);
  CHECK_EQ(R(0x35), 0xDB);
  CHECK_EQ(R(0x36), 0xFF); // 0x1234 * -2 = -0x2468 = 0xFFDB98
  W16(0x1B, 0x8000);
  W(0x1C, 0x00); W(0x1C, 0x80);
  CHECK_EQ(R(0x34), 0x00);
  CHECK_EQ(R(0x35), 0x00);
  CHECK_EQ(R(0x36), 0x40); // -32768 * -128 = 0x400000
  W16(0x1B, 0x0003);
  W(0x1C, 0x05);           // single write: M7B = 0x05 << 8 | latch(0x00)
  CHECK_EQ(R(0x34), 0x0F);

  // --- H/V counter latch, flip-flops, STAT78 ---
  (void)R(0x3F); // reset flip-flops
  ppu_latch_hv(0x123, 0x0F0);
  CHECK_EQ(R(0x3C), 0x23);
  CHECK_EQ(R(0x3C) & 1, 1);
  CHECK_EQ(R(0x3D), 0xF0);
  CHECK_EQ(R(0x3D) & 1, 0);
  CHECK_EQ(R(0x3C), 0x23); // flip-flop toggled back to low
  {
    uint8_t s = R(0x3F);
    CHECK_EQ(s & 0x40, 0x40); // latched
    CHECK_EQ(s & 0x0F, 3);    // PPU2 version
    CHECK_EQ(s & 0x10, 0);    // NTSC
    CHECK_EQ(R(0x3F) & 0x40, 0); // cleared by the read
  }
  CHECK_EQ(R(0x3C), 0x23); // $213F reset the H flip-flop (low byte again)
  CHECK_EQ(R(0x3E) & 0x0F, 1); // PPU1 version
  CHECK_EQ(R(0x37), 0xEE);     // open bus
}

/* ------------------------------------------------------------------------ */
/* Mode 1 scene                                                              */
/* ------------------------------------------------------------------------ */

enum {
  BG1_CHR = 0x0000, BG2_CHR = 0x1000, BG3_CHR = 0x2000,
  BG1_MAP = 0x4000, BG2_MAP = 0x4400, BG3_MAP = 0x4800,
  OBJ_CHR = 0x6000,
};

static void put_map(unsigned base, int tx, int ty, unsigned entry) {
  W(0x15, 0x80);
  set_vaddr(base + (unsigned)(ty * 32 + tx));
  vwrite(entry);
}
static unsigned ME(int tile, int pal, int prio, int h, int v) {
  return (unsigned)(tile | (pal << 10) | (prio << 13) | (h << 14) | (v << 15));
}
static void put_text(int tx, int ty, const char *s) { // BG3 status bar text, high priority
  for (int x = 0; x < 32; x++) {
    const char ch = (x >= tx && *s) ? *s++ : ' ';
    const int gi = glyph_index(ch);
    put_map(BG3_MAP, x, ty, ME(gi >= 0 ? 1 + gi : 0x40, 1, 1, 0, 0));
  }
}

static void upload_obj_chars(void) {
  uint8_t pix[64], big[256];
  // char 0x00: 8x8 'F'
  glyph_pixels('F', 1, 3, pix);
  upload_tile(OBJ_CHR + 0x00 * 16, pix, 4);
  // chars 0x02/0x03/0x12/0x13: 16x16 'R' with a Γ border (top row + left column)
  for (int y = 0; y < 16; y++)
    for (int x = 0; x < 16; x++) {
      uint8_t v = 3;
      if (y == 0 || x == 0) v = 2;
      if (y >= 1 && x >= 1 && glyph_bit('R', (y - 1) / 2, (x - 1) / 2)) v = 1;
      big[y * 16 + x] = v;
    }
  upload_obj16(OBJ_CHR, 0x02, big);
  // chars 0x04/0x05/0x14/0x15: 16x16 ball
  for (int y = 0; y < 16; y++)
    for (int x = 0; x < 16; x++) {
      const double dx = x - 7.5, dy = y - 7.5, d = sqrt(dx * dx + dy * dy);
      uint8_t v = 0;
      if (d < 7.6) v = 3;
      if (d >= 6.2 && d < 7.6) v = 2;
      if ((x - 5) * (x - 5) + (y - 5) * (y - 5) < 5) v = 1;
      big[y * 16 + x] = v;
    }
  upload_obj16(OBJ_CHR, 0x04, big);
  // chars 0x40..0xFF: color grid (fill color + white diagonal marker in top-left)
  for (int n = 0x40; n < 0x100; n++) {
    const uint8_t fill = (uint8_t)(1 + ((n & 15) + (n >> 4) * 3) % 14);
    for (int r = 0; r < 8; r++)
      for (int c = 0; c < 8; c++) pix[r * 8 + c] = (r == c && r < 4) || (r == 0 && c < 3) ? 15 : fill;
    upload_tile(OBJ_CHR + (unsigned)n * 16, pix, 4);
  }
  // second name table (OBSEL name select 0 -> +0x1000 words): char 0x100 = '2'
  glyph_pixels('2', 1, 2, pix);
  upload_tile(OBJ_CHR + 0x1000, pix, 4);
}

static void upload_obj_palettes(void) {
  set_color(129, rgb(31, 31, 31)); set_color(130, rgb(31, 28, 0)); set_color(131, rgb(24, 2, 2));
  set_color(145, rgb(31, 31, 31)); set_color(146, rgb(4, 28, 28)); set_color(147, rgb(2, 6, 24));
  set_color(161, rgb(31, 31, 31)); set_color(162, rgb(0, 0, 0));   set_color(163, rgb(4, 22, 4));
  { // palette 3: 14 clearly distinct colors + white marker
    static const uint8_t k[14][3] = {{31, 0, 0},  {31, 16, 0}, {31, 31, 0}, {16, 31, 0}, {0, 31, 0},
                                     {0, 31, 16}, {0, 31, 31}, {0, 16, 31}, {0, 0, 31},  {16, 0, 31},
                                     {31, 0, 31}, {31, 0, 16}, {16, 16, 16}, {8, 8, 8}};
    for (int i = 0; i < 14; i++) set_color(177 + i, rgb(k[i][0], k[i][1], k[i][2]));
  }
  set_color(191, rgb(31, 31, 31));
  set_color(193, rgb(31, 31, 31)); set_color(194, rgb(31, 28, 0)); set_color(195, rgb(28, 4, 28));
}

static void setup_mode1(void) {
  ppu_init();
  W(0x00, 0x80); // forced blank during upload
  uint8_t pix[64];

  // ---- palettes ----
  set_color(0, rgb(12, 20, 31));                                          // backdrop: sky
  set_color(5, rgb(31, 31, 31)); set_color(6, rgb(31, 28, 0)); set_color(7, rgb(2, 2, 10)); // BG3 pal 1
  set_color(9, rgb(31, 31, 31)); set_color(10, rgb(22, 24, 28));          // BG3 pal 2 (cloud)
  set_color(33, rgb(4, 24, 4)); set_color(34, rgb(2, 14, 2)); set_color(35, rgb(20, 12, 4));
  set_color(36, rgb(12, 6, 2)); set_color(37, rgb(26, 18, 8));            // BG1 pal 2 (ground)
  set_color(49, rgb(28, 14, 4)); set_color(50, rgb(10, 4, 2)); set_color(51, rgb(31, 26, 4));
  set_color(52, rgb(16, 10, 0)); set_color(53, rgb(31, 31, 31));          // BG1 pal 3 (bricks)
  set_color(65, rgb(20, 28, 20)); set_color(66, rgb(14, 22, 14)); set_color(67, rgb(10, 18, 10));
  set_color(68, rgb(6, 14, 6)); set_color(69, rgb(2, 6, 2));               // BG2 pal 4
  upload_obj_palettes();

  // ---- BG1 chars (4bpp) ----
  for (int r = 0; r < 8; r++)
    for (int c = 0; c < 8; c++)
      pix[r * 8 + c] = (uint8_t)(r < 2 ? 1 : r == 2 ? ((c & 3) == 1 ? 1 : 2) : ((r * 3 + c * 5) % 7 == 0 ? 4 : 3));
  upload_tile(BG1_CHR + 1 * 16, pix, 4);
  for (int r = 0; r < 8; r++)
    for (int c = 0; c < 8; c++)
      pix[r * 8 + c] = (uint8_t)((r * 3 + c * 5) % 7 == 0 ? 4 : ((r + c * 3) % 11 == 0 ? 5 : 3));
  upload_tile(BG1_CHR + 2 * 16, pix, 4);
  for (int r = 0; r < 8; r++)
    for (int c = 0; c < 8; c++)
      pix[r * 8 + c] = (uint8_t)((r == 3 || r == 7 || (r < 3 && c == 7) || (r > 3 && r < 7 && c == 3)) ? 2 : 1);
  upload_tile(BG1_CHR + 3 * 16, pix, 4);
  for (int r = 0; r < 8; r++)
    for (int c = 0; c < 8; c++)
      pix[r * 8 + c] = (uint8_t)((r == 0 || r == 7 || c == 0 || c == 7) ? 4 : ((r == 3 || r == 4) && (c == 3 || c == 4)) ? 5 : 3);
  upload_tile(BG1_CHR + 4 * 16, pix, 4);

  // ---- BG2 chars (4bpp, used as 16x16 tile 0x22 = chars 0x22,0x23,0x32,0x33) ----
  for (int r = 0; r < 8; r++)
    for (int c = 0; c < 8; c++) pix[r * 8 + c] = (uint8_t)((r == c || r == 0) ? 5 : 1);
  upload_tile(BG2_CHR + 0x22 * 16, pix, 4);
  memset(pix, 2, 64); upload_tile(BG2_CHR + 0x23 * 16, pix, 4);
  memset(pix, 3, 64); upload_tile(BG2_CHR + 0x32 * 16, pix, 4);
  memset(pix, 4, 64); upload_tile(BG2_CHR + 0x33 * 16, pix, 4);

  // ---- BG3 chars (2bpp): font glyphs at 1.., solid bar 0x40, cloud 0x41 ----
  for (int i = 0; i < FONT_N; i++) {
    glyph_pixels(k_font[i].ch, 1, 3, pix);
    upload_tile(BG3_CHR + (unsigned)(1 + i) * 8, pix, 2);
  }
  memset(pix, 3, 64); upload_tile(BG3_CHR + 0x40 * 8, pix, 2);
  for (int r = 0; r < 8; r++)
    for (int c = 0; c < 8; c++) {
      const double dx = (c - 3.5) / 4.0, dy = (r - 4.0) / 3.5;
      pix[r * 8 + c] = (uint8_t)(dx * dx + dy * dy < 1.0 ? (r >= 6 ? 2 : 1) : 0);
    }
  upload_tile(BG3_CHR + 0x41 * 8, pix, 2);

  // ---- BG1 map: ground rows 24..31, bricks row 15, priority column ----
  for (int x = 0; x < 32; x++) {
    put_map(BG1_MAP, x, 24, ME(1, 2, 0, 0, 0));
    for (int y = 25; y < 32; y++) put_map(BG1_MAP, x, y, ME(2, 2, 0, 0, 0));
  }
  for (int x = 8; x < 16; x++) put_map(BG1_MAP, x, 15, ME(3, 3, 0, 0, 0));
  for (int y = 17; y < 24; y++)
    for (int x = 20; x < 22; x++) put_map(BG1_MAP, x, y, ME(4, 3, 1, 0, 0));

  // ---- BG2 map (16x16 tiles): rows 3..10, flips alternate ----
  for (int y = 3; y <= 10; y++)
    for (int x = 0; x < 32; x++) put_map(BG2_MAP, x, y, ME(0x22, 4, 0, x & 1, y & 1));

  // ---- BG3 map: status bar rows 1-2 (priority), clouds row 4-5 (low) ----
  put_text(1, 1, "MARIO X 05            TIME 312");
  put_text(1, 2, "COINS 47          SCORE 0012340");
  for (int y = 4; y < 6; y++)
    for (int x = 0; x < 4; x++) {
      put_map(BG3_MAP, 4 + x, y, ME(0x41, 2, 0, 0, 0));
      put_map(BG3_MAP, 18 + x, y, ME(0x41, 2, 0, 0, 0));
    }

  upload_obj_chars();

  // ---- sprites ----
  oam_clear();
  oam_set(0, 40, 60, 0x02, ATTR(0, 3, 0, 0), 1);
  oam_set(1, 64, 60, 0x02, ATTR(0, 3, 1, 0), 1);
  oam_set(2, 88, 60, 0x02, ATTR(0, 3, 0, 1), 1);
  oam_set(3, 112, 60, 0x02, ATTR(0, 3, 1, 1), 1);
  oam_set(4, 40, 84, 0x00, ATTR(1, 3, 0, 0), 0);
  oam_set(5, 56, 84, 0x00, ATTR(1, 3, 1, 0), 0);
  oam_set(6, 72, 84, 0x00, ATTR(1, 3, 0, 1), 0);
  oam_set(7, 88, 84, 0x00, ATTR(1, 3, 1, 1), 0);
  oam_set(8, -8, 100, 0x02, ATTR(1, 3, 0, 0), 1);   // half off the left edge
  oam_set(9, -4, 120, 0x00, ATTR(1, 3, 0, 0), 0);   // half off the left edge
  oam_set(10, 110, 84, 0x100, ATTR(2, 3, 0, 0), 0); // second name table
  oam_set(11, 152, 150, 0x02, ATTR(0, 2, 0, 0), 1); // prio 2: behind BG1 high-prio column
  oam_set(12, 8, 2, 0x02, ATTR(0, 3, 0, 0), 1);     // prio 3: behind BG3 high w/ priority bit
  oam_set(13, 200, 60, 0x04, ATTR(4, 3, 0, 0), 1);  // palette 4: color math capable
  oam_set(14, 224, 60, 0x04, ATTR(0, 3, 0, 0), 1);  // palette 0: never color math
  oam_set(15, 120, 100, 0x02, ATTR(1, 0, 0, 0), 1); // prio 0: behind BG2
  oam_upload();

  // ---- registers ----
  W(0x01, 0x03);                  // OBJ 8x8/16x16, base 0x6000, gap 0
  W(0x05, 0x09 | 0x20);           // mode 1, BG3 priority, BG2 16x16 tiles
  W(0x07, BG1_MAP >> 8);          // 32x32
  W(0x08, BG2_MAP >> 8);
  W(0x09, BG3_MAP >> 8);
  W(0x0B, 0x10);                  // BG1 chr 0x0000, BG2 chr 0x1000
  W(0x0C, 0x02);                  // BG3 chr 0x2000
  W(0x0D, 0x00); W(0x0D, 0x00);   // BG1 HOFS 0
  W(0x0E, 0xFF); W(0x0E, 0x03);   // BG1 VOFS -1 (0x3FF)
  W(0x0F, 0x08); W(0x0F, 0x00);   // BG2 HOFS 8
  W(0x10, 0xFF); W(0x10, 0x03);
  W(0x11, 0x00); W(0x11, 0x00);
  W(0x12, 0xFF); W(0x12, 0x03);
  W(0x2C, 0x17);                  // TM: BG1 BG2 BG3 OBJ
  W(0x2D, 0x00);
  W(0x00, 0x0F);                  // display on, full brightness
}


static void fade_line(int y) {
  if (y >= 100 && y < 110) W(0x00, 0x80);
  else W(0x00, (unsigned)(15 - ((y - 1) * 16) / 224));
}

static void test_mode1_scenes(void) {
  // --- base ---
  setup_mode1();
  render_frame(NULL);
  save_ppm("m1_base");
  // pixel checks: backdrop at (0, 30)? (BG3 cloud row 4-5 = y 32..47; BG2 starts y 48)
  CHECK_EQ(fb_px(128, 30), c15to32(rgb(12, 20, 31)));
  // sprite #12 (prio 3) is visible above the BG3 bar but hidden behind it (BG3 priority bit)
  CHECK_EQ(fb_px(8, 4), c15to32(rgb(31, 28, 0)));  // left border column of 'R' sprite, row 4
  CHECK_EQ(fb_px(8, 12), c15to32(rgb(2, 2, 10)));  // bar background covers it

  // --- windows: BG2 masked by W1 [96,159]; OBJ masked by W2 [176,207];
  //     BG1 masked by W1 XOR W2 ---
  setup_mode1();
  W(0x26, 96); W(0x27, 159); W(0x28, 176); W(0x29, 207);
  W(0x23, 0x2A);  // BG1: W1+W2 enable, BG2: W1 enable
  W(0x25, 0x08);  // OBJ: W2 enable
  W(0x2A, 0x02);  // BG1 logic XOR
  W(0x2E, 0x13);  // TMW: BG1 BG2 OBJ
  render_frame(NULL);
  save_ppm("m1_window");
  CHECK_EQ(fb_px(150, 100), c15to32(rgb(12, 20, 31))); // BG2 masked -> backdrop

  // --- color math: subscreen = BG2, half-add on BG1 + OBJ (pal 4-7 only),
  //     fixed color red used where the subscreen is transparent (no halving),
  //     math prevented inside color window W1 [96,159] ---
  setup_mode1();
  W(0x26, 96); W(0x27, 159);
  W(0x25, 0x20);  // color window: W1 enable
  W(0x2D, 0x02);  // TS: BG2
  W(0x30, 0x22);  // math: prevent inside color window (bits 4-5 = 2), add subscreen
  W(0x31, 0x51);  // half, OBJ, BG1
  W(0x32, 0x20 | 10); W(0x32, 0x40); W(0x32, 0x80); // fixed = (10,0,0)
  render_frame(NULL);
  save_ppm("m1_math");
  {
    // ground at (0,200): BG1 + fixed red, no half. Check red channel increased vs base.
    // brick (x=70,y=121) over BG2 -> averaged with BG2.
    const uint32_t ground = fb_px(4, 210);
    printf("  math: ground px %06X, brick px %06X, brick-in-prevent px %06X\n", ground, fb_px(70, 124), fb_px(100, 124));
  }

  // --- subtract fixed color from backdrop, clip-to-black inside color window W2 ---
  setup_mode1();
  W(0x28, 176); W(0x29, 207);
  W(0x25, 0x80);  // color window: W2 enable
  W(0x30, 0x80);  // clip main to black inside color window; fixed color
  W(0x31, 0xA0);  // subtract, backdrop
  W(0x32, 0x20 | 8); W(0x32, 0x40 | 8); W(0x32, 0x80);
  render_frame(NULL);
  save_ppm("m1_sub_clip");
  CHECK_EQ(fb_px(128, 30), c15to32(rgb(4, 12, 31)));
  CHECK_EQ(fb_px(190, 30), 0);
  CHECK_EQ(fb_px(190, 12), 0);

  // --- brightness fade per line + forced blank band ---
  setup_mode1();
  render_frame(fade_line);
  save_ppm("m1_fade");
  CHECK_EQ(fb_px(128, 104), 0);
  CHECK_EQ(fb_px(128, 223), 0); // brightness 0 on the last lines


  // --- mosaic 4x4 on BG1 + BG2 ---
  setup_mode1();
  W(0x06, 0x33);
  render_frame(NULL);
  save_ppm("m1_mosaic");
}

/* ------------------------------------------------------------------------ */
/* Sprite limits / rotation / sizes                                          */
/* ------------------------------------------------------------------------ */

static void setup_obj_test(void) {
  setup_mode1();
  W(0x2C, 0x10); // OBJ only
  set_color(0, rgb(4, 4, 8));
  oam_clear();
  for (int i = 0; i < 40; i++) oam_set(i, i * 6, 20, 0x00, ATTR(i & 1, 3, 0, 0), 0); // 40 on a line
  for (int i = 0; i < 18; i++) oam_set(40 + i, i * 14, 60, 0x02, ATTR(0, 3, 0, 0), 1); // 36 tiles
  for (int i = 0; i < 8; i++) oam_set(60 + i, 40 + i * 10, 120, 0x04, ATTR((i & 3) == 3 ? 4 : (i & 3), 3, 0, 0), 1);
  oam_set(70, 200, 250, 0x02, ATTR(1, 3, 0, 0), 1); // Y wrap: bottom rows shown at top
  oam_upload();
}

static void test_obj(void) {
  setup_obj_test();
  render_frame(NULL);
  save_ppm("obj_limits");
  CHECK_EQ(R(0x3E) & 0xC0, 0xC0); // time over + range over

  setup_obj_test();
  W(0x02, 64 * 2 & 0xff); W(0x03, 0x80 | ((64 * 2) >> 8)); // priority rotation: sprite 64 first
  CHECK_EQ(ppu_debug_first_sprite(), 64);
  render_frame(NULL);
  save_ppm("obj_rotate");

  // flags clear at the next frame start when no overflow happens
  setup_mode1();
  render_frame(NULL);
  ppu_frame_start();
  CHECK_EQ(R(0x3E) & 0xC0, 0);

  // OBSEL size 6 (16x32 / 32x64), incl. vflip on rectangular sprites
  setup_mode1();
  W(0x2C, 0x10);
  set_color(0, rgb(4, 4, 8));
  W(0x01, 0x03 | (6 << 5));
  oam_clear();
  oam_set(0, 16, 40, 0x40, ATTR(3, 3, 0, 0), 0);  // 16x32
  oam_set(1, 48, 40, 0x40, ATTR(3, 3, 0, 1), 0);  // 16x32 vflip
  oam_set(2, 80, 40, 0x40, ATTR(3, 3, 1, 0), 0);  // 16x32 hflip
  oam_set(3, 120, 40, 0x44, ATTR(3, 3, 0, 0), 1); // 32x64
  oam_set(4, 168, 40, 0x44, ATTR(3, 3, 1, 1), 1); // 32x64 h+v flip
  oam_set(5, 16, 140, 0x40, ATTR(3, 3, 0, 0), 0);
  oam_upload();
  render_frame(NULL);
  save_ppm("obj_sizes");
  // OBSEL size 5 (32x32 / 64x64)
  W(0x01, 0x03 | (5 << 5));
  render_frame(NULL);
  save_ppm("obj_sizes5");
}

/* ------------------------------------------------------------------------ */
/* Mode 7                                                                    */
/* ------------------------------------------------------------------------ */

static void setup_mode7(int extbg_f) {
  ppu_init();
  W(0x00, 0x80);
  // chars (high bytes): VMAIN = 0x80 (increment after $2119), write only $2119
  W(0x15, 0x80);
  for (int ch = 0; ch < 6; ch++) {
    set_vaddr((unsigned)ch * 64);
    for (int r = 0; r < 8; r++)
      for (int c = 0; c < 8; c++) {
        unsigned v = 0;
        const int edge = (r == 0 || c == 0 || r == 7 || c == 7);
        switch (ch) {
        case 0: v = ((r + c) & 4) ? 6 : 7; break; // tile-0 fill pattern
        case 1: v = edge ? 2 : 1; break;
        case 2: v = edge ? 2 : 3; break;
        case 3: v = edge ? 5 : 4; break;
        case 4: v = 8; break;
        case 5: v = 0x80 | (edge ? 5 : 4); break; // EXTBG high priority
        }
        W(0x19, v);
      }
  }
  // tilemap (low bytes): VMAIN = 0x00 (increment after $2118), write only $2118
  W(0x15, 0x00);
  set_vaddr(0);
  for (int ty = 0; ty < 128; ty++)
    for (int tx = 0; tx < 128; tx++) {
      unsigned t = (((tx >> 2) ^ (ty >> 2)) & 1) ? 1 : 2;
      if (tx >= 40 && tx < 88 && ty >= 40 && ty < 88 && glyph_bit('F', (ty - 40) / 6, (tx - 40) / 6))
        t = extbg_f ? 5 : 3;
      if (tx < 2 || ty < 2 || tx >= 126 || ty >= 126) t = 4;
      W(0x18, t);
    }
  set_color(0, rgb(8, 8, 16));
  set_color(1, rgb(26, 26, 26)); set_color(2, rgb(14, 14, 14)); set_color(3, rgb(8, 8, 8));
  set_color(4, rgb(28, 4, 4));   set_color(5, rgb(16, 2, 2));
  set_color(6, rgb(24, 4, 24));  set_color(7, rgb(12, 2, 12));
  set_color(8, rgb(31, 28, 4));
  upload_obj_palettes();
  upload_obj_chars();
  oam_clear();
  oam_upload();
  W(0x01, 0x03);
  W(0x05, 0x07);
  W(0x2C, 0x11);
  W(0x00, 0x0F);
}

static void set_m7_matrix(double a, double b, double c, double d) {
  W16(0x1B, (int)lround(a * 256)); W16(0x1C, (int)lround(b * 256));
  W16(0x1D, (int)lround(c * 256)); W16(0x1E, (int)lround(d * 256));
}
static void set_m7_center(int cx, int cy, int hofs, int vofs) {
  W16(0x1F, cx & 0x1fff); W16(0x20, cy & 0x1fff);
  W16(0x0D, hofs & 0x1fff); W16(0x0E, vofs & 0x1fff);
}

static void m7_rot_line(int y) {
  if (y == 113) W(0x1A, 0xC0); // bottom half: screen over = tile 0 fill
}

static void m7_persp_line(int y) {
  const int hor = 64;
  if (y < 80) { W(0x2C, 0x10); return; }
  W(0x2C, 0x11);
  const double th = 20.0 * 3.14159265358979 / 180.0;
  const double z = 10240.0 / (y - hor);
  const double f = 128.0;
  set_m7_matrix(z / f * cos(th), z * sin(th) / y, z / f * sin(th), -z * cos(th) / y);
}

static void test_mode7(void) {
  // --- rotation 30 deg, 4.5 map px per screen px; top half transparent, bottom tile-0 fill ---
  setup_mode7(0);
  {
    const double th = 30.0 * 3.14159265358979 / 180.0, s = 4.5;
    set_m7_matrix(cos(th) * s, sin(th) * s, -sin(th) * s, cos(th) * s);
    set_m7_center(512, 512, 512 - 128, 512 - 112);
    W(0x1A, 0x80); // screen over: transparent
    oam_set(0, 120, 104, 0x02, ATTR(0, 3, 0, 0), 1);
    oam_upload();
  }
  render_frame(m7_rot_line);
  save_ppm("m7_rot");

  // --- perspective floor with per-line matrix, wrap ---
  setup_mode7(0);
  set_color(0, rgb(12, 20, 31));
  W(0x1A, 0x00);
  set_m7_center(512, 900, 512 - 128, 900);
  render_frame(m7_persp_line);
  save_ppm("m7_persp");

  // --- identity + EXTBG: BG2 high-priority pixels in front of prio-1 sprites ---
  setup_mode7(1);
  set_m7_matrix(1, 0, 0, 1);
  set_m7_center(0, 0, 300, 300);
  W(0x33, 0x40);  // EXTBG
  W(0x2C, 0x12);  // BG2 + OBJ
  oam_set(0, 60, 100, 0x02, ATTR(0, 1, 0, 0), 1);  // straddles F vertical bar (x=68)
  oam_set(1, 150, 60, 0x02, ATTR(0, 1, 0, 0), 1);  // straddles bottom edge of top bar (y=67)
  oam_set(2, 180, 30, 0x02, ATTR(1, 3, 0, 0), 1);  // prio 3: in front
  oam_upload();
  render_frame(NULL);
  save_ppm("m7_extbg");

  // --- identity, X+Y flip ---
  setup_mode7(0);
  set_m7_matrix(1, 0, 0, 1);
  set_m7_center(0, 0, 300, 300);
  W(0x1A, 0x03);
  render_frame(NULL);
  save_ppm("m7_flip");
  // (160,100): F stem interior, pixel value 0x04 -> CGRAM[4]
  CHECK_EQ(fb_px(160, 100), c15to32(rgb(28, 4, 4)));
  CHECK_EQ(fb_px(140 - 3, 100) == c15to32(rgb(28, 4, 4)), 0); // left of the mirrored stem
  // direct color: pixel 0x04 (char 3 interior) -> BBGGGRRR = 00 000 100 -> R=4<<2=16
  W(0x30, 0x01);
  render_frame(NULL);
  save_ppm("m7_direct");
  CHECK_EQ(fb_px(160, 100), c15to32(rgb(16, 0, 0))); // direct color 0x04 = R 4<<2
}

/* ------------------------------------------------------------------------ */
/* Save state + timing                                                       */
/* ------------------------------------------------------------------------ */

static void test_state_and_timing(void) {
  setup_mode1();
  const size_t sz = ppu_save_state(NULL);
  uint8_t *st = (uint8_t *)malloc(sz);
  uint32_t *a = (uint32_t *)malloc(256 * 224 * 4);
  CHECK_EQ(ppu_save_state(st), sz);
  render_frame(NULL);
  memcpy(a, ppu_framebuffer(), 256 * 224 * 4);
  W(0x00, 0x05); W(0x0D, 0x33); W(0x0D, 0x00); set_color(0, 0);
  render_frame(NULL);
  ppu_load_state(st);
  render_frame(NULL);
  CHECK_EQ(memcmp(a, ppu_framebuffer(), 256 * 224 * 4), 0);
  printf("save state size: %zu bytes\n", sz);
  free(st);
  free(a);

  struct timespec t0, t1;
  const int N = 500;
  setup_mode1();
  W(0x2D, 0x02); W(0x30, 0x02); W(0x31, 0x51); // with color math
  timespec_get(&t0, TIME_UTC);
  for (int i = 0; i < N; i++) render_frame(NULL);
  timespec_get(&t1, TIME_UTC);
  const double ms1 = ((t1.tv_sec - t0.tv_sec) * 1e3 + (t1.tv_nsec - t0.tv_nsec) / 1e6) / N;
  setup_mode7(0);
  set_m7_matrix(0.8, 0.5, -0.5, 0.8);
  timespec_get(&t0, TIME_UTC);
  for (int i = 0; i < N; i++) render_frame(NULL);
  timespec_get(&t1, TIME_UTC);
  const double ms7 = ((t1.tv_sec - t0.tv_sec) * 1e3 + (t1.tv_nsec - t0.tv_nsec) / 1e6) / N;
  printf("timing: mode1 scene %.3f ms/frame, mode7 %.3f ms/frame\n", ms1, ms7);
}

int main(int argc, char **argv) {
  if (argc > 1) g_out = argv[1];
  test_registers();
  test_mode1_scenes();
  test_obj();
  test_mode7();
  test_state_and_timing();
  printf("%d checks, %d failures\n", g_checks, g_fail);
  return g_fail ? 1 : 0;
}
