// SNES PPU (S-PPU1/S-PPU2) — scanline renderer for the SMW native port.
// Pure C17, libc only.
//
// Semantics follow bsnes/higan (accuracy + "fast" PPU), anomie's docs and
// fullsnes:
//  * register latches ($210D-$2114 BG scroll latch, mode-7 latch, OAM low-table
//    write latch, CGRAM byte latch, VRAM read prefetch, OPHCT/OPVCT flip-flops)
//  * BG modes 0-7 (incl. offset-per-tile for 2/4/6, mode 5/6 hires rendered at
//    256 width by sampling the odd ("main") hires pixel), 8x8/16x16 tiles,
//    mosaic, direct color, mode-7 matrix/screen-over/flip/EXTBG
//  * sprites: range (32) / time (34 tiles) limits, OAM priority rotation,
//    lower OAM index wins between sprites, 9-bit X, all 8 OBSEL size pairs
//  * windows, main/sub screens, color math, master brightness.
//
// Coordinates: visible line y (1..224) is the PPU V counter.  BG row fetched
// for line y is (y + VOFS) (so with VOFS=0 the first visible line shows BG row
// 1, like hardware); sprites are range-evaluated on line y-1 (so OAM Y=0 shows
// on framebuffer row 0).  Mode 7 uses y directly as hardware does.
//
// Memories are stored as little-endian 16-bit words, and exposed as byte
// arrays through ppu_vram()/ppu_cgram() (byte 2n = low byte of word n).
// This assumes a little-endian host (x86/x64/ARM LE).

#include "ppu.h"
#include <string.h>

/* ------------------------------------------------------------------------ */
/* State                                                                     */
/* ------------------------------------------------------------------------ */

typedef struct PpuState {
  uint16_t vram[0x8000];   // 32K words
  uint16_t cgram[256];     // 15-bit BGR
  uint8_t  oam[544];       // 512 low table + 32 high table

  // $2100-$2133 register images
  uint8_t  inidisp;        // $2100
  uint8_t  obsel;          // $2101
  uint8_t  bgmode;         // $2105
  uint8_t  mosaic;         // $2106
  uint8_t  bgsc[4];        // $2107-$210A
  uint8_t  bgnba[2];       // $210B-$210C
  uint16_t hofs[4];        // BGnHOFS (16-bit latch image; low 10 bits used)
  uint16_t vofs[4];        // BGnVOFS (16-bit latch image; low 10 bits used)
  uint8_t  bgofs_latch;    // shared BG scroll "previous byte"
  uint8_t  m7_latch;       // shared mode-7 "previous byte"
  uint16_t m7hofs, m7vofs; // M7HOFS/M7VOFS (13 bit signed, stored raw)
  uint8_t  vmain;          // $2115
  uint16_t vram_addr;      // VMADD (word address, unremapped)
  uint16_t vram_latch;     // VRAM read prefetch buffer
  uint8_t  m7sel;          // $211A
  uint16_t m7a, m7b, m7c, m7d, m7x, m7y;
  uint8_t  cgadd;          // CGRAM word address
  uint8_t  cg_flip;        // CGRAM low/high byte flip-flop
  uint8_t  cg_latch;       // CGRAM low byte write buffer
  uint8_t  w12sel, w34sel, wobjsel;
  uint8_t  wh0, wh1, wh2, wh3;
  uint8_t  wbglog, wobjlog;
  uint8_t  tm, ts, tmw, tsw;
  uint8_t  cgwsel, cgadsub;
  uint8_t  fixed_r, fixed_g, fixed_b;
  uint8_t  setini;

  // OAM addressing
  uint16_t oam_base;       // OAMADD (9-bit word address)
  uint16_t oam_addr;       // internal 10-bit byte address
  uint8_t  oam_prio;       // $2103 bit 7 (priority rotation)
  uint8_t  oam_latch;      // low-table write buffer
  uint8_t  first_sprite;

  // Status / read side
  uint8_t  ppu1_mdr, ppu2_mdr;
  uint16_t hcount, vcount; // latched counters
  uint8_t  hflipflop, vflipflop, counters_latched;
  uint8_t  time_over, range_over;
  uint8_t  field;

  // Vertical mosaic counter
  uint8_t  mosaic_restart;
  uint8_t  mosaic_cnt;
  uint16_t mosaic_voff;
  int16_t  cur_line;
} PpuState;

static PpuState S;
static uint32_t s_fb[PPU_WIDTH * PPU_HEIGHT];

/* Per-line scratch (not part of the persistent state). */
static uint16_t s_main_c[256], s_sub_c[256];
static uint8_t  s_main_z[256], s_sub_z[256], s_main_m[256];
static uint16_t s_raw_c[512];
static uint8_t  s_raw_z[512];
static uint16_t s_lay_c[256];
static uint8_t  s_lay_z[256];
static uint8_t  s_m7pix[256];
static uint8_t  s_obj_z[256], s_obj_pal[256];
static uint8_t  s_win[256], s_colwin[256];

/* ------------------------------------------------------------------------ */
/* Helpers                                                                   */
/* ------------------------------------------------------------------------ */

// s_expand[b]: byte lane i (LSB first) = bit (7-i) of b, i.e. pixel i of a
// bitplane byte, left to right.  Built lazily (no global constructors).
static uint64_t s_expand[256];
static int s_tables_ready;

static void init_tables(void) {
  for (unsigned b = 0; b < 256; b++) {
    uint64_t v = 0;
    for (unsigned i = 0; i < 8; i++)
      if (b & (0x80u >> i)) v |= (uint64_t)1 << (8 * i);
    s_expand[b] = v;
  }
  s_tables_ready = 1;
}

// 8 pixels of one tile row from 1/2/4 bitplane words (each word = 2 planes).
static inline uint64_t decode_row(const uint16_t *w, int nwords) {
  uint64_t px = s_expand[w[0] & 0xff] | (s_expand[w[0] >> 8] << 1);
  if (nwords > 1) px |= (s_expand[w[1] & 0xff] << 2) | (s_expand[w[1] >> 8] << 3);
  if (nwords > 2) {
    px |= (s_expand[w[2] & 0xff] << 4) | (s_expand[w[2] >> 8] << 5);
    px |= (s_expand[w[3] & 0xff] << 6) | (s_expand[w[3] >> 8] << 7);
  }
  return px;
}

static inline int sext13(uint16_t v) { return (int)(int16_t)(uint16_t)(v << 3) >> 3; }

static inline uint16_t vram_remap(uint16_t a) {
  switch ((S.vmain >> 2) & 3) {
  default:
  case 0: return a;
  case 1: return (uint16_t)((a & 0xff00) | ((a << 3) & 0x00f8) | ((a >> 5) & 7));
  case 2: return (uint16_t)((a & 0xfe00) | ((a << 3) & 0x01f8) | ((a >> 6) & 7));
  case 3: return (uint16_t)((a & 0xfc00) | ((a << 3) & 0x03f8) | ((a >> 7) & 7));
  }
}

static inline unsigned vram_incr(void) {
  static const uint8_t inc[4] = {1, 32, 128, 128};
  return inc[S.vmain & 3];
}

static void oam_set_first_sprite(void) {
  S.first_sprite = S.oam_prio ? (uint8_t)((S.oam_addr >> 2) & 127) : 0;
}

static void oam_addr_reset(void) {
  S.oam_addr = (uint16_t)((S.oam_base << 1) & 0x3ff);
  oam_set_first_sprite();
}

static inline uint16_t direct_color(unsigned p, unsigned t) {
  // p = BBGGGRRR, t = tilemap entry (palette bits 10-12 = r,g,b low bits)
  return (uint16_t)(((p << 7) & 0x6000) + (t & 0x1000) +
                    ((p << 4) & 0x0380) + ((t >> 5) & 0x0040) +
                    ((p << 2) & 0x001c) + ((t >> 9) & 0x0002));
}

static inline uint16_t blend(uint16_t a, uint16_t b, int subtract, int half) {
  int ar = a & 31, ag = (a >> 5) & 31, ab = (a >> 10) & 31;
  int br = b & 31, bg = (b >> 5) & 31, bb = (b >> 10) & 31;
  int r, g, bl;
  if (!subtract) {
    r = ar + br; g = ag + bg; bl = ab + bb;
    if (half) { r >>= 1; g >>= 1; bl >>= 1; }
    else { if (r > 31) r = 31; if (g > 31) g = 31; if (bl > 31) bl = 31; }
  } else {
    r = ar - br; g = ag - bg; bl = ab - bb;
    if (r < 0) r = 0;
    if (g < 0) g = 0;
    if (bl < 0) bl = 0;
    if (half) { r >>= 1; g >>= 1; bl >>= 1; }
  }
  return (uint16_t)(r | (g << 5) | (bl << 10));
}

// Window evaluation for one layer.  sel: bit0 W1 invert, bit1 W1 enable,
// bit2 W2 invert, bit3 W2 enable.  logic: 0 OR, 1 AND, 2 XOR, 3 XNOR.
// out[x] = 1 when x is "inside" the (combined) window.
static void calc_window(unsigned sel, unsigned logic, uint8_t *out) {
  const int e1 = (sel & 2) != 0, e2 = (sel & 8) != 0;
  const unsigned i1 = sel & 1, i2 = (sel >> 2) & 1;
  const unsigned l1 = S.wh0, r1 = S.wh1, l2 = S.wh2, r2 = S.wh3;
  if (!e1 && !e2) { memset(out, 0, 256); return; }
  if (e1 && !e2) {
    for (unsigned x = 0; x < 256; x++) out[x] = (uint8_t)((x >= l1 && x <= r1) ^ i1);
    return;
  }
  if (e2 && !e1) {
    for (unsigned x = 0; x < 256; x++) out[x] = (uint8_t)((x >= l2 && x <= r2) ^ i2);
    return;
  }
  for (unsigned x = 0; x < 256; x++) {
    unsigned w1 = (x >= l1 && x <= r1) ^ i1;
    unsigned w2 = (x >= l2 && x <= r2) ^ i2;
    unsigned w;
    switch (logic & 3) {
    default:
    case 0: w = w1 | w2; break;
    case 1: w = w1 & w2; break;
    case 2: w = w1 ^ w2; break;
    case 3: w = !(w1 ^ w2); break;
    }
    out[x] = (uint8_t)w;
  }
}

static void layer_window(int layer, uint8_t *out) {
  unsigned sel, logic;
  switch (layer) {
  case 0: sel = S.w12sel & 15; logic = S.wbglog & 3; break;
  case 1: sel = S.w12sel >> 4; logic = (S.wbglog >> 2) & 3; break;
  case 2: sel = S.w34sel & 15; logic = (S.wbglog >> 4) & 3; break;
  case 3: sel = S.w34sel >> 4; logic = (S.wbglog >> 6) & 3; break;
  default: sel = S.wobjsel & 15; logic = S.wobjlog & 3; break; // OBJ
  }
  calc_window(sel, logic, out);
}

// Merge one layer's line (color + z, z=0 transparent) into main/sub screens.
// layer 0..3 = BG1..4, 4 = OBJ.  math: per-layer color math enable (BG) or
// per-pixel flags array (OBJ).
static void plot_layer(int layer, const uint16_t *c, const uint8_t *z,
                       int math, const uint8_t *math_px) {
  const unsigned bit = 1u << layer;
  const int on_main = (S.tm & bit) != 0, on_sub = (S.ts & bit) != 0;
  const int wm = on_main && (S.tmw & bit), ws = on_sub && (S.tsw & bit);
  if (!on_main && !on_sub) return;
  // The same layer window masks the main screen (TMW) and/or the subscreen (TSW).
  if (wm || ws) layer_window(layer, s_win);
  for (int x = 0; x < 256; x++) {
    const uint8_t pz = z[x];
    if (!pz) continue;
    if (on_main && pz > s_main_z[x] && !(wm && s_win[x])) {
      s_main_z[x] = pz;
      s_main_c[x] = c[x];
      s_main_m[x] = (uint8_t)(math_px ? math_px[x] : math);
    }
    if (on_sub && pz > s_sub_z[x] && !(ws && s_win[x])) {
      s_sub_z[x] = pz;
      s_sub_c[x] = c[x];
    }
  }
}

/* ------------------------------------------------------------------------ */
/* Background rendering                                                      */
/* ------------------------------------------------------------------------ */

// Priority (z) values, higher = in front.  Sprites: S0=3 S1=6 S2=9 S3=12.
static const uint8_t k_obj_z[4] = {3, 6, 9, 12};
static const uint8_t k_bg_z[8][4][2] = {
  /* 0: S3 1H 2H S2 1L 2L S1 3H 4H S0 3L 4L */ {{8, 11}, {7, 10}, {2, 5}, {1, 4}},
  /* 1: S3 1H 2H S2 1L 2L S1 3H S0 3L      */ {{8, 11}, {7, 10}, {2, 5}, {0, 0}},
  /* 2: S3 1H S2 2H S1 1L S0 2L            */ {{5, 11}, {2, 8}, {0, 0}, {0, 0}},
  /* 3 */                                      {{5, 11}, {2, 8}, {0, 0}, {0, 0}},
  /* 4 */                                      {{5, 11}, {2, 8}, {0, 0}, {0, 0}},
  /* 5 */                                      {{5, 11}, {2, 8}, {0, 0}, {0, 0}},
  /* 6: S3 1H S2 S1 1L S0                  */ {{5, 11}, {0, 0}, {0, 0}, {0, 0}},
  /* 7: S3 S2 (2H) S1 1 S0 (2L)            */ {{5, 5}, {2, 7}, {0, 0}, {0, 0}},
};

static inline unsigned mosaic_size(void) { return (unsigned)(S.mosaic >> 4) + 1; }
static inline int mosaic_on(int bg) { return ((S.mosaic >> bg) & 1) && (S.mosaic >> 4); }

static inline unsigned bg_chr_base(int bg) {
  unsigned nib = (bg < 2) ? (S.bgnba[0] >> (bg * 4)) : (S.bgnba[1] >> ((bg - 2) * 4));
  return ((nib & 15u) << 12) & 0x7fff;
}

static inline uint16_t bg_get_tile(int bg, unsigned hoff, unsigned voff,
                                   unsigned tw_shift, unsigned th_shift) {
  const uint8_t sc = S.bgsc[bg];
  const unsigned base = (unsigned)(sc & 0x7c) << 8;
  const unsigned tx = hoff >> tw_shift, ty = voff >> th_shift;
  unsigned off = ((ty & 0x1f) << 5) | (tx & 0x1f);
  if ((tx & 0x20) && (sc & 1)) off += 0x400;
  if ((ty & 0x20) && (sc & 2)) off += (sc & 1) ? 0x800 : 0x400;
  return S.vram[(base + off) & 0x7fff];
}

// depth: 0 = 2bpp, 1 = 4bpp, 2 = 8bpp
static void render_bg(int bg, int depth, int line) {
  const int mode = S.bgmode & 7;
  const int hires = (mode == 5 || mode == 6);
  const int opt = (mode == 2 || mode == 4 || mode == 6);
  const int direct = (S.cgwsel & 1) && bg == 0 && (mode == 3 || mode == 4);
  const int width = hires ? 512 : 256;
  const unsigned tsz = (S.bgmode >> (4 + bg)) & 1;
  const unsigned th_shift = 3 + tsz;
  const unsigned tw_shift = hires ? 4 : th_shift;
  const uint8_t sc = S.bgsc[bg];
  const unsigned hmask = ((32u << tw_shift) << (sc & 1)) - 1;
  const unsigned vmask = ((32u << th_shift) << ((sc >> 1) & 1)) - 1;
  const unsigned chr_index = bg_chr_base(bg) >> (3 + depth);
  const unsigned tile_mask = 0x0fffu >> depth;
  const unsigned color_shift = 3 + (unsigned)depth;
  const unsigned pal_base = (mode == 0) ? (unsigned)bg << 5 : 0;
  const unsigned pal_shift = 2u << depth;
  const int mos = mosaic_on(bg);
  const uint8_t *zt = k_bg_z[mode][bg];
  uint8_t z2[2] = {zt[0], zt[1]};
  unsigned hscroll = S.hofs[bg], vscroll = S.vofs[bg];
  unsigned y = mos ? S.mosaic_voff : (unsigned)line;
  // BG3 parameters for offset-per-tile lookups
  const unsigned b3_tsz = (S.bgmode >> 6) & 1;
  const unsigned b3_th = 3 + b3_tsz, b3_tw = hires ? 4 : b3_th;

  if (mode == 1 && bg == 2 && (S.bgmode & 8)) z2[1] = 13;
  if (hires) hscroll <<= 1;

  memset(s_raw_z, 0, (size_t)width);

  int x = -(int)(hscroll & 7);
  while (x < width) {
    unsigned hoff = (unsigned)x + hscroll;
    unsigned voff = y + vscroll;
    if (opt) {
      const unsigned offx = (unsigned)x + (hscroll & 7);
      if (offx >= 8) { // the first column is exempt
        const unsigned valid = 0x2000u << bg;
        const unsigned lx = (offx - 8) + (S.hofs[2] & ~7u);
        const uint16_t hl = bg_get_tile(2, lx, S.vofs[2], b3_tw, b3_th);
        if (mode == 4) {
          if (hl & valid) {
            if (!(hl & 0x8000)) hoff = offx + (hl & ~7u);
            else voff = y + hl;
          }
        } else {
          const uint16_t vl = bg_get_tile(2, lx, S.vofs[2] + 8u, b3_tw, b3_th);
          if (hl & valid) hoff = offx + (hl & ~7u);
          if (vl & valid) voff = y + vl;
        }
      }
    }
    hoff &= hmask;
    voff &= vmask;

    const uint16_t entry = bg_get_tile(bg, hoff, voff, tw_shift, th_shift);
    const unsigned mirror_y = (entry & 0x8000) ? 7 : 0;
    const int mirror_x = (entry & 0x4000) != 0;
    const uint8_t z = z2[(entry >> 13) & 1];
    const unsigned pal = (pal_base + (((unsigned)(entry >> 10) & 7) << pal_shift)) & 0xff;
    unsigned tn = entry;
    if (tw_shift == 4 && (((hoff & 8) != 0) ^ mirror_x)) tn += 1;
    if (th_shift == 4 && (((voff & 8) != 0) ^ (mirror_y != 0))) tn += 16;
    tn = ((tn & 0x3ff) + chr_index) & tile_mask;
    const unsigned addr = ((tn << color_shift) + ((voff & 7) ^ mirror_y)) & 0x7fff;

    uint16_t w[4];
    w[0] = S.vram[addr];
    w[1] = S.vram[(addr + 8) & 0x7fff];
    w[2] = S.vram[(addr + 16) & 0x7fff];
    w[3] = S.vram[(addr + 24) & 0x7fff];
    const uint64_t row = decode_row(w, 1 << depth);
    if (!row) { x += 8; continue; } // fully transparent row

    for (int i = 0; i < 8; i++, x++) {
      if ((unsigned)x >= (unsigned)width) continue;
      const unsigned px = (unsigned)(row >> (8 * (mirror_x ? 7 - i : i))) & 0xff;
      if (!px) continue;
      s_raw_z[x] = z;
      s_raw_c[x] = direct ? direct_color(px, entry) : S.cgram[(pal + px) & 0xff];
    }
  }

  const uint16_t *oc = s_raw_c;
  const uint8_t *oz = s_raw_z;
  if (hires || mos) {
    const unsigned msz = mosaic_size();
    for (unsigned X = 0; X < 256; X++) {
      unsigned sx = mos ? X - X % msz : X;
      if (hires) sx = sx * 2 + 1;
      s_lay_z[X] = s_raw_z[sx];
      s_lay_c[X] = s_raw_c[sx];
    }
    oc = s_lay_c;
    oz = s_lay_z;
  }
  plot_layer(bg, oc, oz, (S.cgadsub >> bg) & 1, NULL);
}

static inline int m7_clip(int n) { return (n & 0x2000) ? (n | ~1023) : (n & 1023); }

static void render_mode7(int line) {
  const int want1 = ((S.tm | S.ts) & 1) != 0;
  const int want2 = ((S.tm | S.ts) & 2) && (S.setini & 0x40);
  if (!want1 && !want2) return;

  const int a = (int16_t)S.m7a, b = (int16_t)S.m7b;
  const int c = (int16_t)S.m7c, d = (int16_t)S.m7d;
  const int hofs = sext13(S.m7hofs), vofs = sext13(S.m7vofs);
  const int cx = sext13(S.m7x), cy = sext13(S.m7y);
  const unsigned rep = S.m7sel >> 6;
  int yy = mosaic_on(0) ? S.mosaic_voff : line; // BG2 also uses BG1's vertical mosaic
  if (S.m7sel & 2) yy = 255 - yy;

  const int hx = m7_clip(hofs - cx), vy = m7_clip(vofs - cy);
  const int ox = ((a * hx) & ~63) + ((b * vy) & ~63) + ((b * yy) & ~63) + cx * 256;
  const int oy = ((c * hx) & ~63) + ((d * vy) & ~63) + ((d * yy) & ~63) + cy * 256;

  for (int X = 0; X < 256; X++) {
    const int xx = (S.m7sel & 1) ? 255 - X : X;
    const int px = (ox + a * xx) >> 8;
    const int py = (oy + c * xx) >> 8;
    const int oob = ((px | py) & ~1023) != 0;
    uint8_t pix;
    if (oob && rep == 2) {
      pix = 0;
    } else {
      unsigned tile;
      if (oob && rep == 3) tile = 0;
      else tile = S.vram[(((unsigned)py >> 3 & 127) << 7) | ((unsigned)px >> 3 & 127)] & 0xff;
      pix = (uint8_t)(S.vram[(tile << 6) | (((unsigned)py & 7) << 3) | ((unsigned)px & 7)] >> 8);
    }
    s_m7pix[X] = pix;
  }

  const unsigned msz = mosaic_size();
  if (want1) {
    const int mos = mosaic_on(0);
    const int direct = S.cgwsel & 1;
    for (unsigned X = 0; X < 256; X++) {
      const uint8_t p = s_m7pix[mos ? X - X % msz : X];
      s_lay_z[X] = p ? 5 : 0;
      s_lay_c[X] = direct ? direct_color(p, 0) : S.cgram[p];
    }
    plot_layer(0, s_lay_c, s_lay_z, S.cgadsub & 1, NULL);
  }
  if (want2) {
    const int mos = mosaic_on(1);
    for (unsigned X = 0; X < 256; X++) {
      const uint8_t p = s_m7pix[mos ? X - X % msz : X];
      s_lay_z[X] = (p & 0x7f) ? ((p & 0x80) ? 7 : 2) : 0;
      s_lay_c[X] = S.cgram[p & 0x7f];
    }
    plot_layer(1, s_lay_c, s_lay_z, (S.cgadsub >> 1) & 1, NULL);
  }
}

/* ------------------------------------------------------------------------ */
/* Sprites                                                                   */
/* ------------------------------------------------------------------------ */

static const uint8_t k_obj_w[2][8] = {{8, 8, 8, 16, 16, 32, 16, 16}, {16, 32, 64, 32, 64, 64, 32, 32}};
static const uint8_t k_obj_h[2][8] = {{8, 8, 8, 16, 16, 32, 32, 32}, {16, 32, 64, 32, 64, 64, 64, 32}};

typedef struct ObjTile {
  int16_t  x;        // 9-bit screen x
  uint16_t pl[2];    // bitplanes (0/1, 2/3)
  uint8_t  pal;      // palette 0..7
  uint8_t  prio;     // 0..3
  uint8_t  hflip;
} ObjTile;

// Evaluate + fetch + draw sprites for visible line `line` into s_obj_z/s_obj_pal.
static void render_obj(int line) {
  const int ty = line - 1; // sprites shown on line y were evaluated on line y-1
  const unsigned bs = S.obsel >> 5;
  uint8_t items[32];
  int nitems = 0;

  memset(s_obj_z, 0, sizeof s_obj_z);

  for (int n = 0; n < 128; n++) {
    const unsigned idx = (S.first_sprite + n) & 127;
    const uint8_t *o = &S.oam[idx * 4];
    const unsigned hi = S.oam[512 + (idx >> 2)] >> ((idx & 3) * 2);
    const int x = o[0] | ((hi & 1) << 8);
    const int large = (hi >> 1) & 1;
    const int w = k_obj_w[large][bs], h = k_obj_h[large][bs];
    const int sy = o[1];
    if (x > 256 && x + w - 1 < 512) continue;
    if (!((ty >= sy && ty < sy + h) || (sy + h >= 256 && ty < ((sy + h) & 255)))) continue;
    if (nitems >= 32) { S.range_over = 1; break; }
    items[nitems++] = (uint8_t)idx;
  }

  ObjTile tiles[34];
  int ntiles = 0;
  const unsigned tdbase = (unsigned)(S.obsel & 7) << 13;
  const unsigned nsel = (S.obsel >> 3) & 3;

  for (int i = nitems - 1; i >= 0; i--) {
    const unsigned idx = items[i];
    const uint8_t *o = &S.oam[idx * 4];
    const unsigned hi = S.oam[512 + (idx >> 2)] >> ((idx & 3) * 2);
    const int x = o[0] | ((hi & 1) << 8);
    const int large = (hi >> 1) & 1;
    const int w = k_obj_w[large][bs], h = k_obj_h[large][bs];
    const unsigned attr = o[3];
    const int vflip = (attr >> 7) & 1, hflip = (attr >> 6) & 1;
    int row = (ty - o[1]) & 0xff;
    if (vflip) {
      if (w == h) row = h - 1 - row;
      else if (row < w) row = w - 1 - row;
      else row = w + (w - 1) - (row - w);
    }
    row &= 255;
    unsigned base = tdbase;
    if (attr & 1) base += (1 + nsel) << 12;
    const unsigned chrx = o[2] & 15;
    const unsigned chry = ((((unsigned)o[2] >> 4) + ((unsigned)row >> 3)) & 15) << 4;
    const int tw = w >> 3;
    for (int tx = 0; tx < tw; tx++) {
      const unsigned sx = (unsigned)(x + (tx << 3)) & 511;
      if (x != 256 && sx >= 256 && sx + 7 < 512) continue;
      if (ntiles >= 34) { S.time_over = 1; goto fetched; }
      const unsigned mx = hflip ? (unsigned)(tw - 1 - tx) : (unsigned)tx;
      const unsigned pos = (base + ((chry + ((chrx + mx) & 15)) << 4)) & 0xffff;
      const unsigned addr = (pos & 0x7ff0) + ((unsigned)row & 7);
      ObjTile *t = &tiles[ntiles++];
      t->x = (int16_t)sx;
      t->pl[0] = S.vram[addr];
      t->pl[1] = S.vram[(addr + 8) & 0x7fff];
      t->pal = (uint8_t)((attr >> 1) & 7);
      t->prio = (uint8_t)((attr >> 4) & 3);
      t->hflip = (uint8_t)hflip;
    }
  }
fetched:
  // Later tiles (lower OAM index in evaluation order) overwrite earlier ones.
  for (int i = 0; i < ntiles; i++) {
    const ObjTile *t = &tiles[i];
    const uint64_t row = decode_row(t->pl, 2);
    if (!row) continue;
    int sx = t->x;
    if (sx >= 256) sx -= 512;
    const uint8_t z = k_obj_z[t->prio];
    const uint8_t palbase = (uint8_t)(128 + (t->pal << 4));
    for (int p = 0; p < 8; p++) {
      const int X = sx + p;
      if (X < 0 || X >= 256) continue;
      const unsigned col = (unsigned)(row >> (8 * (t->hflip ? 7 - p : p))) & 0xff;
      if (!col) continue;
      s_obj_z[X] = z;
      s_obj_pal[X] = (uint8_t)(palbase + col);
    }
  }
}

/* ------------------------------------------------------------------------ */
/* Public API                                                                */
/* ------------------------------------------------------------------------ */

void ppu_reset(void) {
  // Registers only; VRAM/CGRAM/OAM keep their contents (as on hardware reset).
  const size_t mem = offsetof(PpuState, inidisp);
  memset((uint8_t *)&S + mem, 0, sizeof S - mem);
  S.inidisp = 0x80; // forced blank, brightness 0
  S.mosaic_cnt = 1;
  S.mosaic_voff = 1;
}

void ppu_init(void) {
  if (!s_tables_ready) init_tables();
  memset(&S, 0, sizeof S);
  memset(s_fb, 0, sizeof s_fb);
  ppu_reset();
}

void ppu_write(uint8_t reg, uint8_t v) {
  switch (reg) {
  case 0x00: S.inidisp = v; break;
  case 0x01: S.obsel = v; break;
  case 0x02:
    S.oam_base = (uint16_t)((S.oam_base & 0x100) | v);
    oam_addr_reset();
    break;
  case 0x03:
    S.oam_base = (uint16_t)(((v & 1) << 8) | (S.oam_base & 0xff));
    S.oam_prio = v >> 7;
    oam_addr_reset();
    break;
  case 0x04: {
    const unsigned a = S.oam_addr;
    S.oam_addr = (uint16_t)((a + 1) & 0x3ff);
    if (!(a & 1)) S.oam_latch = v;
    if (a & 0x200) {
      S.oam[0x200 | (a & 0x1f)] = v;
    } else if (a & 1) {
      S.oam[(a & 0x1fe) + 0] = S.oam_latch;
      S.oam[(a & 0x1fe) + 1] = v;
    }
    oam_set_first_sprite();
    break;
  }
  case 0x05: S.bgmode = v; break;
  case 0x06:
    S.mosaic = v;
    S.mosaic_restart = 1; // vertical mosaic counter restarts on the next line
    break;
  case 0x07: case 0x08: case 0x09: case 0x0A: S.bgsc[reg - 0x07] = v; break;
  case 0x0B: S.bgnba[0] = v; break;
  case 0x0C: S.bgnba[1] = v; break;
  case 0x0D: case 0x0F: case 0x11: case 0x13: {
    const int bg = (reg - 0x0D) >> 1;
    if (reg == 0x0D) {
      S.m7hofs = (uint16_t)((v << 8) | S.m7_latch);
      S.m7_latch = v;
    }
    // Stored as 16 bits like bsnes (only the low 10 bits are used for rendering);
    // keeping the upper bits makes (hofs >> 8) & 7 yield the previous low byte's
    // bits 0-2 for the usual low-then-high write sequence.
    S.hofs[bg] = (uint16_t)((v << 8) | (S.bgofs_latch & ~7u) | ((S.hofs[bg] >> 8) & 7));
    S.bgofs_latch = v;
    break;
  }
  case 0x0E: case 0x10: case 0x12: case 0x14: {
    const int bg = (reg - 0x0E) >> 1;
    if (reg == 0x0E) {
      S.m7vofs = (uint16_t)((v << 8) | S.m7_latch);
      S.m7_latch = v;
    }
    S.vofs[bg] = (uint16_t)((v << 8) | S.bgofs_latch);
    S.bgofs_latch = v;
    break;
  }
  case 0x15: S.vmain = v; break;
  case 0x16:
    S.vram_addr = (uint16_t)((S.vram_addr & 0xff00) | v);
    S.vram_latch = S.vram[vram_remap(S.vram_addr) & 0x7fff];
    break;
  case 0x17:
    S.vram_addr = (uint16_t)((v << 8) | (S.vram_addr & 0x00ff));
    S.vram_latch = S.vram[vram_remap(S.vram_addr) & 0x7fff];
    break;
  case 0x18: {
    uint16_t *w = &S.vram[vram_remap(S.vram_addr) & 0x7fff];
    *w = (uint16_t)((*w & 0xff00) | v);
    if (!(S.vmain & 0x80)) S.vram_addr = (uint16_t)(S.vram_addr + vram_incr());
    break;
  }
  case 0x19: {
    uint16_t *w = &S.vram[vram_remap(S.vram_addr) & 0x7fff];
    *w = (uint16_t)((*w & 0x00ff) | (v << 8));
    if (S.vmain & 0x80) S.vram_addr = (uint16_t)(S.vram_addr + vram_incr());
    break;
  }
  case 0x1A: S.m7sel = v; break;
  case 0x1B: S.m7a = (uint16_t)((v << 8) | S.m7_latch); S.m7_latch = v; break;
  case 0x1C: S.m7b = (uint16_t)((v << 8) | S.m7_latch); S.m7_latch = v; break;
  case 0x1D: S.m7c = (uint16_t)((v << 8) | S.m7_latch); S.m7_latch = v; break;
  case 0x1E: S.m7d = (uint16_t)((v << 8) | S.m7_latch); S.m7_latch = v; break;
  case 0x1F: S.m7x = (uint16_t)((v << 8) | S.m7_latch); S.m7_latch = v; break;
  case 0x20: S.m7y = (uint16_t)((v << 8) | S.m7_latch); S.m7_latch = v; break;
  case 0x21: S.cgadd = v; S.cg_flip = 0; break;
  case 0x22:
    if (!S.cg_flip) {
      S.cg_latch = v;
    } else {
      S.cgram[S.cgadd] = (uint16_t)(((v & 0x7f) << 8) | S.cg_latch);
      S.cgadd++;
    }
    S.cg_flip ^= 1;
    break;
  case 0x23: S.w12sel = v; break;
  case 0x24: S.w34sel = v; break;
  case 0x25: S.wobjsel = v; break;
  case 0x26: S.wh0 = v; break;
  case 0x27: S.wh1 = v; break;
  case 0x28: S.wh2 = v; break;
  case 0x29: S.wh3 = v; break;
  case 0x2A: S.wbglog = v; break;
  case 0x2B: S.wobjlog = v; break;
  case 0x2C: S.tm = v; break;
  case 0x2D: S.ts = v; break;
  case 0x2E: S.tmw = v; break;
  case 0x2F: S.tsw = v; break;
  case 0x30: S.cgwsel = v; break;
  case 0x31: S.cgadsub = v; break;
  case 0x32:
    if (v & 0x20) S.fixed_r = v & 0x1f;
    if (v & 0x40) S.fixed_g = v & 0x1f;
    if (v & 0x80) S.fixed_b = v & 0x1f;
    break;
  case 0x33: S.setini = v; break;
  default: break;
  }
}

uint8_t ppu_read(uint8_t reg, uint8_t open_bus) {
  switch (reg) {
  case 0x34: case 0x35: case 0x36: {
    const int32_t r = (int32_t)(int16_t)S.m7a * (int32_t)(int8_t)(S.m7b >> 8);
    S.ppu1_mdr = (uint8_t)((uint32_t)r >> ((reg - 0x34) * 8));
    return S.ppu1_mdr;
  }
  case 0x37: // SLHV: counters are latched by the bus via ppu_latch_hv()
    return open_bus;
  case 0x38: { // OAMDATAREAD
    unsigned a = S.oam_addr;
    S.oam_addr = (uint16_t)((a + 1) & 0x3ff);
    if (a & 0x200) a = 0x200 | (a & 0x1f);
    S.ppu1_mdr = S.oam[a];
    oam_set_first_sprite();
    return S.ppu1_mdr;
  }
  case 0x39: // VMDATALREAD
    S.ppu1_mdr = (uint8_t)(S.vram_latch & 0xff);
    if (!(S.vmain & 0x80)) {
      S.vram_latch = S.vram[vram_remap(S.vram_addr) & 0x7fff];
      S.vram_addr = (uint16_t)(S.vram_addr + vram_incr());
    }
    return S.ppu1_mdr;
  case 0x3A: // VMDATAHREAD
    S.ppu1_mdr = (uint8_t)(S.vram_latch >> 8);
    if (S.vmain & 0x80) {
      S.vram_latch = S.vram[vram_remap(S.vram_addr) & 0x7fff];
      S.vram_addr = (uint16_t)(S.vram_addr + vram_incr());
    }
    return S.ppu1_mdr;
  case 0x3B: // CGDATAREAD
    if (!S.cg_flip) {
      S.ppu2_mdr = (uint8_t)(S.cgram[S.cgadd] & 0xff);
    } else {
      S.ppu2_mdr = (uint8_t)((S.ppu2_mdr & 0x80) | ((S.cgram[S.cgadd] >> 8) & 0x7f));
      S.cgadd++;
    }
    S.cg_flip ^= 1;
    return S.ppu2_mdr;
  case 0x3C: // OPHCT
    if (!S.hflipflop) S.ppu2_mdr = (uint8_t)(S.hcount & 0xff);
    else S.ppu2_mdr = (uint8_t)((S.ppu2_mdr & 0xfe) | ((S.hcount >> 8) & 1));
    S.hflipflop ^= 1;
    return S.ppu2_mdr;
  case 0x3D: // OPVCT
    if (!S.vflipflop) S.ppu2_mdr = (uint8_t)(S.vcount & 0xff);
    else S.ppu2_mdr = (uint8_t)((S.ppu2_mdr & 0xfe) | ((S.vcount >> 8) & 1));
    S.vflipflop ^= 1;
    return S.ppu2_mdr;
  case 0x3E: // STAT77
    S.ppu1_mdr = (uint8_t)((S.time_over << 7) | (S.range_over << 6) | (S.ppu1_mdr & 0x10) | 0x01);
    return S.ppu1_mdr;
  case 0x3F: // STAT78 (NTSC: bit4 = 0)
    S.hflipflop = 0;
    S.vflipflop = 0;
    S.ppu2_mdr = (uint8_t)((S.field << 7) | (S.counters_latched << 6) | (S.ppu2_mdr & 0x20) | 0x03);
    S.counters_latched = 0;
    return S.ppu2_mdr;
  default:
    return open_bus;
  }
}

void ppu_latch_hv(uint16_t h, uint16_t v) {
  S.hcount = h & 0x1ff;
  S.vcount = v & 0x1ff;
  S.counters_latched = 1;
}

void ppu_frame_start(void) {
  S.field ^= 1;
  S.cur_line = 0;
  if (!(S.inidisp & 0x80)) { // flags reset at end of vblank unless in forced blank
    S.time_over = 0;
    S.range_over = 0;
  }
}

void ppu_vblank_start(void) {
  S.cur_line = 225;
  if (!(S.inidisp & 0x80)) oam_addr_reset();
}

void ppu_render_line(int y) {
  if (y < 1 || y > PPU_HEIGHT) return;
  if (!s_tables_ready) init_tables();
  S.cur_line = (int16_t)y;

  // Vertical mosaic counter (runs every line, independent of enables).
  if (y == 1 || S.mosaic_restart) {
    S.mosaic_restart = 0;
    S.mosaic_voff = (uint16_t)y;
    S.mosaic_cnt = (uint8_t)mosaic_size();
  } else if (--S.mosaic_cnt == 0) {
    S.mosaic_cnt = (uint8_t)mosaic_size();
    S.mosaic_voff = (uint16_t)(S.mosaic_voff + mosaic_size());
  }

  uint32_t *out = &s_fb[(y - 1) * PPU_WIDTH];
  if (S.inidisp & 0x80) { // forced blank: black, no sprite evaluation
    memset(out, 0, PPU_WIDTH * sizeof(uint32_t));
    return;
  }

  const int mode = S.bgmode & 7;
  const uint16_t fixed = (uint16_t)(S.fixed_r | (S.fixed_g << 5) | (S.fixed_b << 10));
  {
    const uint16_t back = S.cgram[0];
    const uint8_t back_math = (S.cgadsub >> 5) & 1;
    for (int x = 0; x < 256; x++) {
      s_main_z[x] = 0;
      s_main_c[x] = back;
      s_main_m[x] = back_math;
      s_sub_z[x] = 0;
      s_sub_c[x] = fixed; // transparent subscreen = fixed color
    }
  }

  const unsigned en = S.tm | S.ts;
  switch (mode) {
  case 0:
    for (int bg = 0; bg < 4; bg++) if (en & (1u << bg)) render_bg(bg, 0, y);
    break;
  case 1:
    if (en & 1) render_bg(0, 1, y);
    if (en & 2) render_bg(1, 1, y);
    if (en & 4) render_bg(2, 0, y);
    break;
  case 2:
    if (en & 1) render_bg(0, 1, y);
    if (en & 2) render_bg(1, 1, y);
    break;
  case 3:
    if (en & 1) render_bg(0, 2, y);
    if (en & 2) render_bg(1, 1, y);
    break;
  case 4:
    if (en & 1) render_bg(0, 2, y);
    if (en & 2) render_bg(1, 0, y);
    break;
  case 5:
    if (en & 1) render_bg(0, 1, y);
    if (en & 2) render_bg(1, 0, y);
    break;
  case 6:
    if (en & 1) render_bg(0, 1, y);
    break;
  case 7:
    render_mode7(y);
    break;
  }

  // Sprites are always evaluated (STAT77 flags) even if OBJ is not displayed.
  render_obj(y);
  if (en & 0x10) {
    static uint16_t oc[256];
    static uint8_t om[256];
    const int objmath = (S.cgadsub >> 4) & 1;
    for (int x = 0; x < 256; x++) {
      if (!s_obj_z[x]) continue;
      oc[x] = S.cgram[s_obj_pal[x]];
      om[x] = (uint8_t)(objmath && s_obj_pal[x] >= 192); // only palettes 4-7
    }
    plot_layer(4, oc, s_obj_z, 0, om);
  }

  // Color window + color math + brightness.
  const unsigned clipmode = S.cgwsel >> 6, mathmode = (S.cgwsel >> 4) & 3;
  const int need_cw = (clipmode == 1 || clipmode == 2 || mathmode == 1 || mathmode == 2);
  if (need_cw) calc_window(S.wobjsel >> 4, (S.wobjlog >> 2) & 3, s_colwin);
  const int sub_add = (S.cgwsel & 2) != 0;
  const int subtract = (S.cgadsub & 0x80) != 0;
  const int half = (S.cgadsub & 0x40) != 0;

  uint32_t lut[32];
  {
    const unsigned br = S.inidisp & 15;
    for (unsigned i = 0; i < 32; i++) {
      const unsigned c8 = (i << 3) | (i >> 2);
      lut[i] = br ? c8 * (br + 1) / 16 : 0;
    }
  }

  for (int x = 0; x < 256; x++) {
    const unsigned cw = need_cw ? s_colwin[x] : 0;
    unsigned main_vis, math_ok;
    switch (clipmode) { // 0 never clip, 1 clip outside, 2 clip inside, 3 always
    case 0: main_vis = 1; break;
    case 1: main_vis = cw; break;
    case 2: main_vis = !cw; break;
    default: main_vis = 0; break;
    }
    switch (mathmode) { // 0 always, 1 inside only, 2 outside only, 3 never
    case 0: math_ok = 1; break;
    case 1: math_ok = cw; break;
    case 2: math_ok = !cw; break;
    default: math_ok = 0; break;
    }
    uint16_t c = main_vis ? s_main_c[x] : 0;
    if (math_ok && s_main_m[x]) {
      uint16_t s;
      int h = half && main_vis;
      if (sub_add) {
        s = s_sub_c[x];
        if (!s_sub_z[x]) h = 0; // transparent subscreen -> fixed color, no halving
      } else {
        s = fixed;
      }
      c = blend(c, s, subtract, h);
    }
    out[x] = (lut[c & 31] << 16) | (lut[(c >> 5) & 31] << 8) | lut[(c >> 10) & 31];
  }
}

uint32_t *ppu_framebuffer(void) { return s_fb; }
uint8_t *ppu_vram(void) { return (uint8_t *)S.vram; }
uint8_t *ppu_cgram(void) { return (uint8_t *)S.cgram; }
uint8_t *ppu_oam(void) { return S.oam; }

size_t ppu_save_state(uint8_t *buf) {
  if (buf) memcpy(buf, &S, sizeof S);
  return sizeof S;
}

void ppu_load_state(const uint8_t *buf) {
  memcpy(&S, buf, sizeof S);
}

/* ------------------------------------------------------------------------ */
/* Extra debug accessors (not declared in ppu.h; used by tests/tools).        */
/* ------------------------------------------------------------------------ */

uint16_t ppu_debug_bg_hofs(int bg) { return S.hofs[bg & 3] & 0x3ff; }
uint16_t ppu_debug_bg_vofs(int bg) { return S.vofs[bg & 3] & 0x3ff; }
int      ppu_debug_m7_hofs(void) { return sext13(S.m7hofs); }
int      ppu_debug_m7_vofs(void) { return sext13(S.m7vofs); }
uint16_t ppu_debug_vram_addr(void) { return S.vram_addr; }
uint16_t ppu_debug_oam_addr(void) { return S.oam_addr; }
uint8_t  ppu_debug_first_sprite(void) { return S.first_sprite; }
