// S-DSP emulation (SNES sound generator), one 32 kHz stereo frame at a time.
//
// The per-voice / echo pipeline of the real chip is collapsed into a single
// sample step, but every arithmetic detail that affects the output follows the
// hardware (as documented by blargg's SPC_DSP / bsnes): BRR decoding (filters,
// clamping, 16-bit wrap), 4-tap Gaussian interpolation with the hardware table
// and its intermediate truncation, the ADSR/GAIN envelope with the global rate
// counter, KON start-up delay, noise LFSR, pitch modulation, per-voice clamped
// mixing, and the echo unit (8-tap FIR with 16-bit wrap of the first 7 taps then
// clamping, feedback, EDL/ESA latching).
#include <string.h>
#include "apu_internal.h"

#define D (apu_state.dsp)

// Global registers
enum {
  R_MVOLL = 0x0C, R_MVOLR = 0x1C, R_EVOLL = 0x2C, R_EVOLR = 0x3C,
  R_KON = 0x4C, R_KOFF = 0x5C, R_FLG = 0x6C, R_ENDX = 0x7C,
  R_EFB = 0x0D, R_PMON = 0x2D, R_NON = 0x3D, R_EON = 0x4D,
  R_DIR = 0x5D, R_ESA = 0x6D, R_EDL = 0x7D, R_FIR = 0x0F,
};
// Voice registers (offset within the voice's 16-byte block)
enum {
  V_VOLL = 0, V_VOLR, V_PITCHL, V_PITCHH, V_SRCN, V_ADSR1, V_ADSR2, V_GAIN, V_ENVX, V_OUTX,
};

#define CLAMP16(x) do { if ((int16_t)(x) != (x)) (x) = ((x) < 0) ? -0x8000 : 0x7FFF; } while (0)

// Hardware Gaussian interpolation table (512 entries).
static const int16_t gauss[512] = {
     0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
     1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   2,   2,   2,   2,   2,
     2,   2,   3,   3,   3,   3,   3,   4,   4,   4,   4,   4,   5,   5,   5,   5,
     6,   6,   6,   6,   7,   7,   7,   8,   8,   8,   9,   9,   9,  10,  10,  10,
    11,  11,  11,  12,  12,  13,  13,  14,  14,  15,  15,  15,  16,  16,  17,  17,
    18,  19,  19,  20,  20,  21,  21,  22,  23,  23,  24,  24,  25,  26,  27,  27,
    28,  29,  29,  30,  31,  32,  32,  33,  34,  35,  36,  36,  37,  38,  39,  40,
    41,  42,  43,  44,  45,  46,  47,  48,  49,  50,  51,  52,  53,  54,  55,  56,
    58,  59,  60,  61,  62,  64,  65,  66,  67,  69,  70,  71,  73,  74,  76,  77,
    78,  80,  81,  83,  84,  86,  87,  89,  90,  92,  94,  95,  97,  99, 100, 102,
   104, 106, 107, 109, 111, 113, 115, 117, 118, 120, 122, 124, 126, 128, 130, 132,
   134, 137, 139, 141, 143, 145, 147, 150, 152, 154, 156, 159, 161, 163, 166, 168,
   171, 173, 175, 178, 180, 183, 186, 188, 191, 193, 196, 199, 201, 204, 207, 210,
   212, 215, 218, 221, 224, 227, 230, 233, 236, 239, 242, 245, 248, 251, 254, 257,
   260, 263, 267, 270, 273, 276, 280, 283, 286, 290, 293, 297, 300, 304, 307, 311,
   314, 318, 321, 325, 328, 332, 336, 339, 343, 347, 351, 354, 358, 362, 366, 370,
   374, 378, 381, 385, 389, 393, 397, 401, 405, 410, 414, 418, 422, 426, 430, 434,
   439, 443, 447, 451, 456, 460, 464, 469, 473, 477, 482, 486, 491, 495, 499, 504,
   508, 513, 517, 522, 527, 531, 536, 540, 545, 550, 554, 559, 563, 568, 573, 577,
   582, 587, 592, 596, 601, 606, 611, 615, 620, 625, 630, 635, 640, 644, 649, 654,
   659, 664, 669, 674, 678, 683, 688, 693, 698, 703, 708, 713, 718, 723, 728, 732,
   737, 742, 747, 752, 757, 762, 767, 772, 777, 782, 787, 792, 797, 802, 806, 811,
   816, 821, 826, 831, 836, 841, 846, 851, 855, 860, 865, 870, 875, 880, 884, 889,
   894, 899, 904, 908, 913, 918, 923, 927, 932, 937, 941, 946, 951, 955, 960, 965,
   969, 974, 978, 983, 988, 992, 997,1001,1005,1010,1014,1019,1023,1027,1032,1036,
  1040,1045,1049,1053,1057,1061,1066,1070,1074,1078,1082,1086,1090,1094,1098,1102,
  1106,1109,1113,1117,1121,1125,1128,1132,1136,1139,1143,1146,1150,1153,1157,1160,
  1164,1167,1170,1174,1177,1180,1183,1186,1190,1193,1196,1199,1202,1205,1207,1210,
  1213,1216,1219,1221,1224,1227,1229,1232,1234,1237,1239,1241,1244,1246,1248,1251,
  1253,1255,1257,1259,1261,1263,1265,1267,1269,1270,1272,1274,1275,1277,1279,1280,
  1282,1283,1284,1286,1287,1288,1290,1291,1292,1293,1294,1295,1296,1297,1297,1298,
  1299,1300,1300,1301,1302,1302,1303,1303,1303,1304,1304,1304,1304,1304,1305,1305,
};

// Envelope / noise rate counter: rate N fires when (counter + offset[N]) % rate[N] == 0.
#define COUNTER_RANGE (2048 * 5 * 3)
static const uint16_t counter_rates[32] = {
  COUNTER_RANGE + 1, // rate 0 never fires
  2048, 1536, 1280, 1024, 768, 640, 512, 384, 320, 256, 192, 160, 128, 96, 80,
  64, 48, 40, 32, 24, 20, 16, 12, 10, 8, 6, 5, 4, 3, 2, 1,
};
static const uint16_t counter_offsets[32] = {
  1, 0, 1040, 536, 0, 1040, 536, 0, 1040, 536, 0, 1040, 536, 0, 1040, 536,
  0, 1040, 536, 0, 1040, 536, 0, 1040, 536, 0, 1040, 536, 0, 1040, 0, 0,
};

static inline unsigned read_counter(int rate) {
  return ((unsigned)D.counter + counter_offsets[rate]) % counter_rates[rate];
}

static inline int ram16(const uint8_t *ram, int addr) {
  return ram[addr & 0xFFFF] | (ram[(addr + 1) & 0xFFFF] << 8);
}

void apu_dsp_reset(void) {
  memset(&D, 0, sizeof(D));
  D.regs[R_FLG] = 0xE0;   // soft reset + mute + echo writes disabled
  for (int i = 0; i < 8; i++)
    D.voice[i].brr_offset = 1;
  D.noise = 0x4000;
  D.every_other_sample = 1;
}

uint8_t apu_dsp_read(int addr) {
  return D.regs[addr & 0x7F];
}

void apu_dsp_write(int addr, uint8_t val) {
  addr &= 0x7F;
  D.regs[addr] = val;
  if (addr == R_KON)
    D.new_kon = val;
  else if (addr == R_ENDX)
    D.regs[R_ENDX] = 0;   // any write clears ENDX
}

static int interpolate(const ApuDspVoice *v) {
  int offset = (v->interp_pos >> 4) & 0xFF;
  const int16_t *fwd = gauss + 255 - offset;
  const int16_t *rev = gauss + offset;
  const int32_t *in = &v->buf[(v->interp_pos >> 12) + v->buf_pos];
  int out;
  out  = (fwd[0]   * in[0]) >> 11;
  out += (fwd[256] * in[1]) >> 11;
  out += (rev[256] * in[2]) >> 11;
  out = (int16_t)out;
  out += (rev[0]   * in[3]) >> 11;
  CLAMP16(out);
  return out & ~1;
}

static void decode_brr(ApuDspVoice *v, int brr_byte, int header, const uint8_t *ram) {
  int nybbles = (brr_byte << 8) | ram[(v->brr_addr + v->brr_offset + 1) & 0xFFFF];
  int32_t *pos = &v->buf[v->buf_pos];
  if ((v->buf_pos += 4) >= 12)
    v->buf_pos = 0;
  const int shift = header >> 4;
  const int filter = header & 0x0C;
  for (int k = 0; k < 4; k++, pos++, nybbles <<= 4) {
    int s = (int16_t)(nybbles & 0xFFFF) >> 12;  // signed nibble
    s = (s * (1 << shift)) >> 1;
    if (shift >= 0xD)
      s = s < 0 ? -0x800 : 0;                   // invalid shift range
    const int p1 = pos[12 - 1];
    const int p2 = pos[12 - 2] >> 1;
    if (filter >= 8) {
      s += p1;
      s -= p2;
      if (filter == 8) {          // s += p1 * 0.953125 - p2 * 0.46875
        s += p2 >> 4;
        s += (p1 * -3) >> 6;
      } else {                    // s += p1 * 0.8984375 - p2 * 0.40625
        s += (p1 * -13) >> 7;
        s += (p2 * 3) >> 4;
      }
    } else if (filter) {          // s += p1 * 0.46875
      s += p1 >> 1;
      s += (-p1) >> 5;
    }
    CLAMP16(s);
    s = (int16_t)(s * 2);
    pos[12] = pos[0] = s;
  }
}

static void run_envelope(ApuDspVoice *v, const uint8_t *vr) {
  int env = v->env;
  if (v->env_mode == APU_ENV_RELEASE) {
    env -= 0x8;
    if (env < 0) env = 0;
    v->env = env;
    return;
  }
  int rate;
  int adsr0 = vr[V_ADSR1];
  int env_data = vr[V_ADSR2];
  if (adsr0 & 0x80) { // ADSR
    if (v->env_mode >= APU_ENV_DECAY) {
      env--;
      env -= env >> 8;
      rate = env_data & 0x1F;
      if (v->env_mode == APU_ENV_DECAY)
        rate = ((adsr0 >> 3) & 0x0E) + 0x10;
    } else { // attack
      rate = (adsr0 & 0x0F) * 2 + 1;
      env += rate < 31 ? 0x20 : 0x400;
    }
  } else { // GAIN
    env_data = vr[V_GAIN];
    int mode = env_data >> 5;
    if (mode < 4) { // direct
      env = env_data * 0x10;
      rate = 31;
    } else {
      rate = env_data & 0x1F;
      if (mode == 4) {          // linear decrease
        env -= 0x20;
      } else if (mode < 6) {    // exponential decrease
        env--;
        env -= env >> 8;
      } else {                  // linear increase / bent-line increase
        env += 0x20;
        if (mode > 6 && (unsigned)v->hidden_env >= 0x600)
          env += 0x8 - 0x20;
      }
    }
  }
  // Sustain level
  if ((env >> 8) == (env_data >> 5) && v->env_mode == APU_ENV_DECAY)
    v->env_mode = APU_ENV_SUSTAIN;
  v->hidden_env = env;
  // unsigned compare: linear decrease going negative also lands here
  if ((unsigned)env > 0x7FF) {
    env = env < 0 ? 0 : 0x7FF;
    if (v->env_mode == APU_ENV_ATTACK)
      v->env_mode = APU_ENV_DECAY;
  }
  if (!read_counter(rate))
    v->env = env;
}

void apu_dsp_run_sample(int16_t out[2]) {
  uint8_t *const ram = apu_state.ram;
  uint8_t *const regs = D.regs;

  // KON / KOFF are sampled every other sample
  if ((D.every_other_sample ^= 1) != 0) {
    D.new_kon &= ~D.kon;
    D.kon = D.new_kon;
    D.t_koff = regs[R_KOFF];
  }

  // Global rate counter + noise
  if (--D.counter < 0)
    D.counter = COUNTER_RANGE - 1;
  if (!read_counter(regs[R_FLG] & 0x1F)) {
    int feedback = (D.noise << 13) ^ (D.noise << 14);
    D.noise = (feedback & 0x4000) ^ (D.noise >> 1);
  }

  const int pmon = regs[R_PMON] & 0xFE;  // voice 0 can't be modulated
  const int non = regs[R_NON];
  const int eon = regs[R_EON];
  const int flg = regs[R_FLG];
  const int dir = regs[R_DIR] << 8;
  int main_out[2] = { 0, 0 };
  int echo_out[2] = { 0, 0 };
  int prev_output = 0;

  for (int i = 0; i < 8; i++) {
    ApuDspVoice *v = &D.voice[i];
    uint8_t *vr = &regs[i << 4];
    const int vbit = 1 << i;

    // Sample directory entry: start address while starting up, loop address otherwise
    int dir_addr = dir + vr[V_SRCN] * 4;
    int brr_next = ram16(ram, dir_addr + (v->kon_delay ? 0 : 2));
    int pitch = vr[V_PITCHL] | ((vr[V_PITCHH] & 0x3F) << 8);
    int brr_header = ram[v->brr_addr];
    int brr_byte = ram[(v->brr_addr + v->brr_offset) & 0xFFFF];

    // Pitch modulation from the previous voice's output
    if (pmon & vbit)
      pitch += ((prev_output >> 5) * pitch) >> 10;

    if (v->kon_delay) {
      if (v->kon_delay == 5) {   // start decoding the new sample on the next steps
        v->brr_addr = brr_next;
        v->brr_offset = 1;
        v->buf_pos = 0;
        brr_header = 0;          // header ignored on this sample
      }
      v->env = 0;                // envelope never runs during KON
      v->hidden_env = 0;
      v->interp_pos = 0;         // decode BRR only during the last three samples
      if (--v->kon_delay & 3)
        v->interp_pos = 0x4000;
      pitch = 0;
    }

    // Interpolate (or noise), apply envelope
    int output = interpolate(v);
    if (non & vbit)
      output = (int16_t)(D.noise * 2);
    output = ((output * v->env) >> 11) & ~1;
    vr[V_ENVX] = (uint8_t)(v->env >> 4);

    // Immediate silence on soft reset or end-of-sample without loop
    if ((flg & 0x80) || (brr_header & 3) == 1) {
      v->env_mode = APU_ENV_RELEASE;
      v->env = 0;
    }
    if (D.every_other_sample) {
      if (D.t_koff & vbit)
        v->env_mode = APU_ENV_RELEASE;
      if (D.kon & vbit) {
        v->kon_delay = 5;
        v->env_mode = APU_ENV_ATTACK;
      }
    }
    if (!v->kon_delay)
      run_envelope(v, vr);

    // BRR decode when the position crossed into the next group of 4 samples
    int looped = 0;
    if (v->interp_pos >= 0x4000) {
      decode_brr(v, brr_byte, brr_header, ram);
      if ((v->brr_offset += 2) >= 9) {
        v->brr_addr = (v->brr_addr + 9) & 0xFFFF;
        if (brr_header & 1) {
          v->brr_addr = brr_next;
          looped = vbit;
        }
        v->brr_offset = 1;
      }
    }
    v->interp_pos = (v->interp_pos & 0x3FFF) + pitch;
    if (v->interp_pos > 0x7FFF)
      v->interp_pos = 0x7FFF;

    // Mix (each addition is clamped, as on hardware)
    for (int ch = 0; ch < 2; ch++) {
      int amp = (output * (int8_t)vr[V_VOLL + ch]) >> 7;
      main_out[ch] += amp;
      CLAMP16(main_out[ch]);
      if (eon & vbit) {
        echo_out[ch] += amp;
        CLAMP16(echo_out[ch]);
      }
    }

    // ENDX / OUTX
    int endx = regs[R_ENDX] | looped;
    if (v->kon_delay == 5)
      endx &= ~vbit;
    regs[R_ENDX] = (uint8_t)endx;
    vr[V_OUTX] = (uint8_t)(output >> 8);
    prev_output = output;
  }

  // ---- Echo ----
  D.echo_hist_pos = (D.echo_hist_pos + 1) & 7;
  const int echo_ptr = (D.t_esa * 0x100 + D.echo_offset) & 0xFFFF;
  int echo_in[2];
  for (int ch = 0; ch < 2; ch++) {
    int s = (int16_t)ram16(ram, echo_ptr + ch * 2);
    D.echo_hist[D.echo_hist_pos][ch] = D.echo_hist[D.echo_hist_pos + 8][ch] = s >> 1;
  }
  for (int ch = 0; ch < 2; ch++) {
    // h[0] = oldest sample (coefficient C0), h[7] = newest (C7)
    const int32_t (*h)[2] = (const int32_t (*)[2])&D.echo_hist[D.echo_hist_pos + 1];
    int sum = 0;
    for (int t = 0; t < 7; t++)
      sum += (h[t][ch] * (int8_t)regs[R_FIR + t * 0x10]) >> 6;
    sum = (int16_t)sum;
    sum += (int16_t)((h[7][ch] * (int8_t)regs[R_FIR + 0x70]) >> 6);
    CLAMP16(sum);
    echo_in[ch] = sum & ~1;
  }

  // Main output: (main * MVOL + echo * EVOL), each term truncated to 16 bits
  int o[2];
  for (int ch = 0; ch < 2; ch++) {
    int s = (int16_t)((main_out[ch] * (int8_t)regs[R_MVOLL + ch * 0x10]) >> 7) +
            (int16_t)((echo_in[ch] * (int8_t)regs[R_EVOLL + ch * 0x10]) >> 7);
    CLAMP16(s);
    o[ch] = s;
  }

  // Echo feedback + write back to the ring buffer
  for (int ch = 0; ch < 2; ch++) {
    int e = echo_out[ch] + (int16_t)((echo_in[ch] * (int8_t)regs[R_EFB]) >> 7);
    CLAMP16(e);
    e &= ~1;
    if (!(flg & 0x20)) {
      ram[(echo_ptr + ch * 2) & 0xFFFF] = (uint8_t)e;
      ram[(echo_ptr + ch * 2 + 1) & 0xFFFF] = (uint8_t)(e >> 8);
    }
  }
  D.t_esa = regs[R_ESA];
  if (!D.echo_offset)
    D.echo_length = (regs[R_EDL] & 0x0F) * 0x800;
  D.echo_offset += 4;
  if (D.echo_offset >= D.echo_length)
    D.echo_offset = 0;

  if (flg & 0x40) { // mute
    o[0] = 0;
    o[1] = 0;
  }
  out[0] = (int16_t)o[0];
  out[1] = (int16_t)o[1];
}
