// SNES APU (SPC700 + S-DSP) used by the SMW native port.
// The sound driver that SMW uploads at runtime is SPC700 code; it is executed
// by an SPC700 interpreter here, and the S-DSP generates 32 kHz stereo audio.
// Pure C17, no dependencies besides libc.
#pragma once
#include <stdint.h>
#include <stddef.h>

#define APU_SAMPLE_RATE 32000

void apu_init(void);
void apu_reset(void);

// Main-CPU side of the four communication ports ($2140-$2143).
// port = 0..3
uint8_t apu_read_port(int port);
void    apu_write_port(int port, uint8_t val);

// Advance the SPC700 + DSP so that it is in sync with the given 65816 master
// clock count (21.477272 MHz, monotonic since power-on).
// SPC700 clock = 1.024 MHz -> spc_cycles = master_clock * 5632 / 118125.
void apu_run_to(uint64_t master_clock);

// Pop up to max_frames generated stereo frames (interleaved L,R int16) into out.
// Returns the number of frames written. Internally buffers at least ~0.25 s.
int apu_read_samples(int16_t *out, int max_frames);

// Number of buffered stereo frames.
int apu_samples_available(void);

// Serialize full APU state. Returns bytes written; if buf == NULL returns size.
size_t apu_save_state(uint8_t *buf);
void   apu_load_state(const uint8_t *buf);

// Debug access to the 64 KiB audio RAM.
uint8_t *apu_ram(void);
