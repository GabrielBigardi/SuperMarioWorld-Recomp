"""Analyse the WAVs written by dsp_test.exe.

usage: python -I analyze_dsp.py <out_dir>

Checks (voice 0: 32-sample BRR sine at pitch $1000 -> 1000 Hz, ADSR AR=15,
SL=7, SR=$10; VOL L/R = $60/$40; echo EDL=4 (64 ms), EFB=$50, FIR C0=$7F,
KOFF 0.6 s after KON):
  - output is non-silent, silent before KON
  - dominant frequency ~1000 Hz
  - left/right level ratio ~ $60/$40
  - envelope decays during sustain (SR), release after KOFF ends the dry signal
  - echo-less run is exactly silent after the release
  - echo run diverges from the dry run ~64 ms (+7 samples FIR delay) after onset
  - echo tail after KOFF is present, ~1000 Hz, decaying ~EFB/128 per 64 ms pass
"""
import json
import os
import sys
import wave

import numpy as np

SR = 32000
failures = 0


def check(cond, msg):
    global failures
    print(('  ok   : ' if cond else '  FAIL : ') + msg)
    if not cond:
        failures += 1


def load(path):
    with wave.open(path, 'rb') as w:
        assert w.getframerate() == SR and w.getnchannels() == 2 and w.getsampwidth() == 2
        data = np.frombuffer(w.readframes(w.getnframes()), dtype='<i2').astype(np.float64)
    return data.reshape(-1, 2)


def rms(x):
    return float(np.sqrt(np.mean(x * x))) if len(x) else 0.0


def dominant_freq(x):
    win = np.hanning(len(x))
    spec = np.abs(np.fft.rfft(x * win))
    freqs = np.fft.rfftfreq(len(x), 1.0 / SR)
    k = int(np.argmax(spec[1:])) + 1
    # parabolic interpolation around the peak
    if 1 <= k < len(spec) - 1:
        a, b, c = np.log(spec[k - 1] + 1e-9), np.log(spec[k] + 1e-9), np.log(spec[k + 1] + 1e-9)
        p = 0.5 * (a - c) / (a - 2 * b + c)
        return freqs[k] + p * (freqs[1] - freqs[0])
    return freqs[k]


def main():
    out = sys.argv[1] if len(sys.argv) > 1 else 'out'
    wet = load(os.path.join(out, 'dsp_echo.wav'))
    dry = load(os.path.join(out, 'dsp_noecho.wav'))
    with open(os.path.join(out, 'dsp_markers.json')) as f:
        markers = json.load(f)
    n = min(len(wet), len(dry))
    wet, dry = wet[:n], dry[:n]
    L = wet[:, 0]

    nz = np.nonzero(np.abs(dry).sum(axis=1))[0]
    check(len(nz) > 0 and np.abs(wet).max() > 2000, 'output non-silent (peak %d)' % np.abs(wet).max())
    onset = int(nz[0])
    dry_end = int(nz[-1]) + 1
    kon_marker = markers['echo']['kon']
    koff_marker = markers['echo']['koff']
    print('  info : onset at frame %d (KON marker %d), dry signal ends at frame %d (KOFF marker %d)'
          % (onset, kon_marker, dry_end, koff_marker))
    check(np.all(wet[:onset] == 0), 'silent before KON')
    check(0 <= onset - kon_marker + 300 < 600, 'onset matches KON time (KON delay + marker granularity)')

    tone = slice(onset + 1600, onset + 12800)        # 50 ms .. 400 ms
    f0 = dominant_freq(dry[tone, 0])
    check(abs(f0 - 1000.0) < 5.0, 'dominant frequency %.2f Hz (expected 1000 Hz)' % f0)

    lr = rms(dry[tone, 0]) / max(rms(dry[tone, 1]), 1e-9)
    check(abs(lr - 1.5) < 0.03, 'L/R level ratio %.3f (VOL $60/$40 = 1.5)' % lr)

    early = rms(dry[onset + 1600:onset + 3200, 0])
    late = rms(dry[onset + 16000:onset + 17600, 0])
    check(early > 2.0 * late > 0, 'sustain decay (SR): RMS %.0f at 50-100 ms -> %.0f at 500-550 ms' % (early, late))

    # release: dry signal stops shortly after KOFF, and its envelope decreases while doing so
    check(koff_marker - 300 <= dry_end <= koff_marker + 400,
          'dry signal ends %d frames after KOFF marker (release)' % (dry_end - koff_marker))
    peaks = [np.abs(dry[s:s + 32, 0]).max() for s in range(dry_end - 32 * 6, dry_end, 32)]
    before = np.abs(dry[dry_end - 32 * 20:dry_end - 32 * 12, 0]).max()
    check(all(peaks[i] >= peaks[i + 1] for i in range(len(peaks) - 1)) and peaks[-1] < before,
          'release envelope decreasing (per-period peaks %s, earlier %d)' % ([int(p) for p in peaks], before))
    check(np.all(dry[dry_end:] == 0) and n - dry_end > 16000, 'no-echo run silent after release (%d frames)' % (n - dry_end))

    # echo onset: wet == dry until the first echo comes back around the 64 ms buffer
    diff = np.nonzero(np.abs(wet - dry).sum(axis=1))[0]
    delay = int(diff[0]) - onset if len(diff) else -1
    check(2040 <= delay <= 2070, 'echo first appears %d frames (%.2f ms) after onset (EDL=4: 2048 + FIR delay)'
          % (delay, delay / 32.0))

    # echo tail after KOFF
    tail = wet[dry_end + 320:, 0]
    tail_rms = rms(tail[:9600])
    check(tail_rms > 200, 'echo tail present after KOFF (RMS %.0f over 300 ms)' % tail_rms)
    ft = dominant_freq(tail[:6400])
    check(abs(ft - 1000.0) < 10.0, 'echo tail frequency %.2f Hz' % ft)
    # successive 64 ms passes decay by ~ EFB/128 * C0/128 = 0.62
    start = dry_end + 64
    ratios = []
    for k in range(1, 6):
        a = rms(wet[start + (k - 1) * 2048:start + k * 2048, 0])
        b = rms(wet[start + k * 2048:start + (k + 1) * 2048, 0])
        ratios.append(b / a if a else 0.0)
    check(all(0.5 < r < 0.75 for r in ratios), 'echo feedback decay per pass %s (expected ~0.62)'
          % ['%.3f' % r for r in ratios])
    check(rms(wet[-3200:, 0]) < tail_rms * 0.1, 'echo tail dies away (final RMS %.1f)' % rms(wet[-3200:, 0]))

    print('\nANALYSIS %s (%d failure%s)' % ('PASSED' if not failures else 'FAILED', failures,
                                          '' if failures == 1 else 's'))
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
