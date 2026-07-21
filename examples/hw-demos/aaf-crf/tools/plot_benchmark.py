#!/usr/bin/env python3
import csv
import sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from pathlib import Path

# Captured run data lives in misc/metrics/ (untracked). Override with
# the first argument: ./plot_benchmark.py /path/to/metrics
METRICS_DIR = Path(sys.argv[1]) if len(sys.argv) > 1 \
        else Path(__file__).resolve().parent.parent / 'misc' / 'metrics'
CSV_PATH = METRICS_DIR / 'benchmark_1hr.csv'
ACQUISITION_SAMPLES = 3000
OUTLIER_THRESH_NS = 200
SAMPLE_RATE = 300  # Hz


def load_phase_error():
    phase = []
    with open(CSV_PATH) as f:
        reader = csv.DictReader(f)
        for row in reader:
            if int(row['servo_state']) == 2:
                phase.append(int(row['phase_error_ns']))
    phase = np.array(phase[ACQUISITION_SAMPLES:])
    phase = phase[np.abs(phase) <= OUTLIER_THRESH_NS]
    return phase


def compute_mtie(x, taus_samples):
    mtie = np.empty(len(taus_samples))
    n = len(x)
    for i, m in enumerate(taus_samples):
        max_val = -np.inf
        for start in range(0, n - m, max(1, m // 4)):
            window = x[start:start + m]
            peak_to_peak = window.max() - window.min()
            if peak_to_peak > max_val:
                max_val = peak_to_peak
        mtie[i] = max_val
    return mtie


def compute_tdev(x, taus_samples):
    """TDEV via second difference of time error: ITU-T G.810 formula."""
    n = len(x)
    tdev = np.empty(len(taus_samples))
    for i, m in enumerate(taus_samples):
        if 2 * m >= n:
            tdev[i] = np.nan
            continue
        # Second difference: x(j+2m) - 2*x(j+m) + x(j)
        diffs = x[2*m:] - 2*x[m:len(x)-m] + x[:len(x)-2*m]
        tdev[i] = np.sqrt(np.mean(diffs**2) / 6.0)
    return tdev


def make_tau_grid(n_samples, sample_rate, points_per_decade=20):
    max_tau_samples = n_samples // 3
    taus_samples = np.unique(np.geomspace(1, max_tau_samples, num=points_per_decade * 4).astype(int))
    taus_sec = taus_samples / sample_rate
    return taus_samples, taus_sec


def main():
    phase = load_phase_error()
    taus_samples, taus_sec = make_tau_grid(len(phase), SAMPLE_RATE)

    print(f'Computing MTIE/TDEV over {len(taus_samples)} tau points...')
    mtie = compute_mtie(phase, taus_samples)
    tdev = compute_tdev(phase, taus_samples)

    fig, axes = plt.subplots(3, 1, figsize=(10, 12))

    # Phase error histogram
    ax = axes[0]
    bins = np.arange(phase.min() - 0.5, phase.max() + 1.5, 1)
    ax.hist(phase, bins=bins, color='steelblue', edgecolor='none', alpha=0.85)
    ax.axvline(0, color='red', linewidth=0.7, linestyle='--')
    ax.set_xlabel('Phase error (ns)')
    ax.set_ylabel('Count')
    ax.set_title(
        f'CRF Servo Phase Error — 78 min locked\n'
        f'n={len(phase):,}  mean={phase.mean():+.1f} ns  '
        f'σ={phase.std():.1f} ns  [{phase.min():+d}, {phase.max():+d}]'
    )
    ax.set_xlim(-150, 150)

    # MTIE
    ax = axes[1]
    ax.loglog(taus_sec, mtie, 'o-', color='darkorange', markersize=3)
    ax.set_xlabel('Observation interval τ (s)')
    ax.set_ylabel('MTIE (ns)')
    ax.set_title('Maximum Time Interval Error')
    ax.grid(True, which='both', alpha=0.3)

    # TDEV
    ax = axes[2]
    valid = ~np.isnan(tdev)
    ax.loglog(taus_sec[valid], tdev[valid], 'o-', color='seagreen', markersize=3)
    ax.set_xlabel('Observation interval τ (s)')
    ax.set_ylabel('TDEV (ns)')
    ax.set_title('Time Deviation')
    ax.grid(True, which='both', alpha=0.3)

    plt.tight_layout()
    out = METRICS_DIR / 'benchmark_1hr_histogram.png'
    plt.savefig(out, dpi=150)
    print(f'Saved: {out}')


if __name__ == '__main__':
    main()
