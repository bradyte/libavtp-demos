#!/usr/bin/env python3
import re
import sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from pathlib import Path

METRICS_DIR = Path(__file__).parent

def parse_extts_log(path):
    values = []
    with open(path) as f:
        for line in f:
            m = re.search(r'extts_jitter_ns=([+-]\d+)', line)
            if m:
                values.append(int(m.group(1)))
    return np.array(values)

def parse_phase_error(path):
    values = []
    with open(path) as f:
        for line in f:
            m = re.search(r'phase=\s*([+-]?\d+)ns', line)
            if m:
                values.append(int(m.group(1)))
    return np.array(values)

def plot_histogram(ax, data, title, xlabel='ns', clip=None):
    if clip:
        data = data[(data >= clip[0]) & (data <= clip[1])]
    n = len(data)
    if n == 0:
        ax.text(0.5, 0.5, 'No data', transform=ax.transAxes, ha='center')
        return
    ax.hist(data, bins=min(200, int(data.max() - data.min() + 1)),
            color='steelblue', edgecolor='none', alpha=0.8)
    ax.set_title(f'{title}\nn={n:,}  mean={data.mean():.1f}  '
                 f'σ={data.std():.1f}  [{data.min()}, {data.max()}]',
                 fontsize=9)
    ax.set_xlabel(xlabel)
    ax.set_ylabel('count')
    ax.axvline(0, color='red', linewidth=0.5, linestyle='--')

def main():
    talker_extts = parse_extts_log(METRICS_DIR / 'talker_extts.log')
    listener1_extts = parse_extts_log(METRICS_DIR / 'listener1_extts.log')
    listener2_extts = parse_extts_log(METRICS_DIR / 'listener2_extts.log')
    listener1_phase = parse_phase_error(METRICS_DIR / 'listener1_stderr.log')
    listener2_phase = parse_phase_error(METRICS_DIR / 'listener2_stderr.log')

    # Filter to locked state (skip first 10 samples which are acquisition)
    listener1_phase = listener1_phase[10:]
    listener2_phase = listener2_phase[10:]

    fig, axes = plt.subplots(3, 2, figsize=(14, 10))
    fig.suptitle('EXTTS Jitter & Servo Phase Error Histograms', fontsize=12)

    # Left column: EXTTS jitter (clipped to ±20ns, shared scale)
    clip_extts = (-20, 20)
    plot_histogram(axes[0, 0], talker_extts, 'Talker EXTTS jitter (FPGA→SDP0)', clip=clip_extts)
    plot_histogram(axes[1, 0], listener1_extts, 'Listener 1 EXTTS jitter (PWM→CS2600→SDP0)', clip=clip_extts)
    plot_histogram(axes[2, 0], listener2_extts, 'Listener 2 EXTTS jitter (PWM→CS2600→SDP0)', clip=clip_extts)
    for ax in axes[:, 0]:
        ax.set_xlim(-20, 20)

    # Right column: servo phase error (clipped to ±200ns, shared scale)
    clip_phase = (-200, 200)
    axes[0, 1].set_visible(False)
    plot_histogram(axes[1, 1], listener1_phase, 'Listener 1 phase error (edge - CRF ts)', clip=clip_phase)
    plot_histogram(axes[2, 1], listener2_phase, 'Listener 2 phase error (edge - CRF ts)', clip=clip_phase)
    for ax in axes[1:, 1]:
        ax.set_xlim(-200, 200)

    plt.tight_layout()
    out = METRICS_DIR / 'histograms.png'
    plt.savefig(out, dpi=150)
    print(f'Saved: {out}')

if __name__ == '__main__':
    main()
