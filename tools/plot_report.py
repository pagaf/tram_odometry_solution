#!/usr/bin/env python3
"""Figures for the README: replays bags through the node core (like replay_eval.py).

  python3 tools/plot_report.py task_description/data --out docs/img
"""
from __future__ import annotations

import argparse
import math
from pathlib import Path
import sqlite3
import struct
import sys

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parent))
import replay_eval as rv  # noqa: E402

SURFACE = '#fcfcfb'
TEXT = '#0b0b0b'
TEXT_2 = '#52514e'
GRID = '#e4e3df'
SERIES = ['#2a78d6', '#eb6834', '#1baf7a']


def trace(db: Path, overrides):
    """Per-output samples: t, v_est, wheel_mean, x, y, z; plus GNSS refs."""
    cfg, raw = rv.load_params(rv.ROOT / 'src/tram_reserve_odometry/config/odom_params.yaml', overrides)
    path_file = str(rv.ROOT / 'src/tram_reserve_odometry' / raw['path_file'])
    core = rv.ReserveOdometryCore(cfg, rv.PolylinePath.from_file(path_file))
    conn = sqlite3.connect(str(db))
    tmap = {tid: rv.TOPICS[n] for tid, n in conn.execute('select id,name from topics') if n in rv.TOPICS}
    out, mf, rf, mv = [], [], [], []
    for tid, raw_msg in conn.execute('select topic_id,data from messages order by timestamp'):
        kind = tmap.get(tid)
        try:
            if kind in ('front', 'rear', 'cmd'):
                if kind == 'cmd':
                    t, val = rv.parse_driver(raw_msg)
                    core.on_cmd(t, val)
                else:
                    t, val = rv.parse_velocity_sensor(raw_msg)
                    (core.on_front if kind == 'front' else core.on_rear)(t, val)
                if core.should_publish(t):
                    x, y, z, _ = core.pose() if core.position_ready else (math.nan,) * 4
                    w = [s[1] for s in (core.front, core.rear) if s is not None]
                    out.append((t, core.velocity, sum(w) / len(w) if w else 0.0, x, y, z))
            elif kind in ('master_fix', 'rover_fix'):
                t, lat, lon, alt, st = rv.parse_fix(raw_msg)
                core.on_gnss_fix(kind.split('_')[0], t, lat, lon, alt, st)
                (mf if kind == 'master_fix' else rf).append((t, lat, lon, alt, st))
            elif kind == 'master_vel':
                mv.append(rv.parse_twist(raw_msg))
        except (struct.error, ValueError):
            continue
    conn.close()
    ref = rv.reference_base_link(core, np.asarray(mf, float).reshape(-1, 5), np.asarray(rf, float).reshape(-1, 5))
    return np.asarray(out, float), np.asarray(mv, float), ref


def style(ax, title, xlabel, ylabel):
    ax.set_facecolor(SURFACE)
    ax.set_title(title, loc='left', color=TEXT, fontsize=12)
    ax.set_xlabel(xlabel, color=TEXT_2)
    ax.set_ylabel(ylabel, color=TEXT_2)
    ax.grid(True, color=GRID, lw=0.8)
    ax.tick_params(colors=TEXT_2)
    for side in ('top', 'right'):
        ax.spines[side].set_visible(False)
    for side in ('left', 'bottom'):
        ax.spines[side].set_color(GRID)


def plot_braking(data_dir: Path, out_dir: Path, bag: str, t_from: float, t_to: float):
    out, mv, _ = trace(next(data_dir.rglob(f'{bag}_0.db3')), [])
    t0 = out[0, 0]
    m = (out[:, 0] - t0 >= t_from) & (out[:, 0] - t0 <= t_to)
    g = (mv[:, 0] - t0 >= t_from) & (mv[:, 0] - t0 <= t_to)
    fig, ax = plt.subplots(figsize=(9, 4.2), facecolor=SURFACE)
    ax.plot(mv[g, 0] - t0, mv[g, 1], color=TEXT_2, lw=2, ls='--', label='GNSS (эталон)')
    ax.plot(out[m, 0] - t0, out[m, 2], color=SERIES[1], lw=2, label='среднее колёс')
    ax.plot(out[m, 0] - t0, out[m, 1], color=SERIES[0], lw=2, label='оценка /result/velocity')
    style(ax, f'Резкое торможение при ручке ≈ 0 ({bag})', 'время от начала прогона, с', 'скорость, м/с')
    ax.legend(frameon=False, labelcolor=TEXT)
    fig.tight_layout()
    fig.savefig(out_dir / 'velocity_braking.png', dpi=110, facecolor=SURFACE)
    plt.close(fig)


def position_error(out, ref):
    idx, ok = rv.nearest(out[:, 0], ref[:, 0])
    ok &= np.isfinite(out[idx, 3])
    d = out[idx[ok], 3:6] - ref[ok, 1:4]
    return ref[ok, 0] - out[0, 0], np.sqrt(np.sum(d * d, axis=1))


def plot_position(data_dir: Path, out_dir: Path, bag: str):
    db = next(data_dir.rglob(f'{bag}_0.db3'))
    fig, ax = plt.subplots(figsize=(9, 4.2), facecolor=SURFACE)
    for color, label, gain in ((SERIES[1], 'без привязки к остановкам', '0.0'),
                               (SERIES[0], 'с привязкой к остановкам', '0.8')):
        out, _, ref = trace(db, [f'stop_anchor_gain={gain}'])
        t, e = position_error(out, ref)
        ax.plot(t, e, color=color, lw=2, label=f'{label}: RMSE {math.sqrt(np.mean(e * e)):.1f} м')
    style(ax, f'Ошибка положения относительно GNSS base_link ({bag})', 'время от начала прогона, с', '3D-ошибка, м')
    ax.set_ylim(bottom=0)
    ax.legend(frameon=False, labelcolor=TEXT)
    fig.tight_layout()
    fig.savefig(out_dir / 'position_error.png', dpi=110, facecolor=SURFACE)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('data_dir', type=Path)
    ap.add_argument('--out', type=Path, default=rv.ROOT / 'docs' / 'img')
    args = ap.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    plot_braking(args.data_dir, args.out, '30618_616ec56b', 1012.0, 1034.0)
    plot_position(args.data_dir, args.out, '30618_22c1c589')
    print(f'Saved figures to {args.out}')


if __name__ == '__main__':
    main()
