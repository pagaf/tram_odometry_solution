#!/usr/bin/env python3
"""
Offline calibration from the full dataset. No ROS installation is required:
it reads rosbag2 SQLite directly and deserializes only the messages we need.

Usage:
  python3 tools/calibrate_from_bags.py /path/to/dataset/data --out calibration.json

It fits the nonlinear acceleration model only on high-confidence intervals.
When GNSS velocity exists, it is treated as another sensor, NOT as perfect truth.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sqlite3
import struct
from typing import Dict, List, Tuple

import numpy as np

BASE = 4  # CDR encapsulation header size


def align(off: int, n: int) -> int:
    rel = off - BASE
    return BASE + ((rel + n - 1) & ~(n - 1))


def parse_header(data: bytes):
    off = 4
    sec = struct.unpack_from('<i', data, off)[0]; off += 4
    nsec = struct.unpack_from('<I', data, off)[0]; off += 4
    off = align(off, 4)
    n = struct.unpack_from('<I', data, off)[0]; off += 4
    frame = data[off:off + max(0, n - 1)].decode(errors='replace'); off += n
    return sec + 1e-9 * nsec, frame, off


def parse_velocity_sensor(data: bytes):
    t, _, off = parse_header(data)
    off = align(off, 8)
    return t, struct.unpack_from('<d', data, off)[0]


def parse_driver(data: bytes):
    t, _, off = parse_header(data)
    return t, struct.unpack_from('<b', data, off)[0]


def parse_twist_stamped(data: bytes):
    t, _, off = parse_header(data)
    off = align(off, 8)
    vals = struct.unpack_from('<6d', data, off)
    return t, vals[:3]


def topic_map(conn):
    return {name: tid for tid, name in conn.execute('select id,name from topics')}


def read_topic(conn, tid, parser):
    if tid is None:
        return np.empty((0, 2), dtype=float)
    rows = []
    for bag_t, raw in conn.execute('select timestamp,data from messages where topic_id=? order by timestamp', (tid,)):
        try:
            _, val = parser(raw)
            if isinstance(val, tuple):
                rows.append((bag_t * 1e-9, *val))
            else:
                rows.append((bag_t * 1e-9, val))
        except Exception:
            continue
    return np.asarray(rows, dtype=float)


def interp_if_available(arr, t, column=1):
    if arr.shape[0] < 2:
        return None
    return np.interp(t, arr[:, 0], arr[:, column])


def moving_average(x, width=11):
    if len(x) < width:
        return x.copy()
    k = np.ones(width, dtype=float) / width
    y = np.convolve(x, k, mode='same')
    # avoid convolution edge bias by holding nearest interior value
    h = width // 2
    y[:h] = y[h]
    y[-h:] = y[-h-1]
    return y


def bag_samples(db3: Path):
    conn = sqlite3.connect(str(db3))
    tm = topic_map(conn)
    f = read_topic(conn, tm.get('/vehicle/front_bogie_velocity'), parse_velocity_sensor)
    r = read_topic(conn, tm.get('/vehicle/rear_bogie_velocity'), parse_velocity_sensor)
    c = read_topic(conn, tm.get('/vehicle/driver_position_cmd'), parse_driver)
    gm = read_topic(conn, tm.get('/sensing/gnss/master/vel'), parse_twist_stamped)
    gr = read_topic(conn, tm.get('/sensing/gnss/rover/vel'), parse_twist_stamped)
    conn.close()
    if len(f) < 30 or len(r) < 30 or len(c) < 30:
        return None

    t = f[:, 0]
    vf = f[:, 1] / 3.6  # organizer-confirmed km/h -> m/s
    vr = np.interp(t, r[:, 0], r[:, 1]) / 3.6
    notch = np.rint(np.interp(t, c[:, 0], c[:, 1])).astype(int)
    vw = 0.5 * (vf + vr)

    clean = np.abs(vf - vr) < 0.15
    # GNSS is another noisy speed source. If present, use consensus gating only.
    gnss_speeds = []
    for g in (gm, gr):
        if len(g) >= 2:
            speed = np.sqrt(g[:, 1] ** 2 + g[:, 2] ** 2)
            gnss_speeds.append(np.interp(t, g[:, 0], speed))
    if gnss_speeds:
        vg = np.median(np.vstack(gnss_speeds), axis=0)
        clean &= np.abs(vw - vg) < 0.60
        vref = 0.5 * (vw + vg)
    else:
        vref = vw

    vs = moving_average(vref, 11)
    accel = np.gradient(vs, t)
    clean &= np.isfinite(accel) & (np.abs(accel) < 2.5) & (vref > 0.15) & (vref < 25.0)
    clean[:20] = False
    clean[-20:] = False
    return t[clean], vref[clean], notch[clean], accel[clean]


def features(v, notch):
    u = np.clip(notch.astype(float) / 15.0, -1.0, 1.0)
    up = np.maximum(u, 0.0)
    un = np.maximum(-u, 0.0)
    return np.c_[np.ones_like(v), up, up**2, un, un**2, v, v * np.abs(v), up * v, un * v]


def _ridge_lstsq(X, y, w=None, ridge=1e-3):
    """Numerically stable ridge least-squares without forming X.T @ X."""
    X = np.asarray(X, dtype=np.float64)
    y = np.asarray(y, dtype=np.float64)
    if w is None:
        sw = np.ones(len(y), dtype=np.float64)
    else:
        sw = np.sqrt(np.clip(np.asarray(w, dtype=np.float64), 0.0, np.inf))

    # RMS column scaling greatly improves conditioning (v^2 and binary-notch
    # features have very different magnitudes). Do not center: the first
    # column is the physical intercept.
    scale = np.sqrt(np.mean(X * X, axis=0))
    scale = np.where(np.isfinite(scale) & (scale > 1e-12), scale, 1.0)
    Xs = X / scale
    Xw = Xs * sw[:, None]
    yw = y * sw

    if ridge > 0.0:
        # Ridge as an augmented least-squares system. This avoids squaring the
        # condition number, unlike solving the normal equations X.T X beta.
        eye = np.eye(X.shape[1], dtype=np.float64)
        Xa = np.vstack((Xw, math.sqrt(ridge) * eye))
        ya = np.concatenate((yw, np.zeros(X.shape[1], dtype=np.float64)))
    else:
        Xa, ya = Xw, yw

    beta_scaled, *_ = np.linalg.lstsq(Xa, ya, rcond=None)
    return beta_scaled / scale


def robust_fit(X, y, ridge=1e-3, iters=25):
    X = np.asarray(X, dtype=np.float64)
    y = np.asarray(y, dtype=np.float64)
    finite = np.isfinite(y) & np.all(np.isfinite(X), axis=1)
    X, y = X[finite], y[finite]
    if len(y) < max(100, 10 * X.shape[1]):
        raise ValueError(f'Not enough finite calibration rows: {len(y)}')

    beta = _ridge_lstsq(X, y, ridge=ridge)
    for _ in range(iters):
        r = y - X @ beta
        med = np.median(r)
        scale = 1.4826 * np.median(np.abs(r - med)) + 1e-6
        delta = 1.5 * scale
        w = np.minimum(1.0, delta / (np.abs(r) + 1e-12))
        beta_new = _ridge_lstsq(X, y, w=w, ridge=ridge)
        if np.linalg.norm(beta_new - beta) <= 1e-10 * (1.0 + np.linalg.norm(beta)):
            beta = beta_new
            break
        beta = beta_new
    return beta


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('data_dir', type=Path)
    ap.add_argument('--out', type=Path, default=Path('calibration.json'))
    ap.add_argument('--max-samples', type=int, default=250000)
    ap.add_argument('--val-fraction', type=float, default=0.20, help='deterministic bag-level holdout for model checking')
    ap.add_argument('--seed', type=int, default=260925)
    args = ap.parse_args()

    bags = sorted(args.data_dir.rglob('*.db3'))
    if not bags:
        raise SystemExit(f'No .db3 files found under {args.data_dir}')
    rng = np.random.default_rng(args.seed)
    order = np.arange(len(bags)); rng.shuffle(order)
    nval = int(round(len(bags) * max(0.0, min(0.5, args.val_fraction))))
    val_idx = set(order[:nval].tolist())
    train_parts, val_parts = [], []
    used = 0
    for i, db in enumerate(bags, 1):
        sample = bag_samples(db)
        if sample is None:
            continue
        _, v, u, a = sample
        part = 'VAL' if (i - 1) in val_idx else 'TRAIN'
        (val_parts if part == 'VAL' else train_parts).append((v, u, a))
        used += 1
        print(f'[{i}/{len(bags)}] {db.name}: {len(v)} clean samples [{part}]')

    if not train_parts:
        raise SystemExit('No usable training samples')
    v = np.concatenate([x[0] for x in train_parts]); u = np.concatenate([x[1] for x in train_parts]); a = np.concatenate([x[2] for x in train_parts])
    if len(v) > args.max_samples:
        idx = rng.choice(len(v), args.max_samples, replace=False)
        v, u, a = v[idx], u[idx], a[idx]
    X = features(v, u).astype(np.float64, copy=False)
    finite = np.isfinite(a) & np.all(np.isfinite(X), axis=1)
    dropped = int((~finite).sum())
    if dropped:
        print(f'Warning: dropping {dropped} non-finite calibration rows')
        X, a, v, u = X[finite], a[finite], v[finite], u[finite]
    print('feature_abs_max:', [float(x) for x in np.max(np.abs(X), axis=0)])
    print('design_condition_scaled:', float(np.linalg.cond(X / np.maximum(np.sqrt(np.mean(X*X, axis=0)), 1e-12))))
    beta = robust_fit(X, a)
    pred = X @ beta
    rmse = float(np.sqrt(np.mean((pred - a) ** 2)))
    mae = float(np.mean(np.abs(pred - a)))
    val_rmse = val_mae = None
    val_samples = 0
    if val_parts:
        vv = np.concatenate([x[0] for x in val_parts]); vu = np.concatenate([x[1] for x in val_parts]); va = np.concatenate([x[2] for x in val_parts])
        VX = features(vv, vu).astype(np.float64, copy=False)
        finite_v = np.isfinite(va) & np.all(np.isfinite(VX), axis=1)
        VX, va = VX[finite_v], va[finite_v]
        vp = VX @ beta
        val_rmse = float(np.sqrt(np.mean((vp - va) ** 2)))
        val_mae = float(np.mean(np.abs(vp - va)))
        val_samples = int(len(va))
    payload = {
        'bags_used': used,
        'train_bags': len(train_parts),
        'val_bags': len(val_parts),
        'samples': int(len(v)),
        'val_samples': val_samples,
        'dynamics_coeffs': [float(x) for x in beta],
        'fit_rmse_mps2': rmse,
        'fit_mae_mps2': mae,
        'val_rmse_mps2': val_rmse,
        'val_mae_mps2': val_mae,
        'feature_order': ['1','u_pos','u_pos^2','u_neg','u_neg^2','v','v_abs_v','u_pos_v','u_neg_v'],
    }
    args.out.write_text(json.dumps(payload, indent=2), encoding='utf-8')
    print(json.dumps(payload, indent=2))
    print('\nPaste into odom_params.yaml:')
    print('dynamics_coeffs: [' + ', '.join(f'{x:.12g}' for x in beta) + ']')


if __name__ == '__main__':
    main()
