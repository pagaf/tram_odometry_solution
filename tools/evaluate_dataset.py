#!/usr/bin/env python3
"""Offline validation against GNSS velocity channels (reference only, never runtime input).

This script replays the same causal estimator logic from SQLite rosbag2 files and
compares it with simple wheel baselines.  It does NOT use future wheel samples.
GNSS is used only to compute metrics.

Example:
  python3 tools/evaluate_dataset.py dataset/data \
      --calibration calibration.json --out metrics.csv

Interpretation:
  * primary comparison: ours vs front/rear/mean wheel speed;
  * report master GNSS, rover GNSS and their median consensus separately;
  * longitudinal distance error is a proxy obtained by integrating the GNSS
    speed consensus over the interval where GNSS velocity is actually present.
"""
from __future__ import annotations

import argparse
import csv
import importlib.util
import json
import math
from pathlib import Path
import sqlite3
import struct
import sys
from typing import Optional

import numpy as np

BASE = 4
DEFAULT_COEFFS = [
    0.000140604885, 1.85266353, -0.268904638,
    -2.43778005, 1.57779757, -0.00808316467,
    0.000480035941, -0.0745746855, 0.0115504134,
]


def load_filter_class():
    here = Path(__file__).resolve().parents[1]
    fp = here / 'src' / 'tram_reserve_odometry' / 'tram_reserve_odometry' / 'filter.py'
    spec = importlib.util.spec_from_file_location('tram_filter_offline', fp)
    mod = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod.RobustTramFilter


RobustTramFilter = load_filter_class()


def align(off: int, n: int) -> int:
    rel = off - BASE
    return BASE + ((rel + n - 1) & ~(n - 1))


def parse_header(data: bytes):
    off = 4
    sec = struct.unpack_from('<i', data, off)[0]; off += 4
    nsec = struct.unpack_from('<I', data, off)[0]; off += 4
    off = align(off, 4)
    n = struct.unpack_from('<I', data, off)[0]; off += 4
    off += n
    return sec + 1e-9 * nsec, off


def parse_velocity_sensor(data: bytes):
    t, off = parse_header(data)
    off = align(off, 8)
    return t, struct.unpack_from('<d', data, off)[0]


def parse_driver(data: bytes):
    t, off = parse_header(data)
    return t, struct.unpack_from('<b', data, off)[0]


def parse_twist(data: bytes):
    t, off = parse_header(data)
    off = align(off, 8)
    x, y, z, *_ = struct.unpack_from('<6d', data, off)
    return t, math.sqrt(x*x + y*y + z*z)


def topic_ids(conn):
    return {name: tid for tid, name in conn.execute('select id,name from topics')}


def read_twist_topic(conn, tid: Optional[int]):
    if tid is None:
        return np.empty((0, 2), dtype=float)
    rows = []
    for (raw,) in conn.execute('select data from messages where topic_id=? order by timestamp', (tid,)):
        try:
            rows.append(parse_twist(raw))
        except Exception:
            pass
    return np.asarray(rows, dtype=float)


def interp_inside(arr: np.ndarray, t: float) -> Optional[float]:
    if len(arr) < 2 or t < arr[0, 0] or t > arr[-1, 0]:
        return None
    return float(np.interp(t, arr[:, 0], arr[:, 1]))


def metrics(est, ref):
    est = np.asarray(est, dtype=float)
    ref = np.asarray(ref, dtype=float)
    if len(est) == 0:
        return {'rmse': math.nan, 'mae': math.nan, 'bias': math.nan, 'n': 0}
    e = est - ref
    return {
        'rmse': float(np.sqrt(np.mean(e*e))),
        'mae': float(np.mean(np.abs(e))),
        'bias': float(np.mean(e)),
        'n': int(len(e)),
    }


def evaluate_bag(db: Path, coeffs):
    conn = sqlite3.connect(str(db))
    tm = topic_ids(conn)
    ids = {
        'f': tm.get('/vehicle/front_bogie_velocity'),
        'r': tm.get('/vehicle/rear_bogie_velocity'),
        'c': tm.get('/vehicle/driver_position_cmd'),
        'gm': tm.get('/sensing/gnss/master/vel'),
        'gr': tm.get('/sensing/gnss/rover/vel'),
    }
    if ids['f'] is None or ids['r'] is None or ids['c'] is None:
        conn.close()
        return None

    gm = read_twist_topic(conn, ids['gm'])
    gr = read_twist_topic(conn, ids['gr'])
    if len(gm) < 2 and len(gr) < 2:
        conn.close()
        return {'bag': db.parent.name, 'gnss_samples': 0}

    filt = RobustTramFilter(coeffs=coeffs)
    notch = 0
    front = rear = None  # (t, m/s)
    last_used_f = last_used_r = None
    fresh = 0.22

    # Values recorded at command timestamps; matches online output cadence.
    rec = []
    internal_ids = [x for x in (ids['f'], ids['r'], ids['c']) if x is not None]
    qs = ','.join('?' for _ in internal_ids)
    query = f'select topic_id,data from messages where topic_id in ({qs}) order by timestamp'

    for tid, raw in conn.execute(query, tuple(internal_ids)):
        try:
            if tid == ids['f']:
                t, x = parse_velocity_sensor(raw)
                filt.predict_to(t, notch)
                front = (t, x / 3.6)
                fv = front[1]
                rv = rear[1] if rear is not None and abs(t - rear[0]) <= fresh else None
                uf = front[0] != last_used_f
                ur = rv is not None and rear[0] != last_used_r
                filt.update_wheels(t, notch, fv, rv, uf, ur)
                if uf: last_used_f = front[0]
                if ur: last_used_r = rear[0]
            elif tid == ids['r']:
                t, x = parse_velocity_sensor(raw)
                filt.predict_to(t, notch)
                rear = (t, x / 3.6)
                fv = front[1] if front is not None and abs(t - front[0]) <= fresh else None
                rv = rear[1]
                uf = fv is not None and front[0] != last_used_f
                ur = rear[0] != last_used_r
                filt.update_wheels(t, notch, fv, rv, uf, ur)
                if uf: last_used_f = front[0]
                if ur: last_used_r = rear[0]
            else:
                t, k = parse_driver(raw)
                filt.predict_to(t, notch)
                notch = int(k)
                gmv = interp_inside(gm, t)
                grv = interp_inside(gr, t)
                refs = [v for v in (gmv, grv) if v is not None and math.isfinite(v)]
                if not refs:
                    continue
                gt = float(np.median(refs))
                fv = front[1] if front is not None and abs(t - front[0]) <= fresh else None
                rv = rear[1] if rear is not None and abs(t - rear[0]) <= fresh else None
                meanw = None if fv is None and rv is None else (
                    rv if fv is None else fv if rv is None else 0.5*(fv+rv)
                )
                rec.append((t, filt.velocity, filt.distance, fv, rv, meanw, gt, gmv, grv))
        except Exception:
            continue
    conn.close()

    if not rec:
        return {'bag': db.parent.name, 'gnss_samples': 0}

    # Compare only timestamps where each estimate exists.
    def pair(col, refcol=6):
        a, b = [], []
        for row in rec:
            if row[col] is not None and math.isfinite(row[col]):
                a.append(row[col]); b.append(row[refcol])
        return metrics(a, b)

    mo = pair(1)
    mf = pair(3)
    mr = pair(4)
    mm = pair(5)

    # Master/rover are reported separately because organizers clarified there is
    # no single physical "reference speed" channel.
    def pair_specific(estcol, refcol):
        a, b = [], []
        for row in rec:
            if row[estcol] is not None and row[refcol] is not None:
                a.append(row[estcol]); b.append(row[refcol])
        return metrics(a, b)
    mmaster = pair_specific(1, 7)
    mrover = pair_specific(1, 8)

    # Longitudinal distance proxy from GNSS speed consensus over its covered span.
    ts = np.asarray([r[0] for r in rec], float)
    vg = np.asarray([r[6] for r in rec], float)
    sd = np.asarray([r[2] for r in rec], float)
    truth_d = 0.0
    if len(ts) > 1:
        truth_d = float(np.trapz(vg, ts))
    est_d = float(sd[-1] - sd[0]) if len(sd) > 1 else 0.0
    drift_m = est_d - truth_d
    drift_pct = 100.0 * drift_m / max(truth_d, 1.0)

    return {
        'bag': db.parent.name,
        'gnss_samples': mo['n'],
        'ours_rmse': mo['rmse'], 'ours_mae': mo['mae'], 'ours_bias': mo['bias'],
        'front_rmse': mf['rmse'], 'rear_rmse': mr['rmse'], 'mean_rmse': mm['rmse'],
        'ours_master_rmse': mmaster['rmse'], 'ours_rover_rmse': mrover['rmse'],
        'distance_ref_m': truth_d, 'distance_est_m': est_d,
        'distance_drift_m': drift_m, 'distance_drift_pct': drift_pct,
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('data_dir', type=Path)
    ap.add_argument('--calibration', type=Path, default=None)
    ap.add_argument('--out', type=Path, default=Path('metrics.csv'))
    ap.add_argument('--limit', type=int, default=0)
    args = ap.parse_args()

    coeffs = DEFAULT_COEFFS
    if args.calibration:
        payload = json.loads(args.calibration.read_text())
        coeffs = payload['dynamics_coeffs']

    bags = sorted(args.data_dir.rglob('*.db3'))
    if args.limit > 0:
        bags = bags[:args.limit]
    rows = []
    for i, db in enumerate(bags, 1):
        r = evaluate_bag(db, coeffs)
        if r is None:
            continue
        rows.append(r)
        if r.get('gnss_samples', 0):
            print(f"[{i}/{len(bags)}] {r['bag']}: ours RMSE={r['ours_rmse']:.3f} m/s, "
                  f"mean-wheel={r['mean_rmse']:.3f} m/s, drift={r['distance_drift_pct']:.3f}%")
        else:
            print(f"[{i}/{len(bags)}] {r['bag']}: no GNSS velocity -> skipped metrics")

    keys = [
        'bag','gnss_samples','ours_rmse','ours_mae','ours_bias',
        'front_rmse','rear_rmse','mean_rmse','ours_master_rmse','ours_rover_rmse',
        'distance_ref_m','distance_est_m','distance_drift_m','distance_drift_pct'
    ]
    with args.out.open('w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        for r in rows:
            w.writerow({k: r.get(k, '') for k in keys})

    valid = [r for r in rows if r.get('gnss_samples', 0) > 0]
    if valid:
        # Sample-count-weighted global summary.
        total = sum(r['gnss_samples'] for r in valid)
        def wrms(key):
            return math.sqrt(sum(r['gnss_samples'] * r[key]**2 for r in valid if math.isfinite(r[key])) / total)
        print('\nGLOBAL (approx. sample-count weighted)')
        print(f"ours RMSE:       {wrms('ours_rmse'):.4f} m/s")
        print(f"mean-wheel RMSE: {wrms('mean_rmse'):.4f} m/s")
        med_drift = float(np.median([abs(r['distance_drift_pct']) for r in valid]))
        print(f"median |distance drift| proxy: {med_drift:.4f}%")
    print(f"\nSaved: {args.out}")


if __name__ == '__main__':
    main()
