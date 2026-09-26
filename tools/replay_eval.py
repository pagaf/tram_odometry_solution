#!/usr/bin/env python3
"""Causal offline replay of the ROS node core with jury-like metrics.

Messages are fed to ReserveOdometryCore in bag (receive) order exactly as the node
callbacks would get them during `ros2 bag play`; outputs are recorded whenever the
node would publish. GNSS is passed to the core only through on_gnss_fix, which
itself stops listening after the initialization window. GNSS is used here only as
the metric reference.

Metrics (per bag and pooled):
  * velocity RMSE/MAE/bias vs |/sensing/gnss/master/vel|, pairing each reference
    sample with the output of nearest stamp within 0.05 s (like the jury script);
  * position error vs GNSS base_link (both antennas + known TF) in the map frame:
    3D mean/RMSE/max, along-track and cross-track RMSE, final error in % of the
    travelled distance.

Example:
  python3 tools/replay_eval.py task_description/data \\
      --params src/tram_reserve_odometry/config/odom_params.yaml --out replay.csv
"""
from __future__ import annotations

import argparse
import csv
import importlib
import json
import math
from pathlib import Path
import sqlite3
import struct
import sys
import time
from concurrent.futures import ProcessPoolExecutor
from typing import Dict, List, Optional

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src' / 'tram_reserve_odometry'))
core_mod = importlib.import_module('tram_reserve_odometry.core')
path_mod = importlib.import_module('tram_reserve_odometry.path_model')
CoreConfig = core_mod.CoreConfig
ReserveOdometryCore = core_mod.ReserveOdometryCore
PolylinePath = path_mod.PolylinePath

BASE = 4
PAIR_TOL = 0.05


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


def parse_fix(data: bytes):
    t, off = parse_header(data)
    status = struct.unpack_from('<b', data, off)[0]
    off = align(off + 1, 2) + 2
    off = align(off, 8)
    lat, lon, alt = struct.unpack_from('<3d', data, off)
    return t, lat, lon, alt, status


def parse_twist(data: bytes):
    t, off = parse_header(data)
    off = align(off, 8)
    x, y, z = struct.unpack_from('<3d', data, off)
    return t, math.sqrt(x * x + y * y + z * z)


TOPICS = {
    '/vehicle/front_bogie_velocity': 'front',
    '/vehicle/rear_bogie_velocity': 'rear',
    '/vehicle/driver_position_cmd': 'cmd',
    '/sensing/gnss/master/fix': 'master_fix',
    '/sensing/gnss/rover/fix': 'rover_fix',
    '/sensing/gnss/master/vel': 'master_vel',
    '/sensing/gnss/rover/vel': 'rover_vel',
}


def load_params(path: Optional[Path], overrides: List[str]) -> CoreConfig:
    values: Dict[str, object] = {}
    if path is not None:
        import yaml
        doc = yaml.safe_load(path.read_text(encoding='utf-8'))
        node = next(iter(doc.values()))
        values.update(node.get('ros__parameters', {}))
    for item in overrides:
        k, v = item.split('=', 1)
        try:
            values[k] = json.loads(v)
        except ValueError:
            values[k] = v
    names = set(CoreConfig.names())
    return CoreConfig(**{k: v for k, v in values.items() if k in names}), values


def reference_base_link(core: ReserveOdometryCore, mf: np.ndarray, rf: np.ndarray):
    """GNSS base_link in the map frame from time-paired master/rover fixes."""
    cfg = core.cfg
    rows = []
    if len(mf) == 0 or len(rf) == 0:
        return np.empty((0, 5))
    j = 0
    for t, lat, lon, alt, st in mf:
        while j + 1 < len(rf) and abs(rf[j + 1, 0] - t) <= abs(rf[j, 0] - t):
            j += 1
        if abs(rf[j, 0] - t) > PAIR_TOL or st < 0 or rf[j, 4] < 0:
            continue
        mx, my = core.to_map(lat, lon)
        rx, ry = core.to_map(rf[j, 1], rf[j, 2])
        dx, dy = rx - mx, ry - my
        if not 0.8 * core.baseline <= math.hypot(dx, dy) <= 1.2 * core.baseline:
            continue
        yaw = math.atan2(dy, dx)
        hx, hy = math.cos(yaw), math.sin(yaw)
        bx = 0.5 * ((mx - cfg.master_x_m * hx) + (rx - cfg.rover_x_m * hx))
        by = 0.5 * ((my - cfg.master_x_m * hy) + (ry - cfg.rover_x_m * hy))
        bz = 0.5 * (alt + rf[j, 3]) - cfg.antenna_z_m
        rows.append((t, bx, by, bz, yaw))
    ref = np.asarray(rows, float).reshape(-1, 5)
    # Drop isolated GNSS glitches: keep samples within 5 m of the local median track.
    if len(ref) > 11:
        k = 5
        pad = np.pad(ref[:, 1:3], ((k, k), (0, 0)), mode='edge')
        win = np.lib.stride_tricks.sliding_window_view(pad, 2 * k + 1, axis=0)
        med = np.median(win, axis=2)
        ref = ref[np.hypot(*(ref[:, 1:3] - med).T) <= 5.0]
    return ref


def nearest(out_t: np.ndarray, t: np.ndarray):
    idx = np.clip(np.searchsorted(out_t, t), 1, len(out_t) - 1)
    left = out_t[idx - 1]
    right = out_t[idx]
    idx = np.where(np.abs(t - left) <= np.abs(right - t), idx - 1, idx)
    ok = np.abs(out_t[idx] - t) <= PAIR_TOL
    return idx, ok


def replay_bag(db: Path, cfg: CoreConfig, path_file: Optional[str]):
    conn = sqlite3.connect(str(db))
    tmap = {tid: TOPICS[name] for tid, name in conn.execute('select id,name from topics') if name in TOPICS}
    path = PolylinePath.from_file(path_file) if path_file else None
    core = ReserveOdometryCore(cfg, path)

    out = []  # t, v, x, y, z
    mf, rf, mv = [], [], []
    proc = []
    for tid, raw in conn.execute('select topic_id,data from messages order by timestamp'):
        kind = tmap.get(tid)
        if kind is None:
            continue
        try:
            if kind in ('front', 'rear', 'cmd'):
                started = time.perf_counter()
                if kind == 'cmd':
                    t, val = parse_driver(raw)
                    core.on_cmd(t, val)
                else:
                    t, val = parse_velocity_sensor(raw)
                    (core.on_front if kind == 'front' else core.on_rear)(t, val)
                if core.should_publish(t):
                    x, y, z, _ = core.pose() if core.position_ready else (math.nan,) * 4
                    wheels = [w[1] for w in (core.front, core.rear) if w is not None]
                    out.append((t, core.velocity, x, y, z, sum(wheels) / len(wheels) if wheels else 0.0))
                    proc.append(time.perf_counter() - started)
            elif kind in ('master_fix', 'rover_fix'):
                t, lat, lon, alt, st = parse_fix(raw)
                core.on_gnss_fix(kind.split('_')[0], t, lat, lon, alt, st)
                (mf if kind == 'master_fix' else rf).append((t, lat, lon, alt, st))
            elif kind == 'master_vel':
                mv.append(parse_twist(raw))
        except (struct.error, ValueError):
            continue
    conn.close()

    res = {'bag': db.parent.name, 'outputs': len(out)}
    if not out:
        return res
    out = np.asarray(out, float)
    out = out[np.argsort(out[:, 0], kind='stable')]
    res['duration_s'] = float(out[-1, 0] - out[0, 0])
    res['proc_ms_p99'] = 1000.0 * float(np.percentile(proc, 99))

    mv = np.asarray(mv, float).reshape(-1, 2)
    if len(mv):
        idx, ok = nearest(out[:, 0], mv[:, 0])
        e = out[idx[ok], 1] - mv[ok, 1]
        if len(e):
            res.update(v_n=int(len(e)), v_rmse=float(np.sqrt(np.mean(e * e))),
                       v_mae=float(np.mean(np.abs(e))), v_bias=float(np.mean(e)),
                       v_sse=float(np.sum(e * e)), v_sae=float(np.sum(np.abs(e))))
            ew = out[idx[ok], 5] - mv[ok, 1]
            res.update(wheel_rmse=float(np.sqrt(np.mean(ew * ew))), wheel_sse=float(np.sum(ew * ew)))

    ref = reference_base_link(core, np.asarray(mf, float).reshape(-1, 5), np.asarray(rf, float).reshape(-1, 5))
    if len(ref) > 10:
        idx, ok = nearest(out[:, 0], ref[:, 0])
        ok &= np.isfinite(out[idx, 2])  # position is not published before initialization
        est = out[idx[ok], 2:5]
        r = ref[ok]
        d = est - r[:, 1:4]
        e3 = np.sqrt(np.sum(d * d, axis=1))
        hx, hy = np.cos(r[:, 4]), np.sin(r[:, 4])
        along = d[:, 0] * hx + d[:, 1] * hy
        cross = -d[:, 0] * hy + d[:, 1] * hx
        steps = np.hypot(np.diff(r[:, 1]), np.diff(r[:, 2]))
        travelled = float(np.sum(steps[steps < 5.0]))
        res.update(
            p_n=int(len(e3)), p_mean=float(np.mean(e3)), p_rmse=float(np.sqrt(np.mean(e3 * e3))),
            p_max=float(np.max(e3)), p_along_rmse=float(np.sqrt(np.mean(along * along))),
            p_along_max=float(np.max(np.abs(along))), p_cross_rmse=float(np.sqrt(np.mean(cross * cross))),
            p_z_rmse=float(np.sqrt(np.mean(d[:, 2] ** 2))), p_final=float(e3[-1]),
            travelled_m=travelled, p_drift_pct=100.0 * float(e3[-1]) / max(travelled, 1.0),
            p_sse=float(np.sum(e3 * e3)), p_sae=float(np.sum(e3)),
        )
    return res


def _job(args):
    return replay_bag(*args)


KEYS = ['bag', 'outputs', 'duration_s', 'proc_ms_p99', 'v_n', 'v_rmse', 'v_mae', 'v_bias', 'wheel_rmse',
        'p_n', 'p_mean', 'p_rmse', 'p_max', 'p_along_rmse', 'p_along_max', 'p_cross_rmse',
        'p_z_rmse', 'p_final', 'travelled_m', 'p_drift_pct']


def summarize(rows: List[dict]) -> Dict[str, float]:
    s: Dict[str, float] = {}
    v = [r for r in rows if r.get('v_n')]
    p = [r for r in rows if r.get('p_n')]
    if v:
        n = sum(r['v_n'] for r in v)
        s['v_rmse_pooled'] = math.sqrt(sum(r['v_sse'] for r in v) / n)
        s['v_mae_pooled'] = sum(r['v_sae'] for r in v) / n
        s['wheel_mean_rmse_pooled'] = math.sqrt(sum(r['wheel_sse'] for r in v) / n)
        s['v_rmse_bag_mean'] = float(np.mean([r['v_rmse'] for r in v]))
        s['v_abs_bias_bag_mean'] = float(np.mean([abs(r['v_bias']) for r in v]))
    if p:
        n = sum(r['p_n'] for r in p)
        s['p_rmse_pooled'] = math.sqrt(sum(r['p_sse'] for r in p) / n)
        s['p_mean_pooled'] = sum(r['p_sae'] for r in p) / n
        s['p_rmse_bag_median'] = float(np.median([r['p_rmse'] for r in p]))
        s['p_max_bag_median'] = float(np.median([r['p_max'] for r in p]))
        s['p_cross_rmse_bag_median'] = float(np.median([r['p_cross_rmse'] for r in p]))
        s['p_drift_pct_bag_median'] = float(np.median([r['p_drift_pct'] for r in p]))
        s['p_drift_pct_bag_mean'] = float(np.mean([r['p_drift_pct'] for r in p]))
    if rows:
        s['proc_ms_p99_max'] = max(r.get('proc_ms_p99', 0.0) for r in rows)
    return s


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('data_dir', type=Path)
    ap.add_argument('--params', type=Path, default=ROOT / 'src/tram_reserve_odometry/config/odom_params.yaml')
    ap.add_argument('--set', action='append', default=[], help='override parameter, e.g. --set path_file=""')
    ap.add_argument('--bags', type=Path, default=None, help='text file with bag names to evaluate')
    ap.add_argument('--out', type=Path, default=None)
    ap.add_argument('--jobs', type=int, default=8)
    args = ap.parse_args()

    cfg, raw = load_params(args.params, args.set)
    path_file = str(raw.get('path_file') or '')
    if path_file and not Path(path_file).is_absolute():
        # Same rule as the node: relative to the package (share) directory.
        path_file = str((ROOT / 'src' / 'tram_reserve_odometry' / path_file).resolve())
    bags = sorted(args.data_dir.rglob('*.db3'))
    if args.bags:
        wanted = set(args.bags.read_text().split())
        bags = [b for b in bags if b.parent.name in wanted]
    with ProcessPoolExecutor(args.jobs) as ex:
        rows = list(ex.map(_job, [(b, cfg, path_file or None) for b in bags]))
    rows.sort(key=lambda r: r['bag'])
    if args.out:
        with args.out.open('w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=KEYS, extrasaction='ignore')
            w.writeheader()
            w.writerows(rows)
    print(json.dumps(summarize(rows), indent=2))


if __name__ == '__main__':
    main()
