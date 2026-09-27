#!/usr/bin/env python3
"""Offline equivalent of the organizers' hackathon_solution_checker (check-code/).

The node core is replayed on a bag (see replay_eval.py); every published output is
paired with /localization/kinematic_state of nearest stamp within 0.05 s, and
RMSE / max absolute error are reported for velocity (vs twist.twist.linear.x) and
for position x, y, z and 3D distance (vs pose.pose.position), like metrics.py.

  python3 tools/checker_replay.py check-code/bags/30618_88aea4d9
"""
from __future__ import annotations

import argparse
from pathlib import Path
import sqlite3
import struct
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import plot_report as pr  # noqa: E402
import replay_eval as rv  # noqa: E402


def parse_odometry(data: bytes):
    """nav_msgs/Odometry -> (t, x, y, z, vx)."""
    t, off = rv.parse_header(data)
    off = rv.align(off, 4)
    n = struct.unpack_from('<I', data, off)[0]
    off = rv.align(off + 4 + n, 8)
    x, y, z = struct.unpack_from('<3d', data, off)
    off += 7 * 8 + 36 * 8
    vx = struct.unpack_from('<d', data, off)[0]
    return t, x, y, z, vx


def read_reference(db: Path) -> np.ndarray:
    conn = sqlite3.connect(str(db))
    tid = dict((n, i) for i, n in conn.execute('select id,name from topics'))['/localization/kinematic_state']
    rows = [parse_odometry(r[0]) for r in conn.execute(
        'select data from messages where topic_id=? order by timestamp', (tid,))]
    conn.close()
    return np.asarray(rows, float)


def report(name: str, err: np.ndarray) -> str:
    return f'{name}: RMSE={np.sqrt(np.mean(err * err)):.4f}, max={np.max(np.abs(err)):.4f}, n={len(err)}'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag_dir', type=Path)
    ap.add_argument('--set', action='append', default=[], help='override parameter, e.g. --set stop_anchor_gain=0.0')
    args = ap.parse_args()
    db = next(args.bag_dir.glob('*.db3'))

    ref = read_reference(db)
    out, _, _ = pr.trace(db, args.set)
    idx = np.clip(np.searchsorted(ref[:, 0], out[:, 0]), 1, len(ref) - 1)
    idx = np.where(np.abs(ref[idx - 1, 0] - out[:, 0]) <= np.abs(ref[idx, 0] - out[:, 0]), idx - 1, idx)
    ok = np.abs(ref[idx, 0] - out[:, 0]) <= rv.PAIR_TOL

    print('Velocity metrics [m/s]:', report('velocity', out[ok, 1] - ref[idx[ok], 4]))
    okp = ok & np.isfinite(out[:, 3])  # position is not published before initialization
    d = out[okp, 3:6] - ref[idx[okp], 1:4]
    parts = [report(axis, d[:, i]) for i, axis in enumerate('xyz')]
    parts.append(report('distance', np.linalg.norm(d, axis=1)))
    print('Position metrics [m]:', ', '.join(parts))


if __name__ == '__main__':
    main()
