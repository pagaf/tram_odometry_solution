#!/usr/bin/env python3
"""Build a route map: organizer pathgraph tracks + surveyed terminal extensions.

The organizer pathgraph contains only the two main-line tracks (ST: Shchukinskaya ->
Tallinskaya and TS: back). Runs start/finish on terminal tracks that are NOT in the
pathgraph. This offline tool joins them into one continuous route:

    Tallinskaya stub -> TS (official) -> Shchukinskaya loop -> ST (official) -> Tallinskaya stub

Terminal pieces are surveyed from GNSS base_link trajectories of TRAINING bags only
(offline map preparation, like the pathgraph itself; GNSS is never used online).
At Shchukinskaya the tail of a run leaving TS is spliced with the head of a run
entering ST at their closest co-directional point.

Example:
  python3 tools/build_route.py task_description/data --bags train.txt \\
      --out src/tram_reserve_odometry/config/route.json
"""
from __future__ import annotations

import argparse
from concurrent.futures import ProcessPoolExecutor
import json
import math
from pathlib import Path
import sqlite3
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import replay_eval as rv  # noqa: E402

ROOT = rv.ROOT
MAPS = ROOT / 'map_coordinates'
ST_FILE = MAPS / 'щукинская - таллинская.json'
TS_FILE = MAPS / 'таллинская - щукинская.json'
ON_TRACK_M = 6.0
END_ZONE_M = 30.0


def load_track(fp: Path) -> np.ndarray:
    data = json.loads(fp.read_text(encoding='utf-8'))
    pts = data['points']
    order = data['paths'][0]['point_indices'] if data.get('paths') else range(len(pts))
    return np.asarray([[pts[i]['x'], pts[i]['y'], pts[i]['z']] for i in order], float)


def arclength(p: np.ndarray) -> np.ndarray:
    return np.r_[0.0, np.cumsum(np.hypot(*np.diff(p[:, :2], axis=0).T))]


def project(track: np.ndarray, xy: np.ndarray):
    """Vectorized nearest-segment projection; returns (s, lateral)."""
    a = track[:-1, :2]
    d = track[1:, :2] - a
    l2 = np.maximum((d * d).sum(1), 1e-12)
    s0 = arclength(track)
    s_out = np.empty(len(xy))
    lat_out = np.empty(len(xy))
    for i in range(0, len(xy), 1000):
        q = xy[i:i + 1000]
        w = np.clip(((q[:, None, :] - a[None]) * d[None]).sum(2) / l2[None], 0.0, 1.0)
        pr = a[None] + w[..., None] * d[None]
        dist = np.sqrt(((q[:, None, :] - pr) ** 2).sum(2))
        j = dist.argmin(1)
        k = np.arange(len(q))
        lat_out[i:i + 1000] = dist[k, j]
        s_out[i:i + 1000] = s0[j] + w[k, j] * np.sqrt(l2[j])
    return s_out, lat_out


def bag_standstills(db: Path, min_duration_s: float = 10.0, radius_m: float = 0.3) -> np.ndarray:
    """Base_link [x, y] of every GNSS standstill lasting at least min_duration_s."""
    ref = bag_reference(db)
    out = []
    i = 0
    while i < len(ref):
        j = i
        while j + 1 < len(ref) and math.hypot(ref[j + 1, 1] - ref[i, 1], ref[j + 1, 2] - ref[i, 2]) <= radius_m:
            j += 1
        if ref[j, 0] - ref[i, 0] >= min_duration_s:
            out.append(ref[(i + j) // 2, 1:3])
        i = j + 1
    return np.asarray(out, float).reshape(-1, 2)


def cluster_stops(s: np.ndarray, min_count: int = 10, max_iqr_m: float = 1.5, gap_m: float = 15.0):
    """Platform stopping points: dense, repeatable standstill clusters along the route."""
    s = np.sort(s)
    if len(s) == 0:
        return []
    clusters = [[s[0]]]
    for x in s[1:]:
        if x - clusters[-1][-1] < gap_m:
            clusters[-1].append(x)
        else:
            clusters.append([x])
    return [float(np.median(c)) for c in clusters
            if len(c) >= min_count and np.percentile(c, 75) - np.percentile(c, 25) <= max_iqr_m]


def bag_reference(db: Path) -> np.ndarray:
    """GNSS base_link [t, x, y, z, yaw] in the map frame."""
    conn = sqlite3.connect(str(db))
    tmap = {tid: rv.TOPICS[n] for tid, n in conn.execute('select id,name from topics') if n in rv.TOPICS}
    mf, rf = [], []
    for tid, raw in conn.execute('select topic_id,data from messages order by timestamp'):
        kind = tmap.get(tid)
        if kind in ('master_fix', 'rover_fix'):
            (mf if kind == 'master_fix' else rf).append(rv.parse_fix(raw))
    conn.close()
    core = rv.ReserveOdometryCore(rv.CoreConfig())
    return rv.reference_base_link(core, np.asarray(mf, float).reshape(-1, 5), np.asarray(rf, float).reshape(-1, 5))


def bag_trajectory(db: Path) -> np.ndarray:
    """Moving GNSS base_link samples [x, y, z_rail, yaw] in the map frame."""
    ref = bag_reference(db)
    if len(ref) < 50:
        return np.empty((0, 4))
    # Keep samples ~0.5 m apart so standing time does not dominate.
    keep = [0]
    for i in range(1, len(ref)):
        if math.hypot(ref[i, 1] - ref[keep[-1], 1], ref[i, 2] - ref[keep[-1], 2]) >= 0.5:
            keep.append(i)
    return ref[keep][:, 1:5]


def contiguous(mask: np.ndarray):
    idx = np.flatnonzero(np.diff(np.r_[0, mask.astype(int), 0]))
    return list(zip(idx[::2], idx[1::2]))


def tails_and_heads(traj: np.ndarray, st: np.ndarray, ts: np.ndarray):
    """Off-map pieces leaving the end of a track (tails) or entering its start (heads)."""
    out = {'tail_ST': [], 'tail_TS': [], 'head_ST': [], 'head_TS': []}
    if len(traj) < 20:
        return out
    tracks = {'ST': st, 'TS': ts}
    proj = {k: project(v, traj[:, :2]) for k, v in tracks.items()}
    off = (proj['ST'][1] > ON_TRACK_M) & (proj['TS'][1] > ON_TRACK_M)
    for a, b in contiguous(off):
        seg = traj[a:b]
        if len(seg) < 20:
            continue
        for k, tr in tracks.items():
            s, lat = proj[k]
            length = arclength(tr)[-1]
            # The track ends are only ~5 m apart at Shchukinskaya, hence the relaxed gate.
            if a > 0 and lat[a - 1] <= 2 * ON_TRACK_M and s[a - 1] > length - END_ZONE_M:
                out[f'tail_{k}'].append(np.vstack([traj[a - 1:a], seg]))
            if b < len(traj) and lat[b] <= 2 * ON_TRACK_M and s[b] < END_ZONE_M:
                out[f'head_{k}'].append(np.vstack([seg, traj[b:b + 1]]))
    return out


def is_clean(piece: np.ndarray, max_jump_m: float = 2.5) -> bool:
    """Reject pieces with GNSS jumps (multipath/float solutions near the terminals)."""
    return len(piece) > 20 and np.hypot(*np.diff(piece[:, :2], axis=0).T).max() <= max_jump_m


def splice(tails, heads):
    """Best tail+head pair joined at their closest co-directional points."""
    best = None
    tails = [p for p in tails if is_clean(p)]
    heads = [p for p in heads if is_clean(p)]
    for tail in tails:
        for head in heads:
            d = np.hypot(tail[:, None, 0] - head[None, :, 0], tail[:, None, 1] - head[None, :, 1])
            codir = np.cos(tail[:, None, 3] - head[None, :, 3]) > 0.7
            d = np.where(codir, d, np.inf)
            i, j = np.unravel_index(np.argmin(d), d.shape)
            gap = d[i, j]
            if best is None or gap < best[0]:
                best = (gap, np.vstack([tail[:i + 1], head[j:]]))
    if best is None:
        raise SystemExit('no connector candidates found; add more training bags')
    return best


def resample(p: np.ndarray, step: float = 1.0, smooth_m: float = 4.0) -> np.ndarray:
    s = arclength(p)
    keep = np.r_[True, np.diff(s) > 1e-6]
    p, s = p[keep], s[keep]
    q = np.arange(0.0, s[-1], step)
    r = np.c_[np.interp(q, s, p[:, 0]), np.interp(q, s, p[:, 1]), np.interp(q, s, p[:, 2])]
    k = max(1, int(round(smooth_m / step)))
    if len(r) > 2 * k + 1:
        ker = np.ones(2 * k + 1) / (2 * k + 1)
        inner = np.c_[[np.convolve(r[:, c], ker, mode='valid') for c in range(3)]].T
        r[k:-k] = inner
    return r


def connector(tail_src: np.ndarray, head_dst: np.ndarray, pieces) -> np.ndarray:
    """GNSS connector from the last point of tail_src track to the first of head_dst."""
    gap, path = pieces
    # Trim the parts that still lie on the official tracks: beyond a track end the
    # projection clamps to the end point, so lateral distance grows past 1 m.
    _, lat_src = project(tail_src, path[:, :2])
    _, lat_dst = project(head_dst, path[:, :2])
    start = int(np.argmax(lat_src > 1.0))
    stop = len(path) - int(np.argmax(lat_dst[::-1] > 1.0))
    core = path[start:stop, :3]
    core = np.vstack([tail_src[-1:], core, head_dst[:1]])
    return resample(core)[1:], gap


def stub_extension(track: np.ndarray, pieces, at_start: bool) -> np.ndarray:
    """Longest clean GNSS piece entering (at_start) or leaving a dead-end track end."""
    best = np.empty((0, 3))
    for piece in pieces:
        if not is_clean(piece):
            continue
        _, lat = project(track, piece[:, :2])
        if at_start:
            stop = len(piece) - int(np.argmax(lat[::-1] > 1.0))
            ext = resample(np.vstack([piece[:stop, :3], track[:1]]))[:-1]
            ext = cut_turnaround(ext[::-1], track[0, :2] - track[1, :2])[::-1]
        else:
            start = int(np.argmax(lat > 1.0))
            ext = resample(np.vstack([track[-1:], piece[start:, :3]]))[1:]
            ext = cut_turnaround(ext, track[-1, :2] - track[-2, :2])
        if len(ext) > len(best):
            best = ext
    return best


def cut_turnaround(ext: np.ndarray, track_dir: np.ndarray, max_turn_rad: float = 1.6) -> np.ndarray:
    """Keep a dead-end extension only until it turns back (a run that went on around the
    loop would otherwise run parallel to the opposite track and capture its initialization)."""
    if len(ext) < 2:
        return ext
    h0 = math.atan2(track_dir[1], track_dir[0])
    d = np.diff(ext[:, :2], axis=0)
    turn = np.abs(np.angle(np.exp(1j * (np.arctan2(d[:, 1], d[:, 0]) - h0))))
    bad = np.flatnonzero(turn > max_turn_rad)
    return ext[: bad[0] + 1] if len(bad) else ext


def build(pieces, st: np.ndarray, ts: np.ndarray):
    """Open route: Tallinskaya stub -> TS -> Shchukinskaya loop -> ST -> Tallinskaya stub.

    Tallinskaya is a fan of dead-end stub tracks: trams enter a stub on ST and leave it
    on TS, the change of ends is not recorded in any bag, so that end stays open.
    Shchukinskaya is a proper turning loop recorded continuously (TS tail + ST head).
    """
    head_ts = stub_extension(ts, pieces['head_TS'], at_start=True)
    conn_s, gap_s = connector(ts, st, splice(pieces['tail_TS'], pieces['head_ST']))
    tail_st = stub_extension(st, pieces['tail_ST'], at_start=False)
    parts = [('tallinskaya_stub_TS', head_ts), ('TS', ts), ('shchukinskaya_loop', conn_s),
             ('ST', st), ('tallinskaya_stub_ST', tail_st)]
    segments, n = {}, 0
    for name, part in parts:
        segments[name] = [n, n + len(part)]
        n += len(part)
    route = np.vstack([part for _, part in parts])
    info = {name: float(arclength(part)[-1]) if len(part) > 1 else 0.0 for name, part in parts}
    info['shchukinskaya_splice_gap_m'] = float(gap_s)
    return route, segments, info


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('data_dir', type=Path)
    ap.add_argument('--bags', type=Path, required=True, help='training bag names (one per line)')
    ap.add_argument('--out', type=Path, required=True)
    ap.add_argument('--rail-offset', type=float, default=0.1,
                    help='extra z offset so GNSS rail height matches pathgraph z (measured ~0.1 m)')
    args = ap.parse_args()

    st = load_track(ST_FILE)
    ts = load_track(TS_FILE)
    wanted = set(args.bags.read_text().split())
    pieces = {'tail_ST': [], 'tail_TS': [], 'head_ST': [], 'head_TS': []}
    dbs = [db for db in sorted(args.data_dir.rglob('*.db3')) if db.parent.name in wanted]
    with ProcessPoolExecutor() as ex:
        trajs = list(ex.map(bag_trajectory, dbs))
        stills = list(ex.map(bag_standstills, dbs))
    for traj in trajs:
        traj[:, 2] -= args.rail_offset
        for k, v in tails_and_heads(traj, st, ts).items():
            pieces[k].extend(v)
    print({k: len(v) for k, v in pieces.items()})

    route, segments, info = build(pieces, st, ts)
    xy = np.vstack(stills) if stills else np.empty((0, 2))
    s_still, lat = project(route, xy) if len(xy) else (np.empty(0), np.empty(0))
    stops = cluster_stops(s_still[lat < 6.0])
    info['stops'] = len(stops)
    jumps = np.hypot(*np.diff(route[:, :2], axis=0).T)
    print(json.dumps(info, indent=2))
    print(f'route length {arclength(route)[-1]:.0f} m, max point spacing {jumps.max():.2f} m')
    payload = {
        'closed': False,
        'frame': 'UTM37N - (300000, 6100000), organizer pathgraph frame',
        'segments': segments,
        'stops': stops,
        'points': [{'x': float(x), 'y': float(y), 'z': float(z)} for x, y, z in route],
        'paths': [{'point_indices': list(range(len(route)))}],
    }
    args.out.write_text(json.dumps(payload), encoding='utf-8')
    print(f'Saved: {args.out}')


if __name__ == '__main__':
    main()
