#!/usr/bin/env python3
"""Quick dataset sanity check: topic rates, wheel units/range, notch distribution."""
from pathlib import Path
import argparse, sqlite3, struct, statistics
from collections import Counter

BASE=4
def align(o,a): return BASE+(((o-BASE)+a-1)&~(a-1))
def header(d):
    o=4; sec=struct.unpack_from('<i',d,o)[0];o+=4;ns=struct.unpack_from('<I',d,o)[0];o+=4
    n=struct.unpack_from('<I',d,o)[0];o+=4; frame=d[o:o+n-1].decode(errors='replace');o+=n
    return sec+ns*1e-9,frame,o
def vel(d):
    t,f,o=header(d);o=align(o,8);return t,f,struct.unpack_from('<d',d,o)[0]
def cmd(d):
    t,f,o=header(d);return t,f,struct.unpack_from('<b',d,o)[0]
def main():
    ap=argparse.ArgumentParser();ap.add_argument('db3',type=Path);args=ap.parse_args()
    c=sqlite3.connect(str(args.db3)); topics={i:(n,t) for i,n,t in c.execute('select id,name,type from topics')}
    for tid,(name,typ) in topics.items():
        rows=c.execute('select timestamp,data from messages where topic_id=? order by timestamp',(tid,)).fetchall()
        if not rows: print(name,': 0 messages');continue
        ts=[x[0]*1e-9 for x in rows]
        hz=(len(ts)-1)/(ts[-1]-ts[0]) if len(ts)>1 else 0
        if 'bogie_velocity' in name:
            vals=[vel(d)[2] for _,d in rows]
            print(f'{name}: n={len(vals)} rate={hz:.2f}Hz raw[min,median,max]=[{min(vals):.3f},{statistics.median(vals):.3f},{max(vals):.3f}] km/h')
        elif 'driver_position' in name:
            vals=[cmd(d)[2] for _,d in rows]
            print(f'{name}: n={len(vals)} rate={hz:.2f}Hz notches={Counter(vals).most_common()}')
        else: print(f'{name}: n={len(rows)} rate={hz:.2f}Hz')
if __name__=='__main__': main()
