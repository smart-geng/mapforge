"""Geometry of one entering road (read-only): ROADLINK centreline vs lanes/boundaries in the junction's local frame.

usage: python leginfo.py <junction_pid> <link_pid> [<link_pid> ...]
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np
import shapefile

TREE = Path("/tmp/claude-0/-home-user-mapforge/c89e54ad-c075-5b54-a3c1-768f7b232511/scratchpad/diag-leg-tree")
sys.path.insert(0, str(TREE))

from mapforge.adapters.shp.profile_source import ProfileSource  # noqa: E402
from mapforge.ops import shp_to_xodr as S  # noqa: E402

SHP = TREE / "shp_0222-0326"


def polylen(p):
    p = np.asarray(p, float)
    return float(np.linalg.norm(np.diff(p, axis=0), axis=1).sum()) if len(p) > 1 else 0.0


def hd(p, i0=0, i1=None):
    p = np.asarray(p, float)
    d = np.diff(p, axis=0)
    d = d[np.linalg.norm(d, axis=1) > 0.05]
    h = np.degrees(np.unwrap(np.arctan2(d[:, 1], d[:, 0])))
    return h


def project(poly, pts):
    from mapforge.ops.leg_fit_fallback import _project
    return _project(np.asarray(poly, float), np.asarray(pts, float))


def raw_lane_rows(pids):
    r = shapefile.Reader(str(SHP / "IBD_LANE_LINK"), encoding="gbk")
    fields = [f[0] for f in r.fields[1:]]
    out = {}
    for rec in r.iterRecords():
        m = dict(zip(fields, rec))
        if str(m["LANE_PID"]).strip() in pids:
            out[str(m["LANE_PID"]).strip()] = {k: (str(v).strip() if isinstance(v, str) else v) for k, v in m.items()}
    return out


def main():
    pid = sys.argv[1]
    links = sys.argv[2:]
    src = ProfileSource(str(SHP), "ibd-smarteditor-v1")
    junc = next(j for j in src.junctions if j.pid == pid)
    lon0, lat0 = float(junc.center[0]), float(junc.center[1])
    proj = lambda p: S._proj(p, lat0, lon0)  # noqa: E731
    lanes_all = [l for lp in links for l in src.lanes_of(lp)]
    raw = raw_lane_rows({l.lane_pid for l in lanes_all})
    for lp in links:
        rl = src.roadlinks.get(lp)
        c = src.roadcenters.get(lp)
        print(f"== link {lp} name={rl.name if rl else None} lane_num={rl.lane_num if rl else None}")
        if c is not None:
            g = proj(c)
            h = hd(g)
            print(f"   ROADCENTER {len(g)} pts, length {polylen(g):.2f} m, start {g[0].round(2)}, end {g[-1].round(2)}, "
                  f"heading start {h[0]:.1f} end {h[-1]:.1f} total turn {h[-1]-h[0]:.1f} deg")
            s = np.r_[0, np.cumsum(np.linalg.norm(np.diff(g, axis=0), axis=1))]
            print("   centre stations/heading:", [(round(float(a), 1), round(float(b), 1)) for a, b in zip(s[:-1], hd(g) if len(hd(g)) == len(s) - 1 else [np.nan] * (len(s) - 1))][:60])
        if rl is not None and len(rl.geometry) >= 2:
            gr = proj(rl.geometry)
            print(f"   ROADLINK geometry {len(gr)} pts, length {polylen(gr):.2f}, start {gr[0].round(2)} end {gr[-1].round(2)}")
        for l in src.lanes_of(lp):
            lg = proj(l.geometry)
            m = raw.get(l.lane_pid, {})
            h = hd(lg)
            print(f"   lane {l.lane_pid} seq={l.seq} type={l.lane_type} width={l.width_mm} sw={l.s_width_mm} ew={l.e_width_mm}"
                  f" dir={m.get('LANE_DIR')} widechange={m.get('WIDECHANGE')} len={polylen(lg):.2f} start={lg[0].round(2)}"
                  f" end={lg[-1].round(2)} heading {h[0]:.1f}->{h[-1]:.1f} (turn {h[-1]-h[0]:.1f})")
            print(f"      S_LANE={m.get('S_LANE_PID')} E_LANE={m.get('E_LANE_PID')} topo_in={src.topo_in.get(l.lane_pid)} "
                  f"topo_out={src.topo_out.get(l.lane_pid)}")
            for nxt in (src.topo_out.get(l.lane_pid) or []) + (src.topo_in.get(l.lane_pid) or []):
                o = src.lane(nxt)
                if o is not None:
                    og = proj(o.geometry) if len(o.geometry) >= 2 else None
                    print(f"        topo lane {nxt} link={o.link_pid} merge={src.is_merge(nxt)} seq={o.seq} "
                          f"len={polylen(og) if og is not None else 0:.1f} start={None if og is None else og[0].round(1)}"
                          f" end={None if og is None else og[-1].round(1)}")
            for b in src.lane_boundary_geometries(l.lane_pid):
                bg = proj(b)
                hb = hd(bg)
                print(f"      boundary {len(bg)} pts len={polylen(bg):.2f} start={bg[0].round(2)} end={bg[-1].round(2)}"
                      f" heading {hb[0]:.1f}->{hb[-1]:.1f}")
            if c is not None:
                st, off = project(proj(c), lg)
                print(f"      lane centre vs ROADCENTER: station {st.min():.1f}..{st.max():.1f} of {polylen(proj(c)):.1f};"
                      f" lateral offset min {off.min():.2f} median {np.median(off):.2f} max {off.max():.2f}")


if __name__ == "__main__":
    main()
