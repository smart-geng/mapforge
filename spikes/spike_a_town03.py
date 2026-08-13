# -*- coding: utf-8 -*-
"""Spike-A：Town03 自采样回拍——用参数化真值验证折线→line/arc 拟合器。

流程：Town03 真值 planView → 采样折线（模拟折线源）→ fit_polyline → 与真值对比。
指标：段类型序列 / 显著 arc 半径还原误差 / 假 arc 数 / 重建横向偏差。
实验组：无噪声、σ=1cm、σ=5cm（后者开 GCV 预平滑）。
"""
from __future__ import annotations

import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

sys.path.insert(0, r"F:\MapFactory")
from mapforge.ops.refline_fit import fit_polyline, eval_planview, lateral_deviation, ARC, LINE

XODR = Path(r"F:\资料\SUMO\MapFormat-main\OpenDRIVE_data\OpenDRIVE数据\Town\Town03.xodr")
OUT = Path(r"F:\MapFactory\out")
SIG_ARC_LEN = 8.0      # “显著 arc”判定长度（真值）
SAMPLE_STEP = 2.0


def load_truth_roads():
    """返回 [(road_id, total_len, segs=[(kind,s0,s1,curv,x,y,hdg,length)])]，仅非 junction road。"""
    root = ET.parse(str(XODR)).getroot()
    out = []
    for r in root.findall("road"):
        if r.get("junction", "-1") != "-1":
            continue
        segs, s = [], 0.0
        for g in r.findall("planView/geometry"):
            L = float(g.get("length"))
            x, y, h = float(g.get("x")), float(g.get("y")), float(g.get("hdg"))
            child = g[0]
            if child.tag == "line":
                segs.append(("line", s, s + L, 0.0, x, y, h, L))
            elif child.tag == "arc":
                segs.append(("arc", s, s + L, float(child.get("curvature")), x, y, h, L))
            else:
                segs = None
                break
            s += L
        if segs:
            out.append((r.get("id"), s, segs))
    return out


def sample_truth(segs, step=SAMPLE_STEP) -> np.ndarray:
    """按每段声明的绝对起点位姿解析采样。"""
    pts = []
    for kind, s0, s1, k, x, y, h, L in segs:
        n = max(2, int(L / step) + 1)
        ss = np.linspace(0.0, L, n)
        if kind == "line" or abs(k) < 1e-12:
            xs = x + ss * np.cos(h)
            ys = y + ss * np.sin(h)
        else:
            xs = x + (np.sin(h + k * ss) - math.sin(h)) / k
            ys = y - (np.cos(h + k * ss) - math.cos(h)) / k
        pts.append(np.column_stack([xs, ys]))
    all_pts = np.vstack(pts)
    keep = np.concatenate([[True], np.linalg.norm(np.diff(all_pts, axis=0), axis=1) > 1e-6])
    return all_pts[keep]


def truth_sig_arcs(segs):
    return [(s0, s1, k) for kind, s0, s1, k, *_ in segs if kind == "arc" and (s1 - s0) >= SIG_ARC_LEN]


def fitted_intervals(pv):
    out, s = [], 0.0
    for seg in pv.segs:
        out.append((seg.kind, s, s + seg.length, seg.curvature))
        s += seg.length
    return out


def overlap(a0, a1, b0, b1):
    return max(0.0, min(a1, b1) - max(a0, b0))


def kindstr(items, min_len=3.0):
    return " ".join(("A" if it[0] == "arc" else "L") for it in items if it[2] - it[1] >= min_len)


def run_case(road_id, total_len, segs, noise, seed, **fit_kw):
    rng = np.random.default_rng(seed)
    src = sample_truth(segs)
    noisy = src + rng.normal(0.0, noise, src.shape) if noise > 0 else src
    pv, _ = fit_polyline(noisy, **fit_kw)
    fitted = fitted_intervals(pv)
    recon = eval_planview(pv, step=1.0)
    dmax, dmean = lateral_deviation(recon, src)      # 与无噪真值比

    sig = truth_sig_arcs(segs)
    radii, missed = [], 0
    for s0, s1, k in sig:
        best = max(fitted, key=lambda f: overlap(s0, s1, f[1], f[2]))
        if best[0] == "arc" and abs(best[3]) > 1e-9:
            radii.append(abs(1.0 / abs(best[3]) - 1.0 / abs(k)) / (1.0 / abs(k)))
        else:
            missed += 1
    all_arcs = [(s0, s1) for kind, s0, s1, *_ in segs if kind == "arc"]
    false_arcs = 0
    for f in fitted:
        if f[0] != "arc":
            continue
        ov = sum(overlap(f[1], f[2], a0, a1) for a0, a1 in all_arcs)
        if ov < 0.3 * (f[2] - f[1]):
            false_arcs += 1
    return {
        "road": road_id, "len": total_len,
        "truth_str": kindstr([(k, s0, s1) for k, s0, s1, *_ in segs]),
        "fit_str": kindstr(fitted),
        "n_sig": len(sig), "missed": missed, "false_arcs": false_arcs,
        "radius_err": radii, "dmax": dmax, "dmean": dmean,
    }


def main():
    OUT.mkdir(exist_ok=True)
    roads = load_truth_roads()
    cands = [r for r in roads
             if any(k == "arc" for k, *_ in r[2]) and 60 <= r[1] <= 600
             and r[1] / len(r[2]) >= 10.0]      # 排除平均段长<10m 的过渡胶水链
    cands.sort(key=lambda r: -sum(1 for k, *_ in r[2] if k == "arc"))
    chosen = cands[:8]

    cases = [
        ("clean σ=0",  0.00, dict(kappa_th=1/3000, min_seg_len=6.0, smooth_win=5)),
        ("noise σ=1cm", 0.01, dict(kappa_th=1/600,  min_seg_len=6.0, smooth_win=7)),
        ("noise σ=5cm", 0.05, dict(kappa_th=1/300,  min_seg_len=8.0, smooth_win=9, do_presmooth=True)),
    ]
    lines = ["# Spike-A 报告：Town03 自采样回拍", "",
             f"样本：{len(chosen)} 条含 arc 的非 junction road；采样步长 {SAMPLE_STEP} m；显著 arc ≥{SIG_ARC_LEN} m", ""]
    for name, noise, kw in cases:
        lines += [f"## 实验组 {name}  参数 {kw}", ""]
        agg_r, agg_missed, agg_sig, agg_false, agg_dmax = [], 0, 0, 0, []
        for road_id, total_len, segs in chosen:
            res = run_case(road_id, total_len, segs, noise, seed=42, **kw)
            agg_r += res["radius_err"]; agg_missed += res["missed"]
            agg_sig += res["n_sig"]; agg_false += res["false_arcs"]; agg_dmax.append(res["dmax"])
            rtxt = ", ".join(f"{e*100:.2f}%" for e in res["radius_err"]) or "—"
            lines.append(
                f"- road {res['road']}（{res['len']:.0f} m）真值[{res['truth_str']}] → 拟合[{res['fit_str']}]  "
                f"显著arc {res['n_sig']} 漏 {res['missed']} 假 {res['false_arcs']}  "
                f"半径误差[{rtxt}]  横向偏差 max {res['dmax']*100:.1f} cm / mean {res['dmean']*100:.1f} cm")
        r = np.array(agg_r) if agg_r else np.array([np.nan])
        lines += ["",
                  f"**小结**：显著 arc {agg_sig} 个，漏检 {agg_missed}，假 arc {agg_false}；"
                  f"半径误差 中位 {np.nanmedian(r)*100:.2f}% / 最大 {np.nanmax(r)*100:.2f}%；"
                  f"横向偏差 max 的最大值 {max(agg_dmax)*100:.1f} cm", ""]
    report = "\n".join(lines)
    (OUT / "spike_a_report.md").write_text(report, encoding="utf-8")
    print(report)


if __name__ == "__main__":
    main()
