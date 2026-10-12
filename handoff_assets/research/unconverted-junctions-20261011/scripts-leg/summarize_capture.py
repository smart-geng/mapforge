"""Compact table of a capture pickle: every fit call (attempt, leg, chain, result) and every fallback tier.

usage: python summarize_capture.py <capture.pkl> [<out.txt>]
"""
from __future__ import annotations

import pickle
import sys

sys.path.insert(0, "/tmp/claude-0/-home-user-mapforge/c89e54ad-c075-5b54-a3c1-768f7b232511/scratchpad/diag-leg-tree")


def short(p):
    return "…" + str(p)[-6:]


def main():
    data = pickle.load(open(sys.argv[1], "rb"))
    lines = []
    s = data["summary"]
    lines.append(f"junction {s['junction']} status={s['status']} attempts={s['attempts']} secs={s['secs']}")
    lines.append(f"error: {s['error']}")
    lines.append(f"fallback report: {s['report']}")
    lines.append("")
    lines.append("fit calls (converter fit_leg_refline):")
    for e in data["fits"]:
        chain = " + ".join(f"{short(p)}({l:.1f}m)" for p, l in e["chain"])
        res = f"OK dev={e['dev']:.2f} {e['selection']}" if e["ok"] else "FAIL"
        lines.append(f"  att{e['attempt']} {e['kind']:<17} seed {short(e['seed'])} chain[{chain}] pts={len(e['gxy'])} -> {res}"
                     + (f"  caps={ {short(k): v for k, v in e['link_caps'].items()} } relaxed={[short(x[1]) for x in e['relaxed']]}" if not e["ok"] else ""))
        if not e["ok"]:
            for c in e["diag"]["candidates"]:
                if not c["built"]:
                    lines.append(f"      {c['candidate']:<14} optimiser found no G2 solution under the caps")
                else:
                    lines.append(f"      {c['candidate']:<14} kappa_max={c['kappa_max']:.4f} sharp={c['sharpness_max']:.5f} "
                                 f"s2t med/p95/max={c['s2t']['median_m']:.2f}/{c['s2t']['p95_m']:.2f}/{c['s2t']['max_m']:.2f}"
                                 f" fails: {'; '.join(c['fails'][:4])}")
    lines.append("")
    lines.append("fallback fit_leg calls (leg_fit_fallback):")
    for fb in data["fallbacks"]:
        lines.append(f"  att{fb['attempt']} leg={fb['leg'][0]}:{short(fb['leg'][1])} relaxed={fb['relaxed']} "
                     f"accepted={fb['accepted']} pts={len(fb['gxy'])}")
        seen = set()
        for r in fb["tried"]:
            key = (r["kappa_cap"], r["segments"])
            if key in seen:
                continue
            seen.add(key)
            hp = "-" if r["heading_p95_deg"] is None else f"{r['heading_p95_deg']:.1f}"
            hm = "-" if r["heading_max_deg"] is None else f"{r['heading_max_deg']:.1f}"
            lines.append(f"      kcap={r['kappa_cap']:.3f} n={r['segments']} med={r['median_m']:.2f} outP95={r['outside_p95_m']:.2f} "
                         f"outMax={r['outside_max_m']:.2f} zoneMax={r['zone_max_m']:.2f} max={r['max_m']:.2f} "
                         f"fieldCov={r['field_coverage']:.2f} headP95={hp} headMax={hm} flips={r['flips_per_100m']:.1f} "
                         f"| heading_ok={r['heading']} near_centre={r['near_centre']} with_lanes={r['with_lanes']}")
    text = "\n".join(lines)
    print(text)
    if len(sys.argv) > 2:
        open(sys.argv[2], "w", encoding="utf-8").write(text + "\n")


if __name__ == "__main__":
    main()
