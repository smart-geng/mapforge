# -*- coding: utf-8 -*-
"""真实数据全流程闭环跑分（/goal 验收闸门）：再生成金凤 14 文件并跑六道平滑门禁。

门禁（全部文件 × 全部通过才 exit 0）：
  G1 XSD 1.5M schema 合法；
  G2 planView 位姿连续（<1mm / <0.001rad）；
  G3 参考线曲率连续（全网 G2：结点 |Δκ| < 1e-6，SolveG2 过渡后应为机器精度）；
  G4 断面边界台阶 = 0（<5cm，零宽边界去重后一对一匹配）；
  G5 换乘连续（route_continuity：进/出侧 <1cm）；
  G6 esmini RoadManager 独立行驶（缝隙 <15cm、零跳变、全部可穿越）；
  G7 曲率品质（防"极小段拼接假平滑"）：段长下限与蛇行/侧向 jerk 上限——
     G2 连续只保证几何平滑，逐顶点碎段会让消费端读到高频曲率锯齿。

用法：.venv/Scripts/python scripts/closed_loop.py [--no-regen]
"""
import math
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

FILES = [f"out/direct_xodr/{n}.xodr" for n in
         ("node3", "node4", "NODE5", "node13", "node16", "node17", "node18")] + \
        [f"out/m2x/{n}.xodr" for n in
         ("node3", "node4", "NODE5", "node13", "node16", "node17", "node18")]


def main():
    if "--no-regen" not in sys.argv:
        print("== 再生成 14 文件 ==")
        r = subprocess.run([sys.executable, str(ROOT / "scripts/gen_all.py")],
                           capture_output=True, text=True, cwd=str(ROOT))
        if r.returncode != 0:
            print(r.stdout[-2000:], r.stderr[-2000:])
            sys.exit(2)

    from lxml import etree
    from mapforge.validate.planview_check import check_file
    from mapforge.validate.smoothness import audit_file, curvature_audit, route_continuity

    # G7 阈值（实测基线上留余量，作回归防护）：leg=主路 60km/h、conn=路口内 30km/h。
    # 段长中位是辅助指标——真正的判据是蛇行(flips)与侧向 jerk：G2 的短 clothoid
    # 链在几何与驾驶上都是平滑的，只有"曲率来回振荡"才真正伤害规划/控制。
    LIM = {"leg": {"flips": 18.0, "jerk": 200.0, "med": 2.5},
           "conn": {"flips": 30.0, "jerk": 400.0, "med": 2.0}}

    schema = etree.XMLSchema(etree.parse(str(ROOT / "OpenDRIVE_1.5M.xsd")))
    all_ok = True
    print(f"{'file':30s} XSD  planV  kappa_max  edge_step route(in/out)  "
          f"legFlip/jerk/med  connFlip/jerk/med")
    for f in FILES:
        p = ROOT / f
        root = ET.parse(str(p)).getroot()
        xsd = schema.validate(etree.parse(str(p)))
        pv_ok = not check_file(str(p))["violations"]
        au = audit_file(p)
        rc = route_continuity(root)
        gin = max((r["gap_in"] for r in rc), default=0.0)
        gout = max((r["gap_out"] for r in rc if not math.isnan(r["gap_out"])), default=0.0)
        cq = curvature_audit(root)
        cq_ok = all(q["flips_per_100m_max"] <= LIM[t]["flips"]
                    and q["jerk_max"] <= LIM[t]["jerk"]
                    and q["seg_median_len"] >= LIM[t]["med"]
                    for t, q in cq.items())
        ok = (xsd and pv_ok and au["kappa_step_max"] < 1e-6
              and au["lane_edge_step_max"] < 0.05 and gin < 0.01 and gout < 0.01 and cq_ok)
        all_ok &= ok
        lg, cn = cq.get("leg", {}), cq.get("conn", {})
        print(f"{'OK ' if ok else 'FAIL'} {f:28s} {str(xsd):5s} {str(pv_ok):5s} "
              f"{au['kappa_step_max']:.1e} {au['lane_edge_step_max']:.3f}m "
              f"{gin * 100:.1f}/{gout * 100:.1f}cm  "
              f"{lg.get('flips_per_100m_max', 0):5.1f}/{lg.get('jerk_max', 0):6.1f}/"
              f"{lg.get('seg_median_len', 0):5.1f}  "
              f"{cn.get('flips_per_100m_max', 0):5.1f}/{cn.get('jerk_max', 0):6.1f}/"
              f"{cn.get('seg_median_len', 0):4.1f}")

    print("== G6 esmini RoadManager 独立行驶 ==")
    r = subprocess.run([sys.executable, "-u", str(ROOT / "scripts/esmini_rm_check.py")]
                       + [str(ROOT / f) for f in FILES],
                       capture_output=True, cwd=str(ROOT))
    txt = r.stdout.decode("gbk", errors="replace")
    for line in txt.splitlines():
        if "PASS" in line or "CHECK" in line:
            print("  " + line.strip())
    all_ok &= (r.returncode == 0)
    print("\n=>", "全部门禁 PASS" if all_ok else "存在 FAIL")
    sys.exit(0 if all_ok else 1)


if __name__ == "__main__":
    main()
