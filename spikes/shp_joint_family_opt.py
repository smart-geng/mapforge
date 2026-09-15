"""SHP OpenDRIVE 车道族全局站值优化 spike。

只调整现有 laneSection 端点宽度，不新增 planView 或 laneSection；用于验证
“来源容差 + 共享包络 + 非负宽度 + 物理车道三阶平滑”的联合最小二乘。
"""
from __future__ import annotations

import argparse
import copy
import math
import sys
from pathlib import Path

import numpy as np
from lxml import etree
from scipy.interpolate import CubicSpline
from scipy.optimize import lsq_linear

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from mapforge.validate.g11 import lane_edges_kinematics_at


def _width_at(lane, u):
    records = sorted(lane.findall("width"), key=lambda x: float(x.get("sOffset", "0")))
    record = max((x for x in records if float(x.get("sOffset", "0")) <= u + 1e-8),
                 key=lambda x: float(x.get("sOffset", "0")), default=records[0])
    q = u - float(record.get("sOffset", "0"))
    a, b, c, d = (float(record.get(key, "0")) for key in "abcd")
    return a + b * q + c * q * q + d * q * q * q


def _third_divided_row(samples):
    xs = np.asarray([item[0] for item in samples], float)
    coeff = np.asarray([
        1.0 / np.prod([xs[i] - xs[j] for j in range(4) if j != i])
        for i in range(4)
    ])
    row = sum(coeff[i] * samples[i][1] for i in range(4))
    const = sum(coeff[i] * samples[i][2] for i in range(4))
    return row, float(const)


def optimize_road(tree, road_id: str, base_lambda: float = 100000.0):
    road = tree.getroot().find(f"road[@id='{road_id}']")
    if road is None:
        raise ValueError(f"road 不存在: {road_id}")
    length = float(road.get("length"))
    sections = road.findall("lanes/laneSection")
    starts = [float(x.get("s")) for x in sections]
    ends = starts[1:] + [length]
    report = {}

    for side, sign in (("left", 1.0), ("right", -1.0)):
        occurrences = {}
        parent = {}
        for si, section in enumerate(sections):
            lanes = sorted(section.findall(f"{side}/lane"),
                           key=lambda x: abs(int(x.get("id"))))
            for lane in lanes:
                key = (si, lane.get("id"))
                occurrences[key] = lane
                for endpoint in (0, 1):
                    parent[(key, endpoint)] = (key, endpoint)

        def find(node):
            while parent[node] != node:
                parent[node] = parent[parent[node]]
                node = parent[node]
            return node

        def union(a, b):
            ra, rb = find(a), find(b)
            if ra != rb:
                parent[rb] = ra

        successors = {}
        predecessors = set()
        for (si, lane_id), lane in occurrences.items():
            if si + 1 >= len(sections):
                continue
            link = lane.find("link/successor")
            next_id = link.get("id") if link is not None else lane_id
            nxt = (si + 1, next_id)
            if nxt in occurrences:
                successors[(si, lane_id)] = nxt
                predecessors.add(nxt)
                union((((si, lane_id), 1)), (nxt, 0))

        roots = {find(node) for node in parent}
        variable = {root: i for i, root in enumerate(sorted(roots, key=str))}
        nvar = len(variable)
        observations = {root: [] for root in roots}
        for key, lane in occurrences.items():
            si = key[0]
            section_length = ends[si] - starts[si]
            observations[find((key, 0))].append(_width_at(lane, 0.0))
            observations[find((key, 1))].append(_width_at(lane, section_length))
        initial = np.asarray([
            np.median(observations[root])
            for root, _index in sorted(variable.items(), key=lambda item: item[1])
        ])

        rows, values = [], []

        def add(row, value, weight):
            rows.append(np.asarray(row, float) * weight)
            values.append(float(value) * weight)

        for index, value in enumerate(initial):
            row = np.zeros(nvar)
            row[index] = 1.0
            add(row, value, 0.5)

        center_expr = {}
        for si, section in enumerate(sections):
            lanes = sorted(section.findall(f"{side}/lane"),
                           key=lambda x: abs(int(x.get("id"))))
            if not lanes:
                continue
            for endpoint, station in ((0, starts[si]), (1, ends[si])):
                sample_s = station + (1e-5 if endpoint == 0 else -1e-5)
                edges = lane_edges_kinematics_at(road, sample_s, side)
                base = float(edges[0][0])
                total_row = np.zeros(nvar)
                for lane in lanes:
                    total_row[variable[find(((si, lane.get("id")), endpoint))]] += 1.0
                total = sum(_width_at(lane, 0.0 if endpoint == 0
                                      else ends[si] - starts[si]) for lane in lanes)
                add(total_row, total, 80.0)
                for k, lane in enumerate(lanes):
                    row = np.zeros(nvar)
                    for j in range(k):
                        row[variable[find(((si, lanes[j].get("id")), endpoint))]] += sign
                    row[variable[find(((si, lane.get("id")), endpoint))]] += 0.5 * sign
                    target = float((edges[k][0] + edges[k + 1][0]) / 2.0)
                    center_expr[((si, lane.get("id")), endpoint)] = (
                        row, base, target, station)
                    add(row, target - base, 18.0)

        chains = []
        for start in [key for key in occurrences if key not in predecessors]:
            chain, current = [], start
            while current is not None and current not in chain:
                chain.append(current)
                current = successors.get(current)
            chains.append(chain)

        risks = []
        for chain in chains:
            samples = [
                (center_expr[(chain[0], 0)][3], center_expr[(chain[0], 0)][0],
                 center_expr[(chain[0], 0)][1])
            ]
            samples.extend((center_expr[(key, 1)][3], center_expr[(key, 1)][0],
                            center_expr[(key, 1)][1]) for key in chain)
            local = []
            for q in range(len(samples) - 3):
                row, const = _third_divided_row(samples[q:q + 4])
                raw = abs(float(row @ initial + const))
                local.append(raw)
            risk = max(local, default=0.0)
            # 三阶差分是三阶导数的 1/6；用 60 km/h 做统一风险排序，
            # 实际速度仍由最终 G11-D 独立复算。
            proxy = 6.0 * risk * (60.0 / 3.6) ** 3
            factor = min(12.0, max(1.0, math.sqrt(max(proxy, 1.0))))
            risks.append((proxy, factor, chain))
            for q in range(len(samples) - 3):
                row, const = _third_divided_row(samples[q:q + 4])
                add(row, -const, base_lambda * factor)

        answer = lsq_linear(
            np.vstack(rows), np.asarray(values),
            bounds=(np.zeros(nvar), np.full(nvar, 8.0)),
            tol=1e-11, lsmr_tol="auto", max_iter=2000,
        ).x

        center_deviation = [
            abs(float(row @ answer + base) - target)
            for row, base, target, _station in center_expr.values()
        ]
        for chain in chains:
            xs = [starts[chain[0][0]]] + [ends[key[0]] for key in chain]
            ys = [answer[variable[find((chain[0], 0))]]] + [
                answer[variable[find((key, 1))]] for key in chain]
            if len(xs) >= 3:
                curve = CubicSpline(xs, ys, bc_type="natural")
                slopes = curve(np.asarray(xs), 1)
            else:
                slope = (ys[-1] - ys[0]) / max(xs[-1] - xs[0], 1e-9)
                slopes = np.full(len(xs), slope)
            for q, key in enumerate(chain):
                lane = occurrences[key]
                section_length = xs[q + 1] - xs[q]
                w0, w1 = float(ys[q]), float(ys[q + 1])
                m0, m1 = float(slopes[q]), float(slopes[q + 1])
                c = (3 * (w1 - w0) - (2 * m0 + m1) * section_length) / section_length ** 2
                d = (-2 * (w1 - w0) + (m0 + m1) * section_length) / section_length ** 3
                for old in list(lane.findall("width")):
                    lane.remove(old)
                width = etree.Element(
                    "width", sOffset="0", a=f"{w0:.12g}", b=f"{m0:.12g}",
                    c=f"{c:.12g}", d=f"{d:.12g}")
                lane.insert(1 if lane.find("link") is not None else 0, width)

        report[side] = {
            "variables": nvar,
            "chains": len(chains),
            "center_deviation_max_m": max(center_deviation, default=0.0),
            "risk_proxy_max": max((x[0] for x in risks), default=0.0),
            "risk_factor_max": max((x[1] for x in risks), default=1.0),
        }
    return report


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("input", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--road", required=True)
    parser.add_argument("--lambda", dest="base_lambda", type=float, default=100000.0)
    args = parser.parse_args()
    tree = etree.parse(str(args.input))
    report = optimize_road(tree, args.road, args.base_lambda)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    tree.write(str(args.output), encoding="utf-8", xml_declaration=True, pretty_print=True)
    print(report)


if __name__ == "__main__":
    main()
