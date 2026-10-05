"""python -m mapforge.score — generate the Jinfeng 14-file matrix with the unchanged CLI and score it.

Example:
    python -m mapforge.score --out out/scoreboard/20261003-baseline --register "默认CLI基线"
"""
from __future__ import annotations

import argparse
import datetime as dt
import json
import subprocess
import sys
from pathlib import Path

from mapforge.validate import scoreboard as sb

REGISTRY = sb.ROOT / "experiments" / "registry.jsonl"


def _git(*args):
    proc = subprocess.run(["git", "-c", "safe.directory=*", *args], cwd=str(sb.ROOT),
                          capture_output=True, text=True)
    return proc.stdout.strip() if proc.returncode == 0 else None


def _discover(run_dir: Path, cases, pipelines):
    return [{"case": label, "pipeline": p, "artifact": str(run_dir / f"{p}-{label}.xodr")}
            for label, _ in cases for p in pipelines]


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="python -m mapforge.score", description=__doc__.splitlines()[0])
    ap.add_argument("--out", required=True, type=Path, help="运行目录（建议 out/scoreboard/<日期-名称>）")
    ap.add_argument("--no-generate", action="store_true", help="只评分目录里已有的 <pipeline>-<case>.xodr")
    ap.add_argument("--from", dest="from_dir", type=Path,
                    help="以已有运行目录为输入，配合 --transform 做后处理变体")
    ap.add_argument("--transform", choices=sorted(sb.TRANSFORMS), help="后处理变体名称")
    ap.add_argument("--cases", default="", help="逗号分隔的路口标签，缺省为全部 7 个")
    ap.add_argument("--pipelines", default="map,shp")
    ap.add_argument("--workers", type=int, default=4)
    ap.add_argument("--policy", type=Path, default=sb.POLICY)
    ap.add_argument("--shp-mouth", choices=("envelope", "legacy"), default="envelope",
                    help="SHP 路口口部：envelope=口部前移（默认）| legacy=旧的最远车道端点")
    ap.add_argument("--mouth-margin", type=float, default=3.0, help="envelope 口部前移距离 [m]")
    ap.add_argument("--post", default="c2",
                    help="CLI 生成后的后处理变体（默认 c2）；none=只生成，供 --transform 实验用")
    ap.add_argument("--register", metavar="NOTE", help="把本次结果追加到 experiments/registry.jsonl")
    a = ap.parse_args(argv)

    wanted = {x for x in a.cases.split(",") if x}
    cases = tuple(c for c in sb.CASES if not wanted or c[0] in wanted)
    pipelines = tuple(p for p in a.pipelines.split(",") if p)
    run_dir = a.out.resolve()
    run_dir.mkdir(parents=True, exist_ok=True)
    started = dt.datetime.now().astimezone().isoformat(timespec="seconds")

    generation = None
    if bool(a.from_dir) != bool(a.transform):
        ap.error("--from 和 --transform 必须同时给出")
    if a.transform:
        generation = sb.transform(a.from_dir.resolve(), run_dir, a.transform, cases, pipelines)
        entries = generation
    elif a.no_generate:
        entries = _discover(run_dir, cases, pipelines)
    else:
        generation = sb.generate(run_dir, cases, pipelines, a.workers, shp_mouth=a.shp_mouth,
                                 mouth_margin=a.mouth_margin, post=a.post)
        entries = generation
    board = sb.score(run_dir, entries, a.policy)
    board.update(created=started, generation=generation,
                 git={"head": _git("rev-parse", "HEAD"),
                      "dirty": bool(_git("status", "--porcelain", "--untracked-files=no"))},
                 python=sys.version.split()[0])
    (run_dir / "scoreboard.json").write_text(json.dumps(board, ensure_ascii=False, indent=1), encoding="utf-8")
    (run_dir / "scoreboard.md").write_text(sb.markdown(board), encoding="utf-8")

    if a.register:
        REGISTRY.parent.mkdir(exist_ok=True)
        row = {"run": run_dir.name, "created": started, "note": a.register, "git": board["git"],
               "input": ({"from": a.from_dir.name, "transform": a.transform} if a.transform else
                         {"cli": "mapforge.cli convert", "shp_mouth": a.shp_mouth, "mouth_margin_m": a.mouth_margin,
                          "post": a.post}),
               "policy": board["policy"], "files": board["files"], "tier_pass": board["tier_pass"],
               "errors": sum(1 for r in board["rows"] if r.get("error"))}
        with REGISTRY.open("a", encoding="utf-8") as fh:
            fh.write(json.dumps(row, ensure_ascii=False) + "\n")

    print(f"评分板：{run_dir / 'scoreboard.md'}")
    print("等级通过数：" + "，".join(f"{k} {v}/{board['files']}" for k, v in board["tier_pass"].items()))
    for r in board["rows"]:
        if r.get("error"):
            print(f"  错误 {r['pipeline']}-{r['case']}: {r['error']}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
