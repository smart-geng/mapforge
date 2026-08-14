# -*- coding: utf-8 -*-
"""odrviewer 视觉全扫描：14 文件 × 多机位无窗截帧 → 每文件一张拼图，供逐张目检。

机位：动态顶视 + 两个对角透视（z=55 俯瞰路口）+ 低机位沿路视角。
用法：.venv/Scripts/python scripts/visual_sweep.py [out_dir]
"""
import glob
import os
import subprocess
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
ODRV = ROOT / "esmini" / "bin" / "odrviewer.exe"

FILES = [("direct", f"out/direct_xodr/{n}.xodr") for n in
         ("node3", "node4", "NODE5", "node13", "node16", "node17", "node18")] + \
        [("m2x", f"out/m2x/{n}.xodr") for n in
         ("node3", "node4", "NODE5", "node13", "node16", "node17", "node18")]

SHOTS = [
    ("top", ["--camera_mode", "top"]),
    ("p1", ["--camera_mode", "custom_fixed",
            "--custom_fixed_camera", "70,-100,60,2.2,0.5"]),
    ("p2", ["--camera_mode", "custom_fixed",
            "--custom_fixed_camera", "-100,70,60,-0.6,0.5"]),
    ("low", ["--camera_mode", "custom_fixed",
             "--custom_fixed_camera", "8,-130,6,1.62,0.04"]),
]


def capture(xodr: Path, args, workdir: Path):
    for f in workdir.glob("*.tga"):
        f.unlink()
    cmd = [str(ODRV), "--odr", str(xodr), "--headless", "--capture_screen",
           "--density", "1.0", "--ground_plane", "on"] + args
    try:
        subprocess.run(cmd, cwd=str(workdir), timeout=7,
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    except subprocess.TimeoutExpired:
        pass
    frames = sorted(workdir.glob("*.tga"))
    return frames[len(frames) // 2] if frames else None


def main():
    from PIL import Image, ImageDraw, ImageFile
    ImageFile.LOAD_TRUNCATED_IMAGES = True
    out_dir = Path(sys.argv[1]) if len(sys.argv) > 1 else ROOT / "out" / "preview" / "sweep"
    out_dir.mkdir(parents=True, exist_ok=True)
    work = Path(tempfile.mkdtemp(prefix="odrsweep_"))
    for pipe, rel in FILES:
        tiles = []
        for label, args in SHOTS:
            frame = capture(ROOT / rel, args, work)
            if frame is None:
                continue
            im = Image.open(frame).resize((960, 540))
            d = ImageDraw.Draw(im)
            d.text((12, 10), f"{Path(rel).stem}/{pipe}/{label}", fill=(255, 60, 60))
            tiles.append(im)
        if not tiles:
            continue
        sheet = Image.new("RGB", (960 * 2, 540 * 2), (20, 20, 20))
        for i, t in enumerate(tiles[:4]):
            sheet.paste(t, ((i % 2) * 960, (i // 2) * 540))
        dst = out_dir / f"{pipe}_{Path(rel).stem}.png"
        sheet.save(dst)
        print(dst)
    for f in work.glob("*.tga"):
        f.unlink()


if __name__ == "__main__":
    main()
