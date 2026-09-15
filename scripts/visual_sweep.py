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
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
ODRV = ROOT / "esmini" / "bin" / "odrviewer.exe"

FILES = [("direct", f"out/direct_xodr/{n}.xodr") for n in
         ("node3", "node4", "NODE5", "node13", "node16", "node17", "node18")] + \
        [("m2x", f"out/m2x/{n}.xodr") for n in
         ("node3", "node4", "NODE5", "node13", "node16", "node17", "node18")]

SHOTS = [
    # 用户实际复核命令：无车辆、无地面，600m 固定俯视，最容易暴露道路外缘蛇形。
    ("user", ["--density", "0", "--ground_plane", "off",
              "--camera_mode", "custom_fixed",
              "--custom_fixed_camera", "0,-50,600,1.5708,1.45"]),
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
    # Never supply conflicting options: caller's no-vehicle/no-ground review
    # must not depend on the consumer's first/last-option precedence.
    defaults=[]
    for flag,value in (("--density","0"),("--ground_plane","off")):
        if flag not in args:defaults.extend([flag,value])
    cmd = [str(ODRV), "--odr", str(xodr), "--headless", "--capture_screen"] + defaults + args
    proc = subprocess.Popen(cmd, cwd=str(workdir),
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    deadline = time.monotonic() + 12.0
    while time.monotonic() < deadline:
        if len(list(workdir.glob("*.tga"))) >= 3 or proc.poll() is not None:
            break
        time.sleep(0.1)
    if proc.poll() is None:
        proc.terminate()
        try:
            proc.wait(timeout=2)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait(timeout=2)
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
        shown = tiles[:6]
        columns = min(3, len(shown))
        rows = (len(shown) + columns - 1) // columns
        sheet = Image.new("RGB", (960 * columns, 540 * rows), (20, 20, 20))
        for i, t in enumerate(shown):
            sheet.paste(t, ((i % columns) * 960, (i // columns) * 540))
        dst = out_dir / f"{pipe}_{Path(rel).stem}.png"
        sheet.save(dst)
        print(dst)
    for f in work.glob("*.tga"):
        f.unlink()


if __name__ == "__main__":
    main()
