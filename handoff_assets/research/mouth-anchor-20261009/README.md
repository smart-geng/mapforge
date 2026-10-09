# 0621 边界偏差：口部区锚点的定位与前后对比证据

决定、实现与结论见 [工作台-0621边界偏差与口部锚点](../../../docs/工作台-0621边界偏差与口部锚点-2026-10-09.md)。

## 运行环境

Claude Code 云端容器，Linux，4 核，Python 3.11.16，资料按 `handoff_assets/20261009-*` 恢复。

- **esmini**：用官方 Linux 版 v3.6.0（`esmini-bin_Linux.zip`，SHA256 `2f45358d21d0dc061692edd97cf9ad6b30d8721e7c0bcc791cff1e1911aaed87`，与仓库内 Windows DLL 同一 git 修订 `131a5651`）。在实验目录里放到 `esmini/bin/esminiRMLib.dll` 的位置（`scripts/esmini_rm_check.py` 用 `ctypes.CDLL` 按路径加载）；仓库内的 DLL 与检查脚本未改。`esmini_parity.txt` 是对 25 个已存产物的复核：`esmini_pass` 与最大缝隙全部与已存 Windows 结果相同。
- **基线树与候选树**：
  - 基线树：HEAD `2630c55` 的导出，`lane_refit.py` 加了环境变量开关和触发日志，开关关闭时代码路径与 HEAD 相同。
  - 候选树：HEAD 加本次 `lane_refit.py` 改动。
  - 两棵树都在同一台机器上跑 `python -m mapforge.score` 与 `python -m mapforge.validate.generalization`。

## 文件

| 文件 | 内容 |
|---|---|
| `diagnosis.md` | road 10 偏差定位记录：逐边界有符号残差剖面、逐阶段对比、中心边界拟合的观测与顶点 |
| `scoreboard-base.json` / `scoreboard-formal.json` | 评分板 14 份：基线 / 正式代码 |
| `scoreboard-unrestricted.json` | 第一版（无条件锚点）的评分板，node17、node18 的 T2 FAIL |
| `scoreboard-formal-compare.txt`、`scoreboard-unrestricted-compare.txt` | 逐行逐项差异（`*` 为分级指标） |
| `anchor-log-node17.jsonl`、`anchor-log-node18.jsonl` | 第一版在 node17、node18 上触发锚点的口部：被删顶点、锚点、区内斜率 |
| `generalization-base.json` / `generalization-formal.json`、`generalization-formal-compare.txt` | 泛化集 16 个：基线 / 正式代码与逐项差异 |
| `generalization-slow.json` | 061310、0707104 两个路口：Linux 上单个转换超过泛化集模块的 3600 s 上限（Windows 上为 1120 s、880 s），另行不设上限转换后的基线与候选评分 |
| `0621-base.json` / `0621-formal.json` | 0621 工作台运行的重建路面经默认后处理后的完整评分行（0.10-draft） |
| `compare_boards.py`、`esmini_parity.py` | 对比与 esmini 复核脚本（运行于实验目录，路径按当时目录写死，仅供复查方法） |
