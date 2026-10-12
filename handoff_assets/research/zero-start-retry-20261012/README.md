# 0630 进口 road 13：零宽起点重拟合——证据（2026-10-12）

说明见 [docs/泛化-0630进口道路零宽起点重拟合-2026-10-12.md](../../../docs/泛化-0630进口道路零宽起点重拟合-2026-10-12.md)。

运行环境：同一台 Linux 机器，Python 3.11.16，esmini v3.6.0 Linux 库（与上一轮相同），判级按 0.10-draft。基线是现行 `main`（`4714ba3`）的代码，它的评分板、泛化集和慢路口结果在上一轮证据里：

- [`../surface-seams-20261012/scoreboard-seams.json`](../surface-seams-20261012/scoreboard-seams.json)
- [`../surface-seams-20261012/generalization-seams.json`](../surface-seams-20261012/generalization-seams.json)
- [`../surface-seams-20261012/slow-junctions.sha256`](../surface-seams-20261012/slow-junctions.sha256)

**结果**：

| 文件 | 内容 |
|---|---|
| `scoreboard-zero-start.json`、`scoreboard-zero-start-compare.txt` | 本次评分板（SHP、MAP 分两次跑，`merge_boards.py` 合并）及与基线逐项对比（`compare_boards.py` 输出里 `base` 是基线，`anchor` 是本次）：0/14 行变化，T1/T2 14/14 |
| `scoreboard-xodr.sha256` | 14 份评分板 XODR 的哈希，与基线相同 |
| `generalization-zero-start.json`、`generalization-zero-start-compare.txt` | 本次泛化集评分及对比：只有 0630 一行变化（等级不变，T2 不过的项 7 → 5） |
| `generalization-xodr.sha256` | 模块内转出的 10 个 XODR 哈希：0630 变化，其余 9 个相同。0412 第一次碰到 3600 s 上限，单独以同一命令、同一上限重跑 |
| `slow-junctions.sha256` | 0707104 不设上限转换的输出哈希：XODR 与基线相同；061310 没有重跑（基线里没有被 `lane_refit` 跳过的路，重试不会被调用） |
| `related-tests.txt` | 相关测试前后逐项对比 |
| `gen_part.py`、`gen_score_all.py` | 泛化集分两批转换（模块自己的 `generate`，同一 3600 s 上限），合并后用模块自己的 `score` 评全部 16 行 |
| `merge_boards.py`、`skips.py`、`junit_compare.py`、`road_diff.py` | 合并分开跑的评分板、列出输出里 `lane_refit` 的跳过记录、测试逐项对比、两份 XODR 逐路对比 |

比较脚本 `compare_boards.py`、`bytes_compare.py`、`t2_where.py`、`score_two.py` 与上一轮相同，见 [`../surface-seams-20261012/`](../surface-seams-20261012/)。

**0630 road 13**（基线输出对本次输出）：

| 文件 | 内容 |
|---|---|
| `0630-road13-source-lanes.json`、`source_lanes_r13.py` | road 13 起点三条原件车道的属性、中心线与边界起点、前驱 |
| `0630-road13-xsec-before.json`、`0630-road13-xsec-after.json`、`cross_sections.py` | 18 个桩号的横断面：写出的车道边缘与原件边界在同一法线上的位置 |
| `0630-road13-edges-before.json`、`0630-road13-edges-after.json`、`road_edges.py` | 各段道路外缘偏差（评分板的配对口径） |
| `road13-s0-20-before-after.png`、`road13-full-before-after.png`、`plot_before_after.py` | 前后车道边缘与原件边界（红） |
| `0630-lane-refit-stage.json`、`c_stage.py` | 只跑 `lane_refit` 一步时 0630 的报告：road 13 由重拟合写出（`retried`） |
| `0630-t2-where-after.json` | 本次 0630 剩下 5 项 T2 失败逐项定位（之前的见 [`../surface-seams-20261012/0630-t2-where.json`](../surface-seams-20261012/0630-t2-where.json)） |
| `0630-lane-centre-diff.json`、`lane_centre_diff.py` | 车道中心（去冲突）逐原件车道前后对比：变化的 6 条车道 |

**0627、0707104 的 road 10**（需要用户定，代码未进入仓库）：

| 文件 | 内容 |
|---|---|
| `0627-t2-where.json` | 0627 基线的 T2 失败定位（road 10 整条保留转换器几何） |
| `0627-road10-relative-trial.json`、`0707104-road10-relative-trial.json`、`refit_relative_try.py` | 试验：出负宽的边界链改为相对内侧边界拟合。0627 road 10 能写出，0707104 仍是 −0.49 m |
| `0627-road10-edges-before.json`、`0627-road10-edges-relative-trial.json` | 只跑 `lane_refit` 一步时，0627 road 10 的外缘偏差：基线对试验 |
| `refit_trace.py` | 追踪一条路的负宽检查：交叉修复前各车道宽度的最小值、每次修复尝试 |

脚本里有当时云端临时目录的绝对路径。
