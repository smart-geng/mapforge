# 车道换位一起拟合——证据（2026-10-14）

说明见 [docs/泛化-0711车道换位一起拟合-2026-10-14.md](../../../docs/泛化-0711车道换位一起拟合-2026-10-14.md)。

运行环境：同一台 Linux 机器（4 核），Python 3.11.16，esmini v3.6.0 Linux 库（与前几轮相同），判级按 0.11-draft。

基线是现行 `main`（`8a959e8`）。它的代码与上一轮剪枝护栏验证用的版本只差 `lane_refit.py` 模块说明里的一句，语法树相同（见 [`../prune-guard-20261013/`](../prune-guard-20261013/) 的 `ast_same.py`）。所以基线的转换输出和评分就是那一轮的：

- 评分板：`sb-pg`，评分 `scoreboard-prune-guard.json`；
- 泛化集：`gen-pg`，评分 `generalization-prune-guard.json`；
- 慢路口：`gen-slow-pg`，评分 `slow-0707104-score.json`。

**代码哈希（SHA256）**：

| 文件 | 改动前（`main` `8a959e8`） | 验证用 = 提交 |
|---|---|---|
| `mapforge/ops/lane_refit.py` | `51cd2da41c9569741f66b145ee57ceae6c00aba061632639f855b0c95d9f9e87` | `939c65e460993e063868158fbc145f9eb4c921a07310207aea916542ea321664` |
| `mapforge/ops/refit_lane_swap.py` | 新文件 | `30ca3c99af130648112eaa2e61296e6bc889aa14f0a669174bea0be5cbc3d275` |
| `tests/test_refit_lane_swap.py` | 新文件 | `1043c2f0f96c6323008382efa5a46cb962a8e7c177342738b3e8d7fc8d6f8584` |

- 三组转换（`launch_sw.sh`）和相关测试都在三个文件最后一次修改之后启动，用的就是提交的字节。
- `scripts/bound_files.py`：改动前的 `lane_refit.py` 和提交的三个文件都判 free。
- 本目录不在 `bound_files.py` 的扫描范围内，所以这里写全文哈希，文档里只写前 8 位。

**BLAS 线程数**：两轮的转换都用 `OPENBLAS_NUM_THREADS=1`（本轮 `launch_sw.sh`、`run_st2.sh`），评分用默认线程。评分板 14 份和泛化集未变的 8 个路口都逐字节相同，没有线程数造成的差异。

**结果**：

| 文件 | 内容 |
|---|---|
| `scoreboard-lane-swap.json`、`scoreboard-lane-swap-compare.txt` | 本次评分板与逐项对比，基线是上一轮的 `scoreboard-prune-guard.json`：<br>- 14 份一次跑完，`sb_gen.py` 转换，`--no-generate` 评分；<br>- `compare_boards.py` 输出里 `base` 是基线，`anchor` 是本次；<br>- T1/T2 14/14，0/14 行变化 |
| `scoreboard-xodr.sha256` | 14 份评分板 XODR 的哈希，全部与基线相同；其余输出只差路径 |
| `generalization-lane-swap.json`、`generalization-lane-swap-compare.txt` | 本次泛化集评分及对比，基线是上一轮的 `generalization-prune-guard.json`：0411、0711 两行变化，0711 T2 FAIL → PASS |
| `generalization-xodr.sha256` | 模块内转出的 10 个 XODR 哈希：0411、0711 随本次改动变化，其余 8 个逐字节相同 |
| `slow-junctions.sha256` | 0707104 不设上限转换的全部输出哈希。XODR、g8、交付裁决等相同；`convert.log`、g11、质量报告只差路径（`norm_compare.py`），所以没有重新评分。061310 没有重跑 |
| `related-tests.txt` | 相关测试前后逐项对比（`junit_compare.py`，见 [`../zero-start-retry-20261012/`](../zero-start-retry-20261012/)）。"之前"是上一轮同机同命令的结果 |
| `stage-diff.json`、`run_st2.sh` | 筛查：<br>- 输入是上一轮筛查的 19 个 `lane_refit` 输入（`*.raw.xodr`），由 [`../prune-guard-20261013/`](../prune-guard-20261013/) 的 `stage_raw.py` 生成；<br>- 用本次代码只跑 `lane_refit` 一步（`c_stage_hold.py`，见 [`../hold-width-retry-20261013/`](../hold-width-retry-20261013/)）；<br>- 与上一轮同一步的结果（现行 `main` 的代码）逐路比较（`stage_diff.py`），并列出护栏记录；<br>- 只有 0411 road 13、0711 road 11 不同 |
| `swaps-found.json`、`swaps_found.py` | 本次 `lane_refit` 记录的换位（`lane_swaps`）和失败的换位（`lane_swaps_failed`）：只有上面两处，没有失败 |
| `divider-slopes.txt`、`divider_slopes.py` | 两处分界线的总宽、斜率、曲率目标要的拐角半长与实际取的，以及拐角在事件点离原件折角的距离（μ·h/6） |
| `hash_lists_sw.py`、`launch_sw.sh`、`synth_numbers.py` | 生成三份哈希清单；本轮三组转换的启动脚本；合成测试里的数字（文档"改法"引用） |

泛化集分批转换与合并评分的 `gen_part.py`、`gen_score_all.py`，以及下列脚本与前几轮相同，这里不再复制：

- `sb_gen.py`、`stage_raw.py`、`stage_diff.py`、`runs_lanes.py`、`edge_curv_where.py`、`roads_changed.py`、`ast_same.py`：[`../prune-guard-20261013/`](../prune-guard-20261013/)；
- `c_stage_hold.py`、`norm_compare.py`、`plot_st.py`、`plot_before_after2.py`：[`../hold-width-retry-20261013/`](../hold-width-retry-20261013/)；
- `cross_sections.py`、`road_edges.py`：[`../zero-start-retry-20261012/`](../zero-start-retry-20261012/)；
- `compare_boards.py`：[`../mouth-anchor-20261009/`](../mouth-anchor-20261009/)。

**两条路的前后对比**（基线输出对本次输出，`analyze_sw.sh` 生成）：

| 文件 | 内容 |
|---|---|
| `0711-road11-xsec-before.json`、`0711-road11-xsec-after.json` | road 11 横断面，s 0.5–77.5 每 0.5 m：写出的车道边缘与原件边界在同一法线上的位置（`cross_sections.py`） |
| `0411-road13-xsec-before.json`、`0411-road13-xsec-after.json` | 0411 road 13 横断面，s 60–142 每 0.5 m |
| `xsec-table.txt`、`xsec_table.py` | 由横断面得出：外缘逐段离原件最外边界的最大距离、换位段内分界线离原件分界线的最大距离 |
| `0711-road11-edges-*.json`、`0411-road13-edges-*.json` | 各段道路外缘偏差（`road_edges.py`，每条记录各自选原件边界） |
| `boundary-runs-by-lane.json` | 评分口径（`boundary_inside_runs`）逐原件车道的最大偏差，两个路口前后对比（`runs_lanes.py`） |
| `edge-curvature.txt` | 两条路车道边缘曲率逐段最大（`edge_curv_where.py`，评分板的车道边缘曲率采样和公式） |
| `fairness-peaks.txt`、`fair_where.py` | 光顺项（`lane_fairness`）各类车道中心曲率最大的三条：哪条路、哪条车道、哪一段 |
| `g8-exclusions.txt`、`g8_exclusions_diff.py`、`g8_widths.py` | 分类变化的 G8 排除项、这些零宽段在 G8 采样点上的宽度、交付裁决的阻断与复核理由 |
| `0711-road11-s50-78-before-after.png`、`0711-road11-s0-12-before-after.png`、`0411-road13-s60-95-before-after.png` | 前后的车道边缘与原件边界（平面） |
| `0711-road11-st-before.png`、`0711-road11-st-after.png`、`0411-road13-st-before.png`、`0411-road13-st-after.png` | 同上，路自己的 (s, t) 坐标 |

脚本里有当时云端临时目录的绝对路径。
