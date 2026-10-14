# 出生段太短时沿车道链接量开口斜率——证据（2026-10-14）

说明见 [docs/泛化-短出生段提前出生-2026-10-14.md](../../../docs/泛化-短出生段提前出生-2026-10-14.md)。

运行环境：同一台 Linux 机器（4 核），Python 3.11.16，esmini v3.6.0 Linux 库（与前几轮相同），判级按 0.11-draft。

基线是现行 `main`（`df7455b`）。它的代码就是上一轮（车道换位）验证提交的代码，所以基线的转换输出大多直接用上一轮的（[`../lane-swap-20261014/`](../lane-swap-20261014/)）：

- 评分板：`sb-sw`，评分 `scoreboard-lane-swap.json`；
- 泛化集：`gen-sw`，评分 `generalization-lane-swap.json`；
- 慢路口 0707104：`gen-slow-sw`，本轮按同一脚本重评（`slow-0707104-main.json`）；
- 慢路口 061310 上一轮没有重跑，本轮用现行代码单线程重新转换（`gen-slow-base13`），评分 `slow-061310-main.json`。

**代码哈希（SHA256）**：

| 文件 | 改动前（`main` `df7455b`） | 验证用 = 提交 |
|---|---|---|
| `mapforge/ops/lane_refit.py` | `939c65e460993e063868158fbc145f9eb4c921a07310207aea916542ea321664` | `4e97db36eeedc181ea4b18e54fd563da79dfbc1980cce7ec9e4f3f15ac40dc09` |
| `mapforge/ops/refit_birth_window.py` | 新文件 | `78396007b605e6dbdc8583a4f77d3b1f9553698df1d772bd69c1b446e24cf552` |
| `tests/test_refit_birth_window.py` | 新文件 | `53f3ca5ef8a23a35fee529cd9528648793105518914973c1e652c7abefffb561` |

**四次启动**（每次都启动评分板、泛化集和两个慢路口的转换，`OPENBLAS_NUM_THREADS=1`）：

1. 第一版（只有窗口）：`launch_bw.sh`、`launch_bw_gen.sh`。慢路口 0707104 评分发现跳过的路变差后停掉。0707104 的评分留作 `slow-0707104-first-version.json`。同一次启动里用现行代码（`sw-tree`）转换的 061310 基线（`gen-slow-base13`，`launch_bw.sh` 的最后一个任务）已经跑完，保留作基线。
2. 第二版（加了跳过的路的保护 `redo_skipped`，即最终版 `keep_own_way` 的第一种情况）：`launch_bw2.sh`。两个慢路口跑完，评分发现 061310 road 12 丢了口部路肩（见文档"口部路肩"）；评分板（14 份转完 11 份）和泛化集（转完 5 份）随后为腾出 CPU 停掉，没有评分。这一版的结果留作"无路肩保护"的对照：
   - `slow-061310-no-guard.json`、`slow-061310-no-guard-compare.txt`：061310 的评分及对现行的对比；
   - `road-dev-stage-no-guard.txt`：`road_dev.py` 在第二版 `lane_refit` 一步上的结果（当时没有后缀，后来重算前改名），与第三版只在 061310 road 12 上不同；
   - `stage-diff.json`：第二版只跑 `lane_refit` 一步，与现行同一步逐路比较。
3. 第三版（`keep_own_way` 换回跳过的路和会丢路肩的路，没有距离保护）：`launch_bw3.sh`（`sb-bw3`、`gen-bw3`、`gen-slow-bw3`），全部跑完。两个慢路口和泛化集都没有等级变化，但评分板 shp-node4 的 T2 由 PASS 变 FAIL（`boundary_inside_runs_max_m` 0.345 → 0.366 m，见文档"离原件更远的路"）。这一版的结果留作"只有路肩保护"的对照：
   - `scoreboard-shoulder-guard-only.json`、`scoreboard-shoulder-guard-only-compare.txt`：第三版的评分板及对现行的逐项对比；
   - `screen_bw3.sh`、`stage-diff-guard-vs-main.json`、`stage-diff-guard-vs-noguard.json`：第三版只跑 `lane_refit` 一步，与现行和第二版逐路比较（对现行 8 个输入 10 条路不同，对第二版只有 061310 road 12 不同）；
   - `screen_bw3_events.sh` 在第三版 `lane_refit` 一步上的结果，最终版重算前改名：`event-dev-shoulder-guard.txt`、`road-dev-stage-shoulder-guard.txt`、`event-widths-shoulder-guard.txt`、`event-curvature-stage-shoulder-guard.txt`，与最终版只在 0630 road 11、node4 road 13 上不同（第三版的逐段车道边缘曲率没有保留）。
4. 最终版（`keep_own_way` 另把离原件更远的路换回）：`launch_bw4.sh`（`sb-bw4`、`gen-bw4`、`gen-slow-bw4`），在三个文件最后一次修改之后启动，用的就是提交的字节。相关测试同样在最后一次修改之后跑。

- 筛查的 `lane_refit` 一步输出与完整转换里同一步的中间结果（`lane_refit.apply_with_mouths` 写出后即删的 `*.lane-refit.xodr`，`catch_lr4.sh` / `catch_lr3.sh` / `catch_lr.sh` 在转换过程中复制）逐字节比较（`lr_check.sh`，`lane-refit-check.txt`）：最终版 19 份全部相同；第三版 19 份、第二版 16 份（转换中途停掉，只转到这些）同样全部相同。所以筛查里逐事件、逐路的分析就是完整转换里这一步的结果。
- `scripts/bound_files.py`：改动前后的 `lane_refit.py` 和两个新文件都判 free；`lane_birth_advance.py` 判 BOUND，没改。
- 本目录不在 `bound_files.py` 的扫描范围内，所以这里写全文哈希，文档里只写前 8 位。

**BLAS 线程数**：转换都用 `OPENBLAS_NUM_THREADS=1`（各次启动与基线相同），评分用默认线程。各次转换都单线程，未受本次改动影响的路口都逐字节相同（三份哈希清单），没有出现线程数造成的差异。

**结果**：

| 文件 | 内容 |
|---|---|
| `scoreboard-birth-window.json`、`scoreboard-birth-window-compare.txt` | 本次评分板与逐项对比，基线是上一轮的 `scoreboard-lane-swap.json`：<br>- 14 份一次跑完，`sb_gen.py` 转换，`--no-generate` 评分；<br>- `compare_boards.py` 输出里 `base` 是基线，`anchor` 是本次；<br>- T1/T2 14/14，与基线同级；<br>- 5/14 行变化（shp-node3、node4、node16、node17、node18），其余 9 份 XODR 逐字节相同；<br>- 对第三版只有 shp-node4 一行不同（`boundary_inside_runs_max_m` 0.366 → 0.345 m，T2 FAIL → PASS） |
| `scoreboard-xodr.sha256` | 14 份评分板 XODR 的哈希：基线、最终版、第三版、第二版 |
| `generalization-birth-window.json`、`generalization-birth-window-compare.txt` | 本次泛化集评分及对比，基线是上一轮的 `generalization-lane-swap.json`：<br>- 转出 10（另两个慢路口单独转换），T1 9、T2 4，与基线相同；<br>- 2/16 行变化（0627、0630），等级不变；<br>- 对第三版只有 0630 不同（road 11 换回） |
| `generalization-xodr.sha256` | 模块内转出的 XODR 哈希：基线、最终版、第三版、第二版 |
| `slow-0707104-main.json`、`slow-0707104-birth-window.json`、`slow-061310-main.json`、`slow-061310-birth-window.json`、`slow-junctions-compare.txt` | 两个慢路口（不设上限转换）前后的评分与逐项对比（`score_two.py` 评分，转成 `compare_boards.py` 的格式比较）：<br>- 0707104：等级不变，只有 road 13 变化，与第三版逐字节相同；<br>- 061310：XODR 与现行逐字节相同，评分相同（`slow-061310-birth-window.json` 与 `slow-061310-main.json` 逐字节相同） |
| `slow-junctions.sha256` | 两个慢路口全部输出的哈希：基线、最终版、第三版、第二版 |
| `slow-0707104-first-version.json`、`slow-0707104-first-version-compare.txt` | 第一版的 0707104 评分及对现行的对比：车道拼接曲率跳变 0.174 → 0.450 /m |
| `slow-061310-no-guard.json`、`slow-061310-no-guard-compare.txt` | 第二版（无路肩保护）的 061310 评分及对现行的对比 |
| `scoreboard-shoulder-guard-only.json`、`scoreboard-shoulder-guard-only-compare.txt` | 第三版（只有路肩保护）的评分板及对现行的对比：shp-node4 T2 FAIL |
| `worst-compared.json`、`worst_log.py` | 最终版在 19 个输入上只跑 `lane_refit` 一步，记下每条留了副本的路两种做法离原件最远的点（`refit_birth_window.worst`）和换回的路、原因 |
| `related-tests.txt` | 相关测试前后逐项对比（`junit_compare.py`）。"之前"是上一轮同机同命令的结果 |
| `hash_lists_bw.py`、`launch_bw.sh`、`launch_bw_gen.sh`、`launch_bw2.sh`、`launch_bw3.sh`、`launch_bw4.sh`、`catch_lr.sh`、`catch_lr3.sh`、`catch_lr4.sh`、`lr_check.sh`、`lane-refit-check.txt` | 生成三份哈希清单；四次启动的脚本（第二、三版的脚本头注释当时写的是"最终版"，跑完后补了更正）；转换过程中复制 `lane_refit` 一步的中间结果，与筛查输出的逐字节比较 |

**筛查**（19 个 `lane_refit` 输入与前几轮相同，由 [`../prune-guard-20261013/`](../prune-guard-20261013/) 的 `stage_raw.py` 生成）：

| 文件 | 内容 |
|---|---|
| `screen_bw.sh` | 第二版的筛查：提前报告、新事件、普查、只跑 `lane_refit` 一步（`c_stage_hold.py`），与现行同一步比较，事件附近的偏差、宽度、曲率 |
| `screen_bw4.sh`、`stage-diff-final-vs-main.json`、`stage-diff-final-vs-shoulder-guard.json` | 最终版只跑 `lane_refit` 一步，与现行（[`../lane-swap-20261014/`](../lane-swap-20261014/) 的 `run_st2.sh`）和第三版逐路比较：<br>- 对现行：11 个输入逐字节相同，其余 8 个各只有 1 条路不同（有新提前事件、没有换回的路）；<br>- 对第三版：只有 0630 road 11、node4 road 13 不同 |
| `screen_bw4_events.sh` | 下面五个逐事件、逐路的分析在最终版 `lane_refit` 一步上重算 |
| `adv-main.json`、`adv-main.log`、`adv-window.json`、`adv-window.log`、`adv_report.py` | 现行与本次 `lane_refit.advance_births` 在 19 个输入上的提前报告、提前后整棵树的哈希，本次另记留作备用的副本（`own_way`）。第三版和最终版没改 `advance`，这两份对它们同样成立 |
| `new-events.txt`、`new_events.py` | 只有经窗口才提前的事件：路口、路、车道、原件 s、事件段长、自己段的观测、窗口斜率、提前量 |
| `same-station.txt`、`same_station.py` | 与新提前事件同站点出生的车道（0630 road 12、0707104 road 10）：各自的开口斜率（自己段 / 经窗口）和单独提前时的量。`lane_birth_advance` 每站只取最陡的一个，这一站出生的车道都按它提前 |
| `window-census.json`、`window-census.txt`、`window_census.py` | 19 个输入的全部出生/消失按"自己段能量 / 经窗口能量 / 都量不出"分类 |
| `event-dev.txt`、`event_dev.py` | 新提前事件前后 15 m 内，每条有原件观测的边界离原件的最大距离，前后对比 |
| `road-dev-stage.txt`、`road_dev.py` | 有新提前事件的路整条：每条有原件观测的边界离原件的最大距离，分事件前、事件附近 15 m、事件后三段，前后对比 |
| `event-widths.txt`、`event_widths.py` | 0630 road 11 lane −3、road 12 lane 4：原件宽度与写出宽度逐点对照 |
| `event-curvature-stage.txt`、`event_curv.py` | 新提前事件前后 15 m 内，事件那一侧车道边缘曲率的最大值（评分板车道边缘曲率的采样和公式） |
| `stage-edge-curvature.txt` | 有新提前事件的路车道边缘曲率逐段最大（`edge_curv_where.py`） |

**完整转换的前后对比**（`analyze_bw.sh`，基线输出对最终版输出）：

| 文件 | 内容 |
|---|---|
| `roads-changed.txt` | 写出的 XODR 里哪些路不同（`roads_changed.py`） |
| `g8-codes.txt`、`g8_codes.py` | G8 状态、匹配数、排除项按码计数（另列 `lane_birth_advance` 零宽段的排除项）、交付裁决状态与理由 |
| `event-curvature.txt` | 同 `event-curvature-stage.txt`，在完整转换的输出上 |
| `boundary-runs-by-lane.json`、`boundary-runs-by-lane.txt` | 评分口径（`boundary_inside_runs`）逐原件车道的最大偏差：有变化或超过 0.1 m 的（`runs_lanes.py`） |
| `fairness-peaks.txt` | 光顺项（`lane_fairness`）各类车道中心曲率最大的三条（`fair_where.py`） |
| `lane-centre-by-lane-0630.json`、`lane-centre-by-lane-0627.json` | 0630、0627 逐原件车道的车道中心距离（评分口径 `lane_center_noconflict_*`），基线对最终版（`lane_centre_diff.py`，见 [`../zero-start-retry-20261012/`](../zero-start-retry-20261012/)；不在 `analyze_bw.sh` 里，单独跑） |
| `0630-road12-st-*.png`、`0627-road12-st-*.png`、`node16-road11-st-*.png` | 三条路在事件附近的车道边缘与原件边界，路自己的 (s, t) 坐标（`plot_st.py`） |

下列脚本与前几轮相同，这里不再复制：

- `gen_part.py`、`gen_score_all.py`（泛化集分批转换与合并评分）、`junit_compare.py`：[`../zero-start-retry-20261012/`](../zero-start-retry-20261012/)；
- `sb_gen.py`、`stage_raw.py`、`stage_diff.py`、`runs_lanes.py`、`edge_curv_where.py`、`roads_changed.py`：[`../prune-guard-20261013/`](../prune-guard-20261013/)；
- `c_stage_hold.py`、`norm_compare.py`、`plot_st.py`：[`../hold-width-retry-20261013/`](../hold-width-retry-20261013/)；
- `score_two.py`：[`../surface-seams-20261012/`](../surface-seams-20261012/)；
- `compare_boards.py`：[`../mouth-anchor-20261009/`](../mouth-anchor-20261009/)；
- `fair_where.py`：[`../lane-swap-20261014/`](../lane-swap-20261014/)。

[泛化集剩余失败与待定事项](../../../docs/泛化-剩余失败与待定事项-2026-10-14.md)的失败表由 `memo_table.py` 从本目录的泛化集与慢路口评分生成（`memo-table.txt`）。

脚本里有当时云端临时目录的绝对路径。
