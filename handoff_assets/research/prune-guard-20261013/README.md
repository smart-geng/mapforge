# 0711 进口 road 11：剪枝护栏——证据（2026-10-13）

说明见 [docs/泛化-0711进口道路剪枝护栏-2026-10-13.md](../../../docs/泛化-0711进口道路剪枝护栏-2026-10-13.md)。

运行环境：同一台 Linux 机器（4 核），Python 3.11.16，esmini v3.6.0 Linux 库（与前几轮相同），判级按 0.11-draft。基线是现行 `main`（`cce0bee`）。它的转换代码与上一轮保持宽度改动后相同，转换输出就是那一轮的（[`../hold-width-retry-20261013/`](../hold-width-retry-20261013/)），本轮复制后按 0.11-draft 重新评分（`*-main-0.11.json`）。

**代码哈希（SHA256）**：

| 文件 | 改动前（`main` `cce0bee`） | 验证用 | 提交 |
|---|---|---|---|
| `mapforge/ops/lane_refit.py` | `a3c8d0db49fb9105aae75be1be2b67a97bd1dfcf5d7385df452386eb55538dfa` | `ae22054e689848bb2a6a23366612e78e8877dc3f9e2762a8adcf45d6c7f3b684` | `51cd2da41c9569741f66b145ee57ceae6c00aba061632639f855b0c95d9f9e87` |
| `mapforge/ops/refit_prune_guard.py` | 新文件 | `ab0c892603aa0cdc7dcce7a73a07b95b60c410ef27fa89f6d545248a940a5d00` | 同左 |
| `tests/test_refit_prune_guard.py` | 新文件 | `909e1ab3bfa7d737adf1b44182aaf63dcc23034ae097110927d32cf29a75cbd2` | 同左 |

- `lane_refit.py` 的验证版与提交版只差模块说明里加的一句（规则列表），去掉说明后的语法树相同（`ast_same.py`）。
- `scripts/bound_files.py`：提交版三个文件都判 free。改动前的 `lane_refit.py` 判 BOUND，唯一出处是 `docs/泛化-0627进口道路保持分隔带宽度-2026-10-13.md` 记的那一轮改动后哈希。那是改动记录，`out/`、`experiments/`、`profiles/`、`ledger/` 里都没有这个哈希。本目录不在 `bound_files.py` 的扫描范围内，所以这里写全文哈希，文档里只写前 8 位。

**BLAS 线程数**：本轮的转换全部用 `OPENBLAS_NUM_THREADS=1`（`launch_pg.sh`），评分用默认线程。基线里评分板（除 SHP node18）和泛化集 0627 是默认线程转换，其余单线程（见上一轮 README）。差异的归属：

- 评分板 SHP node3 与基线不同，来自线程数：现行 `main` 同样单线程重转（`sb-bi1`）与本次逐字节相同，两版只差连接路 125 车道 userData 里未选中候选的统计；
- 泛化集 0627 与线程数无关：现行 `main` 单线程重转（`gen-bi1`）与基线（默认线程）逐字节相同，所以 0627 的变化来自本次改动；
- 这些哈希记在各哈希清单末尾的注释里。

**结果**：

| 文件 | 内容 |
|---|---|
| `scoreboard-prune-guard.json`、`scoreboard-main-0.11.json`、`scoreboard-prune-guard-compare.txt` | 本次评分板（14 份一次跑完，`sb_gen.py` 转换，`--no-generate` 评分）、基线按 0.11-draft 的评分，及逐项对比（`compare_boards.py` 输出里 `base` 是基线，`anchor` 是本次）：T1/T2 14/14，0/14 行变化 |
| `scoreboard-xodr.sha256` | 14 份评分板 XODR 的哈希：13 份与基线相同；SHP node3 不同，与现行 `main` 单线程相同 |
| `generalization-prune-guard.json`、`generalization-main-0.11.json`、`generalization-prune-guard-compare.txt` | 本次泛化集评分、基线按 0.11-draft 的评分及对比：0411、0627、0711 三行变化，等级不变；0711 T2 失败项 3 → 1 |
| `generalization-xodr.sha256` | 模块内转出的 10 个 XODR 哈希：0411、0627、0711 随本次改动变化，其余 7 个逐字节相同 |
| `slow-0707104-score.json`、`slow-0707104-prune-guard-compare.txt` | 慢路口 0707104 两版（都单线程转换）的评分（`score_two.py`，见 [`../surface-seams-20261012/`](../surface-seams-20261012/)）及对比：等级与失败项相同 |
| `slow-junctions.sha256` | 0707104 不设上限转换的全部输出哈希；061310 没有重跑 |
| `related-tests.txt` | 相关测试前后逐项对比（`junit_compare.py`，见 [`../zero-start-retry-20261012/`](../zero-start-retry-20261012/)） |
| `stage-diff.json`、`run_stages.sh`、`stage_raw.py`、`stage_diff.py` | 筛查：现行 `main` 转换中 `lane_refit` 的输入（`stage_raw.py` 生成的 `*.raw.xodr`），分别用现行代码和本次代码只跑 `lane_refit` 一步（`c_stage_hold.py`，见 [`../hold-width-retry-20261013/`](../hold-width-retry-20261013/)），比较写出的路并列出护栏记录。泛化集转出的 12 个路口和评分板 7 个 SHP 路口里，只有 0711 road 11、0627 road 13、0411 road 13、0707104 road 13 不同 |
| `guard-trace-0711.json`、`guard-trace-0627.json`、`guard-trace-0411.json`、`guard-trace-0707104.json`、`guard_trace.py` | 这四条路上护栏的每次调用：RDP 顶点、普通剪枝、护栏剪枝及两者离 RDP 顶点的误差 |
| `roads_changed.py`、`ast_same.py`、`hash_lists_pg.py` | 两份 XODR 哪些路、哪些子元素不同；去掉说明后语法树是否相同；生成三份哈希清单 |
| `sb_gen.py`、`launch_pg.sh` | 评分板只转换不评分（`scoreboard.generate`）；本轮三组转换的启动脚本 |

泛化集分批转换与合并评分的 `gen_part.py`、`gen_score_all.py`，以及 `compare_boards.py`、`norm_compare.py`、`xodr_numdiff.py`、`road_edges.py`、`cross_sections.py`、`plot_before_after2.py`、`plot_st.py` 与前几轮相同（见 [`../zero-start-retry-20261012/`](../zero-start-retry-20261012/)、[`../hold-width-retry-20261013/`](../hold-width-retry-20261013/)、[`../mouth-anchor-20261009/`](../mouth-anchor-20261009/)），这里不再复制。

**四条路的前后对比**（基线输出对本次输出）：

| 文件 | 内容 |
|---|---|
| `0711-road11-edges-before.json`、`0711-road11-edges-after.json` 及 0627、0411、0707104 的 `*-road13-edges-*.json` | 各段道路外缘偏差（`road_edges.py`，每条记录各自选原件边界） |
| `boundary-runs-by-lane.json`、`runs_lanes.py` | 评分口径（`boundary_inside_runs`）逐原件车道的最大偏差，四个路口前后对比 |
| `0711-road11-xsec-before.json`、`0711-road11-xsec-after.json` | road 11 横断面，s 0.5–77.5 每 0.5 m：写出的车道边缘与原件边界在同一法线上的位置（`cross_sections.py`） |
| `0627-road13-xsec-before.json`、`0627-road13-xsec-after.json` | 0627 road 13 横断面，s 17–50 每 0.25 m |
| `edge-curvature.txt`、`edge_curv_where.py` | 四条路车道边缘曲率逐段最大（评分板的车道边缘曲率采样和公式） |
| `0711-road11-before-after.png`、`0711-road11-st-before.png`、`0711-road11-st-after.png` | 0711 road 11 前后的车道边缘与原件边界（平面；路自己的 (s, t) 坐标） |
| `0627-road13-s14-62-before-after.png`、`0411-road13-s60-142-before-after.png`、`0707104-road13-s38-70-before-after.png` | 另三条路前后的车道边缘与原件边界 |

脚本里有当时云端临时目录的绝对路径。
