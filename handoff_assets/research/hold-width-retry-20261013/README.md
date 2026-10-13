# 0627 进口 road 10：没有原件的延长段保持分隔带宽度——证据（2026-10-13）

说明见 [docs/泛化-0627进口道路保持分隔带宽度-2026-10-13.md](../../../docs/泛化-0627进口道路保持分隔带宽度-2026-10-13.md)。

运行环境：同一台 Linux 机器（4 核），Python 3.11.16，esmini v3.6.0 Linux 库（与上一轮相同），判级按 0.10-draft。基线是现行 `main`（`d13d300`）的代码，它的评分板、泛化集和慢路口结果在上一轮证据里：

- [`../zero-start-retry-20261012/scoreboard-zero-start.json`](../zero-start-retry-20261012/scoreboard-zero-start.json)
- [`../zero-start-retry-20261012/generalization-zero-start.json`](../zero-start-retry-20261012/generalization-zero-start.json)
- [`../zero-start-retry-20261012/slow-junctions.sha256`](../zero-start-retry-20261012/slow-junctions.sha256)

评分板 SHP 前 6 个路口和泛化集 a、b 两批（0627，0411、0412 第一次）运行时，`refit_hold_width.py` 的模块说明还是旧文字（SHA256 `ad7a64b4…`）；node18 重转和之后各批用提交版本（`c0bcf660…`）。两版只差说明里 0627 原来偏差的描述，去掉说明后的语法树相同。

**BLAS 线程数**：基线和本次前几批按 numpy 默认设置运行（OpenBLAS 按核数开线程）。几个转换同时跑时，这些线程空转占满 CPU，转换慢到撞上后台任务的 2 小时上限和泛化模块的 3600 s 上限，所以后几批改用 `OPENBLAS_NUM_THREADS=1`：

| 批 | BLAS 线程 | 路口 |
|---|---|---|
| 评分板 SHP | 默认 | node3、node4、NODE5、node13、node16、node17 |
| 评分板 SHP | 单线程 | node18（默认线程那次在 2 小时上限被中止） |
| 评分板 MAP | 默认 | 7 个 |
| 泛化集 a | 默认 | 0627 |
| 泛化集 b | 默认 | 0411、0412，都撞上 3600 s 上限 |
| 泛化集 b2 | 单线程 | 0411、0412，同一命令、同一上限重跑 |
| 泛化集 c | 单线程 | 其余 11 个（转出 7 个；0701、0629、0729、0811 仍转不出） |
| 慢路口 | 单线程 | 0707104 |

单线程转出的 11 个路口里，0412、0630、0707104、node18 的输出与默认线程不同，其余 7 个逐字节相同。用现行 `main` 的代码在同样的单线程设置下重转这 4 个（`gen-zs1`、`gen-slow-zs1`、`sb-zs1-shp`），XODR 与本次逐字节相同，差异只来自线程数；这些哈希记在各哈希清单末尾的注释里。评分按基线的做法用默认线程；SHP 评分板在转换之后用 `--no-generate` 单独评分，所以那一部分的 `scoreboard.json` 没有转换记录。

**结果**：

| 文件 | 内容 |
|---|---|
| `scoreboard-hold-width.json`、`scoreboard-hold-width-compare.txt` | 本次评分板（SHP、MAP 分两次跑，`merge_boards.py` 合并）及与基线逐项对比（`compare_boards.py` 输出里 `base` 是基线，`anchor` 是本次）：T1/T2 14/14，只有 SHP node18 一行指标变化（线程数），等级不变 |
| `scoreboard-xodr.sha256` | 14 份评分板 XODR 的哈希：13 份与基线相同；SHP node18 不同，与现行 `main` 单线程相同 |
| `generalization-hold-width.json`、`generalization-hold-width-compare.txt` | 本次泛化集评分及对比：0627（本次改动，T2 不过的项 8 → 5）和 0412（线程数）两行变化，等级不变 |
| `generalization-xodr.sha256` | 模块内转出的 10 个 XODR 哈希：7 个与基线相同；0627 变化；0412、0630 不同，与现行 `main` 单线程相同 |
| `slow-0707104-score.json` | 0707104 默认线程（基线）与单线程（本次）两版的评分（`score_two.py`，见 [`../surface-seams-20261012/`](../surface-seams-20261012/)）：等级与失败项相同 |
| `slow-junctions.sha256` | 0707104 不设上限转换的输出哈希：单线程 XODR 与现行 `main` 单线程相同（与默认线程的基线不同）；road 10 仍跳过，重试记 `no free end without source`。061310 没有重跑（基线里没有被 `lane_refit` 跳过的路，两次重试都不会被调用） |
| `related-tests.txt` | 相关测试前后逐项对比 |
| `norm_compare.py` | 两个输出目录逐文件比较，先把新目录文件里的树名、运行名换成基线的（区分只差路径的文件） |
| `xodr_numdiff.py`、`hash_lists.py` | 两份 XODR 的结构与最大数值差（0630 两版最大 2.9e-8 m；0412、0707104 结构不同；node18 最大 1.99 m）；生成两份 XODR 哈希清单 |

泛化集分批转换与合并评分的 `gen_part.py`、`gen_score_all.py`，以及 `merge_boards.py`、`skips.py`、`junit_compare.py` 与上一轮相同（见 [`../zero-start-retry-20261012/`](../zero-start-retry-20261012/)），这里不再复制。

**0627 road 10**（基线输出对本次输出）：

| 文件 | 内容 |
|---|---|
| `0627-lane-refit-stage.json`、`c_stage_hold.py` | 只跑 `lane_refit` 一步时 0627 的报告：road 10 由本次重试写出 |
| `0627-road10-edges-before.json`、`0627-road10-edges-after.json` | 各段道路外缘偏差（评分板的配对口径，`road_edges.py`，见上一轮） |
| `0627-road10-xsec-before.json`、`0627-road10-xsec-after.json` | 横断面：写出的车道边缘与原件边界在同一法线上的位置（`cross_sections.py`，见上一轮） |
| `road10-s0-90-before-after.png`、`road10-s300-461-before-after.png`、`plot_before_after2.py`、`plot_st.py` | 前后车道边缘与原件边界（两侧） |
| `0627-t2-where-after.json` | 本次 0627 剩下 5 项 T2 失败逐项定位（之前的见 [`../zero-start-retry-20261012/0627-t2-where.json`](../zero-start-retry-20261012/0627-t2-where.json)） |
| `0627-lane-centre-diff.json` | 车道中心（去冲突）逐原件车道前后对比（`lane_centre_diff.py`，见上一轮） |
| `0627-road10-trial-hold.json`、`0627-road10-trial-plain.json`、`0627-road10-trial-relative.json`、`hold_try.py` | 开发时的试验：保持宽度、照旧、相对内侧边界拟合三种做法下 road 10 的结果 |
| `0707104-lane-refit-stage.json`、`0707104-road10-trial-hold.json` | 0707104 road 10：负宽车道的边界链在自由路端没有超过 0.5 m 的无原件段，没有可保持的宽度，重试不写（`no free end without source`），照旧跳过（−0.514 m，两侧原件压叠） |

**评分口径**（用户已选按连续段配对，另行实现）：

| 文件 | 内容 |
|---|---|
| `boundary-inside-split.json`、`bi_split.py` | 决定前的普查：边界内段按现行口径（拼接）与按连续段分开（每段各自选原件边界）量，现行 `main` 的 19 个输出和本次 0627 |
| `0412-road12-edges.json` | 0412 road 12 各段外缘偏差：原件车道 20063 两段之间三段不可信 |

脚本里有当时云端临时目录的绝对路径。
