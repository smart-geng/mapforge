# 路口铺面：原件说得清的接缝——证据（2026-10-12）

说明见 [docs/泛化-路口铺面原件接缝-2026-10-12.md](../../../docs/泛化-路口铺面原件接缝-2026-10-12.md)，六个转不出路口的诊断见 [docs/泛化-转不出路口诊断-2026-10-11.md](../../../docs/泛化-转不出路口诊断-2026-10-11.md)。

运行环境：同一台 Linux 机器，Python 3.11.16，esmini v3.6.0 Linux 库（与仓库内 Windows DLL 同一修订 `131a5651`），判级按 0.10-draft。基线是现行 `main`（`4a92d8e`）的代码，它的评分板和泛化集结果在上一轮证据里：

- [`../source-event-20261010/scoreboard-source-event.json`](../source-event-20261010/scoreboard-source-event.json)
- [`../source-event-20261010/generalization-source-event.json`](../source-event-20261010/generalization-source-event.json)

**结果**（评分板和两个慢路口在"直接改 v1"那一版上跑，对这些路口执行路径与最终代码相同，见说明文档"前后对比"；其余用最终代码）：

| 文件 | 内容 |
|---|---|
| `scoreboard-seams.json`、`scoreboard-seams-compare.txt` | 本次评分板及与基线逐项对比（`compare_boards.py` 输出里 `base` 是基线，`anchor` 是本次）：0/14 行变化 |
| `scoreboard-xodr.sha256` | 14 份评分板 XODR 的哈希，与基线相同 |
| `slow-junctions.sha256` | 061310、0707104 不设上限转换的输出哈希，与基线相同 |
| `generalization-seams.json`、`generalization-seams-compare.txt` | 本次泛化集评分及对比：只有 0621、0630 两行变化（从转不出到有分数） |
| `generalization-xodr.sha256` | 模块内转出的 XODR 哈希：原有 8 个相同，0621、0630 为新增 |
| `seam-rule-records.json`、`seam_records.py` | 两个路口最终 `.mouth-envelope.json` 里的规则记录，以及 XODR 里写明规则的车道：0621 铺面道路 51 的分隔带车道，0630 铺面道路 56 的两条 restricted 车道。都标 INFERRED，属于铺面、不进 G8 |
| `0630-t2-where.json`、`t2_where.py` | 0630 七项 T2 失败逐项定位到道路（与评分板同样的排除） |
| `related-tests.txt` | 相关测试前后逐项对比 |
| `gen_regular.py`、`finish_gen.py` | 泛化集驱动（两个慢路口另行转换），以及驱动被后台时限停下后的补跑与全集评分 |
| `score_two.py`、`bytes_compare.py`、`compare_boards.py` | 单独给指定路口评分（同一策略）、两个输出目录逐字节对比、两次评分逐项对比 |

**诊断**（基线代码上的定位，脚本里有当时云端临时目录的绝对路径）：

| 文件 | 内容 |
|---|---|
| `0621-road10-diagnosis.json`、`diag_0621.py` | 0621 road 10：分隔带跟踪逐行记录、第一处失败截面、口袋（9.741 m²）的范围 |
| `0630-road30-diagnosis.json`、`diag_0630.py` | 0630 road 30：端帽采样点、未碰到路口面的 9 个点、补齐后围出的洞（0.2006 m²） |
| `0621-pocket-source-objects.json`、`0630-hole-source-objects.json`、`scan_sources.py` | 与口袋 / 洞相邻或相交的全部原件对象（路口面、路面、护栏等）及覆盖率 |
| `road-surface-coverage.json`、`road_surface_coverage.py` | 原件路面对两个路口的车道尾端、路口面、问题区域的覆盖率 |
| `median-islands-road-surface.json`、`island_rs.py` | 两个路口已接受的分隔带岛被原件路面覆盖的比例（均 ≤ 1%） |
| `tail-sweeps-road-surface.json` | 各车道尾端补齐时新增面积中不是原件路面的部分 |
| `0621-road10-gap.png`、`0630-road30-hole.png`、`evidence_plots.py` | 两处缝的示意图 |

**Profile 与工作台**（`workbench-profile-impact/`）：

| 文件 | 内容 |
|---|---|
| `snapshot-ids-v1.json`、`snapshot-ids-v1-edited.json`、`snapshot_ids.py` | v1 原样与在 v1 里直接加两层时，工作台源快照身份、node16 / 0621 快照内容哈希和对象数 |
| `ibd-smarteditor-v1.edited-not-adopted.yaml` | 未采用的"直接改 v1"版本（SHA256 `0eda35b1…`），只为复现上面的哈希 |
| `workbench-tests-v1-edited.txt`、`workbench-tests-final.txt` | 读真实 Profile 的 15 个工作台测试文件：直接改 v1 时 1 失败 17 报错；最终做法 474 通过、1 跳过 |
