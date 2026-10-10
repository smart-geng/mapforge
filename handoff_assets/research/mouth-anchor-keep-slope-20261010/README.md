# 口部锚点保持口部斜率：证据（2026-10-10）

说明见 [docs/工作台-口部锚点保持口部斜率-2026-10-10.md](../../../docs/工作台-口部锚点保持口部斜率-2026-10-10.md)。

全部在同一台 Linux 机器上运行：

- Python 3.11.16；
- esmini 用 v3.6.0 Linux 库，与仓库内 Windows DLL 同一修订 `131a5651`；
- 判级按 0.10-draft。

基线是现行 `main`（`1a2eb10`）的代码，它的评分板和泛化集结果已在上一轮证据里：

- [`../mouth-anchor-20261009/scoreboard-formal.json`](../mouth-anchor-20261009/scoreboard-formal.json)
- [`../mouth-anchor-20261009/generalization-formal.json`](../mouth-anchor-20261009/generalization-formal.json)
- [`../mouth-anchor-20261009/generalization-slow.json`](../mouth-anchor-20261009/generalization-slow.json)（`side = formal` 的两行）

这几份与本次基线逐字节相同。

| 文件 | 内容 |
|---|---|
| `scoreboard-keep-slope.json` | 最终代码的评分板（14 份） |
| `scoreboard-keep-slope-compare.txt` | 与基线逐项对比。脚本沿用上一轮的 `compare_boards.py`，输出里的 `base` 是基线，`anchor` 是本次 |
| `generalization-keep-slope.json` | 最终代码的泛化集（14 个常规路口由模块转换，两个慢路口记为未转出，同基线）；驱动脚本 `gen_regular.py` |
| `generalization-keep-slope-compare.txt` | 与基线逐项对比 |
| `generalization-slow-keep-slope.json` | 061310、0707104 不设 3600 s 上限单独转换后的评分 |
| `slow-junctions.sha256` | 两个慢路口的输出哈希：最终代码、未加路缘转角规则、基线。061310 三者相同；0707104 前两者相同 |
| `generalization-noguard.json`、`generalization-noguard-compare.txt` | 未加路缘转角规则时的泛化集，0412 外缘曲率 0.257 /m 由此发现 |
| `anchor-log-0412-noguard.jsonl` | 0412 的口部区记录：road 30 起点锚点到口部斜率 −0.25，保持斜率 −0.05 |
| `curb-guard-screen.txt`、`curb_guard_screen.py` | 对 17 个 SHP 输出重跑 `lane_refit` 一步，列出路缘转角规则起作用的口部区（只有 0412、061310 各一处） |
| `fastpath-ksall-*.row.json`、`fastpath-ksns-*.row.json` | 选型用的 node17、node18 整行评分：`ksall` 为全部保持口部斜率，`ksns` 为只对外展保持 |
| `0621-ksall.row.json` | 选型用的 0621 整行评分（全部保持口部斜率，未采用） |
| `0621-identical.sha256` | 最终代码与现行代码处理 0621 的输出哈希，二者相同 |
| `anchor-log-ksall-*.jsonl`、`anchor-log-ksns-0621.jsonl` | 实验代码记录的口部区：被删顶点、是否直线进口部、锚点、保持的口部斜率 |
| `experiment-modes.diff` | 实验代码相对现行代码的改动（环境变量 `MF_KS_MODE` 切换几种做法，`MF_ANCHOR_LOG` 写日志），只用于选型和筛查，未进入仓库 |
| `outer_flips_locate.py` | 列出外缘曲率换号的位置，口径与 `smoothness.edge_shape_quality` 相同 |

关于两个慢路口：061310 和 0707104 在这台机器上单个转换超过泛化集模块的 3600 s 上限（Windows 上分别为 1120 s、880 s）。基线运行里两者都在模块内超时未转出。本次同样记为未转出，另用同一命令、不设上限转换后评分（`generalization-slow-keep-slope.json`），再与基线的同类评分（上一轮的 `generalization-slow.json`）比较。
