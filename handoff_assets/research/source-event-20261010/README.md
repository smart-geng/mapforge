# 车道收窄终点取原件归零处：证据（2026-10-10）

说明见 [docs/工作台-0621-road11车道收窄终点-2026-10-10.md](../../../docs/工作台-0621-road11车道收窄终点-2026-10-10.md)。

运行环境：同一台 Linux 机器，Python 3.11.16，esmini v3.6.0 Linux 库（与仓库内 Windows DLL 同一修订 `131a5651`），判级按 0.10-draft。

基线是现行 `main`（`31ca038`）的代码，它的评分板和泛化集结果在上一轮证据里：

- [`../mouth-anchor-keep-slope-20261010/scoreboard-keep-slope.json`](../mouth-anchor-keep-slope-20261010/scoreboard-keep-slope.json)
- [`../mouth-anchor-keep-slope-20261010/generalization-keep-slope.json`](../mouth-anchor-keep-slope-20261010/generalization-keep-slope.json)

| 文件 | 内容 |
|---|---|
| `scoreboard-source-event.json`、`scoreboard-source-event-compare.txt` | 本次代码的评分板及与基线逐项对比（`compare_boards.py` 输出里 `base` 是基线，`anchor` 是本次）。只有 shp-node17 变化 |
| `generalization-source-event.json`、`generalization-source-event-compare.txt` | 本次代码的泛化集（驱动脚本 `gen_regular.py`，两个慢路口另行转换）及对比。没有任何变化 |
| `slow-junctions.sha256` | 061310、0707104 不设上限转换的输出哈希，本次与基线相同 |
| `0621-final.row.json`、`0621-candidate.sha256` | 本次代码处理 0621 工作台重建路面后的整行评分，以及本次与基线的候选哈希 |
| `0621-extrapolated.row.json`、`fastpath-node17-extrapolated.row.json` | 中间版本（外推不受原件端点限制）的 0621 与 node17 整行评分；node17 因此退步，最终版本加了端点限制 |
| `fastpath-node17-final.row.json` | 最终版本的 node17 整行评分（快速路径，与完整评分板逐项一致） |
| `trace-road11-fits-main.json`、`trace_fit.py` | 现行代码在 0621 road 11 上每次 `fit_boundary` 调用的输入条件、顶点、拐角半长 |
| `stage-variants-road11.txt` | 只跑 `lane_refit` 一步时几种做法的车道 +4 残差和 road 11 曲率：曲率目标 0.04/0.06/0.08/0.10、强制交接、外推事件点、最终做法 |
| `experiment-variants.diff` | 选型用的实验代码（环境变量切换 `MF_TAPER_KAPPA`、`MF_CUT_TOL`、`MF_EVENT_SOURCE`），未进入仓库 |
| `event_screen.py` | 对成品重跑 `lane_refit` 一步、列出事件点会移动之处的筛查脚本（近似） |
| `l4_resid.py`、`lane_profile.py`、`road_curv.py` | 车道 +4 残差、按原件车道的残差剖面、道路边缘曲率的诊断脚本 |
