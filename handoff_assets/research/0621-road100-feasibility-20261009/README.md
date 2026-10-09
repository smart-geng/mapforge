# 0621 road 100 中心线可达性研究证据

结论与解读见 [工作台-0621中心线可达性研究](../../../docs/工作台-0621中心线可达性研究-2026-10-09.md)。这里只放复核所需的原始产物；全部是未接纳的研究结果，不是交付件。

| 文件 | 内容 |
|---|---|
| `research.py` | 实际运行的脚本，原字节 |
| `run.log` | 23 组设置的逐行结果 |
| `summary.json` | 每组的拟合调用、road 100 形状、样本统计、整图摘要、T2 自身失败项、G11 状态、候选 XODR SHA256；以及输入、代码和策略哈希 |
| `metrics-by-setting.json` | 每组的完整评分板指标（`esmini_pass` 为 `NOT_RUN_LINUX`） |
| `t2-seg1.5-k0.60.candidate.xodr` | 只按 T2 时最好的一组候选（全图中心线 P95 0.1430），供查看几何 |

运行环境：Claude Code 云端容器，Linux，Python 3.11.16，main `20b2ef8`，资料按 `handoff_assets/20261009-*` 恢复。基准对照在 Linux 上重算，与 Windows 评分差值为 0。

复现：先按[云端接续文档](../../../docs/云端接续与工程迁移闭环-2026-10-09.md)恢复资料，再把 `research.py` 复制到一个新建的 `out/workbench/<新目录>/research.py` 后运行（脚本按该深度定位仓库根，并要求各设置子目录不存在）。各组候选 XODR 的 SHA256 记录在 `summary.json`，可用来核对重跑结果。
