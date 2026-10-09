# 非等距五跨度推导独立复核证据

结论与解读见 [工作台-非等距五跨度独立复核](../../../docs/工作台-非等距五跨度独立复核-2026-10-09.md)。被复核的原研究在 `out/workbench/local-shape-next-representation-20261009/`，随 `handoff_assets/20261009-base` 恢复；本目录不改它。

| 文件 | 内容 |
|---|---|
| `review.json` | 复核结论、各项结果、问题清单、被复核文件和复核脚本的 SHA256 |
| `algebra.json` | sympy 公式比对、基函数代数、基准单调性、节点单纯形网格结果 |
| `falsification.json` | 768 个真实候选的逐项判定，以及节点站位、方向、幅度和脚本哈希 |
| `falsify.log` | 反证搜索的运行日志 |
| `rerun-derivation.json` | 在新目录重跑原 `research.py` 的输出：比原 `derivation.json` 多一个 `source_proof` 字段，数学字段一致 |
| `falsify.py`、`review_algebra.py`、`vertex_exact.py` | 实际运行的复核脚本，原字节 |

运行环境：Claude Code 云端容器，Linux，Python 3.11.16，main `7a2d233`。`review_algebra.py` 另用一个只装 sympy 1.13.3 的环境运行；项目锁定环境没有 sympy，未修改 `uv.lock`。

复现：先按[云端接续文档](../../../docs/云端接续与工程迁移闭环-2026-10-09.md)恢复资料，把三个脚本复制到 `out/workbench/local-shape-next-representation-20261009/independent-review/` 后运行（脚本按该深度定位仓库根和被复核目录）。

- `falsify.py [节点组数]`：项目环境，写 `falsification.json`，4 核约 21 分钟。
- `review_algebra.py`：sympy 环境，写 `algebra.json`。
- `vertex_exact.py`：项目环境，打印各顶点的精确系数。
