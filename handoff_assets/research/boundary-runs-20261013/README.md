# 边界内段按连续段配对（策略 0.11-draft）：重评证据（2026-10-13）

决定、实现与结论见 [评分板-边界内段按连续段配对](../../../docs/评分板-边界内段按连续段配对-2026-10-13.md)。

| 文件 | 内容 |
|---|---|
| `rescore.py`、`rescore.json` | 策略前后对比，三组：冻结证据（`out/*/20261008-safe-mouth-recovery-v3` 与 0621 工作台候选，先复现 0.10 重评的等级与失败项）、现行 `main` `d13d300` 的 Linux 评分板与泛化集、hold-width 重试之后的 Linux 评分板与泛化集（这两组先按记录的哈希核对 XODR，再复现已存等级）。每个 SHP 文件重算的 `boundary_inside` 须与已存值相同；加上 `boundary_inside_runs` 后按 0.11-draft 判级。每行附最大值所在的原件车道和被分段配对的车道。两个慢路口不由泛化模块评分，只列数值。 |
| `runs_census.py` | 逐文件列出两种量法和被分段的车道（不判级），用于核对单个文件。 |
| `related-tests.txt` | 相关测试前后逐项对比：除新测试 9 项外，状态全部相同。 |

结果：三组的断言全部通过，都没有等级变化（评分板 T1/T2 14/14；泛化集冻结组 8/2，Linux 两组 9/3）。

运行环境：Claude Code 云端容器，Linux，Python 3.11.16，资料按 `handoff_assets/20261009-*` 恢复（含 `shp_0222-0326/` 与 `out/`），`OPENBLAS_NUM_THREADS=1`。几何没有重新转换。hold-width 组里 0412、0630、0707104 和评分板 SHP node18 是单线程转换，XODR 因 BLAS 线程数与现行 `main` 组不同（见 `../hold-width-retry-20261013/README.md`）。

复现：

- `rescore.py` 按 `handoff_assets/research/<目录>/` 的深度定位仓库根，JSON 写在脚本所在目录；需要能读取 Git 提交 `d13d300`（0.10-draft 策略）。
- Linux 两组的 XODR 不在仓库里（哈希在 `../zero-start-retry-20261012/` 和 `../hold-width-retry-20261013/` 的 `*-xodr.sha256`），脚本里是当时云端临时目录的绝对路径。
