# 停止线就近匹配与原件缺停止线：策略前后对比证据

决定、核对事实、实现与结论见 [工作台-停止线就近匹配与原件缺停止线](../../../docs/工作台-停止线就近匹配与原件缺停止线-2026-10-09.md)。

| 文件 | 内容 |
|---|---|
| `rescore.py`、`rescore.json` | 策略前后对比。已存 Windows 指标加上急弯冲突与路面孔洞指标，先按 0.9-draft（取自提交 `f98231d`）判级，并断言与 `../paving-holes-20261009/rescore.json` 逐案一致；凡需重算 G8 的路口，先断言原清单重跑 `evaluate_g8` 与已存 G8 一致；再补上停止线指标，按 0.10-draft 判级。每行附该路口的停止线复核记录（不含几何）。 |
| `scoreboard_check.py`、`scoreboard_check.json` | 端到端检查：把 0621 运行和泛化集 0412 复制成单独目录，不传入原件源，直接调用 `scoreboard.evaluate`，确认评分自建原件源并给出停止线指标。esmini 在 Linux 上跳过。 |

运行环境：Claude Code 云端容器，Linux，Python 3.11.16，资料按 `handoff_assets/20261009-*` 恢复（含 `shp_0222-0326/`）。几何没有重新转换。

复现：
- 两个脚本都按 `handoff_assets/research/<目录>/` 或 `out/workbench/<目录>/` 的深度定位仓库根，并把 JSON 写在脚本所在目录。
- 建议复制到新建的 `out/workbench/<新目录>/` 下运行。`scoreboard_check.py` 还会在该目录下建两个副本目录。
- `rescore.py` 需要能读取 Git 提交 `f98231d`。
