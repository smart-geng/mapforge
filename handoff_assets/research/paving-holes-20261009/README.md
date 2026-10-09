# 路面孔洞采样修正与原件空区单列：策略前后对比证据

决定、实现与结论见 [工作台-路面孔洞采样与原件空区](../../../docs/工作台-路面孔洞采样与原件空区-2026-10-09.md)。

| 文件 | 内容 |
|---|---|
| `rescore.py`、`rescore.json` | 策略前后对比。已存 Windows 指标加上急弯冲突指标，先按 0.8-draft（取自提交 `bb8008b`）判级，并断言结果与 0.8 重评证据 `../tight-turn-source-conflict-20261009/rescore.json` 一致；再补上路面孔洞新指标，按 0.9-draft 判级。只有 0621 带源路面重建证据。 |
| `sidecar_check.py`、`sidecar_check.json` | 端到端检查：把 0621 运行的候选及旁证复制成工作台检查包的布局，确认 `scoreboard.evaluate` 读取 `candidate.surface-evidence.json`。有证据和无证据两种情况都跑了；esmini 在 Linux 上跳过。 |

运行环境：Claude Code 云端容器，Linux，Python 3.11.16，资料按 `handoff_assets/20261009-*` 恢复。几何没有重新转换。

复现：
- 两个脚本都按 `handoff_assets/research/<目录>/` 或 `out/workbench/<目录>/` 的深度定位仓库根，并把 JSON 写在脚本所在目录。
- 建议复制到新建的 `out/workbench/<新目录>/` 下运行。`sidecar_check.py` 还会在该目录下建两个候选副本目录。
- `rescore.py` 需要能读取 Git 提交 `bb8008b`。
