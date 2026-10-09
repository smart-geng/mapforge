# 急弯连接路原件冲突：摸底与策略前后对比证据

决定、实现与结论见 [工作台-急弯连接路原件冲突](../../../docs/工作台-急弯连接路原件冲突-2026-10-09.md)。

| 文件 | 内容 |
|---|---|
| `survey.py`、`survey.json` | 决定前的只读摸底：18 个 SHP 路口 393 条可比 junction-via 的原件急弯程度、车道偏差，以及按 4/5/6/8/10 m 候选半径整条排除后的中心线统计；逐路口断言与现行 `lane_center_noflare_*` 一致 |
| `rescore.py`、`rescore.json` | 实现后的前后对比：已存 Windows 指标补上新指标，分别按 0.7-draft（取自提交 `2a0b0f4`）和 0.8-draft 判级；先断言 0.7 重判与已存等级一致 |

运行环境：Claude Code 云端容器，Linux，Python 3.11.16；资料按 `handoff_assets/20261009-*` 恢复。几何没有重新转换，esmini 结果沿用已存的 Windows 指标。

复现：两个脚本都按 `handoff_assets/research/<目录>/` 或 `out/workbench/<目录>/` 的深度定位仓库根，会在脚本所在目录写出 JSON；建议复制到新建的 `out/workbench/<新目录>/` 运行。`rescore.py` 需要能读取 Git 提交 `2a0b0f4`。
