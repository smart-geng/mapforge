# Git 接续资料包

这些资料用于接续研发，不是正式地图交付包。完整命令、验证结果及平台限制见 [云端接续与工程迁移闭环](../docs/云端接续与工程迁移闭环-2026-10-09.md)。

- `20261009-base/manifest.json`：真实 SHP、保存工程与历史证据、Windows 消费者资料。
- `20261009-transfer-proof/manifest.json`：本轮迁移、跨平台接续与页面检查证据。

每份资料按最多 32 MiB 分卷，包含整包及逐文件 SHA256。拉取全部分卷后使用 `scripts/restore_handoff_assets.py` 恢复；不要直接把 `.part` 当 ZIP 打开。恢复工具拒绝已有不同内容，不覆盖旧资料。manifest 中的排除清单保留，服务 session/token、缓存和 Python 环境未分发。

原料与旧证据恢复原字节；历史绝对路径是原运行记录。搬迁工程必须另用页面或 `scripts/workbench_relocate_project.py`，旧候选与检查随迁移失效。
