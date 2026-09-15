# mapforge — 地图格式转换工厂

> **2026-09-15阶段收口**：代码/研究资料与受限修形工具整理为Git检查点，**不是合格平滑地图发布**。[开发归档](docs/阶段开发归档-2026-09-15.md) · [当前遗留与验收条件](docs/遗留工作-阶段收口-2026-09-15.md) · [新版选线/拖动使用](docs/本地修形台-选线拖动交互v2.md)。根HANDOFF已精简，长期全文保存在docs/archive/2026-09-15；下文各版本数值保留为历史，不作当前整图PASS。

> **2026-09-15：本地受限修形台已可操作，整图仍未修好。** 运行 `.\.venv\Scripts\python.exe scripts\serve_repair_workbench.py`，打开打印的本地链接。支持 node4 SHP叠图、共享分界线长区间拖动、撤销/保存重开与候选XODR导出/XSD/esmini位置读回。首次运行安装 `requirements-web.txt`。外缘/口部锁定，MAP未接入页面，所有导出仍标研究候选。[使用范围、真实证据与未完事项](docs/本地修形台-E0受限纵切.md)。下文“Web尚未实现”是历史全控制台状态，不能覆盖此纵切事实。

> 2026-09-14：仍没有合格的完整平滑XODR交付。本轮接通全路口来源预检（5普通道路/24转向、真实Line/Arc原件域、长车道完整来源链），不代表几何完成。[当前推进顺序与复跑](docs/全局重建与约束式交互修形方案.md)。

面向车路协同与自动驾驶的语义地图编译工厂：**OpenDRIVE / 车道级 SHP / V2X MAP 消息**互转，
每次转换强制输出质量报告、损失报告、ID 映射与来源清单。交叉口是第一公民。

方案与决策见 [docs/地图格式转换工厂-首批三格式方案.md](docs/地图格式转换工厂-首批三格式方案.md)（当前 v1.74 双父路与全部关联转向共同求解）；
接手开发先读 [HANDOFF.md](HANDOFF.md)。

> 当前边界：仍未完成平滑且忠实于原件的整图交付。v1.74的563变量共同试验已真实写出和复核，但24转向中16几何失败、19全源转角失败，最大esmini接缝0.321296m；新件已封存拒绝，未替换保留研究件或推广CLI。参考仍为少量长原语（本次4–5段、每段≥6m），但不用短段不代表已经平滑。两父源速度动态、全幅/11/12、MAP/CRS/movement/Web仍未完成。[当前交接与证据](HANDOFF.md)；[实施计划](docs/实施计划-整路联动重建与编辑闭环.md)。
>
> [全局重建与约束式交互修形方案](docs/全局重建与约束式交互修形方案.md)已落档；当前只有上述锁端共享分界线纵切可运行，**完整Web控制台和任意口部联动编辑尚未实现**，其他文档接口仍是设计入口。

OpenDRIVE 写出为**自研规范级 writer**（`mapforge/adapters/opendrive/writer.py`，
1.5 语义逐项对照 XSD：双向 leg road ±车道、median、roadMark、车道 `<speed>`、geoReference）。

> 旧正式7份MAP来源清单原件复核仅2份PASS、5份FAIL，包含真实出口被裁；旧输出没有覆盖。只有G8几何PASS不能证明原件完整转换，须同时通过G8-source-integrity。

## 快速开始

```bash
python -m venv .venv
.venv/Scripts/pip install numpy scipy pyshp lxml pyclothoids pycrate typer pyyaml pytest matplotlib
.venv/Scripts/pip install -r spikes/requirements-boundary-qp.txt  # 运行源几何共同求解/相关测试所需的研究依赖
.venv/Scripts/python -m pytest tests -q          # 含 G8/G10/G11 与少段故障注入
.venv/Scripts/python scripts/closed_loop.py      # 14 文件 G1–G11（含外缘/路面/动力学）+ esmini
.venv/Scripts/python scripts/visual_sweep.py out/preview/sweep-final  # 14 份多机位目检图
```

数据放置（不入库）：IBD 规格 SHP 交付放 `shp_0222-0326/`；现网 MAP XML 放 `v2x_map_xml/`。

## 转换

以下是已有CLI入口；v1.74共同重建仍是独立研究脚本，尚未接入默认转换或证明其输出已达到整图交付标准。

```bash
# SHP 图商交付 → MAP 交付包（uper+xml+geojson 三视图 + 质量/损失报告 + id-mapping + provenance）
python -m mapforge.cli convert shp_0222-0326 --to map --like v2x_map_xml/map凤苑路-金玥路node4.xml

# SHP 图商交付 → OpenDRIVE 直转（完整 junction + 连接路实测几何 + laneLink + 多 laneSection
# 变宽断面 + 逐车道限速；默认走 Profile 引擎；--at lon,lat 亦可定位）
python -m mapforge.cli convert shp_0222-0326 --to xodr --like v2x_map_xml/map凤苑路-金玥路node4.xml

# 现网 MAP XML → 仿真 OpenDRIVE（junction 自动重建：出口路优先用多节点帧邻居真实数据、
# 单节点帧按同一 leg 进口断面严格左右镜像兜底 INFERRED；路口中心用 G2 curb-return + 双轴重叠铺面；
# 连接路 G2 且全接缝车道级曲率连续；稀疏点作少段拟合约束，不逐点串小段；
# 金凤 7/7 connectsTo 覆盖 95/96，唯一 skip 为已知 ID 失配。多节点帧用 --node-id 选主路口）
python -m mapforge.cli convert v2x_map_xml/map凤阁路-金剑路路口node16.xml --to xodr

# OpenDRIVE → MAP（junction 自动重塑；无相位时红线 BLOCKED，--allow-no-phase 显式降级）
python -m mapforge.cli convert Town03.xodr --to map --allow-no-phase

# 新图商（字段不一样）：YAML Profile 映射，模板→体检→转换（详见 docs/接入指南）
python -m mapforge.cli profile-init my-vendor.yaml
python -m mapforge.cli profile-check my-vendor.yaml <shp目录>
python -m mapforge.cli convert <shp目录> --to xodr --profile my-vendor.yaml --at 106.51,29.60

# 其它：--to geojson | uper | map-xml；preview / encode / decode / validate-xodr 单步命令
```

缺失数据的行为：红绿灯相位与路口编号是**源里不存在的信息**——相位有来源（现网 XML/配时表）即绑、
无则明确 BLOCKED；编号只消费台账（`ledger/jinfeng-2026.yaml`，策略 inherit-as-is），工具不发明 ID。

## 结构

```text
mapforge/
├─ adapters/   opendrive(reader) · shp(ibd_reader · profile_source Profile引擎) · v2xmap(xml reader/writer · to_asn · asn/)
├─ profiles/   shp/*.yaml 图商字段映射（ibd-smarteditor-v1 为参考实现）
├─ ops/        shp_to_map · junction_to_map · map_to_xodr · refline_fit(参考线拟合) · simplify(附录D)
├─ report/     deliver(交付包) · preview_geojson
├─ validate/   planview_check · smoothness(参考线/边缘/路面) · shp_boundary_fidelity
├─ cli.py      preview / encode / decode / convert / validate-xodr
ledger/        region/node ID 台账
tests/         拟合单测 + 编解码回环 + 金凤 7 路口黄金回归
docs/          方案 · 调研 ×3 · 资料盘点 · 评审 · 文献 · Spike/M0/M1 报告
```

## 历史门禁事实（非当前整图验收，详见 docs/）

以下数字保留对应版本的检查记录；后续完整来源/限速/宽度/形状复核已发现未覆盖项，**不可据此宣称当前14图或两条主线全绿**。当前状态以根HANDOFF及2026-09-15阶段归档说明为准。

- 金凤 7 路口 SHP→MAP 对拍：车道几何横向中位 0.00 m，phase/结构与现网一致（真回归基准）；
- 金凤 7 路口 SHP→xodr 直转：213 条 junction 连接路全部实测几何，XSD 全 PASS、连续性 0 违例
  （新图商接入与数据需求见 docs/接入指南）；
- v1.25 普通道路禁止极小段假平滑：最终 xodr leg 最短段 ≥3m、连接路 ≥1m；无合格拟合候选直接阻断；
- v1.26 正式 G8 policy 已激活：金凤 7 路口 × MAP/SHP 两管道共 14 文件双向车道保真均 PASS；
- v1.30 新增 G9 路面连续性：14 文件铺面均单连通、零孔洞，且每个路口口部与外部道路存在 ≥0.2m² 面积重叠；
- v1.31 新增 G10 最终世界边缘形态与 SHP 物理边界对拍：14/14 PASS；node4 外缘双向中位约 0.21m、P95 约 0.40–0.42m；
- v1.34 新增 G11 少段/横断面/动力学/兼容性门禁：SHP 普通道路每 road≤3 段、连接≤5 段；MAP 普通道路≤4 段、连接≤3 段；14/14 PASS；
- SHP 外缘精确点到折线段对拍 7/7 PASS，双向 P95 最坏 node3 为 0.646/0.561m；14 张拼图和 7 张精确叠图见 `out/preview/sweep-final/`；
- 技术门禁 PASS 与交付放行分离：金凤 CRS 当前仅 `internally-consistent`，未完成绝对核验前交付保持 BLOCKED；
- MAP XML 与 IBD SHP 已核验同源；GCJ02 错配假设被数据否定（crs=internally-consistent）；
- IBD 的 CURVATURE 字段与几何不相关（核验不通过，不作先验）；HEADING 可用；
- UPER 大小基线：绝对分支 678–1373 B/路口，逐点最小档偏移分支再省 32%。
