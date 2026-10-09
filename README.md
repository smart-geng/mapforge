# mapforge — 地图格式转换工厂

> **研发接续入口（2026-10-09）**：代码统一在 `main`，工程 ZIP 迁移服务内核及测试已提交；客户迁移页面仍待接入。先读 [工作台开发交接](docs/交接说明-2026-10-09-工作台接续.md)，其中有 Git 拉取后的环境/测试步骤和另行交接的材料清单。下文旧阶段结果保留作历史，当前工作台能力与发布边界以该交接为准。

> **当前状态（2026-10-03）**：转换入口都能运行，但还没有一份 XODR 达到用户要求的“忠实原件、非碎段、整体平滑”。执行顺序和验收口径已改为“先定规格，再用评分板迭代”：[优化方向计划](docs/优化方向计划-2026-10-03.md)；接手先读 [HANDOFF.md](HANDOFF.md)。下文“历史门禁事实”里的数字只代表当时版本的检查，不是当前整图验收。

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

```powershell
uv sync                                   # Python 3.11，依赖按 pyproject.toml / uv.lock 锁定
.\.venv\Scripts\python -m pytest -q        # 全量回归（约 12 分钟）；8 项机器相关证据重放登记为 xfail，见 docs/环境迁移记录-2026-10-03.md
.\.venv\Scripts\python -m mapforge.score --out out\scoreboard\<名称>   # 金凤 7 路口 × MAP/SHP 生成并评分
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
