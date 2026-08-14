# mapforge — 地图格式转换工厂

面向车路协同与自动驾驶的语义地图编译工厂：**OpenDRIVE / 车道级 SHP / V2X MAP 消息**互转，
每次转换强制输出质量报告、损失报告、ID 映射与来源清单。交叉口是第一公民。

方案与决策见 [docs/地图格式转换工厂-首批三格式方案.md](docs/地图格式转换工厂-首批三格式方案.md)（当前 v1.25）；
接手开发先读 [HANDOFF.md](HANDOFF.md)。

OpenDRIVE 写出为**自研规范级 writer**（`mapforge/adapters/opendrive/writer.py`，
1.5 语义逐项对照 XSD：双向 leg road ±车道、median、roadMark、车道 `<speed>`、geoReference）。

## 快速开始

```bash
python -m venv .venv
.venv/Scripts/pip install numpy scipy pyshp lxml pyclothoids pycrate typer pyyaml pytest matplotlib
.venv/Scripts/python -m pytest tests -q          # 44 项（含金凤黄金回归）
.venv/Scripts/python scripts/closed_loop.py      # 14 文件七门禁 + esmini
```

数据放置（不入库）：IBD 规格 SHP 交付放 `shp_0222-0326/`；现网 MAP XML 放 `v2x_map_xml/`。

## 转换

```bash
# SHP 图商交付 → MAP 交付包（uper+xml+geojson 三视图 + 质量/损失报告 + id-mapping + provenance）
python -m mapforge.cli convert shp_0222-0326 --to map --like v2x_map_xml/map凤苑路-金玥路node4.xml

# SHP 图商交付 → OpenDRIVE 直转（完整 junction + 连接路实测几何 + laneLink + 多 laneSection
# 变宽断面 + 逐车道限速；默认走 Profile 引擎；--at lon,lat 亦可定位）
python -m mapforge.cli convert shp_0222-0326 --to xodr --like v2x_map_xml/map凤苑路-金玥路node4.xml

# 现网 MAP XML → 仿真 OpenDRIVE（junction 自动重建：出口路优先用多节点帧邻居真实数据、
# 单节点帧镜像兜底 INFERRED；连接路 G2 且全接缝车道级曲率连续；稀疏抽稀点列走 clothoid spline 档；
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
├─ validate/   planview_check(连续性门禁)
├─ cli.py      preview / encode / decode / convert / validate-xodr
ledger/        region/node ID 台账
tests/         拟合单测 + 编解码回环 + 金凤 7 路口黄金回归
docs/          方案 · 调研 ×3 · 资料盘点 · 评审 · 文献 · Spike/M0/M1 报告
```

## 关键事实（详见 docs/）

- 金凤 7 路口 SHP→MAP 对拍：车道几何横向中位 0.00 m，phase/结构与现网一致（真回归基准）；
- 金凤 7 路口 SHP→xodr 直转：213 条 junction 连接路全部实测几何，XSD 全 PASS、连续性 0 违例
  （新图商接入与数据需求见 docs/接入指南）；
- v1.25 普通道路禁止极小段假平滑：最终 xodr leg 最短段 ≥3m、连接路 ≥1m；无合格拟合候选直接阻断；
- MAP XML 与 IBD SHP 已核验同源；GCJ02 错配假设被数据否定（crs=internally-consistent）；
- IBD 的 CURVATURE 字段与几何不相关（核验不通过，不作先验）；HEADING 可用；
- UPER 大小基线：绝对分支 678–1373 B/路口，逐点最小档偏移分支再省 32%。
