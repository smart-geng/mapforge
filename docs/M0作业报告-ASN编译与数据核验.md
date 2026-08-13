# M0 作业报告：消息层 ASN.1 编译 + 金凤数据核验（作业①②③④）

> 实施日期：2026-08-13（与 Spike A/B/C 同日）。HANDOFF 第三节 M0 四作业的执行结果。
> 明细输出：`out/asn_extract_report.md`、`out/id_cleanup_report.md`、`out/crs_check_report.md`、`out/geojson/`、`ledger/jinfeng-2026.draft.yaml`。

## 总结论

**M0 的四个"无外部依赖"作业全部完成**：消息层 ASN.1 从送审稿提取并编译通过（UPER 回环 PASS）；7 路口全部解码出 GeoJSON；存量 ID 台账草案生成（24 个失配引用待人工裁决）；**坐标核验判定 SHP 与 MAP 同源同系，GCJ02 假设被数据否定**。附带一个重要推论：金凤 MAP XML 就是从这套 IBD SHP 生成的（多路口逐点重合）。

## ① 消息层 ASN.1：提取 → 编译 → UPER 回环（全通）

- 从送审稿 docx（pandoc 转 md）提取 **163 个类型定义**（grid table + simple table 两种块格式），拼装为 `mapforge/adapters/v2xmap/asn/msglayer-draft.asn`（54 KB）；
- **自动修复送审稿源文本语法瑕疵 5 处**（SEQUENCE 最后成员尾随逗号），已留痕——证实"送审稿≠可直接编译文本"，发布级仍需与正式版 YD/T 3709-2020 差异核对；
- **pycrate 0.8.1 编译 PASS** → 生成运行时模块 `msglayer_draft.py`；
- **UPER 编码回环 PASS**：node16 真实骨架数据（MessageFrame>mapFrame，含 movements/phaseId/lanes）→ 48 字节 → 解码值比对一致。
- 结构探测记录（编码器用）：MapData 必填 msgCnt/nodes；Node 必填 id/refPos；**Link 必填 upstreamNodeId/linkWidth/lanes**；Movement 仅 remoteIntersection 必填。
- 工程注意：pycrate 顶层包名为 `pycrate_*` 系列（无 `pycrate` 模块）；生成器从 `pycrate_asn1c.asnproc` 导入。

## ② 7 路口解码 + GeoJSON 预览（全通）

7 个 XML 全部解析成功（reader 容错含位串空白/缺 movements）：合计 28 Link / 80 车道 / 530 点；每路口输出 GeoJSON（refPos/Link 中线/Lane 中线，属性含 phase、maneuvers、connectsTo）至 `out/geojson/`，QGIS/kepler.gl 可直接打开叠影像目检。phase 取值全部为 {0,1,2,9,10,17,18,25,26}（与资料盘点规律一致）。

## ③ 存量 ID 台账清洗（草案完成，裁决待人工）

- 定量结论：7 文件声明 ID 中**仅 4 个被相互引用可解析，24 个引用失配**（被引用但无文件自称）；region 三种混用 {3, 500, 21901}；node13 自称 (21901,21901)（region=id，工具写入嫌疑）。
- 产出 `ledger/jinfeng-2026.draft.yaml`——**决策文件契约**（方案 7.5 L1 形态首个实例）：每节点 declared/filename_hint/referenced_by + `decision: null`，24 条 unresolved_refs 同样待决；node ID 裁决属 FORBIDDEN_AUTO，**须项目方确认哪套编号为准后转正**。

## ④ CRS 核验：SHP↔MAP 同源同系（H0 全胜）

三路口 × 三假设叠合（MAP 点到 IBD 车道线最近距离）：

| 路口 | H0 同系 | H1 SHP→GCJ02 | H2 MAP→GCJ02 |
|---|---|---|---|
| node16 | **中位 0.00 m**（p90 2.08） | 17.0 m | 48.0 m |
| node4 | **中位 0.00 m（max 0.01 m，逐点重合）** | 16.3 m | 14.9 m |
| node17 | **中位 0.00 m**（p90 0.06） | 29.8 m | 42.8 m |

- **判定：SHP 与 MAP XML 坐标同源同系**；GCJ02 错配假设被否定；`crs_integrity` 由 suspect 升级为 **internally-consistent**（生产链路内 SHP↔MAP 互转无坐标系错配风险）。
- **推论（同源实锤）**：node4/node17 的 MAP 点与 SHP 车道线逐点重合（max 1 cm）——金凤 MAP XML 就是从这套 IBD 交付生成的（生成工具待问项目方）。"SHP→MAP 量产主线"存在存量先例，黄金测试集可做真回归对拍。
- 遗留：相对一致 ≠ 绝对为真 WGS84——与地心坐标的绝对校验需实测控制点/权威影像（M1 前完成即可）；node16 有少量点离 SHP 覆盖较远（max 45.7 m，疑 SHP 覆盖边界/个别 Link 缺车道，待目检 GeoJSON）。

## ⑤（追加同日完成）XML→ASN 全量值映射 + 7 路口整帧 UPER + 大小基线

- `mapforge/adapters/v2xmap/to_asn.py`：reader 结构 → ASN 值全量映射（含 movements/connectsTo/speedLimits/高程），点列支持**绝对分支 / 逐点最小档偏移分支（LL1–LL6）双模式**；
- **7 路口整帧编码回环全部 PASS**（绝对模式值精确一致；偏移模式坐标还原 ≤1 LSB）；
- 大小基线（`out/uper_size_report.md`）：XML 明文 46–97 KB/路口 → **UPER 绝对 678–1373 B**（现网同语义）→ **UPER 偏移 483–911 B，整体再省 32%**（档位分布 LL2/LL3 为主）——方案 2.3"数百字节至 1 KB+"的判断被实测吻合，偏移编码在预算紧张场景（如 node4 绝对编码 1373 B）价值直接。

## ⑥（追加同日完成）OpenDRIVE reader + junction 重塑 → MAP（方向 2 主链）

- `adapters/opendrive/reader.py`：轻量 xodr reader（ElementTree 直读 planView/lanes/link/junction；line/arc 解析求值、spiral 数值积分；poly3 类标记 unsupported——Town03 无）+ 参考线采样/车道偏移工具；
- `ops/simplify.py`：附录 D 抽稀（DP 弦距容差 + 首末点保留）；`ops/junction_to_map.py`：交叉口重塑——驶入端由 laneLink.from 符号判定、Link=驶入侧车道组、**connectsTo=junction laneLink 直译（EXACT）**、maneuver 由 connecting road 首末航向差判别、虚拟台账映射显式导出（正式版从 ledger 消费）；
- **Town03 junction 422 端到端验收：4 incoming Link / 18 条 connectsTo 零丢弃 / maneuvers 位图合理（直+左、直+右、直左右）/ UPER 双模式回环 PASS（绝对 308 B、偏移 272 B）**，GeoJSON 预览落地 `out/town03_j422.geojson`；
- 已知 M0 级近似（正式化清单）：Link 未沿 pred/succ 向上游拼接多条 road（短 incoming 只 2 点）；Link 中线用参考线近似；仅首 laneSection、忽略 laneOffset；remote/upstream 用虚拟 ID。

## M0 剩余项（更新后的清单）

1. ~~OpenDRIVE reader + 交叉口重塑~~（⑥ M0 级完成，正式化清单见上）；
2. ~~XML→ASN 全量值映射 + 整帧 UPER + 大小统计~~（⑤ 已完成）；
3. ~~CLI 骨架~~（已完成：preview/encode/decode/validate-xodr）；
4. **RSU 实机播发验收（需 FusionTest 设备环境）——M0 唯一剩余项**。

## 给项目方的问题（更新）

1. ID 裁决：`ledger/jinfeng-2026.draft.yaml` 的 decision 列（哪套 node 编号为准、region 定哪个值）；
2. 金凤 MAP XML 由什么工具从 IBD SHP 生成（同源已实锤）？该工具是否仍在用（决定兼容边界）；
3. IBD 的 CURVATURE/SLOPE/BANKING 字段写入逻辑（CURVATURE 已核验与几何不相关）；
4. 配时表形态（phaseId 权威来源，M1 需要）。
