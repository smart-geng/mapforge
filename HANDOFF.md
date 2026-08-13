# 地图格式转换工厂 · 会话交接文档

> 更新日期：2026-08-13（第二次会话）。项目已落位 `F:\MapFactory\`，新资料已全部消化，方案升级至 **v1.1**。
> **新会话请先读本文件，再按第三节的待办继续工作。**

---

## 一、当前状态

《地图格式转换工厂：首批 OpenDRIVE / SHP / V2X MAP 落地方案》**v1.1** 已定稿：v1.0 的 13 节框架不变，基于本地新资料完成重新规划（变更清单见方案文档头部 v1.1 变更记录）。

- 在线版（Artifact，可分享）：https://claude.ai/code/artifact/da8aaa79-5d69-4c80-9f3c-b275b68c7584
  - 更新方式：编辑本地文档后调用 Artifact 工具发布，**把上面 URL 作为 `url` 参数传入**（否则会新建独立 Artifact）。
- 本目录文件清单：

```text
F:\MapFactory\
├─ HANDOFF.md / CLAUDE.md                    # 交接 + Claude 指引（已含数据资产清单）
├─ OpenDRIVE_1.4H.xsd / OpenDRIVE_1.5M.xsd   # VIRES 官方 XSD（门禁资产）
├─ shp_0222-0326\                            # IBD 规格 SHP 真实交付（44 图层，重庆金凤）
├─ v2x_map_xml\                              # 7 路口 MAP XML + 消息层送审稿 docx + TCI asn + xml2xodr 参考代码
└─ docs\
   ├─ 地图格式转换工厂-首批三格式方案.md       # ★ 方案正文 v1.1（唯一权威版本）
   ├─ 资料盘点-金凤示范区数据与标准资料.md     # ★ 新资料消化结论（本地事实的权威来源）
   ├─ 调研报告-V2X-MAP标准与工具链.md
   ├─ 调研报告-OpenDRIVE生态.md
   └─ 调研报告-SHP高精地图与合规.md
```

## 二、v1.1 关键增量（速览，详见方案与资料盘点）

1. **M0 前置依赖解除**：消息层标准送审稿 docx 在手（五消息 ASN.1 代码块全）——"购买标准正文"改为 M0 第一工程作业"从 docx 提取拼装 .asn → pycrate 编译，用 TCI asn + 7 路口 XML 双向核对"。
2. **同目录 .asn 文件是 TCI 一致性测试控制协议**（TciMsgLayerFrame），不是消息层本体，不能直接编译出 MAP codec；它是 FusionTest 侧的一致性测试资产（已写入方案 9.5）。
3. **SHP Profile v1 落定 ibd-smarteditor-v1**：IBD 44 图层三层结构，含显式车道拓扑（LANE_TOPO_DETAIL）、路口内虚拟车道（LANE_LINK_MERGE）、交叉口面进出 Link 引用、停止线/信号灯/箭头的车道引用——SHP→MAP 关键环节从推断升级为字段直读。长度单位毫米；同名字段跨图层单位不一致等脏数据实录见资料盘点二.4。
4. **ID 台账清洗升为 M0 紧急作业**：7 个现网 XML 的 region 三种混用、双编号体系、交叉引用失配（实锤表格见资料盘点三.3）→ 产出 `ledger/jinfeng-2026.yaml` 初始台账，裁决规则需项目方确认。
5. **CRS 真伪核验 = M0 第一数据作业**：SHP 与 XML 都只是"声称 WGS84"（.prj 自定义 WKT 无 EPSG；xml2xodr 存在 GCJ02 纠偏开关）——核验前 crs_integrity=suspect 禁止进生产。
6. **MAP→OpenDRIVE 由 P2 上调 P1**：既有 xml2xodr"MAP 还原"链路证明业务需求真实（金凤+长沙两地），其产物无 junction/无高程/不可复现——mapforge 以完整 junction 直译替代。写出版本策略增补 1.5（本地消费端基线 1.4/1.5，XSD 已在仓库）。
7. **黄金测试集改以金凤 7 路口为基底**（真实 phase 绑定 + SHP↔MAP 同路口对拍），新增 ID 混乱、XML 方言容错两个场景；现网方言事实（绝对坐标分支、phaseId=0=右转无灯控、占位高程 500、位串带空白）已写入方案 1.2/5.4。
8. 仍开放的外部输入只剩：**配时表获取形态**（M1 需要）、**IBD 字段枚举权威文档**（值域反推可先行）、方案 13 节的部署形态/xodr 消费方/XML 生成方三问。

## 三、下一步待办（新会话从这里继续）

0. ~~参考线拟合 spike~~ **已完成（2026-08-13，结果见《Spike报告-参考线拟合验证》，方案已升 v1.5）**：A/B/C 全部通过——拟合路线验证成立（Town03 回拍类型零漏检、半径误差中位 6–15%；node16 端到端 XSD PASS + 连续性 0 违例；SolveG2 连接路可用；pyclothoids Windows wheel 直装成功）。**重大发现：IBD CURVATURE 字段核验不通过**（与几何不相关），曲率先验降级、字段核验升为强制门禁。已有代码资产：`mapforge/ops/refline_fit.py`、`mapforge/adapters/v2xmap/xml_reader.py`、`mapforge/validate/planview_check.py`、`.venv`（Python 3.10，正式环境换 3.11）。拟合器正式化待办：边界局部精修、段数最优化（Maier）、大半径假 arc 归直、spiral 档接入。
1. ~~M0 四项无依赖作业~~ **已完成（2026-08-13，《M0作业报告-ASN编译与数据核验》，方案升 v1.6）**：
   - ① ASN.1：163 定义编译 PASS + UPER 回环 PASS（`adapters/v2xmap/asn/msglayer-draft.asn` + 运行时 `msglayer_draft.py`；送审稿修复 5 处尾随逗号已留痕）；
   - ② 7 路口解码 + GeoJSON（`out/geojson/` ×7）；
   - ③ ID 台账 **已转正 `ledger/jinfeng-2026.yaml`**（2026-08-13 项目方裁决：`inherit-as-is`——编号沿用现网存量不重排、失配照抄；生成侧 `id_table_from_xml` 从现网 XML 继承 upstream/remote，交付包 upstream_ids_pending=0）；
   - ④ CRS 核验：**SHP↔MAP 同源同系（逐点重合），GCJ02 否定**，crs_integrity=internally-consistent；绝对校验（实测点）遗留 M1 前。
   - ⑤（追加完成）XML→ASN 全量值映射 + 7 路口整帧 UPER 回环全 PASS + 大小基线（绝对 678–1373 B / 偏移 483–911 B，省 32%，`out/uper_size_report.md`）；⑥ CLI 骨架 `python -m mapforge.cli`（preview/encode/decode/validate-xodr）。
   - ⑦（追加完成）xodr reader + junction 重塑（`adapters/opendrive/reader.py`、`ops/junction_to_map.py`、`ops/simplify.py`）：Town03 junction 422 端到端 → MAP UPER 回环 PASS（connectsTo 直译 18 条零丢弃）。
   - **（v1.7）M0 宣告完成（离线口径）**：mapforge 定位澄清为纯离线工具，"RSU 实机播发"移出工具里程碑、重定义为交付级可选验收（FusionTest 侧消费场景，方案 8.1⑤/9.1）；离线生态兼容证据 = 现网 7 路口 XML 对拍全等。
   - **M1 批量对拍已完成（《M1进展-SHP转MAP主线》）**：金凤 **7/7 路口** SHP→MAP 直读生成并与现网 XML 真回归对拍——横向中位的中位 **0.00 m**、车道数 6/7 完全一致、phase 5/7 全命中、平均覆盖率 78%、UPER 全回环 PASS。代码：`adapters/shp/ibd_reader.py` + `scripts/m1_shp_to_map.py`（批量版）。断链结论：TOPO 上游缺录（数据问题非算法）。
   - **转换矩阵已可用（统一入口 `python -m mapforge.cli convert`）**：MAP XML→geojson/uper/xodr、Town03(xodr)→MAP、IBD SHP→MAP（--like 参考 XML）全部实跑通过；新增 xml_writer（XER 写出，读写回环全等）与 ops/map_to_xodr。
   - **交付包已落地（`--to map` 即产方案 8.3 目录，report/deliver.py）**：三视图+quality/loss+id-mapping/diff+provenance+config+STATUS 共 11 文件；**红线实测生效**（无 phase → BLOCKED exit=2，--allow-no-phase 显式降级记 DROPPED）。样例 out/deliver/。
   - **工程化整备完成**：SHP 重塑与 GeoJSON 预览已入包（ops/shp_to_map、report/preview_geojson），CLI 自足不依赖 scripts/；拟合器加保真后处理（假 arc 归直/同类合并，refine 后禁并弧）；全量回归绿。已知限制：σ=5cm 压力组复合弧分段模糊（根治=Maier 最优分段，正式化项）。
   - **M1 继续**：Link 中线正式化（车道组中线）、出度策略开关、配时表模板替代演示通道、MapIR pydantic 数据类合流、M0 近似项正式化清单：Link 上游回溯拼接、Link 中线（参考线→车道组中线）、多 laneSection/laneOffset、remote ID 接 ledger、MapIR pydantic 数据类合流、拟合器正式化（边界精修/段数最优化/假 arc 归直/spiral 档）。
2. **方案继续细化清单**（v1.0 遗留，未变）：MapIR v0.1 pydantic schema、CLI 命令面（mapforge convert/validate/preview/diff/profile-wizard）、M0 工作包人日分解、profiles/shp YAML 规范文档、SHP→MAP 与 OpenDRIVE→MAP 流水线伪代码。
3. **SHP 字段映射三件套设计**（v1.0 遗留）：字段别名词典 / Profile 自动推断向导 / 字段级校验报告——ibd-smarteditor-v1 是第一个词典输入源（含 10 字符截断实例 STOPLINE_R、单位不一致实例 CENTRE_X）。
4. 向项目方提问清单（已答 2 项）：~~ID 裁决~~（inherit-as-is 已落地）；~~phase 缺失处理~~（按设计：有来源即绑、无则 BLOCKED/显式降级，金凤场景现网 XML 即来源）；仍开放：配时表形态（新路口用）、部署形态、xodr 消费方、XML 生成方、J2735 出口需求、IBD 枚举文档。

## 四、与原项目（FusionTest）的关联（不变 + 一条新增）

- FusionTest（F:\1\next_FusionTest\FusionTest，Python+Zenoh MEC 测试框架，活跃线 origin/feature/20260429）消费本工厂交付包做 RSU 实机播发 HIL 验证；其 MEC 感知消息 `map_location` 引用 MAP 的 region/node/lane ID。
- feature 分支 `report/renderers/precision_map_renderer.py` 可复用为 MAP GeoJSON 底图渲染参考。
- **新增**：`v2x_map_xml/` 下的 TCI 一致性测试控制协议 asn（v1.0.1）是 FusionTest 可消费的设备测试协议资产，建议移交测试框架侧评估。
- 工程边界不变：mapforge 独立仓库，FusionTest 只消费交付包。
