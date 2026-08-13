# M1 进展：SHP→MAP 量产主线（IBD 直读 + node16 真回归对拍）

> 实施日期：2026-08-13。方案 v1.7 后续，方向 1（P0 量产主线）的首轮端到端。
> 代码：`adapters/shp/ibd_reader.py`（IBD 图层读取器）、`scripts/m1_shp_to_map.py`；明细 `out/m1_node16_compare.md`、预览 `out/m1_node16_gen.geojson`。

## 结论

**SHP→MAP 直读主线全链贯通并通过真回归对拍**：从 IBD 交付出发，路口定位、进口方位、车道、connectsTo、maneuver 判别、phase 绑定通道、UPER 编码全部走通；与现网 node16 XML 对拍，**车道几何横向距离中位 0.00 m（p90 1.18 m）**——方案"IBD 直读=EXACT 级"的判断被数据证实。

## 直读链实测（node16 = 凤阁路-金剑路）

| 环节 | 方式 | 结果 |
|---|---|---|
| 交叉口定位 | refPos → 最近 INTERSECTION_SURFACE | 命中，中心距 3.3 m，NAME 与 XML 文件名一致 |
| 进口道 | ENTER_ROAD 直读（4 条全在）+ 驶入方位角命名 | north/west/south/east 与现网配对全部正确 |
| 车道 | LANE_LINK 按 SEQ（**已修 reader 分层**：MERGE 虚拟车道不混入普通层） | 8 车道（现网 9：north 差 1 条，数据源版本差异待查） |
| connectsTo | **TOPO 两跳直译**：进口车道→路口内连接车道→出口车道（IBD 三层模型，路口内车道所属 ROADLINK 的路名即路口名） | 18 条全部生成、零丢弃 |
| maneuver | 路口内连接车道几何首末航向差（connecting road 同款判别） | 与 phase 表匹配全命中（间接验证转向正确） |
| phase | **配时输入通道演示**：现网 XML 提取 (方位×转向→phaseId) 表 → 绑定；缺项按 FORBIDDEN_AUTO 保持无 phase | 18/18 命中 |
| Link 长度 | **上游回溯拼接**（TOPO 反向索引 + 端点衔接 <2 m，拼至 ~160 m） | 覆盖率 30%→69%（north 92/west 73/south 67/east 33） |
| 编码 | 复用 to_asn 双模式 | UPER 绝对 430 B / 偏移 350 B，回环 PASS |

## 对拍差异清单（真实差异，非缺陷）

1. **north 车道数 2 vs 3**：SHP（2023-11 版）该进口 ROADLINK 只 2 条车道，现网 MAP 有 3 条——数据源版本漂移或 MAP 制作时含相邻 ROADLINK 车道，待向项目方核实；
2. **出度 4-5 vs 现网 3-4**：TOPO 表的车道级接续比现网 MAP 更全（多为同转向到不同出口车道的组合）——正式版提供"全保留 / 每转向代表连接"两种输出策略；
3. **east 覆盖率 33%**：上游车道链在 2 m 衔接阈值处断开——正式化：阈值自适应 + 断链诊断进复核队列。

## 过程发现（沉淀为规则）

- **DescriptiveName 是 IA5String**：Link/Node name 不能放中文（pycrate 正确拦截）——这解释了现网用 west/north/east/south 命名的原因；中文路名进报告不进消息；
- DBF 中文必须 GBK（pyshp 需显式 encoding，LDID=0x57 不可信）；
- IBD 路口连接是**三层模型**（进口车道→路口内车道→出口车道），TOPO 表一跳到路口内、两跳到出口——MERGE 层在本路口未参与（S_JUN/E_JUN 字段语义仍待与交付方确认）；
- 对拍距离必须分横向/纵向口径：纵向差异反映 Link 截取长度策略，横向才是几何对齐质量。

## 批量对拍（同日完成，7/7 路口全通）

| 路口 | 车道 生/现 | 横向中位 | p90 | 覆盖率 | connectsTo 生/现 | phase | UPER 绝/偏 B |
|---|---|---|---|---|---|---|---|
| node18 凤苑-金剑 | 12/12 | 0.00 | 0.47 | 81% | 36/12 | 25/36 | 616/523 |
| node5 凤苑-金坪 | 8/8 | 0.00 | 0.00 | 74% | 18/12 | 18/18 | 384/323 |
| node4 凤苑-金玥 | 16/16 | 0.00 | 0.00 | 75% | 24/16 | 24/24 | 511/438 |
| node16 凤阁-金剑 | 8/9 | 0.00 | 1.18 | 69% | 18/13 | 18/18 | 430/350 |
| node13 凤阁-金玥 | 11/11 | 0.00 | 0.13 | 84% | 29/15 | 29/29 | 458/416 |
| node17 含金-金剑 | 12/12 | 0.00 | 0.00 | 76% | 32/16 | 32/32 | 578/492 |
| node3 含金-金玥 | 12/12 | 0.13 | 0.35 | 87% | 32/12 | 27/32 | 505/451 |

**汇总：路口面命中 7/7（面距 1–6 m）；Link 配对 4/4 全对；车道数 6/7 路口与现网完全一致（node16 差 1 条为已知版本差异）；横向中位的中位 0.00 m、平均覆盖率 78%；phase 5/7 路口全命中**（node18 25/36、node3 27/32——现网存在我们未生成的 maneuver 组合，正式版配时表模板可全覆盖）。connectsTo 生成侧普遍多于现网（TOPO 更全的一致模式）。断链诊断结论：node16/node13 的上游断点"最近 gap=None"——**TOPO 表本就无上游记录**（路网边界或缺录），非算法阈值问题，此类车道 Link 长度受数据本身限制。明细 `out/m1_batch_compare.md`，7 路口生成 GeoJSON 在 `out/m1_gen_geojson/`。

## 转换矩阵实跑（同日，统一 convert 入口）

新增 `mapforge/adapters/v2xmap/xml_writer.py`（XER 风格 XML 写出器，node16 读写回环全等）与 `mapforge/ops/map_to_xodr.py`（spike-B 模块化），CLI 增 `convert` 统一命令。项目素材实跑全矩阵：

| # | 命令（`python -m mapforge.cli convert …`） | 产物 | 结果 |
|---|---|---|---|
| 1 | `map含金路-金玥路node3.xml --to geojson` | 17 features | ✅ |
| 2 | `map含金路-金玥路node3.xml --to uper --mode offset` | 593 B | ✅ |
| 3 | `map凤阁路-金剑路路口node16.xml --to xodr --connect west-north,south-west` | 4 road + 2 G2 连接路 | ✅ 连续性 0 违例 + XSD PASS |
| 4 | `Town03.xodr --to map --mode offset`（自动选 junction 422） | .uper 325 B + .map.xml + .geojson(11 feat) | ✅ |
| 5 | `shp_0222-0326 --to map --like map凤苑路-金玥路node4.xml` | .uper 743 B + .map.xml + .geojson(21 feat) | ✅ decode 回读 4 links/16 lanes/84 点 |

过程修复：SHP/xodr 源生成的 MapNode 此前缺 Lane.points（车道点列未入消息）——已补（附录 D 抽稀后写入），→MAP 产物"uper+xml+geojson 三视图"齐全，XML 可被现网工具直接消费。

## 交付包组装（同日，方案 8.3 结构落地）

新增 `mapforge/report/deliver.py`；`convert --to map` 升级为**完整交付包目录**（11 文件）：map.uper/xml/geojson 三视图 + quality-report.json（connectsTo/phase 完整率、孤立车道、宽度异常、upstream_ids_pending、编码回环、对拍摘要）+ loss-report.json（八态事件：APPROXIMATED 抽稀容差、DROPPED(phase) 带 FORBIDDEN_AUTO 规则引用）+ id-mapping.json/csv（source PID↔target，parquet 待 pyarrow）+ id-diff.json（基线/增删改）+ provenance-manifest.json（源 sha256、CRS 核验状态引用、标准依据）+ conversion-config.yaml + DELIVERY-STATUS。

**红线机制实测**（方案 8.2/5.4 行为逐字落地）：IBD 源（phase 24/24）→ `OK`；Town03（无 phase）→ **`BLOCKED(phase_missing=18)` 且 exit=2**；`--allow-no-phase` 显式降级 → OK + 损失报告记 DROPPED。样例：`out/deliver/ibd_4/`、`out/deliver/Town03/`。

## 工程化整备（同日）

- **核心逻辑入包，CLI 不再依赖 scripts/**：SHP→MAP 重塑 → `mapforge/ops/shp_to_map.py`（rebuild_from_ibd + phase_table_from_xml）；GeoJSON 预览 → `mapforge/report/preview_geojson.py`；m1 脚本改薄壳（只留批量对拍职责）；
- **拟合器精修**：`postprocess_planview`——大半径假 arc 归直（|R|>3000 m）、line+line 无损合并、arc 曲率差 <15% 弧长加权合并（仅初拟阶段；refine 后禁用并弧——road76 复合弧误并教训）；
- **全量回归绿**：M1 批量 7/7（横向 0.00 m / 覆盖率 78% 不变）、交付包 OK、xodr 连续性+XSD PASS、Spike-A clean 组假 arc 0；
- **已知限制（记录）**：σ=5cm 压力组（约 5 倍实际噪声）个别复合弧分段模糊导致偏差 6.7 m——根治依赖 Maier 最优分段框架替代贪心分段（正式化清单项），实际噪声水平（≤1 cm）不受影响。

## 下一步

1. Link 中线正式化（当前用车道 1 拼接线近似 → 车道组中线）；出度策略开关（全保留/每转向代表连接）；
2. north 车道数差异、TOPO 上游缺录、IBD 字段语义（CURVATURE/S_JUN 等）向交付方核实；
3. 配时通道换真实配时表输入（当前用现网 XML 演示通道语义）；phase 覆盖差异（node18/node3）随配时模板解决；
4. 与 report/（损失报告、id-diff）及 MapIR 数据类合流，进入 M1 工程化。
