# mapforge — 地图格式转换工厂

> 本文件为新项目的 Claude 指引草案，随项目推进增补。

## 项目定位

面向车路协同与自动驾驶的语义地图编译工厂：源格式 → 统一中间表示 MapIR → 目标格式/版本/Profile，每次转换强制输出质量报告、损失报告、ID 映射与来源清单。**交叉口是第一公民**。

首批格式：OpenDRIVE（读 1.4–1.8，写 1.5/1.6/1.7）、SHP（Schema Profile 驱动，Profile v1 = ibd-smarteditor-v1）、V2X MAP（T/CSAE 53-2020 / YD/T 3709 体系，UPER 编码 + XER 风格 XML 明文双载体）。

## 基本规则

- **2026-09-15阶段归档**：用户授权本次整理/commit/本地归档，不是地图发布或远端推送。当前继续点统一为精简HANDOFF和`docs/遗留工作-阶段收口-2026-09-15.md`；旧HANDOFF全文在`docs/archive/2026-09-15/HANDOFF-history.md`。7条共享分界线Web和UI v2可运行，但外缘/口部/MAP未开放、整图仍BLOCKED。下一步收敛到真实外缘事件人工修形，不恢复无限自动研究，后续不自动提交。

- **当前继续点（覆盖下面旧点）**：`out/node4-whole-dynamics-r1-20260914/`及HANDOFF顶部。长曲线动态检查已接同一5父路/24转向状态，924检查区间591 BOUNDED/333实际超限；11混速转向为最大源限速包络，真实速度域未完。初值不变、零拟合/零新XODR、281哈希未变；不是地图交付/形式安全认证。下一步实际写出域/全源所有权、混速映射、面域和动态残差接完整求解器；不重复1676列扫描，不冻结父路逐条拟合，不改源速度/9处批准，不commit。

- **最新继续点**：`out/node4-whole-joint-step-r2-20260914/`与HANDOFF顶部。全部1676自由列已算、左右矩阵1724×1676；240列差分分歧、77列单侧有效，不是光滑Jacobian认证。5父路＋24转向原子联合微扰与整图独立重算差0，仍507组不等式失败；零拟合/零新XODR。系数差分用全部源仿射见证和精确端口依赖加速，轴/口部/布局仍完整重建，不能固定C/CZ。下一步全源写出端域/区间、动态/面域及非光滑共同求解；不是重复抽查或冻结父路逐条补解。278哈希不变，172＋3测试；9处批准不扩/不重问，源数据/速度不改，MAP等仍未完，未commit。

- 最新全图实施见HANDOFF顶部与`out/node4-whole-source-model-r4-20260914/`：用户本轮“继续”已仅确认西2南1，决策`profiles/repair/node4-west-south-zero-width-source-roles-v1.yaml`；与既有北2东4共9处，物理边界和完整movement路径分离，不再重问这9处、不扩大到其他输入。原件/TOPO/速度/CRS不改。

- 完整5父路与24长连接路已在同一1681维状态实算（含5个固定0的非路口cut槽），共享轴/口部/边界及源尾段随同一状态变化。西11微扰影响106–111六个转向而不改其他；每转向5长原语、每段≥6m，普通width跨度≥5.5m。只是状态和形状核接通，初值仍明显失败；未进行数值优化、无新XODR、未交付。

- **2026-09-14当前执行顺序**：用户要求停止局部补丁。先完成可保持结构的原件残差/Jacobian、全源所有权与实际写出外端/区间、动态和面域约束，再按全局方案预登记最多3个完整候选及计算预算。不能套旧固定C/CZ的GN-QP、冻结父路逐条拟合、盲加种子或放宽门槛。123相关回归＋3真实复核不是整图/MAP验收，不推广CLI、不commit。

- **始终使用中文回复用户**
- 方案与决策以 `docs/地图格式转换工厂-首批三格式方案.md`（v1.74及2026-09-14顶部全路口共同状态补充）为唯一权威版本；接手先读`HANDOFF.md`顶部。下文/历史v1.71–v1.74数值试验均非交付；保留v1.71研究件，新r4仅JSON状态。原7.526348m多要素域分配、完整非线性求解/面域/动态/MAP/CRS/movement/Web仍未完成，不把组件或局部PASS当整图完成。
- 事实性调研结论（标准条款、开源项目状态、政策）以 `docs/调研报告-*.md` 为准；**本地数据/资料的事实以 `docs/资料盘点-金凤示范区数据与标准资料.md` 为准**——不要凭记忆重新断言；确需更新时先核查再改文档
- 技术栈：Python 3.11+；GDAL/GeoPandas/Shapely/pyproj（GIS）、pycrate（ASN.1/UPER）、libOpenDRIVE+lxml（xodr 解析/拓扑）、scenariogeneration（xodr 写出）、asam-qc-opendrive + esmini（门禁）

## 本地数据资产（只读原料，勿修改原件）

- `shp_0222-0326/`：IBD 规格车道级 SHP 真实交付（44 图层，重庆金凤，声称 WGS84 **未核验**，长度单位毫米）
- `v2x_map_xml/map*.xml`：7 个现网路口 MAP 消息 XML 快照（region=500 体系，**ID 混乱待清洗**；黄金测试集原料）
- `v2x_map_xml/*.docx`：消息层技术要求**送审稿**（五消息 ASN.1 代码块全，消息层本体 ASN 从此提取）
- `v2x_map_xml/*.asn`：TCI 一致性测试控制协议（**不是**消息层本体，作核对参照/FusionTest 资产）
- `v2x_map_xml/xml2xodr/`：既有 MAP→xodr 参考代码（只吸收思路，代码不入库）
- `OpenDRIVE_1.4H.xsd` / `OpenDRIVE_1.5M.xsd`：VIRES 官方 XSD（validate/ 门禁资产）

## 硬约束（写代码/改方案时不可违反）

1. **phaseId 绑定禁止自动推断**（FORBIDDEN_AUTO）：必须来自配时表输入或人工标注；信控路口缺相位默认阻断交付
2. **region/node ID 只消费台账不发明**；重生成地图必须输出 id-diff 报告（ID 稳定性 = 下游兼容性）
3. **CRS 缺失/可疑即停止生产转换**，绝不静默按 WGS84 处理；所有坐标变换记录 PROJ 管线
4. MAP 点列抽稀判据 = T/CSAE 159-2020 附录 D（弦距容差 + Link 首点为上游进入第一点、末点为停止线中心）
5. MAP 播发合规：周期 ≤1s、Priority=16、AID=3618（实机门禁检查项）
6. 未识别的图层/字段/扩展一律 PASSTHROUGH 进扩展区，不丢弃；转换状态用八态枚举（EXACT/TRANSFORMED/APPROXIMATED/INFERRED/EXTENSION/PASSTHROUGH/DROPPED/FAILED）
7. 许可证：GPL 组件（CommonRoad 系）只能进程隔离调用或离线对拍；LGPL（pycrate/vanetza）库引用不改源码；GitHub cv2x 仓库无许可证，仅作 asn 核对参照不得复用代码

## 目录约定（方案 7.3）

```text
mapforge/
├─ mapir/        # model / geometry / crs / provenance / idmap
├─ adapters/     # opendrive/ shp/ v2xmap/
├─ profiles/     # shp/*.yaml v2xmap/*.yaml opendrive/*.yaml（版本化）
├─ ops/          # junction_discovery / approach / connect_infer / simplify / budget / repair
├─ validate/     # roundtrip / topo_metrics / size_budget / asam_qc / esmini_load
├─ report/       # loss / quality / preview_geojson / id_diff
├─ ledger/       # region/node ID 分配台账
├─ cli.py
└─ tests/golden/ # 黄金测试集（交叉口场景）
```

## 关联项目

FusionTest（F:\1\next_FusionTest\FusionTest，MEC 测试框架）：消费本工厂交付包做 RSU 实机播发 HIL 验证；其 MEC 感知消息 `map_location` 引用 MAP 的 region/node/lane ID。两仓库保持独立。
