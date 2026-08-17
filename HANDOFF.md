# 地图格式转换工厂 · 会话交接文档

> 更新日期：2026-08-14。项目已落位 `F:\MapFactory\`，方案升级至 **v1.26**（v1.25 几何基线 + 正式 G8 双向车道保真门禁）。
> **完整交接见 [docs/交接文档-mapforge.md](docs/交接文档-mapforge.md)**（架构、验收方法、复跑命令、遗留事项）。
> **遗留工作总排期见 [docs/遗留工作全面规划-2026-08-14.md](docs/遗留工作全面规划-2026-08-14.md)**。
> **新会话请先读本文件，再按第三节的待办继续工作。**

---

## 〇、已完成：正式 G8 闭环（实时交接，2026-08-14）

> 本节是当前会话的最终断点。v1.26 的技术门禁已经完成；下方 v1.25 历史记录仍有效。注意：**技术门禁 PASS 不等于生产交付获批**，当前金凤 CRS 只有 `internally-consistent`，14 个样本的交付决策仍按硬约束为 `BLOCKED`。

### 当前结论

- GUI-01 已完成信息架构评审；GUI 实现未开始。正式 G8 依赖已解除，GUI-02 仍须等待 B1–B4。
- 正式 G8 的契约、来源 manifest、目标 occurrence/component、结构化 gate result、sidecar、交付阻断和机械校准脚本已落地。
- 固定 7 路口 × MAP/SHP 两管道的 14 文件可以完整生成；正式 policy 为 version=`1.0`、lifecycle=`active`，14/14 G8 均为 `PASS`。
- 最新正式全量结果：`scripts/gen_all.py --report-only` 生成矩阵 **14/14**、各 G8 sidecar 实际违规总数 **0**；随后 `scripts/calibrate_g8.py` **PASS**，calibration 8 文件/449 lanes 与 locked-validation 6 文件/320 lanes 的 ceiling 违规均为 **0**。实现过程中曾为 1298 项；没有提高或临时覆写阈值。
- `out/g8-opendrive-jinfeng-v1.candidate.yaml` 已通过语义差异审查并提升为正式 `profiles/validation/g8-opendrive-jinfeng-v1.yaml`。`load_policy` 复核语义 SHA256=`3409f658101c7550ffa1481a12f5ceade5173ac8d60a249f312b5caa71d58b15`，与候选一致；classes/sampling/exclusions/ceiling 未改变。两文件仅换行符不同（候选 CRLF、仓库 policy LF）。
- 全量 pytest：**66 passed in 94.32s**。
- `scripts/closed_loop.py` 与 `scripts/closed_loop.py --no-regen` 均退出码 0：14 文件 G1–G5/G7/G8 全部 PASS，suite 级 G6 esmini RoadManager 14/14 PASS；XSD 由 G1 逐文件验证，`out/closed-loop-report.json` 状态为 `PASS`。
- `scripts/visual_sweep.py` 已生成 14 张统一视角拼图并逐张人工检查；涉及本轮修复的 direct node3/node4/node13/node17 与 m2x node17 均平顺，全部 14 张未见新增孔洞、锯齿、合成振荡、断裂 taper 或明显路面不连续。
- 闭环报告的 14 个 `delivery_decision` 均为 `BLOCKED`，唯一共同硬阻断是 `crs_not_absolutely_verified`（当前仅 `internally-consistent`）；部分 MAP 文件另有 `mirror-no-source-geometry` 人工复核项。这是既定 fail-closed 行为，不是 G8 失败，禁止绕过或把 v1.26 表述为可直接生产交付。
- 正式 G8 技术闭环至此完成，版本基线更新为 **v1.26**；工作树仍未提交、未推送，R0 的恢复点/拆分提交事项仍未完成。

### 独立复验与发布后补正（2026-08-14，另一会话复核）

上述结论已由独立复跑逐条核对，**全部成立**：

| 复验项 | 实测结果 |
| --- | --- |
| `pytest tests -q` | `66 passed`（补正后 `68 passed`） |
| `scripts/closed_loop.py` | 14/14 文件 G1–G8 全 PASS，G6 esmini suite 级 PASS |
| `scripts/closed_loop.py --no-regen` | 同上，PASS |
| active policy 语义 SHA256 | `3409f658…d58b15`，与方案/README 声明一致 |
| 校准双 split | calibration 8 文件/449 lanes、locked-validation 6 文件/320 lanes，ceiling 违规均 **0** |
| 14 份 sidecar | 56 个 JSON 全部 `allow_nan=False` 解析通过；policy hash 与 active 全一致；`errors: []` |
| exclusion codes | 11 类全部在 `allowed_exclusions` 内，无未知码 |
| CLI 非 DELIVERABLE | `convert --to xodr` 打印 `G8: PASS` + `DELIVERY-STATUS: BLOCKED`，退出码 **2**，不打印最终 OK |
| `git diff --check` | 通过；`.claude/worktrees/` 未进入 Git 状态 |

**逼近上限的余量（后续数据/几何改动需盯住）**：769 条可比较 lane 的最坏值——endpoint `1.440 / 1.50`（余量 4%）、stopline `1.367 / 1.50`（9%）、s2t median `0.492 / 0.60`（18%）、p95 `1.080 / 1.50`、max `1.498 / 3.00`、coverage 全为 `1.000`。即 G8 是真通过，但 endpoint 一项余量很薄。

**复验中发现并已修复的两个流程缺陷（`scripts/calibrate_g8.py`）**：

1. policy 提升为 active 后再跑校准，会误报 `source_policy_not_draft`；
2. 更严重：该失败分支会**删除 `out/g8-opendrive-jinfeng-v1.candidate.yaml`** 这一提升证据（复验时已实际触发删除）。

现按 `lifecycle` 分两种模式：`draft → calibrate`（原行为，含候选清理）、`active → verify`（只读复验，绝不写/删候选，回归仍 FAIL）。报告新增 `mode` 字段。新增 2 项测试覆盖两种模式，`pytest 66 → 68`。当前复验输出：

```text
G8 verify: PASS
  calibration: 8 files, 449 lanes, 0 ceiling violations
  locked-validation: 6 files, 320 lanes, 0 ceiling violations
  active policy 1.0 (3409f658101c…) 仍被当前 14 文件满足
```

**draft policy 的可复现性**：`profiles/validation/` 只保留 active，且该目录**未纳入 Git**，draft 原件已不存在。经实测，draft 可从 active **无损复原**——去掉 `calibration.promotion`、`lifecycle` 改 `draft`、`version` 改 `1.0-draft`，复原后的语义 SHA256 = `e16d583dbb71d30ec33ef008fa3fc5fe37613652313a77b927035b78eace71a7`，与 `promotion.source_policy_sha256` **完全一致**，提升链可审计。为避免 `profiles/validation/` 出现两个 policy 造成加载歧义，未把 draft 落回该目录；日常复验请直接用上面的 `verify` 模式。

**遗留风险**：`profiles/validation/` 与 `out/` 均未入库，正式 policy 目前只存在于本地工作树；提交前若丢失，需按上述配方从方案文档记录的 hash 复原并核对。建议尽早把 `profiles/validation/` 纳入版本控制。

### 已落地代码

- 新增：
  - `mapforge/validate/g8_model.py`
  - `mapforge/report/decision.py`
  - `profiles/validation/g8-opendrive-jinfeng-v1.yaml`
  - `scripts/calibrate_g8.py`
  - `tests/test_g8_manifest.py`
  - `tests/test_g8_components.py`
  - `tests/test_g8_faults.py`
  - `tests/test_g8_delivery.py`
  - `tests/test_calibrate_g8.py`
- 主要修改：
  - `mapforge/validate/lane_fidelity.py`
  - `mapforge/adapters/opendrive/writer.py`
  - `mapforge/adapters/shp/ibd_reader.py`
  - `mapforge/adapters/shp/profile_source.py`
  - `mapforge/ops/map_to_xodr.py`
  - `mapforge/ops/shp_to_xodr.py`
  - `mapforge/report/deliver.py`
  - `mapforge/cli.py`
  - `scripts/gen_all.py`
  - `scripts/closed_loop.py`
  - `tests/test_map_to_xodr.py`
  - `tests/test_shp_to_xodr.py`

### 已确认并修复的真实缺陷

1. MAP real-exit 原来只按 node id 查邻居，导致 `(3,3)/(500,3)`、`(3,4)/(500,4)` 跨 region 串绑，出现 200m 级偏差；现已改为完整 `(region,node)` + upstream + 空间一致性匹配，不匹配时退显式 mirror exclusion。
2. MAP/SHP 点列端点外仍在 snap 半径内的点曾被压到同一 s；现已按端点纵向支持域裁剪。
3. SHP `eval_planview(0.5)` 的采样点距并非严格 0.5m，旧代码用 `index*0.5` 累计产生数米 support_s 漂移；现已改用真实累计弧长，并替换 `u/0.5` 索引。
4. SHP via 的 G2 桥接 apron 原来整段冒充实测 via；现在只比较实测中段。无法保留来源的拓扑间隙使用 `source-topology-gap-bridge` 显式 exclusion，并强制进入 review。
5. SHP 生灭/零宽车道 taper 原来参与完整中心线比较；现在用 `lane-transition-taper` 显式裁掉不可比较区间，来源 lane 仍保留在完整 manifest 分母中。
6. SHP 跨 span 配对原来按整段 median 横距，现改为接缝 `v1↔v0` 配对；长 laneSection 等分到不超过 20m。
7. G8 evaluator 现在强制采样每条 lane 的 `support_s` 精确边界，endpoint/stopline 不再受 1m 网格截断影响。
8. SHP 稀疏 2–5 点轮廓、MAP 稀疏点列改用区间内分段线性插值；观测区间外保持端值，禁止斜率无限外推。
9. SHP 端点纵向越界点此前虽已从 manifest/G8 支持域裁掉，却仍参与目标横向轮廓回归，形成不可审计的“幽灵影响”；现已让 `_prof_eval` 与 manifest 共用 `support_ps/support_pd`。定向重跑后 `2023061509384520037`、`2023061509384524082` 的端点、p95 和 coverage 违规全部消失。
10. SHP laneOffset/median/相邻宽度为保持 C0 连续会覆盖来源中心端值，但此前仍把被连续化占用的整段算作可比较来源。现按最终写出 width 的文件语义计算来源端点横移，只在来源 lane 首/末 occurrence 裁掉超过固定 0.5m 构造容差的连续化子段，并使用既有 `lane-transition-taper` 记账；不读取 policy 阈值。node3/node4/node17 的剩余 SHP 违规归零，且未再制造 one-source-many-target。
11. SHP 2–5 点来源在世界坐标中是分段直线，但相对弯曲参考线的 `d(s)` 并不线性；现先沿原折线 0.5m 加密再投影并分段插值，不平滑、不外推、端点和总长不变。node13 三条稀疏 lane 的 median 违规归零。`2023061416492734058` 经连续化裁剪后稳定域仅 1.85m（小于 3m），现整条以 `lane-transition-taper` 显式排除，来源仍保留在完整 manifest，`full_source_length_m=12.204`、`compared_length_m=0`。
12. MAP 稀疏点列存在同一“世界坐标折线 ≠ 弯曲参考线中的端值线性 `d(s)`”问题。现将原始支持点/manifest 与目标重建轮廓分离：公共 `_lane_profile` 默认行为和 manifest 均保留原点列；生产写出显式启用 0.5m 等价折线加密后再投影。MAP/node17 的最后 2 项 median 违规归零，且相邻 lane 无新增违规。

### 最近验证结果

```text
pytest tests/test_g8_manifest.py tests/test_g8_components.py tests/test_g8_faults.py
       tests/test_g8_delivery.py tests/test_calibrate_g8.py -q
=> 21 passed

pytest tests/test_map_to_xodr.py -q
=> 5 passed

pytest tests/test_map_to_xodr.py tests/test_shp_to_xodr.py::test_lane_fidelity_and_no_hairpin -q
=> 6 passed

python scripts/gen_all.py --report-only
=> 14/14 文件全部生成；draft policy 下均为 UNAVAILABLE（预期）

python scripts/calibrate_g8.py
=> FAIL；calibration 15 项 + locked-validation 5 项 ceiling 违规

# 上述正式全量之后的本轮定向验证（仅 SHP node3/node4/node13/node17）
=> node3 / node4 / node13 / node17 均无实际 G8 违规（仅 draft policy_not_active）
=> tests/test_shp_to_xodr.py::test_lane_fidelity_and_no_hairpin：1 passed
=> MAP/node17 无实际 G8 违规；tests/test_map_to_xodr.py：5 passed
=> scripts/gen_all.py --report-only：14/14，输出矩阵完整，各 sidecar 实际违规总数 0
=> scripts/calibrate_g8.py：PASS；calibration 8/449/0，locked-validation 6/320/0
=> active candidate 已审查并提升；语义 hash 3409f658... 保持不变
=> active policy 下 scripts/gen_all.py：14/14 G8 PASS，退出码 0
=> 全量 pytest：66 passed in 94.32s
=> scripts/closed_loop.py：G1–G8 + esmini 全部 PASS，报告 out/closed-loop-report.json
=> scripts/closed_loop.py --no-regen：G1–G8 + esmini 全部 PASS
=> scripts/visual_sweep.py：14 张统一视角拼图全部生成并人工检查通过
=> out/closed-loop-report.json：技术闭环 PASS；14/14 交付仍因 CRS 未绝对核验而 BLOCKED
=> 正式 G8 技术闭环完成，版本基线 v1.26
```

### 当前剩余 lane

最近一次正式 active-policy 全量门禁中，**无剩余 G8 违规 lane**。不得因后续新样本失败而提高 ceiling；应按本轮相同方法定位来源/目标语义。

### 下一接手动作

1. 若继续做 GUI-02，先完成 B1–B4：统一 `ConversionJob/ConversionResult`、run 目录、结构化 gate JSON 与统一预览入口；正式 G8 已不再是阻塞项。
2. 获取实测控制点并完成 CRS 绝对核验；在此之前保留 `crs_not_absolutely_verified` 交付硬阻断，禁止把 `internally-consistent` 改写成绝对正确。
3. 按遗留规划建立可回退恢复点并拆分当前混合工作树。此项涉及提交/remote，由仓库负责人明确授权后执行；当前会话未提交、未推送。
4. 后续代码改动的最小复跑命令：

```bash
.venv/Scripts/python -m pytest tests -q
.venv/Scripts/python scripts/closed_loop.py
.venv/Scripts/python scripts/closed_loop.py --no-regen
.venv/Scripts/python scripts/visual_sweep.py
```

### 仓库与清理注意

- 当前所有改动均未提交、未推送；原始 `shp_0222-0326/` 与 `v2x_map_xml/` 未修改。
- 必须保留本轮之前已有的 GUI-01 文档改动，不得回退。
- `out/debug-node18.*` 是本轮诊断产物，可删除；`out/` 不入 Git。
- `.claude/worktrees/` 是代理隔离目录，禁止加入补丁或提交。

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

**v1.25 极小段假平滑已收口（2026-08-14，历史基线）**：v1.22 的中位段长门禁仍放过 0.667m 碎段，本轮改为最终 xodr 硬门禁：普通 leg 最短段 ≥3m、junction connecting road ≥1m；leg 同时要求来源偏差 ≤1.5m、|κ|≤0.04、|dκ/ds|≤0.0045、翻转≤8/100m。`fit_leg_refline` 只有全部条件同时通过才返回；无解抛 `ReflineFitError`，fallback 仅供诊断。SHP 拼链修正为比较真实连接端切向，候选自身绕街角则在完整 ROADLINK 边界停止；已删除静默裁源线的 `_trim_far_spikes`。车道写出新增 `mapforge.source_lane` provenance，并有按来源 lane 配对的保真/绑错故障测试。**当时验收：pytest 44/44 PASS；14/14 文件通过加强 G7、XSD、planView、G2、断面、换乘和 esmini；普通道路全局最短段约 3.2m。**该段的正式 G8 待办现已由本文件第〇节的 v1.26 闭环完成。

**GUI G0 会话补录（会话 `516d0bd7-6af0-4429-94d2-f011b6aa7a69`，GUI-01 已于 2026-08-14 收口）**：该会话生成的 `docs/GUI方案-mapforge控制台.md` 和 `docs/gui_g0_wireframes.html` 已完成后续结构评审，结论为“修改后接受”；五屏顺序、connect-mode 主区、源/产物常驻叠加和质量页同页拆层已裁决，线框升为 v0.2，并补齐 G8、运行/门禁/交付三轴、`full REVIEW_REQUIRED`、v1.25 G7 阈值与红线来源。在线 Artifact 的 403 不再阻塞本地评审。**GUI 实现仍未开始**；正式 G8 依赖已解除，GUI-02 仍须等待 B1–B4（统一 Job/Result、run 目录、结构化 gate JSON、统一预览入口）。

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
   - **工程卫生完成**：pytest 26 项全绿（`pytest tests -q`，含金凤黄金回归门禁）；README 就位；**git 仓库已建**（main 分支，首次提交 24c2a9d，70 文件；.gitignore 排除 .venv/out/SHP 大数据/xml2xodr 参考代码）。
   - **转向补全模式 connect-mode（2026-08-14 第十五轮，用户"全连接/默认连接填满空隙，路口需要支持这种模式，因为地图本来可能就有问题"，方案升 v1.24）**：新增 `ops/junction_fill.py`（两管道 + CLI 共用）。三档：`data`（默认，仅源数据——"不发明拓扑"仍是默认立场）/ `default`（按车道位置补**应有而缺失**的转向；判"已有转向"用**几何分类**不用数据字段——字段正是可疑的那个）/ `full`（全连接：每条进口车道→每个非掉头出口腿的每条车道，治整片缺录 + 铺满路口）。补出连接标 INFERRED（`stats.conn_filled`）、走与数据连接同一套 G2 回旋链、受**曲率守卫**（|κ|>0.125 即 R<8m 判不可行，计 `conn_fill_skipped`）、掉头默认排除（`--allow-uturn`）。CLI：`--connect-mode data|default|full`。**实测 14 文件**：default 补 12/拒 7，full 补 759/拒 77，**门禁零降级**；esmini 抽检 4 文件全 PASS（96–116 对缝隙 0.0cm、零跳变）。pytest 37（新增 data 档 conn_filled==0 的行为门禁）。
   - **路口铺面材质与验收统一（2026-08-14 第十四轮，用户"补齐为啥不是路/验收和道路不一样"，方案升 v1.23）**：① **lane type 渲染逐类型隔离实测**（上一轮在拼图里目测，结论记反了——教训：颜色/材质这类判断必须单变量隔离+采像素，不能靠多目标拼图目测）：`driving/restricted/shoulder/parking/stop`=沥青(83,83,75)、**`none/border`=浅灰(125,125,113)**、`curb/sidewalk/biking`=混凝土(170,170,154)、`median`=绿化(70,139,88)。铺面原用 `none` → 浅灰补丁；改 **`restricted`**（沥青 + 规范语义"铺装不可行车"），路口内与进口道无缝同材质。② **铺面纳入门禁**：`audit_file` 断面台阶去掉"仅非 junction road"过滤，覆盖所有 road（铺面实测 0.3–0.7cm 达标）。③ 用户提议的"全连接/默认连接填空隙"**未采用**：多边形铺面已 100% 覆盖内部（out/preview/junction_close{,_m2x}.png 实证），而全连接会发明数据中不存在的转向、违反"不发明拓扑"硬约束；如需可行车全连接应设显式开关并标 INFERRED。七门禁 14/14 PASS、pytest 36 绿。
   - **段长与曲率蛇行治理（2026-08-14 第十三轮，用户质疑"极小段拼接会影响自动驾驶车辆"，方案升 v1.22）**：质疑成立——实测 SHP 侧中位段长 1.6–1.9m、50.3% 的段 <2m、node18 单路口 684 段。**先立判据再动手**：`smoothness.curvature_quality/curvature_audit`——段长只是辅助指标，真判据是 sharpness(dκ/ds) 变号率（方向盘微抖）与侧向 jerk v³·dκ/ds（leg 60km/h、conn 30km/h）。**三件核心改动**：① `simplify_planview` 曲率域精简 + κ 去噪（中值+均值，窗**从小到大够用即止**；目标先最少变号再最少段数；`_absorb_short_segs` 每次合并验偏差——**曲率图上直接删控制点会改 ∫κ ds 致航向漂移，实测偏差爆到 30m**）；② `weld_g2` 让 G2 成为**无条件承诺**（g2ify 失控分支残差 + 简化跳过零长阶跃 ⇒ 断差穿透 8e-3~3.3e-2，焊平后回 0.00e+00）；③ 连接路桥切口 3→8m（切口小把三段桥压成 1.6m 碎段）+ 最少段优先（3 段 G2 链偏差 ≤0.5m 就用）。**走不通并删除的路**：曲率剖面直接重建（离散曲率→RDP→clothoid + Kabsch 配准）最好只有 1.5–2.0m 偏差（曲率误差两次积分放大），需非线性最小二乘精修才可行——代码已删，不留死路。**成绩**：段数 3752→2089（−44%）、<1m 段 7.8%→**0.0%**、<2m 50.3%→3.1%、中位 2.00→4.20m；leg 蛇行 18–24→0–11.5/100m、leg jerk 667–1290→0–149。闭环升七门禁（G7 曲率品质）14/14 PASS、pytest 36 绿、视觉 14 张过。
   - **MAP 管道补齐到 SHP 同等（2026-08-14 第十二轮，"MAP XML 转 xodr 也要达到同等效果"，方案升 v1.21）**：`ops/map_to_xodr.py` leg 构建重写——站点网格多 laneSection（≈30m，共享站点 ⇒ 断面天然连续，8→34 段/路口）、实测轮廓跟踪（`_lane_profile` 投影 (s,d) + `_prof_eval` 局部回归 + `_bounds` 中点边界 + Hermite/FC 限幅，与 SHP 同机制；`fc_clamp` 移入 `refline_fit` 共享）、车道 speed 逐段落盘。**三个实测暴露的真问题**：① **车道存在性按点列覆盖判定**——MAP 各车道点列覆盖范围不同（进口 lane1/4 只覆盖近路口段=口部展宽；出口 lane1 只覆盖远端=下游加出车道），旧的"最近值外推"把远端车道摆进路口口部落进对向幅（重叠 2.9m）；改为覆盖外零宽（自然锥形收放）后降至 0.12m，识别 15 条展宽/加出车道，零宽端不写衔接。② 出口瞄准取 **written 链**（median 钳 0 时堆叠外移，否则错开一个车道宽）；口部零宽目标车道自动改瞄最近实存车道。③ **`simplify_planview` 后必须统一 G2**——精简/短段吸收留 8e-3~3.3e-2 κ 残差（旧代码只在精简失败时 G2 化 → SHP 侧 G3 全线回归）；用 `weld_g2` 焊平（不增段、护段长中位；g2ify 会把 node18 中位段长 3.9→2.9m 触发 G7），大阶跃仍走 g2ify 再焊。esmini 探针起点改用 `RM_GetLaneWidthByRoadId` 找车道真正存在的最小 s（零宽处起步会被归邻道，量到的是探针错误）。终态：七门禁 14/14 PASS、pytest 36 绿、视觉 14 张目检过。
   - **视觉全扫描 + FC 限幅（2026-08-14 第十一轮，/goal"多截图多检查不自以为是"，方案升 v1.20）**：`scripts/visual_sweep.py`（14 文件 × 4 机位 odrviewer 截帧拼图 out/preview/sweep/）纳入常规验收——数字门禁与逐张目检双过。目检结论：全部无空洞/锯齿/合成摆动；NODE5 双黄 S 弯 = 数据本形（源车道线交织借道段，node5_overlay*.png 叠画核实）；叠画所见"边缘钩子" = 叠画脚本索引拼接假象（车道数变化处不同物理边界被按序号硬连）——**自检工具也要被自检**。预防修复：轮廓 Hermite 全加 Fritsch–Carlson 单调限幅（`_fc`）。闭环 14/14 PASS、pytest 36 绿。
   - **实测轮廓横断面 + junction 铺面（2026-08-14 第十轮，用户"边缘不是道路形状/还有空洞/做到 SHP 的样子"，方案升 v1.19）**：① 横断面升级为**实测轮廓跟踪**——每车道全程 (s,d) 投影轮廓（`_span_recs` 存 ps/pd）、section 端点**局部线性回归**求值+斜率（`_prof_eval`；中位数在扇形段有一阶偏差）、边界=相邻实测中心中点（`_bounds_at`）、宽度/laneOffset/median 全 Hermite；`_reconcile` 交界调和 + `_emit` 写出层硬连续（续接车道起宽:=上段 written 末宽）。**实证排查链**（值得留档的调试史）：文件与内存记账矛盾 → Lane 对象打标（`_dbg`）对比 doc 与文件 → 从写出系数反解出真实多项式参数 → 定位 v1.18 衔接坡度上限误伤 born 锥形（w0=0 落进钳制分支，张开 3.35 被钳 0.93、记账 3.35 → 隐形 2.4m 台阶）——born 独立分支后**断面台阶全 0.000**。② **junction 铺面**：esmini 实测 outline object 不渲染（替身盒）→ 铺面 road 方案（`writer.add_paving_road`：多边形 PCA 主轴参考线 + type=none 车道逐站扫掠；SHP 用 IBD 交叉口面实测多边形 `stats.paving=polygon`、MAP 用连接路凸包+2.5m `hull` INFERRED；不入拓扑不寻路）。lane type 渲染实测：none/border=沥青、restricted=浅灰、median=绿化带（objtest 截图）。**闭环六门禁 14/14 全 PASS、pytest 36 绿**（路数断言计入铺面、_worst_seam_kappa_gap 跳过铺面路）。odrviewer 截帧 out/preview/odr_node4_paved{,_persp}.png——路口全铺装、边缘贴实测、无露底。shapely 已入环境（铺面切片用）。
   - **全网 G2 + 闭环跑分门禁全绿（2026-08-14 第九轮，/goal"全流程闭环+平滑达标"，方案升 v1.18）**：① `refline_fit.g2ify_planview`——结点开窗切除 + SolveG2（Bertolazzi–Frego）三段回旋线精确重连（位置/航向/曲率全匹配、下游零漂移 1e-10；短段整段吞并、右侧贪心跨段、双趟迭代），挂入 fit_leg_refline 与连接路中段/回退拟合；**v1.13 记档的 line-arc G1 设计点就此消除，14 文件 κ 结点断差全 0.00e+00**。② 桥"猪尾巴"守卫（node18 实锤 |κ|=1.8/R=0.55m 病态解）：桥内 |κ|>0.5 拒 + 切口 ×1/×2/×3 重试 + 次级纯 G2 合成（端点仍精确）。③ **`scripts/closed_loop.py` = /goal 验收闸门**：再生成 14 文件 × 六门禁（XSD/planView/κ<1e-6/台阶<5cm/换乘<1cm/esmini RM）**全部 PASS**；G2+台阶门禁锁入 pytest（36 绿）。odrviewer 截帧 out/preview/odr_node18_final.png（最难路口弯道全程顺滑）。复跑命令：`.venv/Scripts/python scripts/closed_loop.py`。
   - **对向幅连续化 + median C1 样条（2026-08-14 第八轮，用户"还不流畅+有空的"，方案升 v1.17）**：诊断手段升级——**headless odrviewer 截帧**（`odrviewer --headless --capture_screen --camera_mode top`，tga 用 pillow 转档）直接拿 esmini 自家渲染看用户所见；定位：北腿近路口细脖子/南腿轮廓鼓包 = 对向 span 覆盖开天窗。修复：① span 空档/重叠全部中点缝合；② 两端断面保持外推（远端到 0、路口端到 L，`left_extended_m` 记账 APPROXIMATED）；③ median 改**对向内边缘全程单调 C1 样条**（Hermite 逐段；关键：用边缘不用中心——数据链中心 d 与宽度在 span 界各自跳、边缘不跳）；④ leave 边界 6m 稀疏化（2m 微 section 会把锥形压成陡坡，node3 s103/105 实锤）。审计断面匹配先去重零宽车道重复边界（消幻影）。**终态 14/14 esmini 全 PASS 零跳变（node18 也过）**、断面台阶全 0、pytest 36 绿；odrviewer 顶视+透视截帧存 out/preview/odr_node4_after.png / odr_node4_persp.png（街道全宽连续、双黄+绿 median、车道线平顺、车辆正常行驶）。esmini 渲染知识：median 车道渲染为绿化带（非空洞）；密度车流沿车道行驶可当目检。
   - **双侧道路模型 + 自研 OpenDRIVE writer（2026-08-14 第七轮，用户裁决"严格按 OpenDRIVE 定义/scenariogeneration 不好用就不用"，方案升 v1.16）**：① 一条街一条 leg road（参考线沿进口幅、右侧进口 -1..-n、左侧对向出口 +1..+n 逆 s 行车、median 车道两端实测插值、中线双黄、出口连接路接 END 接触点；进出链端点位姿+方向自动配对，落单退化单侧）；② **scenariogeneration 弃用** → `adapters/opendrive/writer.py`（1.5 语义逐项对照 XSD；lane 子元素序/左侧 id 降序/rule=RHT；geoReference 落 PROJ 管线；lane speed/roadMark 原生；_strip/_inject 后处理废除）；共享几何工具移 `refline_fit`（planview_prims/seg_kappa/fit_leg_refline）。③ 消费级修复三件：**leg 曲率封顶 1/30**（node4-west 链 s255-298 实锤 R=10m 振荡噪声，双侧模型 1−tκ→0 车道中心驻点、esmini s 跳 80m；滑动平均渐进重拟合，偏差仍对原始顶点报——合帧 node4 dev 0.68 为修复代价，测试改锁 refit_smoothed≥1 & dev<1.0）；**衔接坡度上限 0.10**（written 末宽跨 section 传递）；对向拼链预算对齐进口参考线长（少留全幅漏斗）。④ 验证域双侧化：smoothness（_sections 含左侧、lane_edges_at(side)/cross_edges_at、route_continuity 支持 contact-end+正 id）、渲染器全断面、rm_check 出侧取 s=L、**车道锥形带豁免**（右灭边界前 22m/左生边界后 22m——探针骑收拢楔=并线非缺陷）。**终态：14/14 XSD+planView 全绿、esmini 换乘缝隙全 0.0cm（235 对）、行驶 13/14 零跳变**——唯一记档 node18 出口幅横断面重划分带（2×3.5→1.6+4.75，~12m 横摆 1.1 m/m，面连续）；pytest 36 绿（road 数公式改 links+conn_roads）。README 依赖行去 scenariogeneration 加 matplotlib。遗留：单侧回退路（30+）未做双侧/无 median；spikes/spike_b 仍引用 scenariogeneration（历史存档）；出口连接路 laneLink to 恒 -1（连接路自身车道）符合语义无需改。
   - **esmini 独立验证 14/14 PASS + 表面连续性修复（2026-08-14 第六轮，用户 odrviewer 截图"镂空+不平滑"驱动，方案升 v1.15）**：esmini v3.6.0 已就位 `esmini/`（用户自装，bin 含 odrviewer/esminiRMLib）。**镂空四类根因定位**（自研填充俯视渲染 `scripts/xodr_topdown.py` + 定位器 `scripts/xodr_diag.py`）：① 生/灭车道以全宽瞬现/瞬失（数据 Link 粒度，外缘 1.8–4.3m 矩形缺口，金凤 6 处大缺口）→ **20m 锥形收放**（`_width_pieces`，生从 0 张开/灭收拢到 0，smoothstep 两端零斜率）；② 断面重划分（node18: 2×3.5 → 1.6+4.75，外缘 0.65m 台阶）→ **匹配车道起宽衔接上段末宽**（sw_join，Σ宽边界连续）；③ 变宽线性折角（22 处，最大 11°）→ 全部 smoothstep 化；④ laneOffset Catmull-Rom 过冲 0.16m → **单调限幅**（Fritsch–Carlson）。附带：**默认标线**（中线实黄/车道间虚白/外缘实白，`_mark`，APPROXIMATED；连接路不加；`_inject_lane_speeds` 插入点改 width|roadMark 之后保 XSD 序）；**审计器升多段 width 求值**（`smoothness._sections/lane_edges_at`——旧读法只取第一段外推会报幻影缺口）。**esmini RM 独立验证**（`scripts/esmini_rm_check.py` 重写，对齐 v3.6.0 真实 ABI：id_t=uint32/double/出参取 lane id——旧 float 签名喂垃圾且对非法 lane id 会段错误）：14/14 PASS——换乘缝隙 esmini 自算全 0.0cm（235 对）、随机行驶零跳变、路口全穿越；两类非缺陷事件已归类为消费策略：junctionSelector"借道"（随机挑别车道的连接路横跳入场）与"灭车道并线点"（收拢归零无后继，探针不并线按同 ID 硬映射，SHP 侧 32 次；**不发明并线拓扑**）。剩余露底=中央分隔带/导流岛/junction 内部车道带间隙（模型语义非缺陷），查看加 `--ground_plane`：`.\esmini\bin\odrviewer.exe --odr out\direct_xodr\node4.xodr --density 2 --ground_plane`。批量再生 `scripts/gen_all.py`（14 文件）。pytest 36 全绿×2 轮。前后对比图 out/preview/node4_direct_{before,after}.png。
   - **换乘连续性构造性归零（2026-08-13 第五轮，回答"实际能否平滑接上/怎么验证"）**：① SHP 连接路两端加 **G2 桥**（`_bridged_geoms`：起点接进口车道、末端接出口车道的**文件语义堆叠位姿**——教训：桥必须瞄准 laneOffset+宽度累减出的堆叠中心而非实测偏移，仿真器消费的是前者；中段保留实测几何，桥失控 >40m 回退）；② laneOffset 升 **C1 三次样条**（Catmull-Rom 斜率，消断面交界斜率折点）；③ 新增 `validate/smoothness.route_continuity` **虚拟行车验证**（按文件语义走"进口车道中心→连接路→出口车道中心"，即任何仿真器的消费方式）——**金凤 7/7：换乘 A/B 两端跳变全 0.0cm、航向差 0.00°**，pytest 门禁锁 <1cm/<0.1°。XSD/连续性依旧全绿。esmini pip 无 wheel 装不上；用户侧独立验证命令：官方 esmini `odrviewer.exe --odr <file> --density 2` 或 esmini 场景放车沿路线行驶。SVG 预览脚本补断面交界采样（消渲染断口假象）。新增 `validate/smoothness.py` 三层审计（段间曲率跳变 Δκ / laneSection 边界车道台阶 / junction 接缝车道级位置闭合）。**审计抓出并修复三个真缺陷**：① spline 档在拼链接点近重复顶点上产生 κ=22 退化微回旋线（R≈4cm）→ 顶点 <3m 去重 + |κ|>0.15 整条熔断弃用；② SHP 进出口参考线短于车道（ROADCENTER 铺不到停止线）→ 按种子车道端点中位延伸；③ 濒死车道残宽台阶（node16 渐变到 1.16m 即断，右边界跳 1.15m）→ 无后继且末宽 <2m 的车道零收口（repair），**断面台阶全归 0**。种子断面改近路口 30% 段测车道偏移（口部对齐优先）。**量化并记档的表达极限**（非缺陷）：斜停止线（node18 各车道末端沿 s 错位 ±1.9m，单一 road 端面无法表达，连接路用真实几何吸收）+ 口部车道扇形分离 ≤1m（xodr 相邻堆叠模型限制）；line-arc 交界为 G1 设计点（Δκ 等效 R 4–9m 在路口口部转角，G2 化=插缓和段，正式化项）。最终：双向 7/7 XSD PASS、连续性 0 违例、断面台阶 0、MAP 接缝 G2 精确；SVG 俯视图产出 `out/preview/`。pytest 36 绿。
   - **两主线打磨完成（2026-08-13 第三轮，SHP→xodr / MAP→xodr）**：① **SHP→xodr 多 laneSection**——拼链改为同名跨车道数（此前遇车道数变化即停，产物含 8–54m 短桩路），每个源 Link 一个 laneSection：S_WIDTH→E_WIDTH 线性变宽（width a/b）、分段 laneOffset 线性过渡（lane0 边界连续）、相邻 section 车道一对一贪心匹配写跨段衔接（落单=生灭车道不写，消 scenariogeneration 覆写警告）。金凤实测：node4 进口路 217.7m/6 段 [2,2,3,4,4,4]（上游 2 车道到停止线 4 车道完整演化）；7 路口进出口路 laneSection 数百段、变宽车道数百条全落盘。② **车道限速注入**：SHP MAX_SPEED（km/h）与 MAP vehicleMaxSpeed（0.02 m/s 步长，833→60km/h）逐车道写 `<speed>`（`_inject_lane_speeds` 后处理，两管道共用）。③ **CLI 默认 SHP 源切 Profile 引擎**（ibd-smarteditor-v1）——实测其宽度阶梯从边界横距救回 6 条 WIDTH=0 脏数据车道（如 4272mm 真值），行为优于内置直读器；等价测试改为**超集契约**（结构全等+非宽度内容逐字节全等+宽度仅在内置缺数据处分歧）。④ 修 `_i` 小数容错（S_WIDTH='4272.0' 被 int(str) 吞 0）。真实数据验收：**SHP→xodr 7/7 与 MAP→xodr 7/7+合帧全部 XSD PASS、连续性 0 违例**；连接路长度分布正常（14–56m 无失控 U 转）。pytest 36 项全绿。
   - **MAP→xodr 真实出口优先 + 全接缝平滑（2026-08-13 第二轮）**：① 多节点 MAP 帧支持（`parse_map_xml_all`）——邻居节点中"上游=本节点"的 inLink 即本路口**真实出口路**（几何/车道全真），镜像降为无数据时的兜底（金凤合帧 node4+18+3 实测：真实 2 + 镜像 2，`--node-id` 选主节点）；② 平滑三件套：连接路起止改用**路模型端部位姿**（不再用原始点列末段航向，消停止线折角）、SolveG2 传入两端**车道级曲率**（κ/(1−tκ) 偏移修正）实现全接缝 G2（实测 16/16 曲率差 0.000000）、镜像只取近路口 120m+剔除偏移尖点；③ **拟合器新增稀疏链 spline 档**（`fit_sparse_g1_spline`，pyclothoids G1Hermite 顶点插值，正式化清单"spiral 档"落地）——修复附录 D 抽稀长弦点列上渐变曲率（缓和曲线）被误拟成直线+圆角链的问题（node17 north 顶点偏差 1.69→0.00），三档择优判据统一为**对原始顶点偏差**（弦上插值点不是测量，平滑曲线正确鼓出弦线不受罚）；resample 丢末点 bug 顺带修复。7/7 偏差峰值 0.13–0.31m 全 XSD PASS。pytest 升 36 项（合帧真实出口 + 接缝曲率 <1e-6 门禁）。
   - **MAP→xodr 的 junction 脑补已落地（2026-08-13，`ops/map_to_xodr.py` 重写）**：MAP 消息无路口内几何/出口路，junction 全部由消息自有信息推断（INFERRED 语义）——出口方向 = connectsTo remote 回查各进口 Link 的 upstreamNodeId（"驶向 X"="沿从 X 来的进口反向离开"）；出口路 = 对应进口 Link 左偏镜像（偏移=进口半幅+出口半幅）；连接路 = 进口车道末位姿→出口车道起位姿 G2 clothoid；进口路车道按点列实测放置（laneOffset+每车道真实宽）。**金凤 7/7：connectsTo 覆盖 95/96 条，XSD 全 PASS、连续性 0 违例；唯一 skip = NODE5 west→20701 已知 ID 失配引用（资料盘点三.3），设计裁决：失配诚实 skip 不得几何猜测编拓扑**。CLI `--connect` 手动通道退役。pytest 升 35 项（快测锁全覆盖+失配 skip 行为）。
   - **SHP Profile 引擎已实装（2026-08-13，`adapters/shp/profile_source.py`）**：一份 YAML 声明图商的图层/字段/单位/编码/几何路线，`convert --profile` 即用，内核零改动接新图商。双路线：A=车道中心线+宽度字段；B=边界线+左右关系（中线合成 `midline_of` + 边界横距推宽）。宽度阶梯 field→boundaries→spacing→default（非 field 记 APPROXIMATED，`derivation_stats` 计数随 CLI 输出）。可选层全可缺省降级：无 junction 层→`--at`+端点聚类（REVIEW）、无 topo→连接为空、无 road 层→按车道 road 字段合成归组、无中心线→中间车道替代。CLI 三件套：`profile-init`（注释模板）/`profile-check`（图层字段体检+宽度量级+降级预告）/`convert --profile`。**证据**：`ibd-smarteditor-v1.yaml` 与内置 IbdSource 产物去时间戳逐字节全等；`ibd-boundary-demo.yaml`（屏蔽 WIDTH 全走边界推导）恒宽车道误差中位 14mm、离群 2 条=口部展宽变宽车道（数据真相）。pytest 升 32 项。仍开放：字段别名词典/自动推断向导（待第二家真实交付）。
   - **SHP→xodr 直转已修正落地（2026-08-13，`ops/shp_to_xodr.py`）**：纠正架构违规——CLI 曾把 SHP→xodr 借道 SHP→MAP→xodr（空口消息窄门，产物无 junction 的孤立 road）。直转要点：ROADCENTER 中心线同名拼链 ~160m + 偏差驱动升级档拟合（>0.3m 换细档，5° 微折角→line-arc-line）、laneOffset 按车道实测横向位置（IBD 中心线在本幅中间，±1.75/±5.25 对称跨线）、每车道真实宽度、完整 `<junction>` + 连接路（TOPO 两跳中间车道实测几何；**路口内连接车道不在 MERGE 层**，在普通层路口内 Link 上）+ Connection/laneLink + 路级/车道级前驱后继。**金凤 7/7：213 条连接路全实测几何（G2 合成 0），XSD 1.5M 全 PASS、连续性 0 违例、拟合偏差峰值 0.29–0.38m**。CLI 新增 `--at lon,lat` 定位（摆脱对参考 XML 依赖）；pytest 升 27 项（test_shp_to_xodr 锁结构底线）。新文档《接入指南-新图商SHP数据需求与Profile》（图商字段差异四步流程 + 数据需求分层表 + 最小可转集）。遗留简化：S_WIDTH/E_WIDTH 变宽、高程（SLOPE 未核验）、多 laneSection、标线 roadMark。
   - **M1 继续**：Link 中线正式化（车道组中线）、出度策略开关、配时表模板替代演示通道、MapIR pydantic 数据类合流、M0 近似项正式化清单：Link 上游回溯拼接、Link 中线（参考线→车道组中线）、多 laneSection/laneOffset、remote ID 接 ledger、MapIR pydantic 数据类合流、拟合器正式化（边界精修/段数最优化/假 arc 归直/spiral 档）。
2. **方案继续细化清单**（v1.0 遗留，未变）：MapIR v0.1 pydantic schema、CLI 命令面（mapforge convert/validate/preview/diff/profile-wizard）、M0 工作包人日分解、profiles/shp YAML 规范文档、SHP→MAP 与 OpenDRIVE→MAP 流水线伪代码。
3. **SHP 字段映射三件套**（v1.0 遗留）：~~字段级校验报告~~（已实装=`profile-check`）；仍开放：字段别名词典 / Profile 自动推断向导——ibd-smarteditor-v1 是第一个词典输入源（含 10 字符截断实例 STOPLINE_R、单位不一致实例 CENTRE_X），待第二家真实交付后再抽象。
4. 向项目方提问清单（已答 2 项）：~~ID 裁决~~（inherit-as-is 已落地）；~~phase 缺失处理~~（按设计：有来源即绑、无则 BLOCKED/显式降级，金凤场景现网 XML 即来源）；仍开放：配时表形态（新路口用）、部署形态、xodr 消费方、XML 生成方、J2735 出口需求、IBD 枚举文档。

## 四、与原项目（FusionTest）的关联（不变 + 一条新增）

- FusionTest（F:\1\next_FusionTest\FusionTest，Python+Zenoh MEC 测试框架，活跃线 origin/feature/20260429）消费本工厂交付包做 RSU 实机播发 HIL 验证；其 MEC 感知消息 `map_location` 引用 MAP 的 region/node/lane ID。
- feature 分支 `report/renderers/precision_map_renderer.py` 可复用为 MAP GeoJSON 底图渲染参考。
- **新增**：`v2x_map_xml/` 下的 TCI 一致性测试控制协议 asn（v1.0.1）是 FusionTest 可消费的设备测试协议资产，建议移交测试框架侧评估。
- 工程边界不变：mapforge 独立仓库，FusionTest 只消费交付包。
