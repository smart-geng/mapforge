# mapforge — 地图格式转换工厂

> AI 协作指引；AGENTS.md 与 CLAUDE.md 保持同一内容，改一处要同步另一处。

## 项目定位

面向车路协同与自动驾驶的语义地图编译工厂：源格式 → 统一中间表示 MapIR → 目标格式/版本/Profile，每次转换强制输出质量报告、损失报告、ID 映射与来源清单。**交叉口是第一公民**。

首批格式：OpenDRIVE（读 1.4–1.8，写 1.5/1.6/1.7）、SHP（Schema Profile 驱动，Profile v1 = ibd-smarteditor-v1）、V2X MAP（T/CSAE 53-2020 / YD/T 3709 体系，UPER 编码 + XER 风格 XML 明文双载体）。

当前主线（用户目标）：纯离线把 **SHP→XODR、MAP XML→XODR** 做好——忠实原件、世界车道边缘与路口平滑、不用密集短段冒充平滑（消费者含自动驾驶）；MAP 缺真实出口按同一进口左右镜像并标 INFERRED。

## 当前入口

- 云端接续先读 `docs/云端接续与工程迁移闭环-2026-10-09.md`：main 含迁移页面、未编译工程修复，真实原料和工程证据从 `handoff_assets/` 恢复。新克隆必须保持 `.gitattributes` 的按字节保存规则，锁定 Profile 不得换行转换；Linux 仍不能执行 Windows DLL 消费者。

- 接手先读 `docs/交接说明-2026-10-09-工作台接续.md`（工作台现状、未提交文件、启动换机、证据与接续计划）和 `HANDOFF.md` 顶部。工作台范围及验收以 `docs/评审与实施计划-交互式地图修补工作台-2026-10-08.md` 为准；旧转换背景见 `docs/交接说明-2026-10-08.md`，几何主线执行口径见 `docs/优化方向计划-2026-10-03.md`。
- 默认转换流程（2026-10-03 起）：`python -m mapforge.cli convert` → `mapforge/pipeline.py`。SHP 用口部前移 3 m 和路面重建（`shp_mouth_envelope`、`envelope_surface`；转换器自己的进口道路参考线拟合失败时由 `leg_fit_fallback` 接手：稳健拟合、截远端链接、放宽曲率档；路面重建对零宽分隔带、端帽个别点、落在路口面内的分隔带尾部另有规则，都只在原逻辑失败时起作用，2026-10-08），MAP 自动应用已批准的源修正（`profiles/source-corrections/`，每次转换另写源点复核报告 `.source-review.json`），两条管道都做后处理 `g2-k04-c2`（车道边界处处 C2、口部边缘精确对齐、口部车道中心曲率与相邻车道一致、转弯连接路曲率单向、连接路参考线可直接落在相邻车道中心、逐条择优；SHP 连接路按源 via 线保真拟合（`connector_source_fit`：段长 ≥3 m 的回旋线链，罚曲率变化率跳变，择优取离源 P95 + 每段 1 cm 最小，只用于 G8 可比车道）；MAP 道路侧按 MAP 车道点列重拟合，自由远端延到车道首点，口部移到各进口最靠前的停止线，离去侧按进口侧镜像；进口侧在离 MAP 点列不超过 max(5 cm, 现距离) 内光顺（`map_approach_fair`），离去侧在弯曲段缓和（`map_departure_ease`，口部与远端不动）；两条管道的单车道连接路在参考线拼接处整形宽度，使车道边缘少跳曲率、车道中心不变（`connector_edge_joins`）；SHP 道路侧车道出生/消失提前半个转角（`lane_birth_advance`，零宽段标 INFERRED；原件车道越过分段边界仍未合拢时，事件点取原件宽度归零处，不越过原件端点，`_source_event`，2026-10-10）、短段转角能按曲率目标放下就保留、顶点值最小二乘（口部端点固定）、口部 8 m 区内被删的原件拐点在区界留锚点（`mouth_anchor`，2026-10-09；源数据从区界直线进口部时口部斜率随锚点后的直线，口部外展时保持不加锚点的口部斜率，锚点到口部斜率超过 0.12 的路缘转角不加锚点，2026-10-10），两条并行 link 的车道之间有间隙时补一条 restricted 车道（`lane_gap`），转弯连接路两端 10 m 内允许 ≤0.02 /m 的回打，选中的连接路在口部边缘曲率超过 1e-3 时把那一端曲率变化率朝相邻道路修整（`_end_rate_trim`），单独的离去车行道起点挪到最远 via 接头与路缘外展结束处之后 3 m（`shp_leave_mouth`），每次 SHP 转换另写源复核 `.source-review.json`（边界优先，源冲突只记录；口部路缘外展记为源特征 `mouth-curb-flare`，评分板另报去掉外展区的车道中心 `lane_center_noflare_*`；SHP 的 G8 比较窗口在门禁前对齐到写出车道的道路端，`window_align`；源中心线偏离边界中点的道路车道在偏离段按两边界中点比较，`window_midpoint`；2026-10-05 起 G8 数字与以前各版不直接可比））。复现旧证据用 `--shp-mouth legacy --post none`。评分板策略 0.4-draft 起在 T2 检查路口接口曲率，0.5-draft 起 SHP 的边界和端点按报告项口径（内段、横向分量）判，0.6-draft 起 SHP 的车道中心去掉口部路缘外展区判；2026-10-07 起另报光顺项 `fair_*`（不分级），0.7-draft 起 T2 检查连接路车道边缘拼接曲率跳变 ≤ 4e-3 /m（自定，待复核）。0.8-draft（2026-10-09 用户决定）起 SHP 车道中心另去掉急弯原件冲突连接路整条车道：原件 via 窗口任意 3 m 内最小半径 <5 m 的可比 junction-via 记为 `tight-turn-source-conflict`，不跟随（`tight_turn_conflict`，评分 `lane_center_noconflict_*`）。0.9-draft（2026-10-09 用户决定）起 T1 路面孔洞改按 `paving_holes_counted` 判（阈值 0、0.01 m² 截面不变）：铺面轮廓在每个 planView、laneOffset、width 记录起点也取样，消除弦近似伪孔；与源路面重建证据绑定的原件空洞（`surface-evidence.json` 的 `original_holes`，哈希与面积核对、双向重合）重合的孔洞单列 `paving_holes_source_void`，不计失败；无此证据（默认管道）时全部计数（`paving_holes`，原 `paving_holes_gt1cm2` 继续报告）。0.10-draft（2026-10-09 用户决定）起 T1 的 G8 改按 `g8_status_stopline_resolved` 判：SHP 进口车道原件无 LANE_REL 停止线关联时，取原件下游端 2 m 内最近的原件停止线（`stopline-nearest-match`，INFERRED，照常量停止线差）；2 m 内没有的记 `source-stopline-absent`，其停止线要求单列不计（`stopline_match`）。G8 本身、原 `g8_status` 与交付裁决不变。2026-10-08 起评分板之外另有泛化集 `python -m mapforge.validate.generalization`（16 个评分板没见过的 SHP 路口，清单 `profiles/validation/generalization-set-v1.yaml`，用评分板同一策略打分、不分级，登记 `experiments/generalization.jsonl`）：改几何除评分板不退步外也要看泛化集。
- 2026-10-08 接续：连接路独立候选恢复已接入 `mouth_frame_align` / `mouth_recovery`。默认 C2 中新增异常恢复必须先通过原双向保真、覆盖率、反打、中心曲率跳变及口部边缘残差检查；无合格者保留原失败。定版 `20261008-safe-mouth-recovery-v3`：评分板同机旧代码对照 14 份逐字节不变，泛化 T1 7 → 8、T2 仍 2/16、转出仍 10/16；061310 未修通。见 `docs/阶段2-连接路独立候选恢复-2026-10-08.md`。
- 2026-10-08 分支归并：当前统一从 `main` 接续；原 `stage2-2026-10-05` 与远端旧 main 的提交历史均已纳入。11 个受绑定研究脚本保留现行字节，采纳未绑定的测试路径与依赖改进；取舍见 `docs/分支归并-main-2026-10-08.md`。
- 方案正文以 `docs/地图格式转换工厂-首批三格式方案.md` 为权威；`docs/archive/` 里的逐轮状态是历史，不是当前指令。

## 基本规则

- **始终使用中文回复用户**
- 不自动 commit、不 push，除非用户明确要求。
- 几何质量以评分板为准：`python -m mapforge.score --out out/scoreboard/<名称>`。改几何要给出评分板前后对比，不能用单点、单项检查或"测试数量"宣称完成；草案阈值不能为让候选通过而改。
- 2026-10-03 冻结（D1）：L01 的 E3 登记与 E1/E2 准入链、L07 整图求解器、"相对 r6 不退步"护栏。旧证据、旧 FAIL 原样保留，用户不要求就不重启。
- **证据绑定**：`out/` 研究证据按字节哈希锁定了大量代码（包括 `cli.py`、`shp_to_xodr.py`、`map_to_xodr.py`、`writer.py`、`refline_fit.py`、`smoothness.py`、`g11.py`、`decision.py`、`tests/conftest.py`）。新功能写进新模块；改受绑定文件前先查影响。不要用 `git checkout/restore/stash/reset` 改写工作区文件——本机 `core.autocrlf=true` 会把 LF 换成 CRLF，破坏绑定。
- 原交接机器 `F:` 是 `E:` 的 subst；2026-10-08 接续机器的实际仓库为实体盘 `F:\MapFactory`，E: 已有其他数据，不能照搬映射。换机先核对盘符和证据绝对路径，再用 uv / Python 3.11.16 重建 `.venv`（见 `docs/交接说明-2026-10-08.md` 第 4 节和 `docs/阶段2-连接路独立候选恢复-2026-10-08.md` 第三节）；查某文件是否受证据绑定：`python scripts\bound_files.py <文件>`。
- 事实性调研结论（标准条款、开源项目状态、政策）以 `docs/调研报告-*.md` 为准；**本地数据/资料的事实以 `docs/资料盘点-金凤示范区数据与标准资料.md` 为准**——不要凭记忆重新断言；确需更新时先核查再改文档。
- 技术栈：Python 3.11（uv，`pyproject.toml` + `uv.lock`，版本锁定在 2026-09 归档值；3.12 的 `sum()` 改了浮点求和，会改变冻结证据的数值，暂不升级）；numpy/scipy/shapely/pyproj/lxml/pyshp（几何与 GIS）、pyclothoids、pycrate（ASN.1/UPER）；OpenDRIVE 用自研 writer（`mapforge/adapters/opendrive/writer.py`，不用 scenariogeneration）；门禁 XSD + esmini（asam-qc-opendrive 尚未接入）。

## 本地数据资产（只读原料，勿修改原件）

- `shp_0222-0326/`：IBD 规格车道级 SHP 真实交付（44 图层，重庆金凤，声称 WGS84 **未核验**，长度单位毫米）
- `v2x_map_xml/map*.xml`：7 个现网路口 MAP 消息 XML 快照（region=500 体系，**ID 混乱待清洗**；黄金测试集原料）
- `v2x_map_xml/*.docx`：消息层技术要求**送审稿**（五消息 ASN.1 代码块全，消息层本体 ASN 从此提取）
- `v2x_map_xml/*.asn`：TCI 一致性测试控制协议（**不是**消息层本体，作核对参照/FusionTest 资产）
- `v2x_map_xml/xml2xodr/`：既有 MAP→xodr 参考代码（只吸收思路，代码不入库）
- `OpenDRIVE_1.4H.xsd` / `OpenDRIVE_1.5M.xsd`：VIRES 官方 XSD（validate/ 门禁资产）
- 注意：GitHub 远端 `smart-geng/mapforge` 当前为 public，已含上面的 MAP XML 和送审稿 docx；改私有/清理历史由用户决定，不要代为操作。

## 硬约束（写代码/改方案时不可违反）

1. **phaseId 绑定禁止自动推断**（FORBIDDEN_AUTO）：必须来自配时表输入或人工标注；信控路口缺相位默认阻断交付
2. **region/node ID 只消费台账不发明**；重生成地图必须输出 id-diff 报告（ID 稳定性 = 下游兼容性）
3. **CRS 缺失/可疑即停止生产转换**，绝不静默按 WGS84 处理；所有坐标变换记录 PROJ 管线
4. MAP 点列抽稀判据 = T/CSAE 159-2020 附录 D（弦距容差 + Link 首点为上游进入第一点、末点为停止线中心）
5. MAP 播发合规：周期 ≤1s、Priority=16、AID=3618（实机门禁检查项）
6. 未识别的图层/字段/扩展一律 PASSTHROUGH 进扩展区，不丢弃；转换状态用八态枚举（EXACT/TRANSFORMED/APPROXIMATED/INFERRED/EXTENSION/PASSTHROUGH/DROPPED/FAILED）
7. 许可证：GPL 组件（CommonRoad 系）只能进程隔离调用或离线对拍；LGPL（pycrate/vanetza）库引用不改源码；GitHub cv2x 仓库无许可证，仅作 asn 核对参照不得复用代码

另：9 处已批准的零宽出生来源角色（北2、东4、西2、南1，`profiles/repair/`）只在原对象生效，不扩大、不再重问；原限速、TOPO、原件不改。

## 目录约定（方案 7.3）

```text
mapforge/
├─ mapir/        # model / geometry / crs / provenance / idmap（目前只有 crs_probe）
├─ adapters/     # opendrive/ shp/ v2xmap/
├─ ops/          # 转换与研究内核（部分研究模块仍 import spikes/、scripts/）
├─ validate/     # 门禁与评分板（scoreboard.py、replay.py）
├─ report/       # 交付裁决 / 预览
├─ repair_web/   # 本地受限修形台
├─ score.py      # python -m mapforge.score
└─ cli.py
profiles/        # shp/ validation/ repair/（版本化）
ledger/          # region/node ID 分配台账
experiments/     # registry.jsonl：评分板实验登记
```

## 关联项目

FusionTest（F:\1\next_FusionTest\FusionTest，MEC 测试框架）：消费本工厂交付包做 RSU 实机播发 HIL 验证；其 MEC 感知消息 `map_location` 引用 MAP 的 region/node/lane ID。两仓库保持独立。
