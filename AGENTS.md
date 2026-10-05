# mapforge — 地图格式转换工厂

> Codex 指引；与 CLAUDE.md 保持同一内容，改一处要同步另一处。

## 项目定位

面向车路协同与自动驾驶的语义地图编译工厂：源格式 → 统一中间表示 MapIR → 目标格式/版本/Profile，每次转换强制输出质量报告、损失报告、ID 映射与来源清单。**交叉口是第一公民**。

首批格式：OpenDRIVE（读 1.4–1.8，写 1.5/1.6/1.7）、SHP（Schema Profile 驱动，Profile v1 = ibd-smarteditor-v1）、V2X MAP（T/CSAE 53-2020 / YD/T 3709 体系，UPER 编码 + XER 风格 XML 明文双载体）。

当前主线（用户目标）：纯离线把 **SHP→XODR、MAP XML→XODR** 做好——忠实原件、世界车道边缘与路口平滑、不用密集短段冒充平滑（消费者含自动驾驶）；MAP 缺真实出口按同一进口左右镜像并标 INFERRED。

## 当前入口

- 接手先读 `HANDOFF.md`（一页）。执行顺序和验收口径见 `docs/优化方向计划-2026-10-03.md`。
- 默认转换流程（2026-10-03 起）：`python -m mapforge.cli convert` → `mapforge/pipeline.py`。SHP 用口部前移 3 m 和路面重建（`shp_mouth_envelope`、`envelope_surface`），MAP 自动应用已批准的源修正（`profiles/source-corrections/`，每次转换另写源点复核报告 `.source-review.json`），两条管道都做后处理 `g2-k04-c2`（车道边界处处 C2、口部边缘精确对齐、口部车道中心曲率与相邻车道一致、转弯连接路曲率单向、连接路参考线可直接落在相邻车道中心、逐条择优；SHP 连接路按源 via 线保真拟合（`connector_source_fit`：段长 ≥3 m 的回旋线链，罚曲率变化率跳变，择优取离源 P95 + 每段 1 cm 最小，只用于 G8 可比车道）；MAP 道路侧按 MAP 车道点列重拟合，自由远端延到车道首点，口部移到各进口最靠前的停止线，离去侧按进口侧精确镜像；SHP 道路侧车道出生/消失提前半个转角（`lane_birth_advance`，零宽段标 INFERRED）、短段转角能按曲率目标放下就保留、顶点值最小二乘（口部端点固定），两条并行 link 的车道之间有间隙时补一条 restricted 车道（`lane_gap`），转弯连接路两端 10 m 内允许 ≤0.02 /m 的回打，选中的连接路在口部边缘曲率超过 1e-3 时把那一端曲率变化率朝相邻道路修整（`_end_rate_trim`），单独的离去车行道起点挪到最远 via 接头与路缘外展结束处之后 3 m（`shp_leave_mouth`），每次 SHP 转换另写源复核 `.source-review.json`（边界优先，源冲突只记录；口部路缘外展记为源特征 `mouth-curb-flare`，评分板另报去掉外展区的车道中心 `lane_center_noflare_*`；SHP 的 G8 比较窗口在门禁前对齐到写出车道的道路端，`window_align`；源中心线偏离边界中点的道路车道在偏离段按两边界中点比较，`window_midpoint`；2026-10-05 起 G8 数字与以前各版不直接可比））。复现旧证据用 `--shp-mouth legacy --post none`。评分板策略 0.4-draft 起在 T2 检查路口接口曲率，0.5-draft 起 SHP 的边界和端点按报告项口径（内段、横向分量）判，0.6-draft 起 SHP 的车道中心去掉口部路缘外展区判。
- 方案正文以 `docs/地图格式转换工厂-首批三格式方案.md` 为权威；`docs/archive/` 里的逐轮状态是历史，不是当前指令。

## 基本规则

- **始终使用中文回复用户**
- 不自动 commit、不 push，除非用户明确要求。
- 几何质量以评分板为准：`python -m mapforge.score --out out/scoreboard/<名称>`。改几何要给出评分板前后对比，不能用单点、单项检查或"测试数量"宣称完成；草案阈值不能为让候选通过而改。
- 2026-10-03 冻结（D1）：L01 的 E3 登记与 E1/E2 准入链、L07 整图求解器、"相对 r6 不退步"护栏。旧证据、旧 FAIL 原样保留，用户不要求就不重启。
- **证据绑定**：`out/` 研究证据按字节哈希锁定了大量代码（包括 `cli.py`、`shp_to_xodr.py`、`map_to_xodr.py`、`writer.py`、`refline_fit.py`、`smoothness.py`、`g11.py`、`decision.py`、`tests/conftest.py`）。新功能写进新模块；改受绑定文件前先查影响。不要用 `git checkout/restore/stash/reset` 改写工作区文件——本机 `core.autocrlf=true` 会把 LF 换成 CRLF，破坏绑定。
- 本机 `F:` 是 `E:` 的 subst，旧证据里的 `F:\MapFactory` 路径靠它解析。
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
