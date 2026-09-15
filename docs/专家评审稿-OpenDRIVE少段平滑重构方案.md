# mapforge OpenDRIVE 少段平滑重构方案（专家意见落实与复验版）

> **2026-09-10 补正**：下文 v0.3 为历史技术门禁结论，不代表全部原始几何复原。最新 [v1.35 实测复核](复核报告-源几何保真与路口平滑-v1.35.md) 已发现共同入口截断、裁剪比较掩盖误差及 7m 弦方向误分类。正式版未裁剪行车链对拍仍有 59/189 条超过严格复核线；node18 的少段来源入口候选已改善，但铺面 G9 未通过，尚未提升到默认路径。

> 文档版本：v0.3（2026-08-22）<br>
> 对应项目方案：mapforge v1.34 真实数据实现闭环<br>
> 当前结论：**专家重大修改项的核心生成与门禁已实施；金凤固定回归集技术验收通过，通用化仍为有条件通过**<br>
> 当前实现：SHP→OpenDRIVE、V2X MAP XML→OpenDRIVE 共 14 份真实输出已通过 G1～G11 与 esmini；v1.31 仅保留为实施前基线<br>
> 评审对象：道路平面线形、OpenDRIVE 建模、自动驾驶地图/规划、车路协同 MAP 数据专家

## 1. 评审摘要

mapforge 已完成真实 SHP→OpenDRIVE 与 V2X MAP XML→OpenDRIVE 的少段平滑技术闭环：固定 7 个金凤路口、两条管线共 14 份 OpenDRIVE 已通过 XSD、参考线/车道连续性、路面无孔洞、源数据保真、G11 少段/动力学/兼容性、esmini RoadManager 和人工多视角检查。

实施前结构审计曾发现：v1.31 的“平滑”只证明几何连续，并未限制 `<planView><geometry>` 碎片度。特别是 SHP 路口连接路采用“两端 G2 回旋线桥 + 中段点列拟合”，形成 6～17 个 geometry；连接路 geometry 中位长度约 2.72m，76.7% 小于 5m。这不符合项目方提出的工程要求：**不得依赖连续的米级小段拼接实现视觉平滑，以免增加自动驾驶地图解析、轨迹参考和离散控制的不确定性。** v0.3 已将这一段保留为重构背景，而非当前状态。

本轮实现将目标从“连续且看起来平滑”升级为：

> 在源数据误差容限内，以最少的 line / arc / spiral 参数段重建整条物理道路；同时满足道路边缘保真、G2 连续、动力学舒适性和 OpenDRIVE 消费端稳定性。

外部专家评审结论为“**有条件通过，重大修改后实施**”。本版采纳了以下强制修改：

1. 用“硬约束可行集 + 字典序模型选择”替代 `wN·N` 加权凑最少段。
2. 三 clothoid 仅作为通用三段候选，不能无条件固定输出；单段和有设计含义的 `spiral-arc-spiral` 应参与同层比较。
3. G11 拆为 A～E 五组，同时约束 planView、laneOffset、width、laneSection、实际车道轨迹、来源状态和消费端兼容性，禁止把复杂度转移到横断面。
4. `<5m` 降为风险信号；结合滑动窗口密度、相对段长、可合并性、数值条件和动力学综合裁决。
5. junction 允许在受限短窗口内联合优化进口、出口和 connecting road，但禁止移动停止线、改变真实车道口或大范围调整道路主体。
6. “一条物理道路一条 reference line”改为默认候选；必要时允许拆为两个方向性 carriageway。
7. 测试集扩展为合成真值、真实黄金和压力集；当前 14 份样本只负责固定回归，不能单独校准通用阈值。
8. 输出拆分为自动驾驶、仿真展示等消费端 Profile；辅助 paving road 和 `paramPoly3` 不再全局默认启用。

上述核心修改已经进入主链路并由真实数据复验。当前仍不作“行业通用完成”声明：金凤 14 份只证明固定回归集，G11 policy 保持候选生命周期；跨城市/第二图商、更多合成压力集，以及 `odr15-ad-strict` 与 `odr15-sim` 的显式消费 Profile 分层仍需后续实施。生产交付还独立受绝对 CRS 核验约束。

## 2. 项目背景

### 2.1 项目定位

mapforge 是一个纯离线地图格式转换工具，面向车路协同、自动驾驶仿真和地图数据治理。首批处理三类格式：

- 车道级 SHP：图商交付的车道中心线、边界、道路中心、拓扑和属性图层。
- V2X MAP：T/CSAE 53-2020 / YD/T 3709 体系的路口级点列与车道连接关系。
- OpenDRIVE：面向仿真器和自动驾驶工具链的参数化道路、车道和 junction 模型。

本次评审只涉及两条输出主线：

1. SHP→OpenDRIVE，必须直转，不得借 V2X MAP 消息模型作为中间表示。
2. V2X MAP XML→OpenDRIVE，必须自动补齐 OpenDRIVE junction；MAP 缺少出口时按同一进口横断面左右镜像，存在真实出口时使用实际数据。

### 2.2 三种格式的语义不对称

SHP 和 MAP 都不是道路设计线形文件：

- SHP 的字段名和图层组织随图商变化，可能提供车道中心线，也可能只提供左右边界；部分宽度、曲率字段存在零值或不可信值。
- MAP 用稀疏点列描述车道，重点是路口拓扑和通信，不保证提供完整出口道路，也不直接提供 OpenDRIVE 的道路参考线、laneOffset、laneSection 或 junction connecting road。
- OpenDRIVE 要求一条物理 road 以一条参考线为基础，车道通过横向偏移和宽度从参考线展开；参考线由 line、arc、spiral、poly3 或 paramPoly3 等解析几何组成。

因此，转换不是 XML/SHP 字段拷贝，而是“从测量要素和拓扑恢复道路模型”的逆问题。

### 2.3 为什么“数学连续”仍可能不满足自动驾驶要求

需要区分四个层次：

1. **G0 连续**：相邻段位置接上。
2. **G1 连续**：位置和切向连续，没有折角。
3. **G2 连续**：曲率连续，不产生横向加速度阶跃。
4. **工程结构简洁**：使用少量有道路意义的参数段，而不是大量米级小段逼近点列。

只满足前三项，仍可能出现以下工程问题：

- 导入器在大量 geometry 边界处重复切换状态或重采样。
- 规划/控制端按段计算道路属性时积累数值误差。
- 极短 clothoid 产生较高的曲率变化率，虽然曲率值连续，横向加加速度仍可能偏大。
- 地图看起来平顺，但参数结构不具备道路设计线形的可解释性，后续编辑和质量审计困难。

OpenDRIVE 规范并未简单禁止短 geometry，因此“无碎段”属于本项目面向自动驾驶消费端增加的工程门禁，不能用 XSD 合法性代替。

## 3. 当前实现与真实数据证据

### 3.0 v1.34 实施结果

v1.34 已把 G11-A～E、少段连接拟合、共享物理边界求解、精确 SHP 外缘对拍和最终车道动力学审计接入正式闭环。最终结构如下：

| 管线/范围 | road 数 | geometry 数 | 单 road 最大段数 | 最短段 | 中位段长 |
|---|---:|---:|---:|---:|---:|
| SHP 普通道路 | 31 | 41 | 3 | 17.265m | 115.397m |
| SHP connecting road | 189 | 602 | 5 | 1.039m | 11.585m |
| SHP auxiliary paving | 14 | 14 | 1 | 35.010m | 49.715m |
| MAP 普通道路 | 28 | 37 | 4 | 17.669m | 89.011m |
| MAP connecting road | 95 | 285 | 3 | 3.715m | 13.383m |
| MAP auxiliary paving | 14 | 14 | 1 | 36.882m | 51.627m |

这里的 1.039m 是某条紧凑实测连接路的合法端部回旋段，不是逐点拟合链：整条连接路只有 3～5 个全局求解的 G2 原语，且最短段满足相对长度、曲率、锐度、来源偏差和扰动稳定性门禁。MAP connecting road 每条最多三段。普通道路不再存在米级 planView 拼接。

真实数据闭环结果：

- 7 SHP + 7 MAP 共 14 文件：G1～G11、XSD 1.5M、esmini 加载/穿越全部 PASS。
- 自动化测试：`pytest -q` 84/84 PASS；包含少段退化、共享边界→width 等价、精确边界投影、动态安全速度与旧缺陷故障注入。
- 连接换乘：进口→连接路→出口两端最大位置缝隙 0.0cm，独立行驶无跳变。
- 路面：所有路口铺面单连通，面积大于 1cm² 的孔洞为 0。
- SHP 外缘：7 路口 source→target / target→source P95 均不超过 0.65m；最坏为 node3 的 0.646/0.561m。
- 最终 XODR 不含 `lane.border`；内部共享边界精确转为等价 `lane.width`，在 esmini 3.6 中不再塌缩。
- G11-D 在源限速与源车道生灭几何不自洽时写出显式 `v_supported`，不移动来源边缘；原限速、支持速度和裁决依据均在 lane provenance 中。
- 14 张四机位拼图与 7 张 SHP/XODR 外缘叠图位于 `out/preview/sweep-final/`，已逐张复核。node17 曲边、NODE5 窄幅、node3 中央白带均由源边界/中央分隔带证据支持，不是算法蛇形或路面孔洞。

正式机器证据：`out/closed-loop-report.json`、`out/g11-opendrive-assessment.json`、每份 XODR 旁的 `.g8.json`、`.g11.json` 与 `.source-lanes.json`。

### 3.1 v1.31 已证明的能力（历史基线）

v1.31 当时固定 7 个金凤路口，SHP 和 MAP 两条管线共 14 份基线输出：

- XSD 1.5M：14/14 PASS。
- G1～G10：14/14 PASS。
- esmini RoadManager：14/14 可加载并穿越 junction。
- 行车换乘位置接缝：最大约 0.22mm。
- 参考线曲率接缝最大跳变：0。
- 车道边缘位置接缝：最大约 1.54mm。
- SHP node4 最外缘 source→target 中位 0.208m、P95 0.418m；target→source 中位 0.206m、P95 0.400m。
- pytest：75/75 PASS。
- 14 份文件×5 个 odrviewer 机位共 70 张截图已人工检查，无明显蛇形边缘、孤立单边 road、路面孔洞或 laneSection 裂缝。

证据位置：

- `out/closed-loop-report.json`
- `out/direct_xodr/*.xodr`
- `out/m2x/*.xodr`
- `out/preview/sweep_edge_v38/`
- `out/preview/sweep_edge_v38/node4_shp_xodr_overlay.{png,json}`

上述结论只代表连续性、表面闭合和现有保真门禁通过，不代表参数段已经最少化。

### 3.2 v1.31 geometry 碎片度审计（历史问题）

对 14 份正式 XODR 的 `<planView><geometry>` 重新统计：

| 管线/范围 | geometry 数 | 最短段 | 中位段长 | `<5m` 占比 | 单 road 最大 geometry 数 |
|---|---:|---:|---:|---:|---:|
| SHP 全部 | 1607 | 1.20m | 2.78m | 70.2% | 17 |
| SHP 普通道路 | 166 | 3.00m | 12.54m | 13.9% | 视路口而定 |
| SHP 路口连接/附属道路 | 1441 | 1.20m | 2.72m | 76.7% | 17 |
| MAP 全部 | 399 | 3.73m | 13.84m | 3.8% | 14 |
| MAP 普通道路 | 100 | 4.32m | 33.43m | 3.0% | 14 |
| MAP 路口连接/附属道路 | 299 | 3.73m | 13.42m | 4.0% | 主要为固定 G2 解 |

说明：统计中的 junction 范围还包含少量辅助铺面 road，但不会改变 SHP 连接路存在大量 1～5m spiral 的结论。

### 3.3 旧门禁为何会通过

当前 `scripts/closed_loop.py` 的 G7 阈值为：

```python
"leg":  {"med": 3.0, "min": 3.0, ...}
"conn": {"med": 2.0, "min": 1.0, ...}
```

它能拒绝普通道路上的亚米级碎段，却明确允许路口连接路最短 1m、中位 2m。G10 检查的是最终世界坐标边缘是否蛇形、是否存在切向跳变，同样不会限制 geometry 数量。因此，v1.31 的 14/14 PASS 与“禁止细小段拼接”并不矛盾，而是验收口径尚未覆盖这一要求。

## 4. 问题定义与边界

### 4.1 本轮目标

1. 在给定源数据误差容限内最小化 geometry 数量。
2. 普通道路恢复具有工程含义的直线、圆曲线、缓和曲线组合。
3. 路口连接路采用固定少量解析原语，不逐输入点插值。
4. 参考线、车道中心、最终道路边缘和 laneSection 横断面均连续。
5. 按目标速度验证曲率、曲率变化率、横向加速度与横向加加速度。
6. SHP 和 MAP 两条管线使用同一个参考线拟合内核和同一套反碎片化门禁。
7. 每类来源携带置信度和误差模型，区分 `MEASURED/FITTED/APPROXIMATED/INFERRED`。
8. 对 reference line、laneOffset、width 和 laneSection 做联合复杂度控制，禁止复杂度转移。
9. 按自动驾驶、仿真展示及具体消费者 Profile 约束 primitive 白名单和辅助道路策略。

### 4.2 非目标

- 不修改原始 SHP、MAP XML 或标准资料。
- 不通过删除真实急弯、移动停止线或放宽保真误差“修漂亮”。
- 不把 GUI 人工修图作为 G0 主链路的必需环节。
- 不承诺缺少真实出口的 MAP 镜像道路等同于现场真值；该部分继续标记 `INFERRED`。
- 不因技术门禁通过而绕过 CRS 绝对坐标未核验、phaseId 缺失等生产交付红线。
- 不把 `INFERRED` 当作结构、连续性、动力学或消费端兼容性的豁免理由。
- 不用增加 width/laneOffset/laneSection 分段的方式隐藏 planView 碎片化。

## 5. 总体重构架构

```text
源图层/点列 + 拓扑
  │
  ├─ 字段/Profile 映射
  ├─ 来源状态、置信度、误差模型
  └─ MEASURED / APPROXIMATED / INFERRED
  │
物理道路或方向性 carriageway 分组
  │
reference line 候选比较
  ├─ 单一双向 reference line
  └─ 两个方向性 carriageway
  │
路口口部车道级边界状态
  (x, y, heading, curvature, confidence)
  │
整路低段数候选图
  ├─ line / arc / spiral
  ├─ spiral-arc-spiral
  ├─ 通用三 clothoid G2
  └─ 至多五段模板
  │
长道路 Top-K 最少分段路径
  │
junction 受限局部联合优化
  │
硬约束可行性过滤 + 字典序模型选择
  │
canonicalization
  ├─ 合并相邻等价 line/arc/spiral
  └─ 删除零长、退化和属性边界导致的假切段
  │
reference line + laneOffset + laneWidth + laneSection
  │
车道中心与最终道路边缘复核
  │
G1～G10 + G11-A～G11-E
  │
消费端 OpenDRIVE Profile 写出
```

### 5.1 消费端 Profile 分层

同一份内部道路模型不得强迫所有消费者接受同一组辅助表达。首版至少定义：

| Profile | 目标 | `paramPoly3` | auxiliary paving road | primitive 策略 |
|---|---|---|---|---|
| `odr15-ad-strict` | 自动驾驶/规划 | 默认关闭 | 默认省略 | line/arc/spiral 白名单；必须 G11 全通过 |
| `odr15-sim` | esmini/仿真展示 | 经实测白名单开启 | 可保留，但 restricted、无拓扑、不可路由 | 允许显示辅助结构，路由仍只走 connecting road |
| `odr15-consumer-*` | 指定商业/内部消费者 | 按兼容矩阵 | 按加载/路由测试结果 | 以实际消费端验证结果锁定 |

此前为解决 odrviewer 路口视觉孔洞加入的辅助 paving road，只能证明仿真展示 Profile 的路面闭合；在自动驾驶 Profile 中默认不输出，除非目标消费者已证明会忽略其路由语义且不会污染道路图。

因此 G9/G10 也要按 Profile 解释：`odr15-sim` 继续要求渲染路面并集无孔洞；`odr15-ad-strict` 重点要求所有可路由车道走廊、connecting road、laneLink 和车道边缘连续，不把“省略不可路由铺面后查看器显示路口空白”误判为驾驶拓扑断裂。若目标自动驾驶消费者同时要求视觉铺面无洞，只能在对应 `odr15-consumer-*` Profile 经实际加载和路由验证后启用辅助铺面。

内部 MapIR/线形候选保持版本无关；1.5M、1.6、1.7 的差异只在 writer Profile 映射。默认正式输出仍为 1.5M，版本升级不能替代最少段重构。

## 6. SHP→OpenDRIVE 方案

### 6.1 输入 Profile

继续支持两条等价输入路线，由用户在 YAML Profile 中映射图层和字段：

- 路线 A：`LANE_LINK` 等价物，至少提供车道线和宽度；宽度缺失时可从相邻中心线或边界推导并标记 `APPROXIMATED`。
- 路线 B：`LANE_BOUNDARY` 等价物，至少提供左右边界及其与车道/道路的关联；由左右边界合成车道中心和宽度。

字段名不写死在算法中，Profile 负责字段语义、单位、编码和关系映射。

### 6.2 物理道路重建

先根据拓扑、行驶方向、平行性、距离和道路口部聚类，将多条车道归并为物理道路腿。共享 reference line 是默认候选，但不是绝对规则。每个道路腿至少比较：

1. 单一双向 reference line：优先真实设计控制线；缺失时比较分隔带中心、道路几何中轴和鲁棒中位轴。
2. 两个方向性 carriageway：每个方向独立 reference line，并用 road link/junction 关系表达物理关联。

模型选择除源边界误差外，还应按字典序比较：

- laneOffset 总变化量和高阶变化。
- width 分段数和 laneSection 数量。
- 实际车道中心/边缘曲率与动力学。
- 数值条件和跨段稳定性。

以下情况允许并倾向拆为两个 carriageway：中央隔离带宽且变化明显、两方向不平行、独立曲率/高程、强行共轴导致复杂 laneOffset，或一侧车道变化污染另一侧横断面。

无论采用一个还是两个 reference line，同一 carriageway 内的车道都必须通过 `laneOffset`、`width` 和 lane ID 展开；禁止为每条车道各建一条重叠单边 road。

### 6.3 实测路口连接线

一条连接车道的全部实测点只作为距离约束，不再成为 clothoid spline 的逐点节点。按以下层级产生候选：

1. 单段 line、arc 或 spiral，若端点位姿和误差允许则直接采用。
2. 三段层同时比较有道路设计含义的 `spiral-arc-spiral` 和通用三 clothoid G2（Bertolazzi–Frego `SolveG2`）。
3. 至多五段的工程线形模板，包括必要的短直线、圆弧和非对称缓和段。
4. `paramPoly3` 默认关闭；只能由消费端 Profile 白名单开启，且最多形成一个连续 fallback 块，不允许多个 `paramPoly3` 再次组成碎片链。

所有候选均整条评估，先过滤不可行解，再进行字典序选择。三 clothoid 必须额外检查：每段正长度、无回头/环路/自交、无近零退化段、曲率符号符合真实转向、车道边界不交叉、参数扰动不导致解剧烈变化。同为三段时优先曲率单调、无多余反弯、曲率变化率小、语义可解释且数值稳定的候选。

不得恢复“两端各三段桥 + 中段多段点列拟合”的现有结构；五段内不存在可行解时必须失败或进入显式例外评审，不能继续自动增加短段。

## 7. MAP XML→OpenDRIVE 方案

### 7.1 进口道路

同一 Link 下多条 Lane 点列先聚合成一条物理道路。通过多车道中心的鲁棒中位轴或横断面拟合共享参考线，再计算各车道相对 reference line 的边界和宽度。MAP Node 点列只作为保真采样，不逐点插值输出 geometry。

### 7.2 出口道路

- 合帧/邻居节点中存在真实出口：使用真实出口几何和车道数。
- 单节点 MAP 缺出口：对同一进口完整横断面做严格左右镜像，包括边界、宽度、变宽斜率和车道生灭；不从路口其他方向复制。
- 镜像出口继续标记 `INFERRED/mirror-no-source-geometry`，不进入源数据保真统计。

### 7.3 路口连接

- 有效 `connectsTo`：按真实车道映射生成连接路。
- 源拓扑缺录且启用 `--connect-mode`：按直行、左转、右转规则补全，并区分 `INFERRED`。
- 无实测连接几何：在单段、`spiral-arc-spiral` 和通用三 clothoid 中选择最少且最稳定的可行解，原则上不超过三段。
- 有实测连接点：使用与 SHP 相同的整条最少段优化，禁止逐 MAP Node 生成短 geometry。

### 7.4 路口口部状态与联合优化

进口/出口主体先独立拟合，再从**实际映射车道中心或车道组**提取口部状态 `(x, y, heading, curvature, confidence)`；不得简单使用 road reference line 端点代替横向偏移车道端点。

当连接路五段内无可行解时，允许对以下变量做一次受限联合优化：

- connecting road 全部参数。
- 进口和出口末端短窗口内的参考线参数。
- 与上述窗口关联的 laneOffset/width 导数，但不增加无必要分段。

硬边界：停止线位置、真实车道口坐标、车道拓扑和道路主体窗口外参数保持固定；调整量不得超过对应来源置信度和误差模型。联合优化仍无解则明确失败，不允许静默恢复短段链。

## 8. 最少段全局拟合算法

### 8.1 预处理

1. 去除完全重复点和低于数据精度的抖动，不删除具有实际线形意义的折点。
2. 以弧长参数化点列。
3. 计算鲁棒航向序列；避免直接对噪声曲率做高阶差分。
4. 可使用逼近平滑样条抑制测量噪声，但禁止 `s=0` 的严格插值样条作为最终线形。

### 8.2 航向域识别

依据航向随弧长的关系产生候选区间：

- line：航向近似常数。
- arc：航向关于弧长近似一次函数。
- spiral：曲率关于弧长近似一次函数，因此航向近似二次函数。

这比直接从稀疏点列估计曲率低一阶，对测量噪声更稳健。

### 8.3 动态规划与全局优化

禁止使用 `wN·N + 误差项` 的单一加权目标模拟“段数最少”，因为权重变化会改变工程结论。采用两层求解：

第一层建立可行集，所有候选必须同时满足：

- 拓扑和行驶方向正确。
- 源数据横向、端点、停止线和口部误差不超限。
- reference line 及存续车道满足 G0/G1/G2。
- width 为正，车道中心和边界不自交、不交叉。
- 曲率、曲率变化率和 movement-specific 设计速度动力学不超限。
- geometry、laneOffset、width、laneSection 和 primitive 类型符合消费端 Profile。

第二层在可行解中做严格字典序模型选择：

```text
1. planView geometry 数量
2. laneOffset / width / laneSection 的总结构复杂度
3. 短段、退化段和滑动窗口边界密度
4. 最大曲率变化率、曲率变化率跳变和车道轨迹动力学
5. P95、连续超限区间和最大拟合误差
6. 参数条件数、扰动敏感性和序列化稳定性
```

只有前一层级完全相同时才比较下一层级。超过五段仍无可行连接路时输出失败/例外，不得继续增加 geometry。

### 8.4 连续性的构造保证

同一 road 内不把每段 `x/y/hdg` 作为独立优化变量。仅优化初始位姿与各 primitive 参数，后一段起点由前一段解析积分得到；相邻段共享端点曲率。这样 G0/G1/G2 由参数化构造保证，内部接缝应达到序列化精度，而不是使用 road-to-road 的 1cm/0.1°宽松阈值。

road-to-road、junction 接口以及源数据匹配继续使用独立、较宽的接口误差门限，二者不得混用。

### 8.5 canonicalization 后处理

字典序选择后执行不改变几何语义的规范化：

- 合并相邻共线 line。
- 合并曲率相同且位姿连续的 arc。
- 合并曲率变化率相同、端点曲率连续的 spiral。
- 删除零长、近零长和数值退化 primitive。
- 禁止仅因采样点、laneSection 或属性边界切断 planView geometry。

canonicalization 后必须重新执行完整可行性和来源保真检查；不能为了合并而突破误差或动力学约束。

## 9. 拟议 G11：反碎片化与动力学门禁

G11 不再是单一段长检查，而拆为 A～E 五组。所有阈值进入版本化 validation Profile；当前数值是专家建议后的首版工程候选，需通过扩展测试集校准。

### G11-A：参考线结构

检查每条 road：primitive 数量和序列、每百米段数、20/25m 滑动窗口 geometry 边界数、连续短段、短段相对长度、canonicalization 后仍可合并的相邻段、零长/退化段、自交和参数敏感性。

首版裁决：

| 指标 | WARNING | HARD FAIL / 例外 |
|---|---:|---:|
| 普通 road geometry 密度 | >8 段/100m | >10 段/100m 为候选硬失败，待跨区域数据校准 |
| 推断 connecting road | — | 最少段结果原则上 >3 段失败或显式例外 |
| 有实测 connecting road | >3 段 | >5 段失败或显式例外 |
| 单段长度 `<5m` | WARNING | 不单独失败 |
| 任意段长度 | `<10%` road 总长复核 | `<0.5m` 或 `<3%` road 总长视为退化失败 |
| canonicalization 后仍可合并 | — | HARD FAIL |
| 回头、环路、自交、边界交叉 | — | HARD FAIL |

`<5m` 是风险信号而非孤立绝对判据，防止 `4.9+5.1+4.9+5.1m` 绕过检查。滑窗边界密度的最终数值由合成真值和真实黄金集校准；在校准前必须输出指标，不能静默忽略。

### G11-B：横断面结构

同时审计：

- `laneOffset` 记录数、每条 lane 的 width 记录数和 laneSection 密度。
- width/offset 的值、一阶和二阶导数，以及跨记录/section 的连续性。
- laneSection 是否对应真实车道数、类型、方向、访问属性或拓扑事件。
- 减少一个 planView geometry 是否无来源依据地增加多个 offset/width/section 记录。
- 普通道路共轴与方向性 carriageway 两种方案的横断面总复杂度。

字典序选择中的第二层直接使用横断面复杂度；若减少 planView 段数只靠增加无来源事件的横断面分段实现，则该候选被支配或判为复杂度转移失败。

### G11-C：数值连续性

- 同一 road 内 geometry 起点由前段解析积分得到，内部 G0/G1 接缝应达到序列化精度；不得使用 1cm/0.1°作为内部接缝合格线。
- 相邻段共享端点曲率，内部 `|Δκ|` 以浮点/序列化精度审计。
- road-to-road/junction 接口才使用位置 ≤1cm、航向 ≤0.1° 的首版候选门限。
- laneSection 边界处存续车道中心和边缘必须单独验证 G1/G2。

### G11-D：实际车道与道路边缘动力学

检查对象不只 reference line，还包括：

- 每条可行驶车道中心线。
- 最内侧和最外侧道路边缘。
- laneOffset/width 变化附加的世界坐标曲率。
- laneSection 边界处存续车道。

按 movement-specific 目标速度计算平面线形代理：

```text
ay = v²·κ
jy = v³·dκ/ds
v_ay = sqrt(ay_limit / max|κ|)
v_jerk = cbrt(jy_limit / max|dκ/ds|)
v_supported = min(v_ay, v_jerk)
```

比较 `movement_target_speed <= v_supported`。路口左/右转不能直接继承主路 40/60km/h 限速，必须由 movement Profile 给目标速度。

首版工程候选：

| 等级 | `ay` | `jy` |
|---|---:|---:|
| 舒适性 WARNING | 1.5m/s² | 0.5m/s³ |
| HARD FAIL | 2.5m/s² | 1.0m/s³ |

这是定速条件下的平面线形代理，不是包含纵向加减速、轮胎和车身模型的完整车辆动力学仿真。数值进入业务/车辆 Profile，不作为法规结论。

### G11-E：来源、例外和消费端兼容性

每条 road/primitive/横断面输出至少一种来源状态：

```text
MEASURED
FITTED
APPROXIMATED
INFERRED
PARAMPOLY3_FALLBACK
SHORT_SEGMENT_EXCEPTION
```

`INFERRED` 可以豁免源数据拟合误差，但不能豁免 G11-A～D 和消费端兼容性。`PARAMPOLY3_FALLBACK` 与 `SHORT_SEGMENT_EXCEPTION` 必须记录触发原因、候选失败信息、拟合误差、动力学、适用 Profile 和实际消费者加载结果。

### 当前数据集的保真门限

SHP 外边界双向 median ≤0.30m、P95 ≤0.65m，MAP 车道中心 P95 ≤0.50m，仅作为当前金凤黄金集的候选发布门限，不得宣称为通用标准。新增：口部、停止线、道路端点、最大误差和连续超限区间的单独指标；镜像/推断区域不参与源真值误差统计，但仍执行结构与动力学门禁。

## 10. 验证与交付证据

测试数据分三层，不能只用金凤 14 份结果校准通用阈值：

1. **合成真值集**：已知 line、arc、spiral、直缓圆、S 弯和短连接路；加入不同采样密度和噪声，验证 primitive 类型与数量恢复正确。
2. **真实黄金集**：冻结当前 7 个金凤路口×SHP/MAP 两管线共 14 份基线，并逐步加入其他城市、图商、采样密度和道路类型。
3. **压力集**：重复点、近重复点、稀疏/密集噪声、缺失出口、宽度突变、车道生灭、非对称道路、宽隔离带、异常 CRS 和错误拓扑。

每次正式验证固定执行：

1. XSD、planView、laneSection、junction/laneLink、路面连通性 G1～G10 全量回归。
2. G11-A～E 输出每条 road 的 primitive、横断面复杂度、滑窗密度、相对短段、来源状态、动力学和例外。
3. SHP 边界和 MAP 车道中心双向身份对拍，不使用全图最近线掩盖错配。
4. esmini RoadManager 加载和路线穿越测试。
5. odrviewer 固定远景、俯视、斜视和低机位截图；SHP 额外输出源边界/XODR 边缘叠图。
6. 对旧 v1.31 做故障对照，证明 G11 会稳定拒绝现有 SHP 短段链，同时不误杀合格 MAP 样本。
7. 按消费端矩阵分别验证三 clothoid、`paramPoly3`、辅助 paving road 和目标 OpenDRIVE 版本；XSD/查看器通过不能代替实际消费者测试。

最终交付报告必须同时回答：

- 是否符合源地图形状。
- 是否 G2 连续。
- 是否存在道路面孔洞或边缘蛇形。
- 是否依赖短段链。
- 哪些几何来自真实数据，哪些为镜像或拓扑补全。
- reference line 复杂度是否转移到 laneOffset/width/laneSection。
- movement 目标速度是否低于车道轨迹 `v_supported`。
- 当前文件适用于哪个消费端 Profile，哪些 primitive/辅助表达已实际验证。

## 11. 实施分解

### 11.0 v0.3 完成度矩阵

| 工作包 | v0.3 状态 | 证据/剩余边界 |
|---|---|---|
| M0 冻结 v1.31 基线 | 完成 | `out/v131-opendrive-baseline.json`；旧输出仅作差异基线 |
| M1 G11-A～E | 完成（金凤候选 policy） | `validate/g11.py`、`profiles/validation/g11-opendrive-v1.draft.yaml`、14 个 sidecar；跨区域校准未完成 |
| M2 少段拟合内核 | 核心完成 | 普通道路每 road≤4 段，SHP 实测连接≤5、MAP 连接≤3；仍可继续扩充合成真值与病态扰动集 |
| M3 SHP 主线替换 | 完成（金凤 Profile） | 共享 reference line/方向性 carriageway、共享边界、精确 width 物化、边界对拍；第二图商待验 |
| M4 MAP 主线统一 | 完成（金凤 MAP XML） | 真实出口优先、缺出口同一断面左右镜像、三段 G2 连接；更多 MAP Profile 待验 |
| M5 消费端与闭环 | 金凤闭环完成，Profile 分层未完 | 14/14 G1～G11+esmini；`odr15-ad-strict`/`odr15-sim` 显式 CLI Profile 和跨消费者矩阵仍待实施 |

因此，v0.3 的准确表述是：**两条 G0 主线已在金凤真实数据上完成少段平滑技术闭环；通用产品化仍有条件通过。** 不应表述为所有图商、所有消费者和所有道路形态均已“完美转换”。

### M0：冻结 v1.31 基线

- 固化 14 份正式 XODR、G1～G10 报告、边界误差、esmini 路线、70 张截图和每条 road primitive/横断面清单。
- 记录输入哈希、生成配置和当前代码版本，保证重构前后可重复比较。
- v1.31 输出只作为回归证据，不再称为无碎段版本。

### M1：G11-A～G11-E

- 实现 reference line、横断面、数值连续性、实际车道动力学和来源/兼容性五组审计。
- 让当前 SHP 短段链稳定失败，同时证明合格 MAP 样本不会被孤立 `<5m` 指标误杀。
- 建立 planView 简化但 width/offset 碎片化、4.9/5.1m 交替、可合并 primitive、退化三 clothoid 等故障注入测试。

### M2：合成真值拟合内核

- 航向域 line/arc/spiral 候选检测。
- Top-K 动态规划最少分段、硬约束可行性过滤和字典序模型选择。
- 构造式 G0/G1/G2、全局位姿/曲率/段长优化和 canonicalization。
- 单段、`spiral-arc-spiral`、通用三 clothoid 与至多五段模板候选。
- 必须证明直线→1 line、圆弧→1 arc、标准缓和曲线→1 spiral、直缓圆不碎分，并验证噪声鲁棒性。

### M3：SHP 主线替换

- 比较共享 reference line 与两个方向性 carriageway，按横断面复杂度和边界保真选择。
- 替换 `_bridged_geoms` 短段链。
- 加入路口口部状态和受限局部联合优化。
- 保持既有 laneOffset、width、laneSection 和边界保真能力，不发生复杂度转移。

### M4：MAP 主线统一

- 进口多车道共轴拟合。
- 实际出口/严格镜像出口保持现有裁决。
- MAP 连接路切换到同一最少段内核。
- 为真实、镜像、连接补全分别配置误差模型和 movement 目标速度。

### M5：消费端矩阵与真实数据闭环

- 14/14 G1～G11。
- 75 项既有测试不得回退，并增加碎片化与动力学测试。
- esmini、截图、叠图和结构报告完整归档。
- 分别验证 `odr15-ad-strict` 与 `odr15-sim`；自动驾驶 Profile 默认不含 paving road 和 `paramPoly3`。
- 对三 clothoid、`paramPoly3`、辅助 paving road 和新 OpenDRIVE 版本逐消费者留测试证据。
- 在跨城市/跨图商样本不足前，G11 通用阈值保持版本化候选状态，不宣称行业通用标准。

## 12. 风险与控制措施

1. **三 clothoid 退化**：固定少段不等于天然稳定；用正长度、相对长度、曲率符号、自交、边界交叉、条件数和扰动测试过滤。
2. **保真与可解释性的冲突**：通过来源置信度/误差模型和字典序选择处理，不以统一权重隐藏业务裁决。
3. **横断面复杂度转移**：G11-B 与第二层字典序直接纳入 laneOffset/width/laneSection，禁止 planView 简化后横断面爆炸。
4. **paramPoly3 兼容性**：默认关闭，仅 Profile 白名单开启一个连续 fallback 块，并要求真实消费者测试。
5. **非设计线形道路**：五段内无解则进入例外/失败；不自动恢复米级短段链。
6. **辅助铺面 road 污染路由**：自动驾驶 Profile 默认省略；仿真 Profile 中 restricted、无拓扑且不可路由。
7. **动力学代理局限**：G11-D 只声明定速平面线形代理，不能冒充完整车辆动力学仿真。
8. **阈值过拟合金凤样本**：用合成真值、跨区域黄金集和压力集校准；14 份样本只做固定回归。
9. **源数据缺失**：MAP 镜像出口和默认转向保持 `INFERRED`，不能变成现场真值。

## 13. 专家裁决及本版落实

| 原评审问题 | 专家裁决 | v0.2 落实 |
|---|---|---|
| 普通 road 8段/100m、中位12m | 先作 WARNING；>10段/100m为候选硬失败，需扩充数据校准 | 纳入 G11-A，增加滑窗、相对短段和可合并性 |
| 推断≤3段、实测≤5段 | 接受“最少段结果原则上不超过3/5”，不能强行凑模板 | 字典序可行解选择；超限失败/显式例外 |
| 短连接绝对/相对段长 | 相对长度和动力学优先；`<0.5m`、`<3%`退化失败，`<10%`复核 | 纳入 G11-A |
| `paramPoly3` | 默认关闭，消费端白名单开启；只能一个连续 fallback 块 | 纳入 Profile 与 G11-E |
| SHP/MAP 保真门限 | 仅当前数据集候选；增加口部、停止线、端点、最大和连续超限 | 纳入 G11 保真分层 |
| 动力学门限 | WARNING 1.5/0.5，FAIL 2.5/1.0；按 movement 速度换算 | 纳入 G11-D，并声明只是平面代理 |
| 非对称 reference line | 设计控制线优先；必要时拆两个 carriageway | 纳入 6.2 候选比较 |
| auxiliary paving road | 自动驾驶 Profile 默认不接受；仿真可保留且隔离路由 | 纳入 5.1 Profile 分层 |
| OpenDRIVE 版本 | 内部模型版本无关；保留 1.5M 默认，其他版本另建 Profile | 不以版本升级替代少段重构 |

实施期间若新专家意见改变上述门限，只能通过版本化 validation/consumer Profile 修改；不得在算法中散落硬编码。

## 14. 参考依据

### OpenDRIVE 规范

- [ASAM OpenDRIVE 参考线：几何之间不得有 gap，推荐避免 kink](https://publications.pages.asam.net/standards/ASAM_OpenDRIVE/ASAM_OpenDRIVE_Specification/v1.9.0/specification/09_geometries/09_02_road_reference_line.html)
- [ASAM OpenDRIVE lane geometry：车道宽度和边界相对参考线定义](https://publications.pages.asam.net/standards/ASAM_OpenDRIVE/ASAM_OpenDRIVE_Specification/1.8.0/specification/11_lanes/11_06_lane_geometry.html)
- [ASAM OpenDRIVE Annex D：车道平滑度](https://publications.pages.asam.net/standards/ASAM_OpenDRIVE/ASAM_OpenDRIVE_Specification/v1.8.1/specification/16_annexes/smoothness_of_lanes/top_ter_smoothness_of_lanes.html)
- [ASAM OpenDRIVE connecting roads](https://publications.pages.asam.net/standards/ASAM_OpenDRIVE/ASAM_OpenDRIVE_Specification/v1.8.1/specification/12_junctions/12_04_connecting_roads.html)

### 曲线拟合与最少段线形

- Bertolazzi, E.; Frego, M. “G1 fitting with clothoids”, DOI: [10.1002/mma.3114](https://doi.org/10.1002/mma.3114)。
- Bertolazzi, E.; Frego, M. “On the G2 Hermite interpolation problem with clothoids”, DOI: [10.1016/j.cam.2018.03.029](https://doi.org/10.1016/j.cam.2018.03.029)。
- Bertolazzi, E.; Frego, M. “Interpolating clothoid splines with curvature continuity”, DOI: [10.1002/mma.4700](https://doi.org/10.1002/mma.4700)。
- Maier, G. “Optimal arc spline approximation”, DOI: [10.1016/j.cagd.2014.02.011](https://doi.org/10.1016/j.cagd.2014.02.011)。
- Camacho-Torregrosa, F. J. et al. “Use of Heading Direction for Recreating the Horizontal Alignment of an Existing Road”, DOI: [10.1111/mice.12094](https://doi.org/10.1111/mice.12094)。
- Reinsch, C. H. “Smoothing by Spline Functions”, DOI: [10.1007/BF02162161](https://doi.org/10.1007/BF02162161)。
- Pai, C.-K. et al. “Automatic generation of OpenDRIVE HD maps from mobile mapping data”, DOI: [10.5194/isprs-archives-XLIII-B1-2022-263-2022](https://doi.org/10.5194/isprs-archives-XLIII-B1-2022-263-2022)。

### 开源实现候选

- [ebertolazzi/Clothoids](https://github.com/ebertolazzi/Clothoids)：BSD-2-Clause，G1/G2 clothoid、biarc、clothoid list。
- [pyclothoids](https://github.com/phillipd94/pyclothoids)：MIT，Clothoids 的 Python 封装。
- [scenariogeneration](https://github.com/pyoscx/scenariogeneration)：MPL-2.0，其 junction creator 使用 `SolveG2` 生成固定三段回旋线。

更完整的文献核验记录见项目内 `docs/参考文献-参考线拟合与OpenDRIVE生成.md`。
