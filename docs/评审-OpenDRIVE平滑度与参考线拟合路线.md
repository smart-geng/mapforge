# 评审：转换能否得到平滑的 OpenDRIVE 地图（不基于短段拼接）

> 评审日期：2026-08-13。评审对象：mapforge 的 →OpenDRIVE 生成路线（MAP→xodr、SHP→xodr）。
> 标杆样本：`F:\资料\SUMO\MapFormat-main\OpenDRIVE_data\OpenDRIVE数据\Town\Town03.xodr`（CARLA 官方 Town03，2.3MB）。
> 结论进入方案 v1.2 第 5.7 节。

## 一、结论

**能。** 前提是走"**一条 Link/车道链 = 一条 road，planView 内多段参数几何（line/arc/spiral 或 paramPoly3），段间 G1（尽量 G2）连续由拟合器保证**"的路线——这与 Town03 的构成方式同构。被否决的是 xml2xodr 的"每两点一条独立 road 的直线拼接"，它不平滑的根因不是"段短"，而是**在错误的层级断开**（road 级 G0 断裂 + 折线航向跳变 + 无拓扑）。

平滑度可达等级（按源分级）：

| 源 | 可达连续性 | 依据 |
|---|---|---|
| IBD SHP | **G2 级**（曲率连续），质量可超 Town03（Town03 仅 line+arc，为 G1 级） | 点距约 2–3 m，且 LANE_POSITION/POSITION 逐形点自带 HEADING/CURVATURE/SLOPE/BANKING——拟合有强先验，可直接按曲率分段构造 arc/clothoid，elevation/superelevation 也有数据源 |
| MAP 消息点列 | **G1 级**（位置+航向连续，视觉平滑、esmini/CARLA 可正常加载行驶），曲线形状为受控近似 | 点列已经附录 D 抽稀（几米到几十米一点）+ 1e-7° 量化噪声（≈1.1 cm）——拟合欠定，还原的是"合理的平滑曲线"而非原始设计曲线，全程记 APPROXIMATED + 容差 |

## 二、标杆解剖：Town03 实测数据

用脚本统计其全部 planView（OpenDRIVE 1.4，RoadRunner 产线出品）：

| 指标 | 数值 | 启示 |
|---|---|---|
| roads / junctions | 279（其中 185 条为 junction 内 connecting road）/ 34 | connecting road 占三分之二——交叉口是 road 数量的主体 |
| 几何段构成 | 共 1076 段：**line 66.7% + arc 33.3%**，无 spiral/poly3/paramPoly3 | 平滑地图不需要花哨几何：line+arc+正确的连续性就够 |
| line 段长度 | p10/p50/p90 = 0.03/**0.21**/20.5 m；445 段 <1 m | **官方地图自己就充满超短段**——短段无罪，它们在同一 road 内部且 G1 连续 |
| arc 段长度 | p50 = 7.9 m | 曲线由 arc 承担（曲率恒定，line-arc 相接处曲率跳变→仅 G1，但仿真消费端接受） |
| 车道宽度 | 1913 条 width 记录中 99.6% 为常数 | 定宽车道是常态，变宽多项式仅用于渐变段（7 处） |
| 高程 | 1227 条 elevation（343 条非平坦）；无 superelevation | 高程是三次多项式 profile，不是逐点 z |
| 拓扑 | 279/279 全部有 pred/succ；junction 185 connection / 229 laneLink | 平滑=几何连续性 × 拓扑完整性，缺一不可 |

**"平滑"的技术定义**（评审采用）：① 段间 G0/G1 连续是 OpenDRIVE planView 的规范要求（位置+航向），G2（曲率）是舒适性加分项；② road 之间靠 pred/succ + junction 连接，contactPoint 正确；③ 消费端验收 = esmini/CARLA 加载后行驶无折角顿挫、ASAM QC 无 planview 连续性告警。

## 三、对照：xml2xodr 为什么不平滑

| 维度 | xml2xodr 现状 | Town03 / mapforge 路线 |
|---|---|---|
| 分段层级 | **每两个折点一条独立 road**（road_id 拼号），G0 级拼接，接缝需缩段 20–30% + AdjustablePlanview 补丁 | 一条 Link/车道链一条 road，road 内多几何段 |
| 几何类型 | 全部 line，折点处航向跳变（G1 断裂） | line+arc(+spiral/paramPoly3)，段间 G1/G2 由构造保证 |
| 拓扑 | 无 junction、无 laneLink，路口全是断头 road | junction/connection/laneLink 完整生成（connectsTo 直译） |
| 高程 | 解析后丢弃（elevationProfile 恒空） | elevation/superelevation profile 拟合写出 |

## 四、mapforge 参考线拟合路线（写入方案 5.7）

1. **分段策略**：以曲率特征分段（直线段/圆弧段/过渡段），而非固定点数分段。IBD 源直接用逐点 CURVATURE 做分段与初值；MAP 源用折线转角序列估计。
2. **几何选型（两档 Profile）**：
   - 默认档 `paramPoly3-spline`：分段三次参数样条，G2 连续易保证，实现快，scenariogeneration 原生支持——M1 落地；
   - 精修档 `line-arc-spiral`（RoadRunner 风格，兼容性最保守）：G1 clothoid 插值（Bertolazzi–Frego 算法，Python 有现成封装 pyclothoids，入库前核许可证）——M2 提供。
3. **端点约束**：Link 接缝处强制位置+航向连续（G1），与相邻 road 的 contactPoint 对齐；junction 内 connecting lane 由"进口端点位姿 → 出口端点位姿"构造 clothoid/arc 对（掉头等点稀场景的合成生成路径）。
4. **拟合误差控制**：最大偏差 ≤ 可配容差（默认 0.05 m，对 MAP 源放宽至编码分辨率量级），逐 road 记录 fitting_error 进质量报告；超容差降级为加密分段而不是放弃平滑。
5. **门禁**：新增 planview 连续性检查器（相邻段端点位置差 <1 mm、航向差 <0.001 rad、精修档加曲率差）+ esmini 加载行驶烟雾测试 + ASAM QC；MAP→xodr 方向以 Town03 的构成指标（line/arc 占比、段长分布形态）作对照参考而非硬指标。
6. **车道宽度**：默认常数宽（对齐 Town03 常态），IBD 的 S_WIDTH/E_WIDTH 差值超阈值时生成线性变宽多项式——不做逐点变宽。

## 五、线形保真约束：曲线必须符合实际（v1.2 补充）

平滑（连续性）与保真（贴合实际线形）是**两个独立目标且存在张力**：过度平滑会抹掉真实曲率变化（把实际弯道拉直、转弯半径失真），过度贴点会把噪声当形状（直路拟出"蛇形"假弯）。"符合实际的平滑"由以下约束保证：

1. **还原设计线形，而非套任意光滑曲线**。实际道路平面线形本身按"直线—缓和曲线（回旋线）—圆曲线"三元素设计（公路/城市道路路线设计规范体系，JTG D20 / CJJ 193），竖曲线为抛物线——这正是 OpenDRIVE line/spiral/arc + 三次 elevation 的由来。因此分段检测在**曲率域**进行：κ≈0 → line、κ≈常数 → arc、κ 线性变化 → spiral；拟合的本质是**逆向还原道路的设计元素**，line-arc-spiral 档是"语义正确"的表达而不是美学选择。
2. **横向偏差带（贴合上限）**：拟合曲线到源点列的最大横向偏差 ≤ 容差（IBD 默认 0.05 m；MAP 源放宽到编码分辨率量级）——曲线不允许为了平滑漂离实际道路位置。
3. **曲率保真（IBD 源的独有验收条件）**：IBD 逐形点带实测 CURVATURE——拟合后曲线的曲率函数 κ(s) 与源实测曲率序列直接比对，偏差统计入质量报告；直线段拟合后 κ 必须为 0（不产生假弯），圆曲线段 κ 恒定、与实测半径偏差 ≤ 阈值。这是"符合实际"的可量化验收，市面工具没有这种验收条件。
4. **防过拟合（贴合下限=噪声治理）**：源噪声建模——MAP 坐标 1e-7° 量化 ≈1.1 cm，SHP 测量噪声 cm 级。**必须用逼近样条（带平滑权重），禁止插值样条**：插值样条强行穿过每个噪声点，会把 cm 级噪声放大成曲率振荡（视觉上是"贴点"，动力学上是假弯连发）。曲率幅值低于噪声可解释水平（由点距与噪声量推出的 κ 检测下限）一律归零拟直线——**不臆造数据支撑不了的弯道**。
5. **验收可视化**：每条 road 输出曲率图（拟合 κ(s) 曲线 vs 源离散曲率点）+ 横向偏差分布图进质量报告——人工复核看曲率图一眼即可判断"是否符合实际"；配合 esmini 行驶轨迹复核。

## 六、实施条件盘点与启动实验（v1.3 补充）

**结论：四要素齐备，无外部阻塞，可立即开工。**

| 要素 | 状态 |
|---|---|
| 算法 | ✅ 分段识别（Camacho-Torregrosa 航向图法）+ 逐段求解（Bertolazzi–Frego G1/G2）+ 预平滑（GCV 平滑样条）+ 段数最少化框架（Maier）——文献齐、路线映射已画（《参考文献》六） |
| 库 | ✅ pyclothoids(MIT)/Clothoids(BSD-2) 许可证已核；scenariogeneration(MPL-2.0) junction creator 现成；scipy/numpy 常规 |
| 数据 | ✅ 三类测试数据在手：Town03（**参数化真值**，可自采样回拍）、金凤 7 路口 MAP XML（真实目标输入）、IBD SHP（逐点实测曲率=验收基准） |
| 验收 | ✅ 判据已定义：κ(s) 曲率保真 + 横向偏差带 + planview 连续性指标 + esmini 加载 + XSD（1.4H/1.5M 在仓库） |

剩余为纯内部工程事项（不阻塞开工，属开工内容本身）：① 建 Python 3.11 venv 并安装依赖（pyclothoids 为 C++ 扩展，Windows 下 pip 安装的 wheel 可用性需实测，不行则本机编译或 WSL）；② mapforge 仓库骨架尚未创建；③ MAP XML reader 未写（本就是 M0 作业 ②）。

**启动实验（spike，各半天至一天，按序做）：**

1. **Spike-A · Town03 自采样回拍（拟合器核心闭环，有真值）**：取 Town03 若干条含 arc 的 road → 按 1–5 m 间距采样成折线（模拟折线源）→ 走"预平滑→航向图分段→逐段求解"流水线拟合回 line/arc(/spiral) → 与原始参数几何对比：分段边界命中率、arc 半径还原误差、κ(s) 对比图、横向偏差分布。**这是唯一有参数化真值的验证，"符合实际"能力在此定量证明**；再叠加人工噪声（1 cm/5 cm）测防过拟合。
2. **Spike-B · 金凤 MAP XML 单路口端到端**：node16 一条 Link 的车道点列 → 拟合 → scenariogeneration 写 xodr（road 级，junction 用 CommonJunctionCreator 试连一对进出口）→ esmini/odrviewer 加载目测 + 曲率图人工复核 → 与 xml2xodr 同路口输出对照（平滑度对比即替代价值证明）。
3. **Spike-C · IBD 曲率验收通道**：取一条 IBD 车道（LANE_LINK 几何 + LANE_POSITION 逐点 CURVATURE/HEADING）→ 拟合 → "拟合 κ(s) vs 实测曲率"偏差统计——打通"符合实际"的量化验收报表原型。

三个 spike 全绿即证明 5.7 路线成立，拟合器随后按 ops/simplify+fit 模块正式开发进 M1。

## 七、限制与诚实边界

- 折线源（SHP/MAP 皆是）的拟合是**近似重建**：源头采样时真曲线已丢失，拟合给出的是"通过容差管道的最平滑解释"——所有 →xodr 输出的几何状态记 APPROXIMATED（IBD 曲率约束下可记 TRANSFORMED），容差入损失报告；
- MAP 源点稀处（长直路 Link 可能仅 3–5 点）曲线形状不确定性大——按直线+大半径 arc 保守拟合，不臆造弯道；
- 高程：MAP 源基本无真实高程（金凤实测为占位值）→ MAP→xodr 默认平面 + 显式声明；IBD 源用 SLOPE/BANKING 拟合 elevation/superelevation。
