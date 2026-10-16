# 路口连接路几何：曲线族、两端 G2 衔接、导引线保真与路段/连接路联合优化

> 调研范围：交叉口连接路（转弯/直行车道）几何的生成方法——两端须与已定道路端点的位置、航向、曲率一致，同时贴近源导引线（via）。事实与适用性判断分开写（"Cited Findings" 为有出处的事实，"Inferences" 为本人推断/适用性判断）。
> 访问限制说明：本次会话中 arxiv.org、researchgate.net、iris.unitn.it、seminariomatematico.polito.it、carla.readthedocs.io、pyclothoids.readthedocs.io 均 DNS 解析失败，相关论文只拿到搜索摘要/书目信息，未读全文，下文已逐条标注。GitHub 源码与 USPTO、MathWorks 页面可访问。

## 问题 1：HD 地图 / 仿真工具（SUMO、RoadRunner、CARLA、Apollo、Lanelet2、CommonRoad）在路口连接上用什么曲线族，两端如何衔接

### Takeaway
已核实源码的开源工具（SUMO netconvert）用的是**二次/三次 Bézier**，控制点沿进/出车道端切线外延放置，只保证两端**位置 + 切向（G1）**，不做曲率匹配；导出 OpenDRIVE 时直接把这条 Bézier 写成一段 `paramPoly3`，失败时退化为直线段。RoadRunner、CARLA、Apollo 制图工具的路口连接算法没有公开文档可查；CommonRoad 只做格式转换，不生成连接路。

### Cited Findings
**SUMO netconvert（源码 `NBNode.cpp`，许可 EPL-2.0 OR GPL-2.0-or-later）**
- `computeInternalLaneShape` 取进口车道形状 `fromShape` 和出口车道形状 `toShape`；有 custom shape 就直接用，否则调用 `computeSmoothShape`，参数为 `extrapolateBeg = 5·fromE->getNumLanes()`、`extrapolateEnd = 5·toEdge->getNumLanes()`；左转/掉头额外加 `AVOID_WIDE_LEFT_TURN` 标志 — [SUMO NBNode.cpp](https://github.com/eclipse-sumo/sumo/blob/main/src/netbuild/NBNode.cpp)
- `computeSmoothShape`：控制点为空时，直接用两端点连直线；否则 `init.bezier(numPoints)` 采样成折线（再只对 z 做 `smoothedZFront()`）— [SUMO NBNode.cpp](https://github.com/eclipse-sumo/sumo/blob/main/src/netbuild/NBNode.cpp)
- `bezierControlPoints` 中：常量 `EXT = 100`，直行阈值 `DEG2RAD(5)`；进口车道末段与出口车道首段各外延 `EXT`，控制点放在外延线上，从而使曲线两端与车道相切 — [SUMO NBNode.cpp](https://github.com/eclipse-sumo/sumo/blob/main/src/netbuild/NBNode.cpp)
  - 掉头：3 个控制点（二次 Bézier），中间点为两端点中点、沿垂向偏移弦长；
  - 夹角 < π/4 时：位移角与转角都 ≤ 5° → 返回空（画直线）；`bendDeg > 22.5` 且 `(bendDeg/45)^2/dist > 0.13` → 判定"过度 S 弯"，`ok=false`、记录 `myDisplacementError`、返回空；正常 S 弯用 4 个控制点（三次 Bézier）；
  - 一般转弯：两条外延线求交点作唯一中间控制点（二次 Bézier）；无交点 → `ok=false`、位移误差记 1.0；`minControlLength = min(1.0, dist/2)`，交点离某端太近则加长该侧，两端都太近则失败；
  - `AVOID_WIDE_LEFT_TURN`：因子 `min(0.6, 16/dist)`，控制点回拉 `min(distBeg·factor/1.2, dist·factor/1.8)`；`AVOID_WIDE_RIGHT_TURN`：转角 < −95° 且一侧距离 > 20 m 时回拉 `min(distBeg/1.4, dist/2)`。
- 该代码没有任何曲率（G2）连续处理；端点只靠控制点在切线外延上保证切向对齐 — [SUMO NBNode.cpp](https://github.com/eclipse-sumo/sumo/blob/main/src/netbuild/NBNode.cpp)
- SUMO 路网文件里车道几何本身是中心线折线（至少两点）— [SUMO Road Networks 文档](https://sumo.dlr.de/docs/Networks/SUMO_Road_Networks.html)
- SUMO 的 OpenDRIVE 导出（`NWWriter_OpenDrive.cpp`）：连接路调用 `NBNode::bezierControlPoints(begShape, endShape, turnaround, 25, 25, ok, nullptr, straightThresh)`，控制点非空则 `writeGeomPP3` 写成**单段 `paramPoly3`**（断言控制多边形为 3 或 4 点，即二次或三次 Bézier，由 Bernstein 形式换算 aU..dV，`pRange="normalized"`）；控制点为空则用 `writeGeomLines` 写折线段；`ok=false` 时发警告，建议用 `junctions.scurve-stretch` 或加大路口半径；`straightThresh` 来自选项 `opendrive-output.straight-threshold` — [SUMO NWWriter_OpenDrive.cpp](https://github.com/eclipse-sumo/sumo/blob/main/src/netwrite/NWWriter_OpenDrive.cpp)
- 同一导出器中连接路车道宽度沿程线性变化，源码注释承认"理想情况下需要三次多项式才更精确" — [SUMO NWWriter_OpenDrive.cpp](https://github.com/eclipse-sumo/sumo/blob/main/src/netwrite/NWWriter_OpenDrive.cpp)

**RoadRunner / MathWorks**
- RoadRunner 的 `Road` 对象有两种平面曲线：基于控制点的 line-arc 曲线（`LineArcRoadCurve`）和分段曲线（`SegmentedRoadCurve`，支持 line、arc、spiral、parametric cubic 段）— [MathWorks RoadRunner Road 参考](https://www.mathworks.com/help/roadrunner/ref/road.html)
- `roadrunner.hdmap.Junction` 只定义为"交叉处车道的分组，用于定义车道几何与连通性"，文档未说明连接曲线如何生成 — [MathWorks roadrunner.hdmap.Junction](https://au.mathworks.com/help/driving/ref/roadrunner.hdmap.junction.html)
- MathWorks Driving Scenario Designer（不是 RoadRunner）导出 OpenDRIVE 时，路口"不带车道连接信息"处理，导出的路口形状可能不准；弯道导出几何会有变化 — [MathWorks：导出 OpenDRIVE](https://www.mathworks.com/help/driving/ug/export-driving-scenario-to-opendrive-file.html)

**Apollo**
- Apollo（Apache-2.0）公开的是参考线平滑器，而不是路口连接生成器：QP-spline 平滑器用分段五次多项式，目标为 ∫(f''')² + ∫(g''')²，结点处位置及 1–3 阶导数连续，在 m 个锚点处约束 |f(t_l) − x_l| < boundary、|g(t_l) − y_l| < boundary，写成标准 QP — [Apollo reference_line_smoother.md (r6.0.0)](https://github.com/ApolloAuto/apollo/blob/r6.0.0/docs/specs/reference_line_smoother.md)

**CommonRoad**
- CommonRoad Scenario Designer 的 OpenDRIVE→CommonRoad 转换流程为：解析 OpenDRIVE → ParametricLane 网络 → lanelet；理论依据是 Althoff、Urban、Koschi 2018 "Automatic Conversion of Road Networks from OpenDRIVE to Lanelets" — [CommonRoad 文档：OpenDRIVE 转换](https://commonroad-scenario-designer.readthedocs.io/en/latest/details/open_drive/)
- `commonroad-scenario-designer` 0.8.5 在 PyPI 上的许可为 GPL-3.0(+) — [PyPI commonroad-scenario-designer](https://pypi.org/project/commonroad-scenario-designer/)

**专利（工业做法的旁证）**
- 某 HD 地图专利用三次 Bézier 生成路口虚拟车道线：直行连接取中点作控制点，转弯取进/出口延长线交点作控制点 — [USPTO 11933627](https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/11933627)
- 某车道网生成专利由车辆轨迹的概率分布在方差小的位置选控制点，再拟合 Bézier — [USPTO 12270678](https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/12270678)

### Inferences
- SUMO 型"切线外延求交 + 二次 Bézier"在端点处曲率一般不为 0（二次 Bézier 端点曲率由前两条控制边的叉积决定），而直线道路端曲率为 0，所以这类做法天然只到 G1；连接路与道路接口的曲率跳变正是我们 T2 要防的。它只适合作为初值或 fallback，不能作为输出。
- SUMO 的做法说明一个现实取舍：一条连接路 = 一段 `paramPoly3`，结构最简单，esmini 等消费者兼容性好；但三次 Bézier 一段的自由度（2D 共 8 个）用于端点 G2（每端位置 2 + 切向 1 + 曲率 1，共 8 个条件）后不剩自由度贴导引线，所以要贴导引线就得分段或换曲线族。
- 工业界（RoadRunner、CARLA、Apollo 制图）的连接路算法基本不公开，可借鉴的开源实现只有 SUMO；作为对标基线，SUMO 的水平低于我们当前的 C2 后处理要求。
- 许可：SUMO 为 EPL-2.0 OR GPL-2.0+，可以只读思路；照搬代码要按 EPL-2.0 走，并做隔离评估。CommonRoad Scenario Designer 为 GPL-3.0，按项目硬约束只能进程隔离调用或离线对拍。Apollo 为 Apache-2.0，可以参考。

### Gaps
- RoadRunner 路口自动生成连接车道时用哪类曲线、导出的 OpenDRIVE 连接路是 spiral 还是 paramPoly3：没有找到官方文档（RoadRunner Help Center 未检索到相关页）。
- CARLA：carla.readthedocs.io 不可达，没有核实"OpenDRIVE standalone mode 是否只按给定 xodr 生成网格、不生成连接路"。
- Apollo 地图制作工具（imap 等）的路口连接方法、Lanelet2 生态（lanelet2 本身不生成路口连接）都没有找到可引用资料。
- SUMO 中 `numPoints` 与 `--junctions.internal-link-detail` 的对应关系，本次读到的源码片段没有显示。

## 问题 2：2010–2026 年路口连接/车道连接的曲线族论文（G2 clothoid、η 样条、PH 曲线、五次 Bézier、优化型）

### Takeaway
在"两端位置 + 航向 + 曲率都给定"（G2 Hermite）这个问题上，研究成熟的三条路线是：①**三段回旋线**（Bertolazzi & Frego 2018，一定有解，Newton 最坏 5 次迭代，BSD-2 C++ 库，项目已锁定的 pyclothoids 0.2.0 就封装了它）；②**多项式 η 样条**（五次 η² 样条到 G2，七次 η³ 样条到 G3，并留有 4/6 个形状参数可再优化，例如最小化曲率变化率）；③**五次 PH 曲线**（弧长是多项式，但平面单段 G2 插值没有剩余自由度、可能有多个孤立解）。专门针对"HD 地图路口连接"的论文很少，多数是参考线/车道线拟合（回旋线 G1/G2 样条）。

### Cited Findings
**三段回旋线 G2 Hermite（Bertolazzi & Frego）**
- "On the G2 Hermite interpolation problem with clothoids"（J. Comput. Appl. Math. 341 (2018) 99–116）：G2 Hermite 问题用 1 段或 2 段回旋线不一定有解（有反例）；提出 3 段方案，原为 8 方程 10 未知数的非线性系统，可化为 2 个方程，用 Newton 法求解，摘要称"总是收敛，最坏情况 5 次标准 Newton 迭代" — [UniTN IRIS 记录（经搜索摘要，全文未读）](https://iris.unitn.it/handle/11572/210452)
- 早先的 G1 工作 "G1 fitting with clothoids"（Math. Methods Appl. Sci. 2015）把 G1 问题化为一元单个非线性函数求零点 — [arXiv:1209.0910（经搜索摘要）](https://arxiv.org/abs/1209.0910v1)
- Clothoids C++ 库（附 MATLAB 接口）实现回旋线 G1/G2 拟合、G1 和 G2 回旋线样条、圆弧、双圆弧；README 列出 "Interpolating clothoid splines with curvature continuity"（MMAS 2018）、"Point-Clothoid Distance and Projection Computation"（SIAM J. Sci. Comput. 2019）等 — [ebertolazzi/Clothoids](https://github.com/ebertolazzi/Clothoids)
- Clothoids 库许可为 BSD 2-Clause（"Copyright (c) 2020, Enrico Bertolazzi and Marco Frego"）— [Clothoids license.txt](https://github.com/ebertolazzi/Clothoids/blob/master/license.txt)
- `G2solve3arc` 源码：Newton 未知数为 `(sM, thM)`（中段长度、中段航向）；`Dmax`（默认 π，上限 2π）和 `dmax`（默认 π/8，上限 π/4）限制两端段的转角/曲率；两端段长 `s0`、`s1` 由启发式给出（先建一条 G1 回旋线，取总长的 1/3，再按曲率和转角上限截短，乘以按两端航向差缩放的系数）；另有 `build_fixed_length(s0, s1)` 由调用方给定端段长；收敛判据为残差范数 `< m_tolerance`（容差须 ≤ 0.1），最大迭代 ≤ 1000；失败时 `build` 返回 −1，且只有收敛才写解 — [Clothoids src/ClothoidG2.cc](https://github.com/ebertolazzi/Clothoids/blob/master/src/ClothoidG2.cc)
- pyclothoids 0.2.0（MIT 许可，Copyright 2020 Phillip Dix）暴露 `SolveG2(x0, y0, t0, k0, x1, y1, t1, k1, Dmax=0, dmax=0)`，内部调用 `G2solve3arc().build(...)`，返回 3 条 `Clothoid`；另有 `Clothoid.G1Hermite(..., tol=1e-10)` — [PyClothoids GitHub](https://github.com/phillipd94/PyClothoids)（本地 wheel 的 LICENSE 与 `clothoid.py` 已核对）

**η 样条（Piazzi、Guarino Lo Bianco 等，Parma）**
- 五次 G2 样条（η² / "η-spline"）：Piazzi、Guarino Lo Bianco、Bertozzi、Fascioli、Broggi，"Quintic G2-splines for the Iterative Steering of Vision-based Autonomous Vehicles"，IEEE T-ITS 3(2):27–36, 2002，DOI 10.1109/6979.994793；参数化五次样条，按 G2 插值点列，讨论完备性、最小性、对称性等 — [ce.unipr.it PDF](https://www.ce.unipr.it/people/broggi/publications/ieee.its.2001.pdf)
- 姊妹篇 "Optimal Trajectory Planning with Quintic G2-splines"（IV 2000）用这一族样条最小化曲率变化率（在平坦性控制中等价于最小化转向变化率）— [TRID 记录](https://trid.trb.org/View/733928)
- η³ 样条：Piazzi、Guarino Lo Bianco、Romano，IEEE T-RO 2007；七次多项式样条，可插值任意点列及其切向、曲率、曲率导数（G3）；由 6 维参数向量 η 塑形，在统一框架中可生成或逼近圆弧、回旋线、螺线 — [UniPR AIR 记录（经搜索摘要）](https://air.unipr.it/handle/11381/2295771)
- Guarino Lo Bianco & Gerelli 2010 利用 η³ 的自由参数求"曲率导数最小"的路径，并给出可在线用的闭式启发式；Tagliavini & Guarino Lo Bianco 2021 推广到 3D（η³ᴰ）— [UniPR AIR 记录（经搜索摘要）](https://air.unipr.it/handle/11381/2299488)

**Pythagorean-hodograph（PH）曲线**
- Jaklič 等 2014（NMTMA）："G2 五次 PH 插值"在任意维匹配两端点、两切向、两曲率向量，问题化为关于切向长度的两个多项式方程，可能有多个解；用光滑数据的渐近分析选最有希望的解，再用同伦延拓从特例追踪到一般数据 — [Global Science Press](https://ojs.global-sci.org/index.php/nmtma/article/view/14061)
- Pelosi 2026（CAGD）：平面单段五次 PH 曲线做 G2 Hermite 插值，未知数化为 2 个实变量，给定 G2 数据后插值曲线完全确定、没有剩余形状参数（可能存在多个孤立解），因此一般不能再加弧长等约束 — [UniBo 预印本](https://cris.unibo.it/bitstream/11585/936372/3/final_version.pdf)
- Knez 等：七次 PH 双弧（4 个自由参数）构造指定弧长的平面 G2 样条路径；PH 曲线的累计弧长是多项式，适合自动驾驶路径规划的精确构造 — [UniSi 预印本](https://usiena-air.unisi.it/retrieve/e0feeaab-bf3a-44d2-e053-6605fe0a8db0/Construction%20of%20G2%20planar%20Hermite%20interpolants%20with%20prescribed%20arc%20lengths%20Preprint.pdf)

**面向 HD 地图的回旋线拟合**
- Songyi Zhang 等，"Clothoid-Based Reference Path Reconstruction for HD Map Generation"，IEEE T-ITS 25(1):587–601, 2024（2023-08-24 在线，非开放获取）— [OpenAlex 记录](https://api.openalex.org/works/doi:10.1109%2FTITS.2023.3305198)。据 ResearchGate 摘要：把密集参考线点压缩为直线/圆弧/回旋线，短路径用线性规划，长路径用渐进方法，相邻位姿之间用 3 段回旋线连接，并涉及环岛和交叉口 — [ResearchGate（经搜索摘要，未读全文）](https://www.researchgate.net/publication/373376805_Clothoid-Based_Reference_Path_Reconstruction_for_HD_Map_Generation)。注：另一次检索的摘要未能证实其覆盖交叉口，此点存疑。
- Cudrano、Gallazzi、Frosi、Mentasti、Matteucci：单目视觉建车道级 HD 地图，每条车道线表示为回旋线样条，只保证 G1 连续，并用贪心步骤剪掉多余段 — [IEEE Vehicular Technology Magazine 2022](https://read.nxtbook.com/ieee/vehicular_technology/vehiculartechnology_dec_2022/clothoid_based_lane_level_hig.html)
- METU 学位论文：用回旋线序列分层拟合采样参考路径（先线型，再曲线，再分段链），在 HERE HD 地图几何上评估 — [METU Open](https://open.metu.edu.tr/handle/11511/119461)
- GAD（arXiv 2405.00515）把轨迹分为直行与转弯两类，指出路口转弯接近圆弧；采样器生成直线、等曲率、类回旋线三种路径，曲率经自行车模型与转向角挂钩 — [arXiv 2405.00515（经搜索摘要）](https://arxiv.org/pdf/2405.00515)

### Inferences
- **与 OpenDRIVE 的可表示性**：三段回旋线可以用 `spiral`（curvStart/curvEnd）元素逐段**精确**写出，两端曲率与道路端相等，因此严格 G2；而五次/七次多项式（η²、η³、PH 五次、Apollo 五次样条）超出 `paramPoly3` 的三次，只能再分段近似，近似误差处又会出现曲率小跳变——对"车道边缘处处 C2"的目标不利。三次 `paramPoly3` 要 G2 只能多段拼接（见问题 1 推断）。
- **三段回旋线的形状自由度**：给定 8 个端点条件后，`G2solve3arc` 仍有两个端段长 `s0`、`s1` 可选（`build_fixed_length`）。这正好是一个**二维连续参数族**，可以在其上最小化"离 via 线的偏差 + 曲率变化率罚项"，而不是在几个离散候选里择优。曲率在三段内是分段线性的，"转弯曲率单调"可以直接写成三段曲率变化率同号的约束来检查。
- η³ 的 6 个参数（或 η² 的 4 个）同样可以用于贴导引线，并且 G3 意味着车道中心曲率变化率也连续；代价是写入 OpenDRIVE 时必须近似。
- 五次 PH 单段 G2 插值没有自由度、可能多解，不适合"既要端点 G2 又要贴源线"的需求；PH 七次双弧（4 个自由参数）理论上可行，但同样受 `paramPoly3` 三次上限限制。
- 关于"车道边缘曲率跳变"：车道边缘 = 参考线按宽度 w(s) 偏移。按偏移曲线的微分几何，边缘曲率除了参考线曲率 κ 外还依赖 w′、w″，所以要边缘 G2，结点处除 κ 连续外还要 w、w′、w″ 连续——而 OpenDRIVE 车道宽度是逐段三次多项式，跨 `<width>` 记录或跨 planView 段时 w″ 不会自动连续。这和"单车道连接路在参考线拼接处边缘曲率跳"现象一致（推断，未找到专门文献）。

### Gaps
- 没有核实到 "η⁴-splines" 的存在与内容（检索未命中）；Parma 组是否有 G4 推广不确定。
- 没读到 "Interpolating clothoid splines with curvature continuity"（MMAS 2018）全文，因此 `ClothoidSplineG2` 的目标函数选项（最小长度/最小曲率/最小曲率导数等）未经核实。
- 三次曲线的 G2 Hermite 插值（de Boor–Höllig–Sabin 1987 一类结果：可能 0–3 个解）本次没有检索到可引用来源，只作背景记忆，不能作为事实写入报告。
- 没有找到 2010–2026 年专门在 IEEE IV/ITSC/T-ITS 发表、以"OpenDRIVE 路口连接路生成 + 两端 G2 + 贴导引线"为题的论文；被引的 "Guo et al. 路口回旋线模型" 原文未找到。

## 问题 3："贴近导引线"如何与端点约束结合（带等式约束的最小二乘、走廊约束、QP），以及这些方法的确定性

### Takeaway
主流做法是把端点条件写成**等式约束**、把"贴近源线"写成**目标函数中的偏差项**或**锚点周围的盒式/走廊不等式约束**，再加平滑项（二阶/三阶导数平方积分、曲率与曲率变化率罚项），整体写成凸 QP（Apollo 参考线平滑器是公开的典型）。纯插值族（三段回旋线、η 样条、PH）本身没有保真项，需要在其自由参数上再做外层优化。凸 QP 的解唯一，理论上与平台无关；离散"多候选择优"和多解非线性方程才是跨平台翻转的来源。

### Cited Findings
- Apollo QP-spline 参考线平滑：分段五次多项式 x=f_i(t)、y=g_i(t)；目标 Σ(∫(f_i''')² dt + ∫(g_i''')² dt)；结点处位置及 1、2、3 阶导数相等（等式约束）；在均匀采样的 m 个锚点上约束 f_i(t_l) − x_l < boundary、g_i(t_l) − y_l < boundary（不等式约束）；写成标准 QP ½xᵀHx + fᵀx，带上下界、Aeq·x = beq、A·x ≤ b — [Apollo reference_line_smoother.md](https://github.com/ApolloAuto/apollo/blob/r6.0.0/docs/specs/reference_line_smoother.md)
- Baidu 相关专利：平滑项直接写进目标函数（不是事后处理），使平滑结果保持在约束内；分段多项式预设为五次；在每个控制点周围设边界盒作为不等式约束（摘要中给出约 0.2 m × 0.4 m 的盒子尺寸）— [USPTO 10591926](https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/10591926)、[USPTO 11493921](https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/11493921)（注：0.2 m × 0.4 m 这一数值出自哪一件专利，本次未逐件核实）
- "Optimal Vehicle Path Planning Using Quadratic Optimization for Baidu Apollo Open Platform"（arXiv 2112.02132）：二次优化同时惩罚导引线的曲率及其变化率，并把平滑后的线限制在车道边界内、尽量靠近车道中心，允许输入点小幅移动以吸收地图误差 — [arXiv 2112.02132（经搜索摘要，全文未读）](https://arxiv.org/pdf/2112.02132)
- 三段回旋线求解器：Newton 迭代到残差 < 容差即停，失败返回 −1；端段长由启发式（含 cos、幂函数缩放）决定，或由 `build_fixed_length` 外部给定 — [Clothoids src/ClothoidG2.cc](https://github.com/ebertolazzi/Clothoids/blob/master/src/ClothoidG2.cc)
- 五次 PH G2 插值可能多解，靠渐近分析 + 同伦延拓选解 — [Jaklič 等 2014](https://ojs.global-sci.org/index.php/nmtma/article/view/14061)；平面单段五次 PH 可能有多个孤立解 — [Pelosi 2026 预印本](https://cris.unibo.it/bitstream/11585/936372/3/final_version.pdf)
- η³ 样条的形状参数可以用来最小化曲率导数，并有闭式启发式 — [UniPR AIR 记录](https://air.unipr.it/handle/11381/2299488)
- SUMO 的连接路形状是闭式几何构造（外延线求交、固定比例回拉），不含迭代求解 — [SUMO NBNode.cpp](https://github.com/eclipse-sumo/sumo/blob/main/src/netbuild/NBNode.cpp)

### Inferences
- **可直接移植的形式**（适用性判断）：以连接路参考线为未知数（例如若干段回旋线的段长/曲率变化率，或分段多项式系数），
  - 等式约束：两端 (x, y, θ, κ) 等于道路口部值（如需"边缘 G2"，再加宽度 w、w′、w″ 在口部的等式）；
  - 目标：Σ 离 via 点的横向偏差² + λ₁∫κ′² + λ₂∫κ″²（对应"罚曲率变化率跳变"）；
  - 不等式：离 via 线的走廊 |d| ≤ δ（可用松弛变量软化）、转弯段 κ′ 同号（单调曲率）、|κ| ≤ κ_max。
  若曲线族取分段多项式，问题是凸 QP；若取回旋线段长作变量，端点 G2 是非线性等式，变成小规模 NLP（2–6 个变量），可以用 Clothoids 库内层求解 + 外层有界优化。
- **确定性**：凸 QP（严格凸目标）有唯一解，不同平台的差异只到求解器容差量级，属于连续偏差；跨 Windows/Linux 翻转的根源是"在候选之间取 argmin"这种不连续映射——两个候选得分差小于浮点噪声时就可能翻转。缓解办法：①用单一连续优化取代离散择优；②离散择优保留，但分数按固定量化（如 1e-9 m 网格）比较，并用固定的确定性 tie-break（候选序号）；③Newton 类求解器固定初值、固定迭代次数上限与容差，并对输入做量化。以上均为工程判断，没有找到专门文献。
- 纯插值族（回旋线三段式、η 样条、PH）不包含"贴源"目标，所以单靠它们无法表达"保真 vs 平滑"的权衡；Apollo 式"锚点盒 + 平滑目标"的 QP 才把两者放进同一个问题，但 Apollo 的平滑器面向在线参考线，并没有把端点曲率等式和车道宽度一起处理。

### Gaps
- 没有找到公开的、专门用于"路口连接路：端点 G2 等式 + via 走廊 + 曲率变化率最小"的论文或开源实现；arXiv 2112.02132 全文未读，其是否包含端点曲率等式约束未核实。
- Apollo 新版本中的 FEM_POS_DEVIATION（离散点 + OSQP）等平滑器的具体公式，本次没有拿到官方文档。
- 没有找到关于 OSQP/IPOPT 等求解器在 Windows 与 Linux 上结果逐位一致性的权威资料。

## 问题 4：是否有工作把道路段与路口连接路放进同一个优化（网络级平滑），规模与耗时

### Takeaway
有，但都不是"OpenDRIVE 生成 + 严格 G2"语境：Zhang 等（TR-C 2016）提出道路与路口统一的车道级路网模型，保证任意路线上的连续性（据摘要用三次 Hermite 样条）；一件专利对整张图的节点坐标做全局连续优化，其势能项之一专门惩罚路口处不连续；CVPR 2024 的 Bézier 图把整张车道图的 Bézier 参数联合最小化（梯度下降）。没有找到报告规模与耗时的、可对标的网络级 G2 联合优化。

### Cited Findings
- Tao Zhang、Stefano Arrigoni、Marco Garozzo、Dian-ge Yang、Federico Cheli，"A lane-level road network model with global continuity"，Transportation Research Part C 71:32–50, 2016, DOI 10.1016/j.trc.2016.07.003：在同一框架中建模道路与交叉口，声称能保证任意地图路线上的连续性，更符合真实车辆轨迹；在米兰城区验证 — [ScienceDirect](https://www.sciencedirect.com/science/article/abs/pii/S0968090X16301012)；据 ResearchGate 摘要，该模型用三次 Hermite 样条实现全局连续，并用"joint lanes"近似转弯轨迹 — [ResearchGate（经搜索摘要）](https://www.researchgate.net/publication/305829849_A_lane-level_road_network_model_with_global_continuity)
- 专利 US 8384776（由传感器数据检测拓扑结构）：对图节点坐标求解全局连续优化，最小化若干势能之和，其中一项惩罚路口处的不连续；流程为先在固定路口的前提下分别平滑各车道段，再推断路口连续性，最后做全局优化 — [USPTO 8384776](https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/8384776)
- Blayney 等，"Bézier Everywhere All at Once: Learning Drivable Lanes as Bézier Graphs"（CVPR 2024）：把 Bézier 曲线拟合到车道图上需要跨整张图联合最小化 Bézier 参数，用梯度下降求解 — [CVF Open Access](https://openaccess.thecvf.com/content/CVPR2024/papers/Blayney_Bezier_Everywhere_All_at_Once_Learning_Drivable_Lanes_as_Bezier_CVPR_2024_paper.pdf)
- LaneGAP（ECCV 2024）主张以"整条路径"而不是"车道片段"为建模单元，理由是分片建模会破坏车道连续性 — [ECVA](https://www.ecva.net/papers/eccv_2024/papers_ECCV/papers/06068.pdf)

### Inferences
- **适用性**：我们的问题——道路端方向轻微变化导致连接路变形——正是"先定道路、再接连接路"的串行流程的典型后果：口部状态 (x, y, θ, κ) 的任何扰动都 100% 传给连接路。专利 US 8384776 的三步法（先各段平滑 → 再推断路口连续 → 最后全局优化）提供了一个渐进式路线：保留现有道路拟合作初值，最后一步只放开"口部附近一小段道路 + 其连接路"的变量做局部联合优化。这比整图求解器小得多，也避开了 D1 冻结的"L07 整图求解器"（是否属于冻结范围需人工确认）。
- 联合优化的规模可以按路口局部化：每个路口的变量 = 各进口/离去道路靠口部的若干段 + 所有连接路；道路远端固定。问题规模随路口连接数线性增长，单路口应在几十到几百个变量量级（推断，未找到实测）。
- 共享口部状态时，同一进口的多条连接路（直/左/右）要求同一个 (θ, κ)，所以联合优化需要显式处理"一端多连"的耦合；文献中这一点只在 Zhang 2016 的"路口统一建模"里间接涉及。

### Gaps
- 没有找到报告网络级 G2/曲率连续联合优化在真实城市路口上规模与运行时间的文献；Zhang 2016 的实验规模、用的是 G1 还是 G2，都未读到全文核实。
- 没有找到"OpenDRIVE 路网整体几何优化"的开源工具。

## 问题 5：不可行组合（急弯 + 曲率上限/平滑要求）的标准处理：报告与放宽

### Takeaway
现有工具的惯例是**检测 → 标失败/记误差 → 退化为简单几何（直线/折线）→ 发警告并给出可调参数**（SUMO），或**返回失败码由调用方处理**（Clothoids）；数学上 G2 Hermite 用三段回旋线总有解，但会以端段大转角、大曲率为代价，Clothoids 用 `Dmax/dmax` 限制。没有找到"在路口连接路上用松弛变量量化源冲突"的公开规范，但 QP 的盒约束/走廊约束天然支持这种做法。

### Cited Findings
- SUMO：过度 S 弯判据 `bendDeg > 22.5 && (bendDeg/45)^2/dist > 0.13` → `ok=false` 并记录 `myDisplacementError`；外延线无交点 → `ok=false`、位移误差 1.0；交点离两端都过近 → 失败；左转用 `AVOID_WIDE_LEFT_TURN` 回拉控制点，防止过大外扩 — [SUMO NBNode.cpp](https://github.com/eclipse-sumo/sumo/blob/main/src/netbuild/NBNode.cpp)
- SUMO OpenDRIVE 导出：Bézier 失败时用弦长作长度并发警告，提示用 `junctions.scurve-stretch` 或加大路口半径；控制点为空则写折线 — [SUMO NWWriter_OpenDrive.cpp](https://github.com/eclipse-sumo/sumo/blob/main/src/netwrite/NWWriter_OpenDrive.cpp)
- G2 Hermite：1 段或 2 段回旋线可能无解，3 段方案总能收敛 — [UniTN IRIS（经搜索摘要）](https://iris.unitn.it/handle/11572/210452)；实现中 `Dmax`、`dmax` 限制端段转角，失败返回 −1 — [Clothoids src/ClothoidG2.cc](https://github.com/ebertolazzi/Clothoids/blob/master/src/ClothoidG2.cc)
- 五次 PH 的 G2 插值没有剩余自由度，额外的几何约束一般无法再施加 — [Pelosi 2026 预印本](https://cris.unibo.it/bitstream/11585/936372/3/final_version.pdf)
- Apollo QP 平滑把源点偏差限制在锚点盒内，盒子大小即允许的保真偏离 — [Apollo reference_line_smoother.md](https://github.com/ApolloAuto/apollo/blob/r6.0.0/docs/specs/reference_line_smoother.md)；arXiv 2112.02132 允许输入点小幅移动以吸收地图误差 — [arXiv 2112.02132（经搜索摘要）](https://arxiv.org/pdf/2112.02132)

### Inferences
- **对 R < 5 m 急弯**：若曲率上限或平滑罚项与源 via 线冲突，推荐做法是保留硬约束（端点 G2、曲率单调），把"贴源"作为带松弛变量的软走廊；输出时把松弛量（最大偏离、偏离区间、所需最小半径 vs 源半径）写入源冲突记录。这与现有"源冲突只记录"的口径一致，而且比"二选一候选"更能量化冲突。
- 三段回旋线在急弯处"总能解出"不等于"可接受"：要额外检查端段曲率峰值与反打（三段曲率变化率不同号）；解出但超阈值时也应标冲突，不能静默接受。
- SUMO 的"失败退化为直线/折线"不适合我们的消费者（自动驾驶），只能作反例；可以借鉴的是它"记录误差量 + 指出可调参数"的报告方式。

### Gaps
- 没有找到 HD 地图行业（NDS、OpenDRIVE 生产方）对"连接路曲率不可行"的公开报告规范或状态码约定。
- 没有找到针对"城市路口最小转弯半径与 AD 消费者容忍的曲率/曲率变化率阈值"的权威数值来源（属于另一调研方向）。
