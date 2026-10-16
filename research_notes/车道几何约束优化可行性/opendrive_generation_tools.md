# 现有工具与论文如何从折线/HD 地图源生成 OpenDRIVE 道路与车道几何（拟合方法、难例处理、精度与局限）

> 调研范围：从折线 / lanelet / 点云生成 OpenDRIVE planView（line/arc/spiral/paramPoly3）与车道几何（laneOffset、width 多项式、laneSection）的开源工具、商业工具与 2015–2026 论文；重点是拟合方法（启发式 vs 最小二乘/约束优化）、车道出生/消失（taper）、路口连接路、保真容差与光顺、报告的质量与局限。事实（Cited Findings，带出处）与适用性判断（Inferences）分开写。路口连接路曲线族的深入调研见同目录 `junction_connectors.md`，本文只记生成工具在这方面的做法。
>
> 访问限制说明（影响证据等级）：本次会话出网代理拒绝 arxiv.org、mdpi.com、ieeexplore、researchgate、sumo.dlr.de、mathworks.com、readthedocs、gitlab.lrz.de、mediatum.ub.tum.de 等域名（CONNECT 403 / DNS 失败），只有 GitHub（raw.githubusercontent.com、github.com 页面）和 PyPI 可直接读取。因此：**工具部分的结论来自直接阅读源码**（SUMO、CommonRoad Scenario Designer 0.8.5 wheel、scenariogeneration 0.16.7 wheel、tier4/autoware_lanelet2_to_opendrive、ftgTUGraz/opendrive-digital-twin-generator、Apollo 解析器、hdmap_generator issue）；**论文部分的方法细节与精度数字来自搜索引擎摘录**（未能读全文），已逐条标注，可信度低于源码结论。所有代码均为 2026-10-10 抓取的 main/master 版本，另注明发布版本号的除外。

## 问题 1：主流工具（RoadRunner、CommonRoad Scenario Designer、CARLA、SUMO netconvert、Lanelet2→OpenDRIVE、Apollo、GitHub 生成项目）用什么方法拟合参考线与车道宽度？有没有用最小二乘或约束优化的？

### Takeaway
查看的 7 个开源项目中（SUMO、CommonRoad cr2odr、tier4、TU Graz、scenariogeneration 读了几何源码；roadgen 只读了 README/issue；Apollo 只读了解析器），**参考线拟合以启发式为主**（SUMO：长直段写 line、拐角切掉后用切线外推的三次 Bézier=paramPoly3；CommonRoad cr2odr：按有限差分航向/曲率阈值把折线分类成 line/arc/spiral）；**只有两个用了最小二乘**：tier4 的 Lanelet2→OpenDRIVE 转换器（加权最小二乘 B 样条，端点位置/切向用大权重罚项“硬约束”，曲率自适应节点，之后按约 1 m 切成 paramPoly3 链）和 TU Graz 数字孪生生成器（主线每 100 m 窗口独立最小二乘 paramPoly3，匝道用全局最小二乘三次样条 + 自适应节点、每个节点区间一段 paramPoly3）。**宽度**普遍很粗糙：SUMO/CommonRoad 常数或两点线性，TU Graz 窗口中位数常数后再三次拟合，tier4 用一维样条转分段三次。没有一个工具把参考线、宽度、laneSection 位置放进同一个带显式保真容差的约束优化。

### Cited Findings

**SUMO netconvert OpenDRIVE 输出（EPL-2.0 OR GPL-2.0-or-later；main 分支）**
- 许可证：源文件头 `SPDX-License-Identifier: EPL-2.0 OR GPL-2.0-or-later`；SUMO README 称以 EPL 2.0 授权 — [NWWriter_OpenDrive.cpp](https://github.com/eclipse-sumo/sumo/blob/main/src/netwrite/NWWriter_OpenDrive.cpp)；[SUMO README](https://github.com/eclipse-sumo/sumo/blob/main/README.md)
- 普通道路的参考线取“最左车道的左边界”（`getInnerLaneBorder(e)`）；只有 2 个点或人行道时直接写 line 链 — [NWWriter_OpenDrive.cpp](https://github.com/eclipse-sumo/sumo/blob/main/src/netwrite/NWWriter_OpenDrive.cpp)
- `writeGeomSmooth`：纯启发式。`longThresh = speed`（代码注释写“应与源数据采样率匹配”，原为 16.0），`curveCutout = longThresh/2`；相邻两段都“长”且转角 > `straightThresh` 的顶点被删掉，在两侧各 `curveCutout` 处插点；之后“长段写 line、短段写曲线”，曲线由 `NBNode::bezierControlPoints` 用前后段方向外推（外推长度 min(25, 段长/4)）得到控制点，再写成单个 paramPoly3；控制点求不出时退回 line — [NWWriter_OpenDrive.cpp](https://github.com/eclipse-sumo/sumo/blob/main/src/netwrite/NWWriter_OpenDrive.cpp)
- `writeGeomPP3`：Bézier 控制点到 paramPoly3 系数是精确换算（三次：bU=3(P1−P0)，cU=3P0−6P1+3P2，dU=−P0+3P1−3P2+P3；二次时 dU=0）— [NWWriter_OpenDrive.cpp](https://github.com/eclipse-sumo/sumo/blob/main/src/netwrite/NWWriter_OpenDrive.cpp)
- 选项 `opendrive-output.straight-threshold` 默认 1e-8（度），描述为“相邻直段角度变化超过该值即构造参数曲线”；OpenDRIVE 输出时强制 `rectangular-lane-cut=true`，否则警告“OpenDRIVE cannot represent oblique lane cuts” — [NWFrame.cpp](https://github.com/eclipse-sumo/sumo/blob/main/src/netwrite/NWFrame.cpp)
- 普通道路车道宽度写成常数 `<width a=laneWidth b=0 c=0 d=0>` — [NWWriter_OpenDrive.cpp](https://github.com/eclipse-sumo/sumo/blob/main/src/netwrite/NWWriter_OpenDrive.cpp)
- SUMO 导出文档称不建模道路加宽：车道数变化即换一条 road（搜索摘录，原页面本次无法打开）— [SUMO Networks/Export](https://sumo.dlr.de/docs/Networks/Export.html)

**CommonRoad Scenario Designer `cr2odr`（GPL-3.0；PyPI 0.8.5，2025-09-29 发布）**
- 许可证 GPL-3.0（PyPI 元数据）— [PyPI commonroad-scenario-designer](https://pypi.org/project/commonroad-scenario-designer/0.8.5/)
- 道路构造：从一个 lanelet 出发向左右扩展相邻 lanelet 组成一条 road，按后继/前驱广度优先遍历（`construct_roads`）；参考线取“行驶方向变化处那条 lanelet 的左边界顶点” — wheel 内 `crdesigner/map_conversion/opendrive/cr2odr/converter.py`、`elements/road.py`，[PyPI 0.8.5](https://pypi.org/project/commonroad-scenario-designer/0.8.5/)
- planView（`set_plan_view`）：对折线逐点算离散曲率和航向，按阈值把连续点归为 LINE（相邻航向差 < `heading_threshold`）、ARC（曲率与段首曲率差 < `curvature_threshold`）、SPIRAL（曲率差分近似常数）；SPIRAL 段用 `pyclothoids.SolveG2` 在段首尾点（位置、航向、曲率）之间做三段回旋线 G2 Hermite 插值；ARC 曲率取段末点曲率。代码注释自认“could be more smooth … with resampling” — 同上 `elements/road.py`
- 阈值默认：`heading_threshold=0.00174533`（注释写 1°，数值实为 0.1°）、`curvature_threshold=0.01`、`curvature_dif_threshold=0.01` — wheel 内 `crdesigner/common/config/opendrive_config.py`
- 宽度：每条 lane 只写一条 `<width>`，取 lanelet 首尾两个截面宽度做一次 `np.polyfit(…,1)`，即整条 road 线性宽度；每条 road 只有一个 laneSection — 同上 `elements/road.py`（`lane_help`、`lane_sections`）
- 无任何拟合误差检查或容差参数（源码中未见）— 同上

**tier4 / hakuturu583 `autoware_lanelet2_to_opendrive`（版本 2.62.0，输出 OpenDRIVE 1.4，作者 Masaya Kataoka；许可证未能确认）**
- README：Lanelet2 → OpenDRIVE 1.4，面向 CARLA；参考线“以 `<paramPoly3>` 链输出，可选分类成 `<line>`/`<arc>`/`<paramPoly3>` 段（`arcspiral.enabled`）”；内置 ASAM QC 校验与 Lanelet2↔road 几何交叉验证（`analyze`） — [package README](https://github.com/tier4/autoware_lanelet2_to_opendrive/blob/master/autoware_lanelet2_to_opendrive/README.md)；版本号见 [pyproject.toml](https://github.com/tier4/autoware_lanelet2_to_opendrive/blob/master/autoware_lanelet2_to_opendrive/pyproject.toml)
- 参考线拟合 = **加权最小二乘 B 样条**：数据点为软约束（权 20），起终点位置与起终点速度向量为“硬约束”（权 80，实为罚项），一次 `lstsq` 求控制点；节点按曲率自适应放置（逆变换采样，`knot_alpha_weight=2`、`knot_beta_weight=2`）；控制点数 = 输入点数×0.4 再按曲率放大，夹在 [5, 50] — [spline.py](https://github.com/tier4/autoware_lanelet2_to_opendrive/blob/master/autoware_lanelet2_to_opendrive/src/autoware_lanelet2_to_opendrive/spline.py)、[config.py](https://github.com/tier4/autoware_lanelet2_to_opendrive/blob/master/autoware_lanelet2_to_opendrive/src/autoware_lanelet2_to_opendrive/config.py)
- 拟合误差只**告警不失败**：平均误差 > `max_avg_error=2.0` 或单点 > `max_point_error=8.0`（坐标单位）时发 UserWarning；端点约束超 `position_tolerance=5.0` 时抛错 — 同上 spline.py / config.py
- 路口连接路端点覆盖：连接路首末点被替换为相连道路参考线端点，硬约束权提高到 1e4；代码注释称默认权重下最小二乘会“用 10–20 cm 边界误差换数据拟合”，1e4 可得亚毫米端点精度且内部偏离默认拟合约 1 cm，权重再大（1e6）会让参考线走出源 lanelet 走廊、破坏几何交叉验证；测试容差 5 cm — [reference_line.py](https://github.com/tier4/autoware_lanelet2_to_opendrive/blob/master/autoware_lanelet2_to_opendrive/src/autoware_lanelet2_to_opendrive/opendrive/reference_line.py)
- 输出段数：paramPoly3 目标段长 `default_segment_length=1.0 m`、最短 0.5 m、每条 road 最多 100 段、最少 1 段（docstring 示例：10 m→10 段，150 m→封顶 100 段） — [geometry.py](https://github.com/tier4/autoware_lanelet2_to_opendrive/blob/master/autoware_lanelet2_to_opendrive/src/autoware_lanelet2_to_opendrive/opendrive/geometry.py)、[config.py](https://github.com/tier4/autoware_lanelet2_to_opendrive/blob/master/autoware_lanelet2_to_opendrive/src/autoware_lanelet2_to_opendrive/config.py)
- 可选曲率分类器（issue #466）把拟合好的样条切成 line/arc/paramPoly3 段，默认关闭 — [geometry_classifier.py](https://github.com/tier4/autoware_lanelet2_to_opendrive/blob/master/autoware_lanelet2_to_opendrive/src/autoware_lanelet2_to_opendrive/opendrive/geometry_classifier.py)
- 宽度：`estimate_lanelet_width_as_spline` 得一维宽度样条，再按其多项式分段写成多条三次 `<width>`；左舵/右舵分别以内侧边界为宽度锚 — [lane.py](https://github.com/tier4/autoware_lanelet2_to_opendrive/blob/master/autoware_lanelet2_to_opendrive/src/autoware_lanelet2_to_opendrive/opendrive/lane.py)
- 高程：样条按 0.1 m 重采样，段端用三次 Hermite 求 elevation 系数 — [reference_line.py](https://github.com/tier4/autoware_lanelet2_to_opendrive/blob/master/autoware_lanelet2_to_opendrive/src/autoware_lanelet2_to_opendrive/opendrive/reference_line.py)
- 本次读取的 repo 根目录与包目录均无 LICENSE 文件（raw 404），README 也未写许可证 — [repo](https://github.com/tier4/autoware_lanelet2_to_opendrive)

**ftgTUGraz `opendrive-digital-twin-generator`（论文 arXiv 2606.16570 的官方实现；许可证未见）**
- 主线参考线：按 `--segment-length`（默认 100 m）切窗口，每窗口把建图轨迹沿法向平移（左路缘侧向偏移取窗口中位数），再在局部 u/v 坐标中用 `np.linalg.lstsq` 拟合 paramPoly3（a 固定为 0，参数 p 取按点序的 `linspace(0,1)`，非弧长）；**相邻窗口之间无位置/航向连续约束** — [fit_mainline_phase2.py](https://github.com/ftgTUGraz/opendrive-digital-twin-generator/blob/main/analysis/fit_mainline_phase2.py)、[utils/equations.py](https://github.com/ftgTUGraz/opendrive-digital-twin-generator/blob/main/utils/equations.py)
- 主线车道：每窗口车道数 = 有效标线数中位数（夹在 1–5），车道宽 = 窗口内 2–6 m 有效宽度的中位数，写常数宽度 — [fit_mainline_phase2.py](https://github.com/ftgTUGraz/opendrive-digital-twin-generator/blob/main/analysis/fit_mainline_phase2.py)
- 匝道：x(s)、y(s) 用 `scipy.interpolate.LSQUnivariateSpline`（k=3）做**全局最小二乘三次样条**，内部节点取上一步分段边界（“adaptive knots”），失败时退回均匀 50 m 节点；再 `PPoly.from_spline` 拆成每个节点区间一段多项式；车道边界的横向偏移 t(s) 用同一组节点的最小二乘样条拟合 — [ramp_fitter.py](https://github.com/ftgTUGraz/opendrive-digital-twin-generator/blob/main/analysis/ramp_fitter.py)
- 车道宽度转换：把各边界的绝对偏移多项式逐条相减后再 `np.polyfit(…,3)` 得每条车道的三次宽度 — [lane_width_converter.py](https://github.com/ftgTUGraz/opendrive-digital-twin-generator/blob/main/xodr_builder/lane_width_converter.py)
- README：流程含 `fit-mainline`、`infer-splits`、`generate-ramps`、`infer-topology`、`generate-xodr`；几何评估输出 RMSE、P95；声明地图面向仿真而非定位级 HD 地图，最终质量需目视检查 — [repo README](https://github.com/ftgTUGraz/opendrive-digital-twin-generator)

**scenariogeneration / pyodrx（MPL-2.0；PyPI 0.16.7，2026-10-08 发布）——生成器而非拟合器**
- 许可证 MPL-2.0（PyPI 元数据）— [PyPI scenariogeneration](https://pypi.org/project/scenariogeneration/0.16.7/)
- 几何由用户给 Line/Arc/Spiral/ParamPoly3 原语；`create_3cloths`、`create_cloth_arc_cloth` 生成回旋线组合；`AdjustablePlanview` “在两条固定道路之间拟合一段几何”；路口连接路在 `CommonJunctionCreator` 与 `create_junction_roads` 中用 `pyclothoids.SolveG2`（三段回旋线 G2 Hermite）生成 — wheel 内 `scenariogeneration/xodr/generators.py`、`junction_creator.py`、`geometry.py`

**hakuturu583 `hdmap_generator`（roadgen；Apache-2.0）——程序化生成器**
- 参考线为 line/arc/clothoid（导出 `<spiral>`）链；路口连接路为两端切向钉在所连车道上的三次曲线；道路间尖角用切圆弧替代；每个横断面变化输出一个 `<laneSection>` — [repo README](https://github.com/hakuturu583/hdmap_generator)；许可证 Apache-2.0 见 [LICENSE](https://github.com/hakuturu583/hdmap_generator/blob/main/LICENSE)

**MathWorks RoadRunner / Driving Scenario Designer（商业）**
- RoadRunner 支持 OpenDRIVE 1.4–1.8 的导入、可视化、导出（MathWorks 页面，搜索摘录）— [RoadRunner](https://www.mathworks.com/products/roadrunner.html)
- 从 HD Map 构建场景需要 RoadRunner Scene Builder 许可；无此许可只能导入并以节点/连线查看 — [getRoadRunnerHDMap](https://www.mathworks.com/help/driving/ref/drivingscenario.getroadrunnerhdmap.html)
- `roadrunnerLaneInfo` 从车道边界点生成 RoadRunner HD Map 车道信息 — [roadrunnerLaneInfo](https://www.mathworks.com/help/driving/ref/roadrunnerlaneinfo.html)
- 官方示例：稀疏点在急弯处会导致车道不准，需先上采样（Pikes Peak 示例）；激光雷达示例在写 HD Map 前用“平滑因子 + 多项式阶数”平滑噪声车道边界 — [Pikes Peak 示例](https://se.mathworks.com/help/map/build-pikes-peak-roadrunner-3d-scene-using-roadrunner-hd-map.html)、[激光雷达示例](https://www.mathworks.com/help/driving/ug/generate-roadrunner-hd-map-from-lidar-data-for-scenario-generation.html)
- Driving Scenario Designer 导出 OpenDRIVE（1.4/1.5/1.6）时，cubic polynomial 与 parametric cubic 几何被导出为 spiral，弯道会产生几何差异 — [drivingScenario.export](https://www.mathworks.com/help/driving/ref/drivingscenario.export.html)、[Driving Scenario Designer](https://www.mathworks.com/help/driving/ref/drivingscenariodesigner-app.html)
- 一条搜索摘要称 R2024a 起 `importScene` 有“smooth-fit road geometry，含 Tolerance 与 MaxDepth”选项；另一条针对性搜索未能确认该选项存在 — [importScene](https://www.mathworks.com/help/driving/ref/roadrunner.importscene.html)（**冲突/未证实**）

**Apollo**
- Apollo 的 OpenDRIVE 方言中车道 `<border>`、`<centerLine>` 下是 `<geometry><pointSet><point …/>` 折线，另有 `<sampleAssociates>` 给 leftWidth/rightWidth 采样宽度——即不做解析几何拟合 — [lanes_xml_parser.cc](https://github.com/ApolloAuto/apollo/blob/master/modules/map/hdmap/adapter/xml_parser/lanes_xml_parser.cc)、[util_xml_parser.cc](https://github.com/ApolloAuto/apollo/blob/master/modules/map/hdmap/adapter/xml_parser/util_xml_parser.cc)
- `imap` 工具只做 OpenDRIVE→Apollo；Apollo 问答（2024-05-09）称没有 Apollo→OpenDRIVE 转换工具；`hdmc` 把 Lanelet2 转成 Apollo 方言 OpenDRIVE — [Apollo 1000 questions](https://apollo-1000-questions.readthedocs.io/en/latest/answers/hdmap/004.html)、[hdmc on PyPI](https://pypi.org/project/hdmc)

**CARLA**
- `carla.Osm2Odr.convert(osm_data, settings)` 把 OSM 转 xodr，`Osm2OdrSettings` 可设 `default_lane_width`（默认 4.0 m）、`generate_traffic_lights`、`all_junctions_with_traffic_lights` 等；生成的道路在地图边界处突然终止 — [CARLA OSM 教程（ue4-dev 源）](https://github.com/carla-simulator/carla/blob/ue4-dev/Docs/tuto_G_openstreetmap.md)、[CARLA 0.9.11 文档](https://carla.readthedocs.io/en/0.9.11/tuto_G_openstreetmap/)

### Inferences
- 业界开源实现几乎都是“分类/切角 + 插值”的启发式；用最小二乘的两个（tier4、TU Graz）都是**无界的加权/普通最小二乘**——容差只用于告警，不作为约束；没有一个把“保真上限”作为硬约束、把光顺作为目标。mapforge 若改成显式约束优化，没有可直接照抄的成熟开源先例，但 tier4 的罚项权重经验（80/20 → 端点 1e4 时“端点亚毫米、内部偏移约 1 cm”，1e6 会把参考线推出走廊）是可借鉴的数值证据：罚项法对端点很容易做到“精确”，代价转移到内部。
- TU Graz 匝道做法（全局最小二乘三次 B 样条 → 每个节点区间精确换成一段 paramPoly3；边界横向偏移用同一组节点拟合）是“少段数 + C2 + 保真可调”的最简可行模板：段数 = 节点区间数，光顺由样条阶数保证，不需要密集短段。tier4 恰好相反：同样是光滑 B 样条，却按 1 m 切成最多 100 段 paramPoly3——这种输出正是 mapforge 主线所拒绝的“密集短段冒充平滑”。
- 宽度方面，工具普遍把宽度当作次要量（常数/两点线性/窗口中位数），这说明 mapforge 对“世界车道边缘”的要求明显高于这些工具的设计目标（仿真可视化/路由），工具输出不能当作质量标杆。
- SUMO 可在 EPL-2.0 下使用（双许可，不必按 GPL 处理）；CommonRoad Scenario Designer 是 GPL-3.0，按项目规则只能进程隔离或离线对拍。

### Gaps
- RoadRunner 的 HD Map 导入 → 道路重建 → OpenDRIVE 导出的实际拟合算法、容差、段数策略：官方文档未公开（本次也无法直接打开 mathworks.com），“smooth-fit Tolerance/MaxDepth”说法互相矛盾，未证实。
- CARLA 的 Osm2Odr 内部是否基于 SUMO netconvert：本次未找到可核实的一手来源（相关源码路径在 ue4-dev 分支 404）。
- tier4 转换器与 TU Graz 生成器的许可证未能确认（根目录无 LICENSE 文件）；Apollo（通常认为 Apache-2.0）、CARLA（通常认为 MIT）的许可证本次未逐一核实。
- 未找到任何工具公布“每路口运行时间”或大规模转换失败率，无法与 mapforge 的 15–60 min/路口、6/16 失败直接比较。
- OSM2XODR / osm2xodr 一类独立项目本次未检索到可读源码（GitHub 搜索 API 被会话策略禁用）。

## 问题 2：2015–2026 年关于自动生成 OpenDRIVE（HD 地图、点云、航拍、OSM）的论文：参考线/paramPoly3/宽度拟合方法、精度、分合流与路口处理

### Takeaway
公开论文多以**点云/移动测量**为源，几何拟合是“分段 + 每段最小二乘多项式（paramPoly3 或三次样条）”，几乎不讨论 G2 光顺、段数最少化或车道出生/消失的忠实度；报告精度在 **0.05–0.74 m RMSE** 量级（基准与口径各不相同，不可横比）。分合流常由外部拓扑（OSM、政府规范、人工）给出，路口/连接路多为后处理或半人工。直接以“车道级边界折线”为源、对 OpenDRIVE 车道几何做约束优化的论文本次未找到。

### Cited Findings
- **Eisemann & Maucher, “Automatic Odometry-Less OpenDRIVE Generation From Sparse Point Clouds”（IEEE ITSC 2023，arXiv 2405.07544）**：仅用点云，不需里程计/多传感器融合/机器学习/高精标定；RANSAC 去地面、按反射率筛标线点、DBSCAN 聚类标线 — [Semantic Scholar](https://www.semanticscholar.org/paper/Automatic-Odometry-Less-OpenDRIVE-Generation-From-Eisemann-Maucher/6a9fab05abb004342063f7dc54ae80c388a73b37)、[arXiv 2405.07544](https://arxiv.org/abs/2405.07544)。一条搜索摘录称其在局部 U/V 坐标中把标线拟合成 paramPoly3 式三次曲线，用 look-ahead/look-back 索引避免段间折角，自洽误差 0.20 m、相对 PEGASUS HD 地图 0.24 m；另一条针对性搜索未能复现这些细节（**未证实，需读全文**）— [arXiv PDF](https://arxiv.org/pdf/2405.07544)
- **Eisemann & Maucher, “Divide and Conquer: … Industrial Scale High-Definition OpenDRIVE Generation from Sparse Point Clouds”（IEEE IV 2024，arXiv 2407.18703，DOI 10.1109/IV55156.2024.10588602）**：用少量外部道路信息（OSM）把试验车数据切段分别处理再合并；参考线由提取的标线 + 政府规范 + OSM 估计；几何元素汇编成单个 OpenDRIVE，并定义车道关系与路口；以 PEGASUS 项目 HD 地图为精度基准。一条被搜索引擎标为“乱码”的摘录给出最近邻距离 0.243 m、σ 0.201 m、RMSE 0.337 m（**未证实**）— [arXiv 2407.18703](https://arxiv.org/abs/2407.18703)
- **Chiang, Pai, Zeng, Tsai, El-Sheimy, “Automated Modeling of Road Networks for High-Definition Maps in OpenDRIVE Format Using Mobile Mapping Measurements”（Geomatics 2(2):221–235, 2022）**：按强度从点云提取车道线，三次样条拟合后建 OpenDRIVE；宽度为三次多项式，拟合“所有点到参考线的最短距离”；摘要报告相对测绘公司已验证 HD 地图 2D/3D RMSE 0.069/0.079 m；ResearchGate 版本表格另有 0.045/0.062 m（**两版本数字冲突**）— [MDPI](https://www.mdpi.com/2673-7418/2/2/13)、[ResearchGate](https://www.researchgate.net/publication/361026548_Automated_Modeling_of_Road_Networks_for_High-Definition_Maps_in_OpenDRIVE_Format_Using_Mobile_Mapping_Measurements)
- **Zhao 等, “Automated Digital Twin Construction for Highway Scenarios Using LiDAR Point Clouds and OpenStreetMap”（arXiv 2606.16570，2026-06-15）**：主线由 LiDAR 重建（参考线、高程、车道边界），匝道几何与拓扑由 OSM 路网推断；六段真实高速序列整体平均横向 RMSE 0.740 m — [arXiv 2606.16570](https://arxiv.org/abs/2606.16570)。实现细节（窗口最小二乘 paramPoly3、匝道最小二乘样条、三次宽度）见问题 1 — [代码仓库](https://github.com/ftgTUGraz/opendrive-digital-twin-generator)
- **Gallazzi, Cudrano, Frosi, Mentasti, Matteucci, “Clothoidal Mapping of Road Line Markings for Autonomous Driving High-Definition Maps”（2022）**：单目视觉生成车道级 HD 地图，用**基于图的优化**同时拟合标线点并保证相邻回旋线间连续可导（G1），再用**迭代贪心删除不必要的回旋线**降低模型复杂度；已知位姿情形每段回旋线约 10 m；在赛道与城郊道路上验证（精度数字未取得）— [POLIMI 仓储](https://re.public.polimi.it/handle/11311/1221329)、[IEEE VT Magazine 2022-12](https://read.nxtbook.com/ieee/vehicular_technology/vehiculartechnology_dec_2022/clothoid_based_lane_level_hig.html)
- **Breggion 等, “High-definition road map generation from mobile mapping data: a case study on the Tangenziale di Napoli”（ISPRS Archives XLIX-B4-2026, 361–367）**：约 10 km 高速（双向、匝道、互通、隧道）用 GAIA M1 移动测量系统生成 OpenDRIVE；强调前处理，自动提取只作辅助，最终路网由栅格底图与分割要素人工装配 — [ISPRS Archives](https://isprs-archives.copernicus.org/articles/XLIX-B4-2026/361/2026/)
- **He, Tiwari, Al-Ghobari, Zhang, Rausch, “Geo-Data-Driven HD Map Generation Workflow with Integrated Reference-Free Constraint-Based Verification”（arXiv 2605.18921，2026-05-18）**：以下萨克森州 Basis-DLM（官方矢量 shapefile 道路中心线）为源生成 lanelet 式 HD 地图，并用取自道路设计规范的几何/拓扑/高程约束做可执行一致性检查（含缺陷注入实验）；输出是 lanelet，不是 OpenDRIVE — [arXiv 2605.18921](https://arxiv.org/abs/2605.18921)
- **RoadWeaver（Yueyuan Li 等，arXiv 2608.11580）**：从零程序化生成大规模车道级 HD 地图，报告可达性 99.8%、断头比 10.7%、端点对齐误差 0.24 m（称比 SOTA 降低 94.4%）；二手报道称可导出 OSM 与 OpenDRIVE — [arXiv 2608.11580](https://arxiv.org/pdf/2608.11580)、[Geoawesome 报道](https://geoawesome.com/roadweaver-generated-hd-maps-autonomous-driving/)
- **OpenTwinMap（Richardson & Sprinkle，Vanderbilt，arXiv 2511.21925，2025-11-26）**：Python 框架，先用 LiDAR 修正 OSM 道路中心线，再转 OpenDRIVE（车道、路口、交通要素），仍在开发中 — [arXiv 2511.21925](https://arxiv.org/abs/2511.21925v1)
- **“Realistic Road Generation: Intersections”（预印本，2022）**：程序化生成 OpenDRIVE 路口；辅助道路（无车道的参考线）由 spiral 与 arc 组成，连接路用参数三次曲线（paramPoly3） — [ResearchGate 预印本](https://www.researchgate.net/publication/360354961_P_r_e_-P_r_i_n_t_Realistic_Road_Generation_Intersections)
- **“Sparse road network model for autonomous navigation using clothoids”（IEEE T-ITS 23(2):885–898, 2022，DOI 10.1109/TITS.2020.3016620）**：关键词含 roundabouts、lane change、clothoids；段数/精度数字未取得 — [USP 仓储记录](https://repositorio.usp.br/item/003005010)
- 反方向参照：Althoff, Urban, Koschi（2018）OpenDRIVE→Lanelet 转换指出参考线由回旋线或多项式串接，转换得到的折线只是车道边界的近似 — [TUM mediaTUM PDF](https://mediatum.ub.tum.de/doc/1449005/299979664679.pdf)

### Inferences
- 论文报告的 0.07–0.74 m 级 RMSE 主要反映**感知/建图误差**，不是“给定矢量源后的拟合误差”；mapforge 的源是已矢量化的车道边界/中心点列，拟合误差应以厘米计，这些论文的精度数字不能当作 mapforge 的容差参考。
- 唯一明确讨论“模型复杂度 vs 拟合”的是 Gallazzi 等（图优化 + 贪心删段），其思路——先多段保证保真，再在保持误差的前提下删段——与“少段数、显式误差”目标一致，可作为参考线段数最少化的方法学先例。
- 以 shapefile 为源的 2605.18921 的价值在于“规范约束的可执行验证”，与 mapforge 评分板/门禁思路同类，但它不生成 OpenDRIVE 几何。

### Gaps
- 2405.07544、2407.18703、Chiang 2022 的拟合细节（段长、是否有连续约束、宽度多项式分段规则）与精度表未能读全文核实。
- 未找到以航拍图像直接生成 OpenDRIVE 车道几何、并报告几何精度的 2015–2026 一手论文（本次检索未命中）。
- 未找到中文期刊中“SHP/车道级矢量 → OpenDRIVE”拟合方法的可核实论文（搜索引擎仅限美区，覆盖不足）。

## 问题 3：laneSection 边界与车道出生/消失（taper）位置如何确定——来自拓扑还是几何？

### Takeaway
所有读过的工具都**由拓扑/源对象边界决定** laneSection 与出生/消失位置（lanelet 组、SUMO edge、OSM/规范切分点、用户给定 s 区间），没有一个按几何优化去移动断面位置。taper 的宽度形状要么是线性（SUMO 连接路、CommonRoad），要么是两端零斜率的三次（scenariogeneration、roadgen 平滑加宽），要么拆成新 road/合成路口（SUMO、tier4）。ASAM 规则只约束零宽车道的链接，不规定 taper 形状。

### Cited Findings
- **ASAM OpenDRIVE 规则（附录 E 规则表，镜像页，规则标注 1.7.0）**：车道在 laneSection 起点宽度为零时不得有 `<predecessor>`，终点为零时不得有 `<successor>`；相连车道在连接点必须宽度非零；突然分流/合流用多个前驱/后继；应避免长距离零宽车道 — [Rules list (mirror)](http://asam.domainfactory-kunde.de/opendrive/ASAM_OpenDRIVE_Specification/latest/specification/16_annexes/Rules_List/Rules.html)
- **ASAM OpenDRIVE 1.8.1 车道几何**：`<width>` 与 `<border>` 在同一车道组内互斥，同时存在时应用必须用 `<width>`；多项式变量变化时新建 `<border>` — [11.6 Lane geometry](https://publications.pages.asam.net/standards/ASAM_OpenDRIVE/ASAM_OpenDRIVE_Specification/v1.8.1/specification/11_lanes/11_06_lane_geometry.html)；规则表另称 border 不应与 laneOffset 同时存在（镜像页摘录，**措辞未核对正式 1.7 文本**）— [Rules list (mirror)](http://asam.domainfactory-kunde.de/opendrive/ASAM_OpenDRIVE_Specification/latest/specification/16_annexes/Rules_List/Rules.html)
- Althoff 等 2018：OpenDRIVE 中合流通常把车道宽度逐渐减到零、分流从零逐渐增宽；宽度为零、车道实际消失后同一车道 ID 仍会在下一 section 复用 — [TUM PDF](https://mediatum.ub.tum.de/doc/1449005/299979664679.pdf)
- **SUMO**：每条 edge 一个 laneSection、常数宽度；车道数变化即换 road（导出文档摘录）；连接路宽度 `b=(outWidth−inWidth)/length` 线性过渡，代码注释承认“理想情况下需要三次多项式……只知道首末宽度所以保持线性” — [NWWriter_OpenDrive.cpp](https://github.com/eclipse-sumo/sumo/blob/main/src/netwrite/NWWriter_OpenDrive.cpp)、[SUMO Networks/Export](https://sumo.dlr.de/docs/Networks/Export.html)
- **CommonRoad cr2odr**：一组横向相邻 lanelet = 一条 road = 一个 laneSection；lanelet 的前驱/后继分叉处另起 road/junction（`add_junction_linkage`：多后继且不属已有路口时新建 junction）— wheel 内 `cr2odr/converter.py`，[PyPI 0.8.5](https://pypi.org/project/commonroad-scenario-designer/0.8.5/)
- **tier4 转换器**：按横向邻接（routing graph 左右关系）DFS 分组成 road；分流/合流合成“divergence/merge junction”（issue #291）；同一 road 内并行 lanelet 必须等长，斜向切分的 lanelet 要先用 MovePoint 预处理对齐边界，否则长度错位、无法变道 — [conversion-process.md](https://github.com/tier4/autoware_lanelet2_to_opendrive/blob/master/autoware_lanelet2_to_opendrive/docs/conversion-process.md)
- **roadgen（hdmap_generator）**：车道数变化开始新横断面（新 laneSection）；延续车道自动连接，终止车道不连接（防止路由驶出）；也支持不改车道数的平滑收窄/加宽 — [repo README](https://github.com/hakuturu583/hdmap_generator)
- **scenariogeneration**：`create_lanes_merge_split` 在每个 LaneDef 的 `s_start` 处新建 laneSection；合流/分流车道宽度用 `get_coeffs_for_poly3`，文档写明“假设段首尾导数为 0”的三次多项式（从全宽到 0 或从 0 到全宽）— wheel 内 `xodr/lane_def.py`、`xodr/utils.py`，[PyPI 0.16.7](https://pypi.org/project/scenariogeneration/0.16.7/)
- **TU Graz 生成器**：laneSection 随 100 m 拟合窗口出现，每窗口车道数取有效标线数中位数；主线分流点由 `infer_mainline_split_points.py` 从数据推断 — [fit_mainline_phase2.py](https://github.com/ftgTUGraz/opendrive-digital-twin-generator/blob/main/analysis/fit_mainline_phase2.py)、[analysis 目录](https://github.com/ftgTUGraz/opendrive-digital-twin-generator/tree/main/analysis)
- **Eisemann & Maucher 2024**：分段位置由 OSM 信息决定（搜索摘录）— [arXiv 2407.18703](https://arxiv.org/abs/2407.18703)

### Inferences
- “laneSection/事件位置由拓扑早早定死”在业界是普遍做法而非 mapforge 独有；没有先例证明“按几何优化 section 位置”可行或有益。若要把出生/消失位置变成优化变量，需自行设计（例如把 section 边界 s 作为连续变量、宽度在其两侧分段），属于新做法。
- 关于 1:4 源 taper 被“内切 0.1–0.3 m”：用 scenariogeneration 式“两端零斜率三次”宽度去逼近线性 taper，最大偏差与 taper 长度无关，恒为全宽的约 9.6%（自行推导：f(u)=3u²−2u³−u 在 u=(3−√3)/6 处 |f|≈0.0962），即 3.0 m 车道约 0.29 m、3.5 m 车道约 0.34 m——与 mapforge 观察到的量级吻合。若改为“线性主段 + 两端短抛物/三次过渡”的多条 `<width>` 记录，过渡长 b、斜率变化 k 时最大偏差约 k·b/8（1:4 即 k=0.25：b=2 m → 约 6 cm，b=1 m → 约 3 cm），代价是过渡段边缘曲率约 k/b（b=2 m → 0.125 /m）。这是“保真 vs 边缘曲率”在 taper 处的显式权衡，宽度记录条数（不是 planView 段数）才是忠实度的自由度。
- SUMO 与 CommonRoad 的线性宽度过渡在两端产生宽度斜率突变（边缘航向折角）；它们不在意，mapforge 的 C2 要求下不可取。

### Gaps
- 未找到任何工具/论文报告 taper 处车道边缘相对源的偏差数字。
- ASAM 正式 1.7 文本中关于 border/laneOffset 互斥、零宽车道的原文未能直接核对（只看到镜像页与 1.8.1 页摘录）。

## 问题 4：路口连接路在这些生成工具中如何生成（与道路端的衔接、宽度、对道路端变化的依赖）

### Takeaway
开源生成工具的连接路有三种：**单段三次 Bézier=paramPoly3，两端切向钉住**（SUMO、roadgen；G1，端点曲率不匹配）、**三段回旋线 G2 Hermite**（scenariogeneration、CommonRoad 的 spiral 段；位置/航向/曲率都匹配）、**源中心线的加权最小二乘样条 + 端点大权重钉在相邻道路参考线端点**（tier4）。所有工具都是“先定道路、再从最终道路端生成连接路”，连接路几何是道路端的函数。

### Cited Findings
- SUMO：连接路参考线为进口车道内侧边界末端到出口车道内侧边界首端的单个 Bézier（`bezierControlPoints(begShape, endShape, turnaround, 25, 25, …)`），写成一个 paramPoly3；失败时退回直线并提示调 `junctions.scurve-stretch` 或加大路口半径；左转等情形改用外侧边界并加 laneOffset；宽度线性 — [NWWriter_OpenDrive.cpp](https://github.com/eclipse-sumo/sumo/blob/main/src/netwrite/NWWriter_OpenDrive.cpp)
- hdmap_generator issue #37：连接路 Bézier 原先被写成 `<line>` 弦链，因 OpenDRIVE 车道边缘垂直于每段弦，口部横断面相对道路旋转，车道边缘错位：T 形 90° 3.5 m 车道 0.21 m、十字 0.17 m、60° 斜交 0.50 m、4 路口 64 条连接路的网格镇 0.15 m（224 条边缘中 96 条 > 10 cm）；车道中心误差 < 1 mm（中心即参考线），原测试只查中心故漏检；修复为每条 Bézier 写一个 `<paramPoly3 pRange="normalized">`，测试扩展到比较车道边缘（PR #50 合并）— [Issue #37](https://github.com/hakuturu583/hdmap_generator/issues/37)、[PR #50](https://github.com/hakuturu583/hdmap_generator/pull/50)
- scenariogeneration：`CommonJunctionCreator` / `create_junction_roads` 用 `pyclothoids.SolveG2` 在两道路端之间生成三段 spiral — wheel 内 `xodr/junction_creator.py`、`xodr/generators.py`，[PyPI 0.16.7](https://pypi.org/project/scenariogeneration/0.16.7/)
- tier4：连接路由带 `turn_direction` 的 lanelet 构造（按空间重叠分组成 junction），首末点覆盖为相连道路参考线的世界坐标端点，硬约束权 1e4，测试容差 5 cm（见问题 1）— [reference_line.py](https://github.com/tier4/autoware_lanelet2_to_opendrive/blob/master/autoware_lanelet2_to_opendrive/src/autoware_lanelet2_to_opendrive/opendrive/reference_line.py)、[conversion-process.md](https://github.com/tier4/autoware_lanelet2_to_opendrive/blob/master/autoware_lanelet2_to_opendrive/docs/conversion-process.md)
- tier4 PR 讨论：Lanelet2 中心线与 OpenDRIVE 车道中心不重合，需要把位姿校正到所在侧车道中心；t 必须沿参考线法向而非位姿航向量取 — [PR #74](https://github.com/hakuturu583/autoware_lanelet2_to_opendrive/pull/74)、[PR #73](https://github.com/hakuturu583/autoware_lanelet2_to_opendrive/pull/73)

### Inferences
- “道路端变化时连接路变形”在这些工具里是设计使然（连接路是道路端的函数），它们靠“道路先定、连接路后生成、端点硬钉”的顺序回避；mapforge 若让道路端在优化中移动，必须把连接路与道路端放进同一轮求解，或在每次道路端变化后重新生成连接路——工具里没有前者的先例。
- issue #37 的数字说明：只检查车道中心会系统性漏掉口部边缘错位（0.15–0.5 m），印证 mapforge 评分板检查“口部边缘精确对齐”的必要性。

### Gaps
- 未找到工具对连接路做“贴近源导引线”的保真度量化（tier4 只有交叉验证告警，无公开数字）。

## 问题 5：是否有公开的“几何原语段数 vs 拟合精度”比较（少段数 vs 保真）？

### Takeaway
没有找到针对 OpenDRIVE 生成、系统报告“段数–误差”曲线的公开研究。工具里的段数完全由经验参数决定（tier4 约 1 m/段、上限 100；TU Graz 主线 100 m/段；SUMO 每拐角一段；CommonRoad 由 0.1°/0.01 阈值隐式决定），都不以误差上限反推段数。方法学上只有回旋线样条文献中的“误差权重/目标误差控制段数”“贪心删段”“惩罚过短段”可借鉴。

### Cited Findings
- tier4：段数 = min(ceil(L/1.0), floor(L/0.5))，夹在 [1, 100] — [geometry.py](https://github.com/tier4/autoware_lanelet2_to_opendrive/blob/master/autoware_lanelet2_to_opendrive/src/autoware_lanelet2_to_opendrive/opendrive/geometry.py)
- TU Graz：主线窗口 100 m；匝道段数 = 自适应节点区间数（失败时 50 m 均匀节点）— [fit_mainline_phase2.py](https://github.com/ftgTUGraz/opendrive-digital-twin-generator/blob/main/analysis/fit_mainline_phase2.py)、[ramp_fitter.py](https://github.com/ftgTUGraz/opendrive-digital-twin-generator/blob/main/analysis/ramp_fitter.py)
- Gallazzi 等 2022：图优化拟合后迭代贪心删除不必要的回旋线以降低复杂度（约 10 m/段）— [POLIMI](https://re.public.polimi.it/handle/11311/1221329)
- McCrae & Singh, “Sketching Piecewise Clothoid Curves”（SBIM 2008，**早于 2015**）：用户可通过直接指定误差代价或目标拟合误差来偏好更多/更少段；过短回旋线段被惩罚，因其损害光顺 — [PDF](https://www.dgp.toronto.edu/~mccrae/projects/clothoid/sbim2008mccrae.pdf)
- 回旋线自由度：单段回旋线不足以做 G2 Hermite 插值，需至少两到三段（搜索摘要，转述相关文献）— [Optimal Smooth Paths Based on Clothoids](https://eprints.whiterose.ac.uk/id/eprint/165251/8/final%20ijcas_draft_tsedl-2020-09-22.pdf)

### Inferences
- “段数 vs 保真”在已发表 OpenDRIVE 工作中是空白，mapforge 若做“在误差上限约束下最少段数”（或“固定段数下最小误差”）的 Pareto 曲线，需要自己在评分板/泛化集上实测；可借鉴的算子是“先密后删（贪心合并，验误差）”与“误差上限下二分/自适应节点”。

### Gaps
- 未找到大地测量/道路平面线形反演（horizontal alignment reconstruction）领域 2015–2026 的“元素数 vs 偏差”定量研究（检索命中的多为 2015 年前或专利）。

## 问题 6：许可证（能否直接复用代码，还是只能进程隔离/离线对拍）

### Takeaway
可直接作为库或参考实现吸收的：SUMO（EPL-2.0 选项）、scenariogeneration（MPL-2.0，文件级 copyleft）、roadgen（Apache-2.0）；只能进程隔离/离线对拍的：CommonRoad Scenario Designer（GPL-3.0）；许可证不明、应视为不可复用代码的：tier4 转换器、TU Graz 生成器；RoadRunner 为商业软件（HD Map 建场景另需 Scene Builder 许可）。

### Cited Findings
- SUMO：`EPL-2.0 OR GPL-2.0-or-later` — [NWWriter_OpenDrive.cpp](https://github.com/eclipse-sumo/sumo/blob/main/src/netwrite/NWWriter_OpenDrive.cpp)
- CommonRoad Scenario Designer 0.8.5：GPL-3.0 — [PyPI](https://pypi.org/project/commonroad-scenario-designer/0.8.5/)
- scenariogeneration 0.16.7：MPL-2.0 — [PyPI](https://pypi.org/project/scenariogeneration/0.16.7/)
- hdmap_generator / roadgen：Apache-2.0 — [LICENSE](https://github.com/hakuturu583/hdmap_generator/blob/main/LICENSE)
- tier4 autoware_lanelet2_to_opendrive：仓库根与包目录未找到 LICENSE，README 未注明 — [repo](https://github.com/tier4/autoware_lanelet2_to_opendrive)
- ftgTUGraz opendrive-digital-twin-generator：根目录无 LICENSE（raw 404），README 未注明 — [repo](https://github.com/ftgTUGraz/opendrive-digital-twin-generator)
- RoadRunner：HD Map 构建场景需 Scene Builder 许可 — [getRoadRunnerHDMap](https://www.mathworks.com/help/driving/ref/drivingscenario.getroadrunnerhdmap.html)

### Inferences
- 对 mapforge 有价值的“方法”（加权/约束最小二乘 B 样条、节点区间→paramPoly3 精确换算、零斜率三次宽度过渡、三段回旋线 G2 连接）都是教科书级算法，可在 numpy/scipy（BSD）上自写，无需引入上述任何项目代码；GPL 的 CommonRoad 只适合作离线对拍基线。

### Gaps
- Apollo、CARLA、pyclothoids 的许可证本次未逐一核实。
