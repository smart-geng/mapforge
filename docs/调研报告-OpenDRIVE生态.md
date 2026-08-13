# OpenDRIVE 解析/写出/验证生态调研报告

> 调研代理产出，2026-08-13。所有条目均经实际网页核查，未核实处已标注"未证实"。本报告是《地图格式转换工厂-首批三格式方案》第 2.1/3/6 节的事实依据。

---

## 1. 版本现状（ASAM 官网核查）

**当前最新正式版：ASAM OpenDRIVE 1.9.0，发布于 2026-05-19**，文件格式支持 `xodr` 与 `xodrz`（压缩格式）。来源：https://www.asam.net/standards/detail/opendrive/

历史版本（来源：https://www.asam.net/standards/detail/opendrive/older/ ）：

| 版本 | 发布日期 | 一句话要点 |
|---|---|---|
| 1.9.0 | 2026-05-19 | 当前最新正式版；官网明确列出 xodr/xodrz 双格式（逐项变更清单在官网版本页未展示，未证实） |
| 1.8.1 | 2024-11-21 | 1.8 的维护修订版（具体变更官网页未展示，未证实） |
| 1.8.0 | 2023-11-22 | 上一个大版本；ASAM qc-opendrive 检查器当前支持上限即 1.8.0 |
| 1.7.0 | 2021-08-03 | 随版发布 User Guide 与 "OpenDRIVE Concept 2.0" 文档，是社区工具（如 scenariogeneration）最常对齐的版本 |
| 1.6.0/1.6.1 | 2020-03/2021-03 | ASAM 接手后首个正式版系列（2018-09 由 VIRES/Daimler 移交 ASAM） |
| 1.4/1.5 | 2015-11/2019-02 | VIRES 时代版本，仍是 CARLA/多数开源解析器的事实兼容基线 |

**ASAM Quality Checker 框架**（来源：https://www.asam.net/standards/asam-quality-checker/ ）：

| 名称 | URL | 许可证 | 语言 | 支持版本 | 最近活跃 | 成熟度/结论 | 可承担角色 |
|---|---|---|---|---|---|---|---|
| asam-ev/qc-framework | https://github.com/asam-ev/qc-framework | MPL-2.0 | C++/Python（checker 可任意语言） | 面向 OpenDRIVE/OpenSCENARIO 等 | 246 commits，23 星 | 官方框架，含 ReportGUI，结果合并为统一 XML | 转换工厂的**输出质检门禁**框架 |
| asam-ev/qc-opendrive | https://github.com/asam-ev/qc-opendrive | MPL-2.0 | Python | OpenDRIVE **1.5.0–1.8.0** | 最近 push 2026-03，17 星，PyPI: `asam-qc-opendrive` | 22 个 checker：XML schema、车道连接、junction connection/linkage、几何多项式等 | **写出 xodr 后的自动验证**（pip 可装，可当库调用） |

---

## 2. 解析/写出库核查

| 名称 | URL | 许可证 | 语言 | 支持版本 | 最近活跃 | 成熟度/结论 | 可承担角色 |
|---|---|---|---|---|---|---|---|
| libOpenDRIVE (pageldev) | https://github.com/pageldev/libOpenDRIVE | Apache-2.0 | C++（零依赖） | 目标 1.4（README 自述） | push 2026-08-09，505 星 | 轻量解析+3D mesh 生成+**RoutingGraph（lane-level shortest_path）**；0.4 起官方移除 WASM/Python 绑定 | 核心解析引擎（需自建 pybind11 绑定或用 pyOpenDRIVE） |
| esmini（RoadManager） | https://github.com/esmini/esmini | MPL-2.0 | C++，有 Python 绑定 | v2.56.0 已支持 OpenDRIVE 1.8 的 19 种 signal 类型（issue #592） | push 2026-08-12，937 星，3900+ commits | OpenSCENARIO 播放器；**RoadManager 以 esminiRMLib 独立库形式提供**，可脱离场景引擎单独用于路网查询 | 路网几何/坐标查询引擎（s,t↔xyz），验证参照实现 |
| pyxodr (driskai) | https://github.com/driskai/pyxodr | MIT | 纯 Python | 未明确标注版本 | push 2025-06-30，25 星，PyPI: `pip install pyxodr` | 将道路/车道解成 (x,y,z) 坐标数组；**明确不支持**：superelevation/road shape、路面、路标、junction group、objects、signals、railroad | 轻量几何提取（不适合做完整拓扑/要素处理） |
| scenariogeneration (pyoscx) | https://github.com/pyoscx/scenariogeneration | MPL-2.0 | Python | xodr 模块基于 **OpenDRIVE 1.7.1** | 最新版 0.16.6 发布 **2026-07-02**，379 星 | 官方定位"生成"：basic roads、junctions、signals、objects；**不做解析/读入**，覆盖范围见 xodr_coverage.txt | **xodr 写出器**（Python 侧唯一活跃维护的 writer） |
| imap (daohu527) | https://github.com/daohu527/imap | Apache-2.0 | Python | OpenDRIVE→Apollo 转换 + 可视化 | push 2026-07-30，262 星，PyPI: `imap_box` | `imap -f -i town.xodr -o apollo_map.txt` 可转 Apollo 地图并出转换报告；signalReference 关联、lane-object overlap 等仍标 TODO | OpenDRIVE→Apollo 支线转换器 + 快速可视化排查 |
| CommonRoad Scenario Designer | https://github.com/CommonRoad/commonroad-scenario-designer | **GPL-3.0（传染性，注意）** | Python | OpenDRIVE↔CommonRoad、OpenDRIVE→Lanelet2、SUMO、OSM | push 2025-10-23，87 星，v0.8.5 BETA | 转换矩阵最全的学术工具（TUM），含地图校验修复与 GUI | 参考实现/离线交叉验证；**GPL 使其不宜进商业核心链路** |
| Apollo OpendriveAdapter | https://github.com/ApolloAuto/apollo/tree/master/modules/map/hdmap/adapter | Apache-2.0（Apollo 整体） | C++ | **解析的是 Apollo 修改版 OpenDRIVE，非标准版**：边界用经纬度点列而非参数曲线、要求车道中心线/边界点列、junction 简化、扩展 crosswalk 等元素（issue #8045；https://towardsdatascience.com/how-baidu-apollo-builds-hd-high-definition-maps-for-autonomous-vehicles-167af3a3fea3/ ） | Apollo 主线维护 | 含 xml_parser/、opendrive_adapter.cc、proto_organizer.cc | 仅当目标是 Apollo 生态时相关；标准 xodr 需先经 imap 类工具转换 |
| pyOpenDRIVE (Nova-UTD) | https://github.com/Nova-UTD/pyOpenDRIVE | Apache-2.0 | Cython 包 libOpenDRIVE | 同 libOpenDRIVE | push 2025-04-27，仅 7 星、22 commits | libOpenDRIVE 的 Python 封装，维护弱 | 可作自建绑定的起点，不建议直接依赖 |
| odrviewer.io | https://odrviewer.io/ | 未公开 | Web | — | 在线可用 | libOpenDRIVE README 官方引用的在线查看器 | 人工快速目检 xodr |
| CARLA libcarla | https://github.com/carla-simulator/carla | MIT（libcarla） | C++/Python | 见第 3 节 | 0.9.16（2025-09-16） | OpenDRIVE 解析内嵌于 libcarla（Map/Waypoint/Junction API） | 见第 3、5 节 |

---

## 3. 消费端支持矩阵（官方文档核查）

| 工具 | OpenDRIVE 支持 | 来源 |
|---|---|---|
| **CARLA**（最新正式版 0.9.16，2025-09-16 发布） | OpenDRIVE standalone mode 可直接摄入 xodr 并程序化生成 3D mesh（"文件里的问题会原样传导进仿真，复杂 junction 尤甚"）；`client.generate_opendrive_world()` 支持传字符串直接建世界；具体支持的 OpenDRIVE 版本号文档未标注 | https://carla.readthedocs.io/en/latest/adv_opendrive/ |
| CARLA OSM2ODR | Python API `Osm2Odr`/`Osm2OdrSettings` 把 .osm 转 .xodr；限制：红绿灯信息看地区数据质量、建议 wall_height=0、道路在图边界截断；源码位于 carla/Util/OSM2ODR | https://carla.readthedocs.io/en/latest/tuto_G_openstreetmap/ |
| **SUMO netconvert** | 导入：1.4 基本全支持，1.4/1.5/1.6 特性支持跟踪中；参数曲线按精度采样为折线；导入 driving/stop/parking 等车道类型（默认不导 sidewalk/border/shoulder）；红绿灯默认导入；**也支持导出 OpenDRIVE**（复杂信号组可能导出失败） | https://sumo.dlr.de/docs/Networks/Import/OpenDRIVE.html |
| **MathWorks RoadRunner** | "ASAM OpenDRIVE **1.4 – 1.8** 的导入、可视化与导出"（产品页原文）；导出对话框含多仿真器兼容选项与 Export Preview 工具；GIS 建路：可导入航拍影像、高程、激光点云、roadmaps 作参考建路，支持 OSM、Zenrin、HERE HD Live Map（Scene Builder）、Lanelet2、OpenCRG | https://www.mathworks.com/products/roadrunner.html ；https://www.mathworks.com/help/roadrunner/import-scene-data.html |
| VTD（Hexagon/VIRES） | OpenDRIVE 原生（VIRES 即 OpenDRIVE 创始方） | https://nexus.hexagon.com/home/product/virtual-test-drive/ |
| 51Sim-One | 支持标准 OpenDrive **1.4** 导入自动建景，WorldEditor 支持 xodr 导入/导出/二次编辑 | https://blog.csdn.net/Sim_One515151/article/details/117825930 ；https://simone-docs.51sim.com/01_overview/Overview.html |
| PanoSim | 支持 .osm/.xodr/net.xml 路网导入 | https://www.panosim.com/ |

---

## 4. OpenDRIVE ↔ GIS

| 名称 | URL | 许可证 | 语言 | 结论 | 可承担角色 |
|---|---|---|---|---|---|
| **GDAL OpenDRIVE 矢量驱动**（DLR-TS） | https://github.com/DLR-TS/gdal-opendrive-how-to | GDAL 主体 MIT/X | C++（基于 **libOpenDRIVE**） | **GDAL 3.10 起官方内置**；一条命令 `ogr2ogr -f "GPKG"/"GeoJSON"/"ESRI Shapefile" out in.xodr` 即可转任意 OGR 格式 | **OpenDRIVE→SHP/GeoJSON 的首选官方通道**（FOSS4G Europe 2024 报告：https://talks.osgeo.org/foss4g-europe-2024/talk/SD7SGV/ ） |
| QGIS 插件 odrviewer | https://github.com/Danaozhong/odrviewer ；https://plugins.qgis.org/plugins/odrviewer/ | MIT | Python | 按 **OpenDRIVE 1.8.1** 规范开发；早期阶段：只画参考线/道路几何，**不支持 junction、objects、signals** | QGIS 内目检（轻量） |
| ad_map_access QGIS 插件 | https://github.com/carla-simulator/map | MIT | C++/Python | 可在 QGIS 直接加载 xodr | 备选 QGIS 通道 |
| virtualcitySYSTEMS/opendriveconverter | https://github.com/virtualcitySYSTEMS/opendriveconverter | MIT | Java 17 | xodr→GeoJSON 命令行工具；push 2024-11，4 星 | 备选（低活跃） |
| Safe FME | https://community.safe.com/ideas/new-reader-writer-opendrive-28563 | — | — | **FME 无原生 OpenDRIVE 读写器**（社区 idea 状态），官方建议走 CityGML 中转或用 Plug-in SDK 自研 | 不可依赖 |

---

## 5. 交叉口 lane-level 拓扑提取能力（决定 V2X MAP 生成）

能把 `junction/connection/laneLink`（incoming lane → connecting road → outgoing lane）暴露为 API 的：

| 库 | 暴露方式 | 评价 |
|---|---|---|
| **libOpenDRIVE** | `RoutingGraph`：以 (road id, lanesection s, lane id) 为 lane key 的有向图，含 junction connecting road 关系，支持 `shortest_path` | **最直接的拓扑 API**；C++，需绑定 |
| **CARLA (libcarla)** | `Map.get_topology()` 返回全图 lane 首尾 waypoint 对；`Junction.get_waypoints(lane_type)` 返回**交叉口内每条 lane 的起止 waypoint 对**；`Waypoint.next/previous/get_junction` | API 语义与 V2X MAP 的 ingress/egress-connection 高度同构，但要拉起 CARLA server，重 |
| esmini RoadManager (esminiRMLib) | 独立路网库+Python 绑定，road/lane/junction 查询接口（junction laneLink 级 API 细粒度未逐项核实） | 轻量替补/交叉验证 |
| pyxodr | 解析 junction connection（junction group 不支持） | 纯 Python 但要素覆盖薄 |
| scenariogeneration | **写出侧**：junction 与车道连接生成（基于 1.7.1） | 用于输出 xodr，不是提取 |
| CommonRoad Scenario Designer | OpenDRIVE→Lanelet2 转换保留车道邻接/后继关系 | GPL-3.0，仅离线参照 |
| qc-opendrive | junction connection/linkage 一致性**检查**（非提取） | 质检环节 |
| 原始 XML 兜底 | `laneLink` 是纯 XML 结构，lxml 直读即可拿到连接三元组；难点在几何（connecting road 的中心线采样）而非拓扑本身 | 自研成本可控 |

**OpenDRIVE→V2X MAP 直达转换器：检索未发现现成开源项目**（搜索 J2735 MAP 生态得到的是 usdot V2X-Hub——从信号机/交叉口配置产 MAP 而非从 OpenDRIVE；Mcity 只提供 MAP 样例）。"opendrive junction topology extraction/intersection graph" 检索命中的多为感知类工作（OpenLane-V2 等），无可直接复用的提取库——**这一空白正是转换工厂的价值点**。

---

## 本方向调研结论（要点）

1. **OpenDRIVE 最新正式版为 1.9.0（2026-05-19 发布）**，但整个工具生态实际停留在 1.4–1.8：qc-opendrive 支持到 1.8.0，RoadRunner 1.4–1.8，scenariogeneration 对齐 1.7.1，CARLA/SUMO 事实基线 1.4。**工厂输出定 1.7/1.8 最稳，1.9 暂不作目标**。
2. 解析侧没有"全能库"：**libOpenDRIVE（Apache-2.0，2026-08 仍活跃）是唯一同时具备成熟解析+lane-level RoutingGraph 的独立库**，但官方无 Python 绑定（Nova-UTD 的 Cython 封装弱维护）。
3. 纯 Python 解析器（driskai/pyxodr）覆盖薄（无 objects/signals/junction group），只够几何提取，不足以支撑完整转换工厂。
4. **写出侧 Python 生态只有 scenariogeneration 一家活跃**（0.16.6，2026-07 发布，MPL-2.0），支持 junction 生成，基于 1.7.1——SHP→OpenDRIVE 方向的落笔工具。
5. **验证链路现成可用**：pip 装 `asam-qc-opendrive`（MPL-2.0）+ qc-framework，覆盖 schema/车道连接/junction linkage 检查，直接做输出门禁。
6. **OpenDRIVE→GIS 已被 GDAL 3.10+ 官方解决**（内置 libOpenDRIVE 驱动，ogr2ogr 一步转 SHP/GeoJSON/GPKG）；FME 反而没有原生支持。QGIS 有 odrviewer 插件（仅几何目检）。
7. **交叉口 laneLink 拓扑**：libOpenDRIVE RoutingGraph 与 CARLA `Junction.get_waypoints()` 语义最接近 V2X MAP 需求；但 laneLink 本身是简单 XML 结构，lxml 自研提取 + libOpenDRIVE/esmini 做几何采样是务实路线。
8. **未发现任何现成 OpenDRIVE→J2735/ISO MAP 消息转换器**（开源检索为空）——该模块必须自研，也是差异化价值。
9. 许可证雷区：CommonRoad Scenario Designer 是转换矩阵最全的参照实现但 **GPL-3.0**，只可离线对拍不可链入产品；其余关键件均为 Apache/MIT/MPL，商用友好。
10. Apollo 生态注意：Apollo 用的是**修改版 OpenDRIVE**（点列边界、简化 junction），标准 xodr 不能直喂 Apollo，需 imap（Apache-2.0，2026-07 活跃）中转。

### 推荐 Python 优先选型组合

- **解析/拓扑核心**：libOpenDRIVE（自建 pybind11 绑定，参考 Nova-UTD/pyOpenDRIVE）；拓扑兜底用 lxml 直读 `junction/connection/laneLink`
- **几何交叉验证**：esmini RoadManager（esminiRMLib Python 绑定）
- **xodr 写出**：scenariogeneration.xodr（SHP→OpenDRIVE 方向）
- **输出质检**：asam-qc-opendrive + qc-framework（CI 门禁）
- **GIS 互转**：GDAL ≥3.10 的 OpenDRIVE 驱动（OpenDRIVE→SHP/GeoJSON 免开发）
- **V2X MAP 生成**：自研模块（laneLink 拓扑 + libOpenDRIVE 几何采样 → node/link/lane-connection），无现成轮子
- **目检**：odrviewer.io + QGIS odrviewer 插件
