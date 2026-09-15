# 参考文献：参考线拟合与 OpenDRIVE 自动生成

> 整理日期：2026-08-13。服务方案 5.7（参考线拟合与平滑度）与方向 5/6（SHP/MAP→OpenDRIVE）的实现。
> 全部条目经在线检索验证真实存在（标题/作者/出处/许可证），验证方式随条注明；未能核实的一律不列。

## 一、Clothoid（回旋线）拟合与插值 —— spiral 段的理论与代码基础

| 文献/库 | 出处 | 对实现的用处 |
|---|---|---|
| Bertolazzi & Frego (2015)《G1 fitting with clothoids》 | Mathematical Methods in the Applied Sciences, DOI: 10.1002/mma.3114 | G1 Hermite 两点带切向插值回旋线的核心算法（三非线性方程组化为单变量求根，解存在唯一）——OpenDRIVE spiral 段生成的直接理论基础 |
| Bertolazzi & Frego (2013)《Fast and accurate G1 fitting of clothoid curves》 | arXiv:1305.6644（开放获取版，可下载全文实现） | 上文的免费全文版本 |
| Bertolazzi & Frego (2018)《On the G2 Hermite interpolation problem with clothoids》 | J. Computational and Applied Mathematics 341, DOI: 10.1016/j.cam.2018.03.029 | G2（曲率连续）两点插值——精修档相邻段曲率连续 |
| Bertolazzi & Frego (2018)《Interpolating clothoid splines with curvature continuity》 | Math. Methods Appl. Sci., DOI: 10.1002/mma.4700 | 过点列构造 G2 连续 clothoid spline——"折线→整条曲率连续参考线"的成套方案 |
| **Clothoids（C++ 库）** | github.com/ebertolazzi/Clothoids，**BSD-2-Clause**（license.txt 已核） | 实现上述全部论文：G1/G2 拟合、clothoid list（line+arc+spiral 混合）、biarc、求交、点投影；活跃维护 |
| **pyclothoids（Python 封装）** | pypi.org/project/pyclothoids（v0.2.0），github.com/phillipd94/pyclothoids，**MIT**（LICENSE 已核） | Clothoid 类含 G1Hermite 求解，开箱即用；底层即 ebertolazzi/Clothoids |

**许可证结论：全链 BSD-2/MIT，无 GPL 传染，符合方案硬约束 7，可作 mapforge 直接依赖。**

## 二、Arc spline / biarc 逼近 —— line/arc 段数最少化

| 文献 | 出处 | 对实现的用处 |
|---|---|---|
| Maier (2014)《Optimal arc spline approximation》 | Computer Aided Geometric Design 31(5):211-226, DOI: 10.1016/j.cagd.2014.02.011 | 给定容差内**段数最少**的 G1 圆弧样条逼近点列；论文动机之一就是"从道路标线测量点生成数字地图"——与"参考线段数最少、避免碎段"目标完全对口 |
| Maier, Schindler, Janda, Brummer (2013)《Optimal Arc-Spline Approximation with Detecting Straight Sections》 | ICCSA 2013 (LNCS), DOI: 10.1007/978-3-642-39643-4_8 | 在最优圆弧样条中显式检测直线段——正是 line 与 arc 必须区分的场景 |
| Bertolazzi & Frego (2019)《A Note on Robust Biarc Computation》 | CAD&A 16(5):822-835（cad-journal.net 免费全文；arXiv:1711.00935） | 数值稳健的 biarc 构造，arc 段 G1 连接原语 |
| Bertolazzi, Frego, Biral (2021)《Interpolating splines of biarcs from a sequence of planar points》 | CAD&A 18(1):66-85（cad-journal.net 免费全文） | **直接就是"平面点列→biarc 样条"**——折线→纯 line/arc 参考线的成套算法 |

## 三、平滑样条与防过拟合 —— "逼近而非插值"的理论与工具

| 文献/工具 | 出处 | 对实现的用处 |
|---|---|---|
| Reinsch (1967)《Smoothing by Spline Functions》 | Numerische Mathematik 10:177-183, DOI: 10.1007/BF02162161（网上有免费扫描版） | 惩罚平滑样条奠基论文，scipy 平滑样条的算法源头；"容差 S 约束下最光滑曲线"直接映射拟合器的容差参数 |
| de Boor (1978/2001)《A Practical Guide to Splines》 | Springer AMS 27 | 样条理论权威教材：B 样条、平滑与最小二乘逼近、平滑参数选择依据 |
| scipy `make_smoothing_spline` | docs.scipy.org（官方文档已核） | GCV 自动选平滑参数 λ——**防过拟合首选入口**（引用 Wahba 1990 等） |
| scipy `UnivariateSpline` / `splprep`(`make_splprep`) | docs.scipy.org（官方文档已核） | 平滑因子 s 语义（s=0 即插值，禁用）；splprep 做二维参数化曲线（弦长参数化）预平滑；paramPoly3 兜底段可由三次 B 样条系数直接转出 |

## 四、道路平面线形重建 —— 直线/圆曲线/缓和曲线分段识别（"符合实际"的学术基础）

| 文献 | 出处 | 对实现的用处 |
|---|---|---|
| **Camacho-Torregrosa, Pérez-Zuriaga, Campoy-Ungría, García, Tarko (2015)《Use of Heading Direction for Recreating the Horizontal Alignment of an Existing Road》** | Computer-Aided Civil and Infrastructure Eng. 30(4):282-299, DOI: 10.1111/mice.12094 | **分段识别的最佳骨架算法**：用弧长-航向角图代替曲率图——直线=水平线、圆曲线=斜线、缓和曲线=抛物线，最小化航向均方误差、免人工阈值、解唯一；航向图比曲率图噪声低一阶 |
| Ai & Tsai (2015)《Automatic Horizontal Curve Identification and Measurement Method Using GPS Data》 | J. Transportation Eng. 141(2):04014078, DOI: 10.1061/(ASCE)TE.1943-5436.0000740 | 从 GPS 点列识别全部曲线类型（含 spiral）并测半径；385 条合成曲线识别率 90.1%、类型分类 87.3%——**分段识别环节的可对标基线指标** |
| Bartin, Demiroluk, Ozbay, Jami (2022)《Automatic Identification of Roadway Horizontal Alignment…: CurvS Tool》 | Transportation Research Record 2676(1):532-543, DOI: 10.1177/03611981211036364 | 面向 **GIS 路网中心线**（正是 SHP 折线这类输入）的自动线形提取工程化流程 |
| Bartin, Jami, Ozbay (2023)《Estimating Roadway Horizontal Alignment from GIS Data: An ANN-Based Approach》 | J. Surveying Eng. 149(4), DOI: 10.1061/jsued2.sueng-1439 | 两步法（先估段数/类型/分段点，再逐段拟参数）；明确"段数与分段点先验未知"这一核心难点的问题建模 |

## 五、自动生成 OpenDRIVE 的先例项目与 junction 几何生成

### 5.1 先例项目/论文（→xodr）

| 项目/论文 | 来源与许可证 | 参考线几何路线 | 对我们的意义 |
|---|---|---|---|
| CommonRoad Scenario Designer (crdesigner) | TUM Althoff 组，github.com/commonroad/commonroad-scenario-designer，**GPL-3.0**；ITSC 2021 论文 | 以 lanelet 折线边界为中间表示，OpenDRIVE↔CommonRoad 双向 | **第三方对拍器**（roundtrip 验证）；GPL 须按硬约束 7 进程隔离，v0.8.5（2025-09）活跃 |
| SUMO netconvert xodr 导出 | Eclipse SUMO/DLR，EPL-2.0 | **line + paramPoly3 折线直出**，junction 靠 `--junctions.scurve-stretch` 补救 | 反面印证：折线直出是 OSM 系转换器质量口碑差的公认根源 |
| CARLA carla.Osm2Odr | CARLA（MIT）内嵌 SUMO fork（EPL-2.0） | 即 netconvert 编译进 CARLA，折线直出+默认车道参数 | 同上 |
| Eisemann & Maucher 点云两部曲 | ITSC 2023（arXiv:2405.07544）、IV 2024（arXiv:2407.18703），无代码 | 点云分段处理→车道要素→分段生成 xodr 再拼接 | "分段拟合→拼接"工程可扩展性的学术验证 |
| Chiang 等 MMS→xodr | MDPI Geomatics 2022，无代码 | MMS 点云/影像→车道矢量→xodr，RMSE 2D 6.9 cm | 精度基准参照 |
| osm2xodr / osm-to-xodr / osm2odr / osm2opendrive | 个人项目（GPL-3.0 / netconvert 薄包装 / 许可证不明） | 折线直出 | 仅说明生态现状，不采用 |
| DeepAerialMapper | Krajewski & Kim，GPL-3.0，arXiv:2410.00769 | 航拍语义分割→**Lanelet2**（非 xodr） | 邻近参照 |
| Althoff 组 2018《Automatic Conversion of Road Networks from OpenDRIVE to Lanelets》 | ResearchGate | — | 背景依据：明文"参考线由回旋线/多项式串接，回旋线曲率随弧长线性变化最贴合实际道路" |

### 5.2 junction 几何生成（connecting road）

| 实现 | 来源与许可证 | 机制（已核实） | 对我们的意义 |
|---|---|---|---|
| **scenariogeneration `CommonJunctionCreator`** | pyoscx，**MPL-2.0**，junction_creator.py 源码抓取证实 | 连接路几何 = `pyclothoids.SolveG2(起点位姿, 终点位姿)` → **三段 Spiral 串接**，共线退化为 Line；另有 `create_3cloths`/`AdjustablePlanview` | **金矿**：我们已选的写出库自带"进出口位姿→G2 连续连接路"，junction 内 connecting road 直接复用，不必自研 |
| SUMO junction 内部车道 | DLR 官方文档 + 开发者邮件列表 | 内部车道折线从**三次贝塞尔样条采样**，密度 `--junctions.internal-link-detail`、平滑度 `--junctions.scurve-stretch` 可调 | 两个可抄思想：参数曲线采样密度可调、junction 平滑度做成参数 |
| **JunctionArt** | UCSC Augmented Design Lab，github.com/AugmentedDesignLab/junction-art，**MPL-2.0**；SAE IJ CAV 2023 + IV 2023 环岛（arXiv:2303.17900） | 控制线+入射道路位姿驱动，自动生成 3–7 路交叉口与环岛及内部连接车道（Line/Arc/Spiral/ParamPoly 原语），与 CARLA/esmini/RoadRunner 互通 | ops/ 的 junction 生成最佳参考实现，许可证友好 |

### 5.3 空白点确认（三轮英文 + 一轮中文检索）

- **V2X MAP 消息（J2735/CSAE 53 体系）→ OpenDRIVE：公开生态零先例**。相邻工作全是反方向（USDOT ISD Message Creator：航拍底图人工描画→输出 MAP/RGA）或不碰几何（V2X-Hub 只做播发/翻译）。本地 xml2xodr 参考代码可能是该方向唯一参照物——**方向 6 具有首创性**（与方案 6.1 空白点结论相互印证）。
- 点云/测绘系"生成 xodr"论文普遍只发论文不放代码（Eisemann 两篇、Chiang 均无仓库）。
- Lanelet2→OpenDRIVE 直连稀缺，唯一可用链是 crdesigner 经 CommonRoad 中转（GPL 随行）。

## 六、实现路线映射（文献 → 拟合器流水线）

```text
折线点列
  → ① 预平滑（scipy make_smoothing_spline / splprep，GCV 定权衡；Reinsch 容差语义）
  → ② 弧长-航向角图分段识别（Camacho-Torregrosa 2015；对标 Ai&Tsai 识别率指标）
       直线=水平线 / 圆弧=斜线 / 缓和曲线=抛物线
  → ③ 逐段参数求解
       默认档：line/arc（Maier 2014 最优弧样条思想：容差内段数最少）
       精修档：spiral 段用 pyclothoids 的 G1Hermite（Bertolazzi-Frego 2015）
               需曲率连续处用 G2 插值（Bertolazzi-Frego 2018）
       兜底：paramPoly3 ← scipy B 样条系数直转
  → ④ 验收：κ(s) 对比源实测曲率（IBD）/ 离散曲率（MAP）+ 横向偏差带 + planview 连续性检查
```

**三条最有实现价值的组合**：① pyclothoids(MIT)+Clothoids(BSD-2) 省去自研 Fresnel 积分层，spiral 段直接得 curvStart/curvEnd/length；② Camacho-Torregrosa 航向图分段法与之互补成完整流水线；③ Maier 最优性框架 + scipy GCV 预平滑解决"精度-段数-平滑"三方权衡。

## 七、2026-09-11补充：联合拟合与形状保持的边界

Cudrano等，*Clothoid-Based Lane-Level High-Definition Maps: Unifying Sensing and Control Models*，IEEE Vehicular Technology Magazine，2022年12月。期刊全文确认：以线标记为观测进行图优化，再删除不必要原语，结果为**G1**连续样条。本项目借鉴联合模型/压缩思路，但其G1和加权误差不能替代我们硬性G2、真实速度、多车道正宽及来源门禁。[期刊全文](https://read.nxtbook.com/ieee/vehicular_technology/vehiculartechnology_dec_2022/clothoid_based_lane_level_hig.html)

v1.47真实对照进一步表明：自由3回旋线可以贴近点列，却在长直段产生额外波浪；增加Line优先和直线保护比单纯多加原语更符合形状意图。两处分合流仍在原60km/h条件下拒绝，**不是采用上述文献后整图已通过**。代码、图和限制见[完整源路径与直线保护复核](复核补充-完整源路径少原语与直线保护-2026-09-11.md)。本轮不复用论文附带代码。

此前章节属于历史调研，不作为当前完成声明；实际writer和默认paramPoly3政策以最新总方案/Profile为准，“未检索到先例”也不能据此证明全世界不存在先例或首创。

### v1.51核对：应优化事件/弧长，而不是只调固定结点的系数

Zhao、Farrell，*Optimization-based Road Curve Fitting*，CDC-ECC 2011，5293–5298，DOI `10.1109/CDC.2011.6161024`。[会议论文镜像](https://folk.ntnu.no/skoge/prost/proceedings/cdc-ecc-2011/data/papers/1492.pdf)。检索所得论文内容描述将弧长、曲率切换和数据拟合联立，利用L1正则促进曲率变化稀疏。镜像直读未成功，本轮非全文逐页核验。其分段常曲率不是本项目G2成品；借鉴的是可变站位和稀疏切换的建模思路，不照搬最终曲线或声称会自动解决当前路网。

McCrae、Singh，*Sketching piecewise clothoid curves*，Computers & Graphics 33(4)，2009，452–461，DOI `10.1016/j.cag.2009.05.006`。[期刊页](https://www.sciencedirect.com/science/article/pii/S0097849309000843)、[作者项目与2008会议版本](https://www.dgp.toronto.edu/~mccrae/projects/clothoid/)。期刊检索摘要确认G2回旋线拟合与误差控制，作者页明确适用概念设计、形状公平性优先于精确插值；两版本不可混作同一书目。不能把视觉公平性当0.35m硬来源、真实速度或OpenDRIVE读回的替代。只核查算法思想，未引入未核许可证的作者代码。

上述方法仅用于后续候选建模选择。v1.51真实固定轴长三次联合块仍被拒绝；完整证据和规范边界见[实施计划](实施计划-整路联动重建与编辑闭环.md)，本轮没有新XODR。

### v1.52核对：长过渡的自由度与目标格式的表示能力分开

*Planar G2 transition with a fair Pythagorean hodograph quintic curve*，Journal of Computational and Applied Mathematics 138(1)，2002，109–126，DOI `10.1016/S0377-0427(01)00359-4`。[期刊页](https://www.sciencedirect.com/science/article/pii/S0377042701003594)。期刊摘要确认五次PH过渡/G2/曲率形态控制；未逐页复现论文算法。本轮借鉴提高长过渡自由度的思路，采用普通五次B样条联合边界，**不是PH曲线，不继承其特殊弧长和offset性质**，未复用作者代码。

多项式/B样条/Bernstein运算参考[SciPy官方文档](https://docs.scipy.org/doc/scipy/tutorial/interpolate/splines_and_polynomials.html)，以独立BPoly和极值负例对拍。原46长区间未缩短，内部保持C2。它仅解决该固定三次模型的几何不可行，不等于源速度动态或OpenDRIVE可写出：width/border仍三次，后续编译误差不能突破总0.35m来源预算。

[CommonRoad官方OpenDRIVE转换说明](https://commonroad-scenario-designer.readthedocs.io/en/latest/details/open_drive/)包含分合流零宽处通行lanelet边界重建说明；这是反向转换，不能据此修改本项目源点或拓扑。v1.52仅区分通行/物理关系，不调用或引入其GPL实现。最终真实边界子块可行、原60km/h动态仍失败，详见实施计划，仍无新XODR。

### v1.56核对：线性化失败与原问题不可行须分开

[Stanford SNOPT官方约束不可行性处理](https://web.stanford.edu/group/SOL/software/snoptHelp/Description_of_method/Treatment_of_constraint_infeasibilities.htm)描述保持线性约束、对非线性约束采用弹性处理；[UCSD官方SNOPT](https://ccom.ucsd.edu/~optimizers/solvers/snopt/)提供SQP说明及论文目录。本轮读取官方说明，只借鉴硬线性/弹性非线性的分层思想：项目采用最大超限LP和第二层平顺QP，不是SNOPT算法复现，也未安装或复用其代码。

本项目的弹性只在搜索中间态存在，来源/宽度/结构保持硬约束，最终端点/G2/动态仍须满足原门禁。NODE5/node16可恢复一部分残差但最终拒绝，不代表原地图全局无解；node13对照复现旧候选，没有新增合格地图。具体实现、数值问题和下步轴/事件联合边界见[复核记录](复核补充-联合可行性恢复与来源参考轴-2026-09-11.md)。
