# SHP / MAP → OpenDRIVE 道路形态重建 v2 评审

> 日期：2026-08-21。结论：v1.29 虽通过位置、接缝和消费端门禁，但**道路形态不合格**，不得作为完成状态。本文给出基于 ASAM 规范和道路重建文献的新建模裁决。

## 1. 用户截图揭示的真实问题

node4 的 odrviewer 全景图可见四类失败：

1. 普通道路外缘出现不符合道路设计线形的周期性鼓包和收缩；
2. 车道线在 laneSection 交界附近蛇形摆动，局部形成反向弯折；
3. 车道增加/消失被“零宽车道 + 全路肩补洞”组合成多个楔形和多余标线；
4. 路口铺面遮住了接缝，但不能证明道路横断面和车道边界正确。

旧门禁只检查位置误差、C0/C1 接缝、XSD 与 esmini 可加载。最近距离 P95 即使小于 0.5m，也可能让目标曲线在来源曲线两侧来回摆动，因此“数值通过但看起来像蛇”并不矛盾。必须增加切向、曲率、形态复杂度和渲染级门禁。

## 2. 规范与文献裁决

### 2.1 ASAM OpenDRIVE 的直接结论

- 参考线应无跳跃且不应有折角；它是所有车道几何的基础：[ASAM OpenDRIVE 9.2 Road reference line](https://publications.pages.asam.net/standards/ASAM_OpenDRIVE/ASAM_OpenDRIVE_Specification/1.8.0/specification/09_geometries/09_02_road_reference_line.html)。
- laneSection 只应在车道数量或功能变化时建立，不应被当作每 10m 的横断面采样容器：[ASAM OpenDRIVE 11.3 Lane sections](https://publications.pages.asam.net/standards/ASAM_OpenDRIVE/ASAM_OpenDRIVE_Specification/1.8.0/specification/11_lanes/11_03_lane_sections.html)。
- 对自动测量获得的边界，规范明确指出 `<border>` 比累加 `<width>` 更容易表达，而且能避免大量 laneSection；`border` 是相对参考线的**绝对外边界**：[ASAM OpenDRIVE 11.6 Lane geometry](https://publications.pages.asam.net/standards/ASAM_OpenDRIVE/ASAM_OpenDRIVE_Specification/v1.8.1/specification/11_lanes/11_06_lane_geometry.html)。
- 同一 lane group 中 `width` 与 `border` 互斥，使用 `border` 时不得再使用 `laneOffset`；边界不得穿过内侧车道。
- lane 增减应以宽度从零逐渐增加或减至零表达，而不是在道路外侧增加贯穿全路的补洞车道。Althoff 等对 OpenDRIVE 车道合流的解释也采用这一模型：[Automatic Conversion of Road Networks from OpenDRIVE to Lanelets](https://mediatum.ub.tum.de/doc/1449005/document.pdf)。
- junction connecting road 应像普通 road 一样建模，并与相连车道平滑衔接：[ASAM OpenDRIVE 12.4 Connecting roads](https://publications.pages.asam.net/standards/ASAM_OpenDRIVE/ASAM_OpenDRIVE_Specification/v1.8.1/specification/12_junctions/12_04_connecting_roads.html)。

### 2.2 道路重建文献的共同方法

- Boyko 与 Funkhouser 将道路建模为同时优化中心线位置和宽度的 **ribbon snake**，目标函数同时包含数据贴合与内部平滑项；这比逐条边界独立插值更适合道路这种带状对象：[Extracting roads from dense point clouds in large scale urban environment](https://doi.org/10.1016/j.isprsjprs.2011.09.009)。
- Garach、de Oña、Pasadas 先以样条恢复道路整体曲率，再识别直线、圆曲线和缓和曲线；在约 1500km 道路上验证，说明应先恢复整体线形，再分解 OpenDRIVE 原语：[DOI 10.1016/j.autcon.2014.07.002](https://doi.org/10.1016/j.autcon.2014.07.002)。
- Eisemann 与 Maucher 指出 OpenDRIVE 质量首先取决于参考线；其流程是先匹配全部车道标线、估计到参考线的偏移，再由所有测量共同生成连续参考线，对超阈值段转人工复核，而不是逐边界硬插值：[Divide and Conquer: Industrial Scale OpenDRIVE Generation](https://agents4ad.github.io/assets/cvpr2024/papers/19.pdf)。
- Pai 等从移动测量车道线生成 OpenDRIVE，报告约 6.9cm 平面精度；其流程同样是先分类/配对车道线，再建模道路，而非把每条 SHP 折线直接变成独立多项式：[DOI 10.5194/isprs-archives-XLIII-B1-2022-263-2022](https://doi.org/10.5194/isprs-archives-XLIII-B1-2022-263-2022)。
- 回旋线 G1/G2 拟合仍用于参考线和连接路，而不是用于掩盖横断面错误：Bertolazzi–Frego，[Fast and accurate G1 fitting of clothoid curves](https://arxiv.org/abs/1305.6644)。

## 3. 新建模裁决

### 3.1 先建立共享边界图，不逐车道各算一遍

将 `LANE_BOUNDARY_REL` 中同一物理边界合并为一个 `BoundaryTrack`。每条车道引用 `inner_boundary_id` 和 `outer_boundary_id`；相邻车道共享同一对象。来源中心线、字段宽度只作为软约束或缺失边界时的回退，不能再独立生成一套互相冲突的边缘。

### 3.2 参考线采用全局带状优化

对每个 road leg 联合优化参考线 `r(s)` 与横断面边界 `b_i(s)`：

```text
min  Σ ρ(distance(source boundary, reconstructed boundary))
   + λr ∫ κ_r(s)^2 ds + μr ∫ κ'_r(s)^2 ds
   + λb Σ∫ b_i''(s)^2 ds
```

约束包括：端点位姿、共享边界一致、`b_i(s)` 顺序不交叉、有效车道最小宽度、车道生灭端宽为零，以及连接路端点 G2。`ρ` 使用 Huber/soft-L1，避免单个脏点把整条道路拉弯。

优化后再将 `r(s)` 分解为尽量少的 line / arc / spiral；若误差预算内无法分解，才使用 paramPoly3 作为有记录的兜底。

### 3.3 SHP 边界路线改用 OpenDRIVE `border`

当真实边界充分时：

- 先把参考线放到合法的 lane-0 分界位置；
- 每条 lane 写相对参考线的绝对外边界 `<border>`；
- 禁止同时写 `laneOffset`；
- laneSection 仅出现在车道数、类型或拓扑变化点；边界多项式变化只新增 `border sOffset`；
- 不再生成贯穿全段的 `physical-edge-fill` shoulder。

只有“车道中心线 + 宽度”Profile 才继续使用 `<width>`；它与边界路线是两个明确的 writer 策略，不能混在同一条 road 上。

### 3.4 车道生灭采用拓扑事件和有限过渡段

先从来源数据识别 birth/death/merge/split 事件，再确定过渡区间。过渡函数使用端点一阶、二阶导数为零的 quintic smoothstep，宽度从 0 到目标宽度；若来源未提供足够长度，则标 `INFERRED` 并受最小过渡长度门禁。不得用外侧 shoulder 为错误的 driving-lane 堆叠补面积。

### 3.5 MAP 路线保持同一质量模型但区分证据等级

MAP 只有车道中心点列时，先联合拟合共享参考线与车道中心偏移，再由相邻中心估计共享边界；真实出口优先，缺出口按同一进口断面严格左右镜像。推断边界全部标 `INFERRED`，但仍必须通过相同的切向、曲率和渲染门禁。

## 4. 新验收体系

除现有 XSD、拓扑、接缝、esmini 门禁外，新增：

1. **位置**：双向 P50/P95/Hausdorff，并分开统计真实区与推断过渡区；
2. **方向**：配对边界切向角 P95，不允许近距离但反复穿越；
3. **曲率**：边界和车道中心的 `|κ|`、`|dκ/ds|`、符号翻转次数；普通直路不得出现周期性 S 摆；
4. **复杂度**：每 100m 的 planView 原语、laneSection、border knot 数上限；
5. **横断面**：边界顺序、最小宽度、共享边界一致性、birth/death 单调性；
6. **路面形态**：来源道路面与 xodr 道路面的 IoU、孔洞数、连通分量、局部轮廓距离；
7. **渲染**：odrviewer 固定全景 + 东西南北局部截图，自动保存并要求人工目检通过。数值门禁不能替代截图。

## 5. 实施顺序

1. writer/reader/validator 增加 `border` 支持，并用 ASAM 示例和 esmini 做独立 spike；
2. 建立 `BoundaryTrack` 共享边界图和真实边界可用性报告；
3. 完成单 leg 的 ribbon/constrained-spline 优化，不含 junction；
4. node4 四条普通路通过形态门禁和截图后，再接回 junction；
5. 批量回归 7 个 SHP 路口；
6. 将同一平滑/形态门禁应用到 MAP→xodr；
7. 最后才恢复“完成”状态。

## 6. 当前状态修正

v1.29 应记为“位置/接缝闭环实验通过，但视觉形态失败”。之前把目标标为 complete 属错误判定。新的完成条件是第 4 节全部通过，并由 odrviewer 截图确认道路外缘和车道线均符合实际道路形态。
