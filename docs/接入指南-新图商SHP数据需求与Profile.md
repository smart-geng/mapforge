# 接入指南：新图商 SHP 的数据需求与 Profile 机制

> 2026-08-13。回答两个实操问题：**① 别家图商 SHP 字段不一样怎么办；② SHP 生成 OpenDRIVE 到底需要哪些数据（中心线？边界？）**。
> 结论基于金凤 IBD 交付实战与 7 路口直转实测；字段事实以《资料盘点》为准。

---

## 一、别家图商字段不一样：Profile 机制

SHP 本身没有统一 schema——"车道级 SHP"只是壳，每家图商的图层划分、字段名、单位、编码都不同。方案的应对（3.2 节）是 **Schema Profile**：一家图商一份版本化映射模板，内核代码不写死任何图商字段。

**Profile 引擎已实装（2026-08-13，`adapters/shp/profile_source.py`）**：一份 YAML 声明图层文件名/字段名/单位/编码/几何路线，用户自己改字段映射即可接新图商，内核零改动。两套车道几何路线都支持：

- **路线 A（LANE_LINK 等价物）**：车道中心线 + 宽度字段（`geometry: field`）；
- **路线 B（LANE_BOUNDARY 等价物）**：边界线 + 车道↔边界左右关系（`geometry: boundaries`——中心线=左右边界中线合成；宽度=边界横距）；
- 宽度按 `width_from` 阶梯获取：`field → boundaries → spacing（相邻车道间距）→ default`，非 field 来源记 APPROXIMATED（`derivation_stats` 逐来源计数）。

引擎可信度证据：用 `profiles/shp/ibd-smarteditor-v1.yaml` 跑 node4 直转，与内置硬编码读取器产物**去时间戳后逐字节全等**（pytest 锁死）；`ibd-boundary-demo.yaml` 故意屏蔽 WIDTH 字段全走边界推导，恒宽车道误差中位 14mm。

### 用户操作流（三条命令）

```bash
# 1. 生成带注释的映射模板，按对方交付改字段名
python -m mapforge.cli profile-init my-vendor.yaml

# 2. 映射体检：图层/字段存在性、值抽样、宽度量级、降级预告（缺层会预告转换时的行为）
python -m mapforge.cli profile-check my-vendor.yaml <shp目录>

# 3. 带 Profile 转换（xodr / map 目标通用）
python -m mapforge.cli convert <shp目录> --to xodr --profile my-vendor.yaml --at 106.51,29.60
```

仍开放的正式化项：字段别名词典与自动推断向导（有第二家真实交付后训练词典才有意义）。

### 接入一家新图商的四步流程

| 步骤 | 做什么 | 工具/产出 |
|---|---|---|
| 1. 盘图层 | 列全 shapefile 清单、字段名、值域抽样、单位探测（长度是米还是毫米？速度 km/h？） | 图层清单表（参照《资料盘点》二.3 的格式） |
| 2. 对词典 | 把对方字段映射到下表的"最小概念集"；IBD 字段作参照列 | 新 Profile 草案 |
| 3. 核验 | 编码（GBK/UTF-8）、CRS 真伪、**姿态字段可信度**（逐字段与几何回算对拍） | 字段核验报告；不可信字段降级不消费 |
| 4. 试转对拍 | 先转 1 个路口 → GeoJSON 叠底图人工确认 → 有同路口其它格式数据则做交叉对拍 → 批量 | 试转报告 + 黄金用例 |

### IBD 实战教训（新图商核验清单）

| 教训 | 实例 | 对新图商的检查项 |
|---|---|---|
| 编码声明不可信 | IBD DBF LDID=0x57 实为 GBK | 抽中文字段实读验证 |
| 单位不统一 | IBD 长度全系毫米；同名字段跨图层单位不一致（CENTRE_X） | 每图层独立探测单位 |
| 字段名截断 | DBF 10 字符上限：STOPLINE_R（本名更长） | 按前缀匹配 + 词典别名 |
| **CRS 声称≠事实** | IBD .prj 自定义 WKT 无 EPSG，"声称 WGS84" | 核验前 `crs_integrity=suspect` **禁止生产转换**（硬约束 3） |
| **姿态字段可能是假的** | IBD CURVATURE 与几何不相关（相关系数中位 −0.005）；HEADING 可用 | 逐字段几何回算对拍，不通过则不消费 |
| 宽度字段有零值脏数据 | IBD 上游链节 6 条车道 WIDTH=0（且存在同 LANE_PID 重复记录）——宽度阶梯从边界横距救回真值（如 4272mm） | width_from 阶梯把 boundaries 排在 default 之前 |
| 数值字段可能带小数 | 上游链节 S_WIDTH='4272.0'（DBF N 型带小数位，pyshp 返回 float），`int(str)` 直接解析会吞成 0 | 数值解析一律 `int(float(...))` 容错 |
| 拓扑分层有玄机 | IBD 路口内连接车道不在 MERGE 层，在普通层的路口内 Link 上 | 拓扑表两跳展开实测验证，勿按文档想当然 |

---

## 二、SHP→OpenDRIVE 需要哪些数据

**核心回答：中心线+宽度 与 边界线 二选一，有其一即可转；再加"路口面 + 车道级接续拓扑"才能出完整 junction。** 高程、标线是增强项，缺了照样转、明确声明即可。

### 数据需求分层表（xodr 要素 ↔ 需要什么 ↔ 缺失时行为）

| xodr 要素 | 需要的数据 | IBD 来源（实证） | 缺失时行为 |
|---|---|---|---|
| 参考线 planView | **道路中心线**（稀疏折线也行） | IBD_ROADCENTER（2–13 点/段，拟合器带升级档处理 5° 微折角） | 用中间车道中心线替代（已实装 fallback） |
| lane + width | **每车道中心线 + 宽度** *或* **车道边界线** | IBD_LANE_LINK（WIDTH 毫米，逐车道） | 有边界⇒宽度=左右边界横距；两者都无⇒默认 3.5m 记 APPROXIMATED |
| laneOffset | 车道相对中心线的横向位置 | **无需字段**——车道线对中心线投影实测（金凤实测车道对称跨中心线 ±1.75/±5.25m） | 自动计算 |
| junction 连接路 | **路口内转弯车道几何** | 路口内 Link 的 LANE_LINK（TOPO 两跳的中间车道，7 路口 213 条全有实测几何） | SolveG2 clothoid 合成，记 INFERRED |
| connection/laneLink | **车道级接续拓扑** | IBD_LANE_TOPO_DETAIL（IN_PID→OUT_PID 显式表） | 几何推断（角度+横向对齐），记 INFERRED 进复核队列 |
| 路口范围与归组 | 路口面 + 进出道路清单 | IBD_OBJECT_INTERSECTION_SURFACE（ENTER_ROAD/LEAVE_ROAD） | 由 Link 端点聚类推断路口 |
| 道路拼链 | 路段衔接关系（或同名+端点衔接） | ROADNAME + 端点匹配（S/E_NODE_PID 备用） | 只转路口邻接的短段 |
| elevationProfile | 逐点高程或坡度 | IBD SLOPE/BANKING **未核验，暂不消费** | 平面输出并在报告声明 |
| roadMark | 标线虚实/颜色/材质 | IBD_MARKLINK（未消费，正式化项） | 不写 roadMark |
| 限速/类型 | 车道限速、车道类型 | MAX_SPEED、LANE_TYPE（枚举表待索取） | 不写 speed；类型统一 driving |

### 新图商最低门槛（最小可转集）

1. **车道几何**：车道中心线+每车道宽度，或车道左右边界线（二选一）；
2. **道路归组**：哪些车道属于同一条路、车道横向排序（字段直给或几何可推）；
3. **路口**：路口面 or 可推断的路口位置 + 进出道路关系；
4. **接续拓扑**：车道级最好；没有则退化为几何推断（转弯关系进人工复核）；
5. **CRS 必须明示且可核验**——这是唯一没有降级路线的硬门槛（硬约束 3）。

> 只有边界线没有中心线的图商（常见形态）：中心线=左右边界中线合成、宽度=边界横距逐点采样。架构已预留（reader 输出统一的"车道记录"），合成代码接入首家此类图商时实装。

### 可推导性实测（2026-08-13，金凤 node4 真实数据）

"中心线+宽度"并非都要图商直给——**真正不可再降的只有"某种车道级几何"（中心线或边界线，二选一）+ 归组 + CRS**，其余能推：

| 要素 | 必需？ | 推导路线 | 金凤实测精度 |
|---|---|---|---|
| 道路中心线 | **否** | 车道线合成（当前实装：中间车道替代；正式化：车道组横向均线） | ROADCENTER 缺失 fallback 已生效 |
| 每车道宽度 | **否**（有边界或 ≥2 车道时） | ① 左右边界横距（LANE_BOUNDARY_REL 的 SIDE=1/2 定左右） | 恒宽车道误差中位 **1.4cm**；变宽车道（口部展宽 3.9→3.5m 渐变）推导值=几何中位宽，与名义字段差 23–59cm 属数据真相非误差 |
| | | ② 相邻车道中心线间距 ≈ (Wa+Wb)/2 | 误差中位 **1.4cm**、最大 10.9cm；边缘车道推不出，取邻档或默认 |
| laneOffset | 否 | 本来就是实测投影计算，无需字段 | — |
| 路口位置 | 否（有更好） | Link 端点聚类推断 | 金凤有 INTERSECTION_SURFACE 直读 |
| 车道中心线 | **二选一** | 有边界线时=左右边界中线合成 | 边界数据齐备（15301 条） |

### shp_0222-0326 图层满足情况（54 个 .shp 实测）

| 类别 | 图层 | 满足的需求 |
|---|---|---|
| **直转管道现役（6 层）** | IBD_OBJECT_INTERSECTION_SURFACE | 路口面 + ENTER/LEAVE_ROAD 归组 |
| | IBD_ROADLINK | 道路归组、路名（拼链键）、车道数 |
| | IBD_ROADCENTER | 道路中心线（参考线首选源） |
| | IBD_LANE_LINK | **车道中心线 + WIDTH/S_WIDTH/E_WIDTH**（核心层） |
| | IBD_LANE_TOPO_DETAIL | 车道级接续（junction 连接关系） |
| | IBD_LANE_LINK_MERGE | 加载备用（金凤路口内连接实际在普通层的路口内 Link 上） |
| **替代/升级备选** | IBD_LANE_BOUNDARY + _REL | 宽度推导①/中心线合成的数据源（SIDE=1/2 左右） |
| | IBD_POSITION / IBD_LANE_POSITION | 逐形点密集坐标+HEADING（参考线密度升级备选；CURVATURE 已证不可信） |
| | IBD_ROADLINK_TEMPLATE | 分段断面（将来多 laneSection 的数据源） |
| **增强（未消费）** | IBD_MARKLINK | roadMark 虚实/颜色（正式化项） |
| | IBD_NODE_SLOPE / 姿态字段 | 高程/超高（字段未核验，暂不进 elevation） |
| | IBD_OBJECT_STOPLINE / TRAFFIC_LIGHTS / ARROW 等 | MAP 侧要素（停止线/信号灯/转向箭头），xodr signals 扩展备选 |
| | 其余 OBJECT_*、_NODE 辅助层、VERSION | 对象/节点/版本信息，PASSTHROUGH 语义 |

---

## 三、SHP→xodr 的两条路径（2026-08-13 修正）

历史问题：CLI 曾把 SHP→xodr 借道 **SHP→MAP→xodr** 实现——MAP 是空口消息（附录 D 抽稀、单宽度、无边界无拓扑承载），拿它当中间表示等于把 SHP 的富数据先倒进窄漏斗再抢救，产物是无 junction 的孤立 road。**已修正为直转**（`ops/shp_to_xodr.py`），经 MAP 的路径只保留给"源本来就是 MAP"的还原场景。

| | **直转 SHP→xodr（现行）** | 经 MAP（仅限 MAP 源还原） |
|---|---|---|
| 参考线 | ROADCENTER 中心线拼链（~160m）+ 全密度拟合 | Link 线（附录 D 抽稀后） |
| 车道宽 | 每车道真实宽（3.45/3.5/3.65 各保留） | 单一平均宽 |
| junction | **完整 `<junction>` + 连接路实测几何 + laneLink + 前驱后继** | 无 |
| 进出口 | 进口+出口路全建 | 仅进口（MAP 只有 inLinks） |
| 定位 | `--like 现网XML` 或 `--at lon,lat` | — |

### 金凤 7 路口直转实测（2026-08-13）

| 路口 | 连接路（全实测几何） | connections | 拟合偏差峰值 | XSD 1.5M | 连续性 |
|---|---|---|---|---|---|
| node3 | 32 | 32 | 0.31m | PASS | 0 违例 |
| node4 | 24 | 24 | 0.30m | PASS | 0 违例 |
| node5 | 18 | 18 | 0.32m | PASS | 0 违例 |
| node13 | 29（4 进 5 出） | 29 | 0.33m | PASS | 0 违例 |
| node16 | 18 | 18 | 0.38m | PASS | 0 违例 |
| node17 | 32 | 32 | 0.29m | PASS | 0 违例 |
| node18 | 36 | 36 | 0.30m | PASS | 0 违例 |

已知简化（HANDOFF 正式化清单）：S_WIDTH/E_WIDTH 变宽未写（路口口部常量宽误差 ≤0.4m）、高程平面、单 laneSection、标线未消费。

```bash
python -m mapforge.cli convert shp_0222-0326 --to xodr --at 106.51,29.60
```
