# mapforge 交接文档

> 版本：2026-08-14 · 对应方案 v1.22 · 首次完整交接
> 读完本文即可独立接手：知道项目做什么、代码怎么组织、质量怎么保证、哪里还没做完。
> 逐轮开发流水见 [HANDOFF.md](../HANDOFF.md)，设计决策的完整论证见
> [地图格式转换工厂-首批三格式方案.md](地图格式转换工厂-首批三格式方案.md)。

---

## 1. 项目是什么

面向车路协同与自动驾驶的**语义地图编译工厂**：源格式 → 目标格式，每次转换强制输出
质量报告、损失报告、ID 映射与来源清单。**交叉口是第一公民**。

首批三格式与当前成熟度：

| 方向 | 状态 | 入口 |
|---|---|---|
| SHP(IBD) → OpenDRIVE | ★ 主线，7/7 路口全绿 | `ops/shp_to_xodr.py` |
| V2X MAP XML → OpenDRIVE | ★ 主线，7/7 路口全绿 | `ops/map_to_xodr.py` |
| SHP(IBD) → V2X MAP | 可用（M1 已对拍） | `ops/shp_to_map.py` |
| OpenDRIVE → V2X MAP | 可用（Town03 验证） | `ops/junction_to_map.py` |

统一 CLI：`python -m mapforge.cli convert <输入> --to <目标>`。

---

## 2. 五分钟上手

```bash
python -m venv .venv
.venv/Scripts/pip install numpy scipy pyshp lxml pyclothoids pycrate typer pyyaml pytest matplotlib shapely
.venv/Scripts/python -m pytest tests -q          # 36 项，含金凤黄金回归
.venv/Scripts/python scripts/closed_loop.py      # 全流程闭环七门禁（约 3 分钟）
```

数据放置（只读原料，不入库）：IBD SHP 交付放 `shp_0222-0326/`；现网 MAP XML 放
`v2x_map_xml/`；esmini 官方二进制解压到 `esmini/`（验证用，见 §5）。

常用命令：

```bash
.venv/Scripts/python scripts/gen_all.py                    # 再生成金凤 14 个 xodr
.venv/Scripts/python scripts/closed_loop.py                # 七门禁跑分（验收闸门）
.venv/Scripts/python scripts/visual_sweep.py               # 14 文件 × 4 机位截帧拼图
.\esmini\bin\odrviewer.exe --odr out\direct_xodr\node4.xodr --density 2 --ground_plane
```

---

## 3. 关键设计决策（改代码前必读）

这些是**踩过坑换来的**，推翻前请先看方案文档里对应轮次的论证。

1. **MapNode 不得客串富格式间的中间表示。** SHP→xodr 必须直转。曾经借道
   SHP→MAP→xodr，产物受空口消息窄门限制（附录 D 抽稀、单一均宽、无 junction）。
2. **双侧 leg road 是规范模型。** 一条街一条 road：参考线沿进口幅，进口车道挂右侧
   （-1..-n），对向出口挂左侧（+1..+n，行车沿 s 递减），两幅间写 median 车道，
   中线双黄。出口连接路接同一条 road 的 **END** 接触点。不要退回"每方向一条独立 road"。
3. **自研 writer，不用 scenariogeneration。** 后者无车道 `<speed>`、凭空发空
   elevationProfile、车道衔接自动推断与显式 laneLink 冲突。
   `adapters/opendrive/writer.py` 每个元素/属性/顺序对照 OpenDRIVE_1.5M.xsd。
4. **G2 是无条件承诺。** 参考线段间曲率断差必须为 0（`weld_g2` 兜底焊平）。
   G1 阶跃 = 侧向加速度阶跃 = 方向盘瞬时打角。
5. **段长不是判据，曲率蛇行才是。** G2 的短 clothoid 链几何上是平滑的；真正伤害
   规划/控制的是 sharpness(dκ/ds) 反复变号与侧向 jerk 超标。用
   `smoothness.curvature_audit` 度量，不要凭段数拍脑袋。
6. **偏差始终对原始顶点报告。** 平滑/去噪/精简都是修复手段，不是新的真值。
7. **硬约束（来自 CLAUDE.md，不可违反）**：phaseId 绑定禁止自动推断；region/node ID
   只消费台账不发明；CRS 缺失或可疑即停止生产转换；未识别字段进扩展区不丢弃；
   转换状态用八态枚举。

---

## 4. 代码地图

```text
mapforge/
├─ adapters/
│  ├─ opendrive/writer.py     # ★ 自研 1.5 规范级 writer（roads/lanes/junction/铺面）
│  ├─ opendrive/reader.py     # xodr 解析
│  ├─ shp/ibd_reader.py       # IBD 44 图层直读
│  ├─ shp/profile_source.py   # ★ Profile 引擎（YAML 字段映射，接新图商用）
│  └─ v2xmap/xml_reader.py    # MAP XML 解析（含多节点帧）
├─ ops/
│  ├─ refline_fit.py          # ★ 参考线拟合核心（见下）
│  ├─ shp_to_xodr.py          # ★ SHP→xodr 直转
│  ├─ map_to_xodr.py          # ★ MAP→xodr（junction 重建）
│  └─ shp_to_map.py / junction_to_map.py
├─ validate/
│  ├─ planview_check.py       # 几何段位姿连续性
│  └─ smoothness.py           # ★ 平滑度审计（断面/换乘/曲率品质）
└─ report/                    # 交付包组装
```

`refline_fit.py` 是几何质量的心脏，函数间关系：

```
fit_polyline_auto      三档拟合（基线 line/arc → 细档 → 稀疏链 spline），择优按对原始顶点偏差
   ↓
fit_leg_refline        leg 参考线：曲率封顶（|κ|>1/30 判数字化噪声，滑动平均重拟合）
   ↓                              ↓
simplify_planview      曲率域精简：κ 去噪 → (s,κ) 上 RDP → 重建 line/arc/clothoid
   ↓                   目标：先最少变号、再最少段数；短段吸收每次验偏差
weld_g2                无条件焊平残留 κ 断差（G2 硬承诺）
   ↓
planview_prims         → writer 几何原语
```

辅助工具（`scripts/`）：

| 脚本 | 用途 |
|---|---|
| `closed_loop.py` | **验收闸门**：再生成 14 文件 × 七门禁 |
| `gen_all.py` | 批量再生成金凤 7 路口 × 两条管道 |
| `esmini_rm_check.py` | esmini RoadManager 独立消费端验证 |
| `visual_sweep.py` | odrviewer 无窗截帧 × 4 机位拼图 |
| `xodr_topdown.py` / `xodr_diag.py` | 自研俯视渲染 / 缺陷定位器 |

---

## 5. 质量保证体系（七门禁）

`scripts/closed_loop.py` 一条命令跑完，**全绿才算交付**：

| 门禁 | 判据 | 当前实测（14 文件） |
|---|---|---|
| G1 XSD | OpenDRIVE_1.5M schema | 14/14 PASS |
| G2 planView | 段间位姿 <1mm / <0.001rad | 0 违例 |
| G3 曲率连续 | 结点 \|Δκ\| < 1e-6 | 全部 **0.00e+00** |
| G4 断面台阶 | laneSection 边界 <5cm | 全部 **0.000m** |
| G5 换乘连续 | 进/出侧 <1cm | 全部 **0.0cm** |
| G6 esmini 独立行驶 | 缝隙 <15cm、零跳变、可穿越 | 14/14 PASS |
| G7 曲率品质 | 蛇行/jerk/段长中位 | leg 蛇行 ≤11.5/100m、jerk ≤149 |

三层验证哲学（缺一不可）：

1. **文件级审计**（我们自己的 `smoothness.py`）——快，但可能自欺；
2. **独立消费端**（esmini RoadManager，第三方实现）——双盲验证语义；
3. **视觉目检**（odrviewer 自家渲染截帧）——数字全绿≠视觉合格，两者都过才算数。

> 教训：自检工具本身也要被自检。曾有一次"边缘钩子"是叠画脚本的索引拼接假象，
> 审计器与 odrviewer 都没有这个东西。

段长/曲率现状（本轮治理后）：

| 指标 | 治理前 | 治理后 |
|---|---|---|
| 总段数（14 文件） | 3752 | 2089（−44%） |
| <1m 段占比 | 7.8% | **0.0%** |
| <2m 段占比 | 50.3% | 3.1% |
| 段长中位 | 2.00m | 4.20m |
| leg 蛇行（次/100m） | 18–24 | 0–11.5 |
| leg 侧向 jerk（60km/h） | 667–1290 | 0–149 |

---

## 6. 接新图商数据

不改内核，写一份 YAML Profile 即可（`profiles/shp/*.yaml`）：

```bash
python -m mapforge.cli profile-init  <名字>       # 生成带注释模板
python -m mapforge.cli profile-check <名字>       # 图层字段体检 + 降级预告
python -m mapforge.cli convert <SHP目录> --to xodr --profile <名字> --at <lon,lat>
```

支持两条几何路线：**A** 车道中心线 + 宽度字段；**B** 边界线 + 左右关系（自动合成
中线、按横距推宽）。宽度四级阶梯 field→boundaries→spacing→default，非 field
来源记 APPROXIMATED。详见 [接入指南](接入指南-新图商SHP数据需求与Profile.md)。

---

## 7. 遗留事项

**需要外部输入**（阻塞相应功能）：
- 真实配时表 → phaseId 绑定（当前无 phase 即 BLOCKED，这是设计红线不是缺陷）
- IBD 字段枚举权威文档 → MARK_TYPE / lane_type 精确映射（当前用通行画法默认）
- 坐标绝对校验（实测点）→ 当前 crs_integrity=internally-consistent
- 部署形态、xodr 消费方、J2735 需求

**已知简化**（记录在案，非缺陷）：
- 高程平面输出（SLOPE/BANKING 字段未核验）
- 标线为默认画法：中线双黄/车道间虚白/外缘实白
- lane_type 除 median 外全 driving
- junction 铺面：SHP 用交叉口面实测多边形；MAP 无面数据用连接路凸包（INFERRED）

**技术改进方向**（按价值排序）：
1. **参考线非线性精修**——曲率剖面直接重建这条路已验证走不通（偏差 1.5–2.0m，
   曲率误差两次积分放大），但加一步 Levenberg-Marquardt 优化各段 (κ, L) 就可行，
   预期能把段数再降一半。这是段长指标继续改善的正道。
2. Maier 最优分段（动态规划）替代当前贪心分段
3. 连接路曲率蛇行（当前 conn 14–24 次/100m，路口内低速影响有限）
4. 字段别名词典 / Profile 自动推断向导（待第二家图商真实交付）

---

## 8. 复跑与排障

```bash
# 完整验收（改任何几何代码后必跑）
.venv/Scripts/python scripts/closed_loop.py

# 只看某个文件哪里不平滑
.venv/Scripts/python scripts/xodr_diag.py out/direct_xodr/node18.xodr

# 用 odrviewer 亲眼看（无窗截帧，不弹窗口）
cd <临时目录> && <项目>/esmini/bin/odrviewer.exe --odr <file> --headless --capture_screen --camera_mode top
```

排障经验：

- **odrviewer 只给一个盒子**：outline `<object>` 不渲染成面，路口铺面要用
  `type=none` 的铺面 road。
- **esmini 段错误**：ABI 签名必须对齐 v3.6.0（id_t=uint32、double 参数、出参取
  车道 id），旧 float 签名喂进去是垃圾值。
- **G3 突然回归**：多半是新加的几何处理跳过了 `weld_g2`。
- **断面出现幽灵台阶**：审计器要先对零宽车道的重复边界去重。
