# SHP 车道级高精地图交付生态、GIS 工具与中国测绘合规 —— 调研报告

> 调研代理产出，2026-08-13。以下全部条目基于当日实际检索/打开的网页；无法核实处已标注"未证实"。本报告是《地图格式转换工厂-首批三格式方案》第 2.2/6/12 节的事实依据。

---

## 1. 基础工具核查

### 1.1 核心开源工具链

| 名称 | URL | 许可证/性质 | 关键结论 | 转换工厂中角色 |
|---|---|---|---|---|
| GDAL/OGR | https://gdal.org/ ；https://github.com/OSGeo/gdal/releases | MIT（部分文件 BSD-2/BSD-3） | 当前稳定版 3.13.2（2026-07-22，bugfix），3.13.0 于 2026-05 发布 | 全部矢量 IO 的底座（SHP/GPKG/GeoJSON 读写）；另自带 XODR（OpenDRIVE）读驱动可用于逆向验证 |
| pyogrio | https://geopandas.org/en/latest/docs/user_guide/fiona_to_pyogrio.html | MIT（GDAL 之上的向量化 IO） | 自 GeoPandas 1.0 起取代 fiona 成为默认 IO 引擎，批量读写提速 5–20x | SHP/GPKG 批量读写首选引擎 |
| fiona | 同上 | BSD | 仍可用（`engine=fiona`），适合逐要素流式处理 | 备选/流式场景 |
| GeoPandas | https://geopandas.org/ | BSD | 1.1.x 系列活跃维护 | 图层级 DataFrame 处理、拓扑清洗、字段映射 |
| Shapely 2.x | https://shapely.readthedocs.io/en/stable/manual.html | BSD（封装 GEOS） | 当前 2.1.2；2.x 提供 NumPy ufunc 向量化几何运算 | 车道线几何处理（offset、简化、打断、拓扑） |
| pyproj/PROJ | https://pyproj4.github.io/pyproj/stable/ | MIT | 当前 pyproj 3.7.2，成熟稳定 | WGS84/CGCS2000/UTM 投影转换（注意：不含 GCJ-02，GCJ-02 无官方算法） |

### 1.2 SHP 格式硬限制清单（影响"车道级高保真交付"的重点）

来源：GDAL 官方驱动文档 https://gdal.org/en/stable/drivers/vector/shapefile.html 、https://knowledge.civilgeo.com/gis-shapefile-common-restrictions ：

1. **文件大小**：.shp/.dbf 建议不超 2GB（兼容性上限）；OGR 实现上限 4GB —— 城市级车道数据必须分幅。
2. **DBF 字段名 ≤10 字符**：车道级属性必然被截断，需要维护"短字段名 ↔ 语义名"映射表 —— 对转换工厂是强约束。
3. **字段数 ≤255、类型受限**：仅 Integer/Integer64/Real/String(≤254 字符)/Date（无 DateTime、无布尔、无列表/嵌套/二进制）—— 车道限速时段、复杂拓扑引用需拆表或编码进字符串。
4. **无 NULL**：dBASE 不存空值（以 0/空串顶替），语义损失。
5. **无参数化曲线**：几何只有折点序列 —— 与 OpenDRIVE 的 clothoid/arc 参考线模型本质差异，SHP→OpenDRIVE 必须曲线拟合，SHP←OpenDRIVE 必须采样离散化。
6. **单文件单图层、单几何类型**：一个图层一组 .shp/.dbf/.shx（+.prj/.cpg），"车道级地图"必然是**多文件分层交付包**；GDAL 把目录当数据集处理。
7. **编码乱**：依赖 .cpg 或 DBF LDID 猜测编码，中文交付常见 GBK/UTF-8 混乱 —— 需显式编码探测。
8. **其他**：拓扑不存储（无共享边）、Multi* 类型判定依 part 数隐式产生、无原生 3D 语义扩展。

### 1.3 替代格式成熟度

| 名称 | URL | 性质 | 结论 | 角色 |
|---|---|---|---|---|
| GeoPackage | https://www.geopackage.org/ | OGC 正式标准（最新采纳 1.4.0），单文件 SQLite | 多图层+属性+瓦片+扩展机制，无 2GB/10 字符限制，已是"替代 SHP"的主流答案 | 工厂内部规范化中间格式首选候选 |
| GeoParquet | https://geoparquet.org/ | 1.1.0 已发布；OGC 正式批准仍在流程中；2.0.0-rc.1 转向 Parquet 原生 GEOMETRY 类型 | 列式、云原生、分析友好；在中国图商交付生态尚无出现痕迹 | 批处理/数据湖侧中间格式，非对外交付格式 |

---

## 2. 中国车道级高精地图 SHP 交付生态（公开资料）

### 2.1 官方/团体标准中 SHP 的地位（最硬的证据）

| 名称 | URL | 性质 | 关键结论 |
|---|---|---|---|
| **T/CITSA 25-2022《交通系统高精度地图服务技术规范》** | https://www.its-china.org.cn/TTBZ/20220907/0004.pdf （已下载解析全文） | 中国智能交通协会团标，2022-09-07 发布实施 | §5.2 明确规定数据文件格式：**二维矢量数据 = shp 文件**，CAD 成果 = DWG，**OpenDRIVE 数据 = xodr**；三维 = las/rcp 点云、OBJ/OSGB/OSG/GLTF/GLB 模型。§5.1 坐标系：**通常采用 CGCS2000 或 GCJ-02，另支持公安 P-GIS（一般为 WGS84）**。§5.3 数据规格含城市道路高精地图、高速公路高精地图、桥隧、停车场等要素 |
| **DB11/T 2041-2022《自动驾驶地图数据规范》（北京地标）** | https://ndls.org.cn/standard/detail/f99426084ac7264cf2a566934fc578f0 ；征求意见稿 PDF：https://ghzrzyw.beijing.gov.cn/biaozhunguanli/bztg/202201/P020220118570614482390.pdf | 北京市规自委，2022-12-27 发布、2023-07-01 实施 | 数据分 **5 个图层组：道路级路网、车道级路网、道路交通标志、道路交通标线、道路交通设施**；"地图-瓦片-图层组-图层"组织；大地基准 CGCS2000 |
| DB11/T 2166-2023《自动驾驶地图质量规范》 | https://ghzrzyw.beijing.gov.cn/biaozhunguanli/bzxg/202410/P020241025391140475761.pdf | 北京地标 | 配套 2041 的质量检验规范，北京已形成"数据规范+质量规范"闭环 |
| DB4201/T 654-2022《智能网联道路建设规范（总则）》（武汉） | https://scjgj.wuhan.gov.cn/jggg/tzgg/202204/P020220428383658368027.pdf | 武汉地标 | 检索摘要显示含 MAP 消息数据采集要求（高精度测绘车：GNSS/INS/激光/视觉）——具体条文细节未逐条核对 |

### 2.2 图商交付实践

| 主体 | URL | 关键结论 |
|---|---|---|
| 行业通行说法 | http://www.its114.com/html/itswiki/technology/2021_02_114318.html （引句来自检索摘要） | "高精地图数据制作采用分图层分要素表达……分 3 大模块：**车道模型、地面标志物、交通标志牌**，**交换规格一般采用 ShapeFile 或者 MIF 等 GIS 数据格式**" |
| 四维图新 | https://www.seewayai.com/map-products | OneMap"四图合一"、HD Lite 轻量车道级产品，走云服务/车端格式路线；**未公开 SHP 图层/字段交付规范** |
| 高德 | https://www.cnblogs.com/amap_tech/p/14900535.html | 高精数据含每车道坡度/曲率/航向/高程、车道线虚实/颜色、人行横道/看板/限速牌/红绿灯等要素；对外分发用 **ADASIS V3 协议**（车端），未提 SHP |
| 百度 | https://zhuanlan.zhihu.com/p/60369652 ；https://blog.csdn.net/xiaoma_bk/article/details/122585733 | Apollo 高精地图为 **OpenDRIVE 修改版**（绝对坐标序列替代参考线方程、扩展禁停区/人行横道/减速带），Apollo 支持 OpenDRIVE→Apollo 格式转换 |
| 武汉中海庭 | https://blog.csdn.net/An1090239782/article/details/122226919 | 上汽控股高精地图供应商（甲级资质 19 家之一）；具体交付格式**未证实** |
| 易图通、立得空间 | （检索无公开技术文档） | 均在甲级资质名单内；车道级交付格式公开资料**未证实** |

**小结**：图层清单（车道中心线/车道边线/路口面/停止线/人行横道/标志标牌/信号灯杆）在公开科普与标准中反复出现，但**任何图商的 SHP 字段级 schema 均未公开**——各家 B 端工程交付实为项目定制，这正是转换工厂需要"可配置字段映射"的原因。

### 2.3 RSU MAP 消息与地图数据的标准关系

| 名称 | URL | 关键结论 |
|---|---|---|
| T/CSAE 53-2017 / 53-2020 | http://zhishi.sae-china.org/read/?id=1798 | 定义 5 类基本消息（BSM/RSM/RSI/SPAT/**MAP**）；MAP 由 RSU 广播，含 node/link/lane/转向连接；2020 修订版 2020-12-31 发布 |
| YD/T 3709-2020 | https://www.codeofchina.com/standard/YDT3709-2020.html | MAP 进入行标体系；坐标编码细节标准原文未公开核实（未证实） |
| MAP 消息解读 | https://blog.csdn.net/qq_34432784/article/details/105903495 （引自检索摘要） | "MAP……车道中心线点序列集合将车道均分 N 个 point"——MAP 本质是**车道中心线抽稀点列+拓扑**，与 SHP 车道中心线层天然对应 |
| 车路云一体化试点推荐标准清单（第一批 63 项） | https://mgr-api.china-icv.cn/profile/upload/2025/03/06/4b153461-7ac9-49ed-879e-297b4eb60b3e.pdf （已下载解析） | 2025-03 宣贯；含"高精度地图与定位"类；点名 **T/CAGIS 13-2024《高级辅助驾驶地图技术审查送审数据规格》**、CH/T《智能汽车基础地图数据传输安全保护技术规范》（编制中）、"云控基础平台与地图基础平台数据交互技术要求"（CSAE，编制中） |

**关键空白**：未检索到任何一部公开团标/地标**直接规定"RSU MAP 消息制作所用地图源数据格式"**——消息端有标准（CSAE 53/YD/T 3709）、地图端有标准（DB11/T 2041、T/CITSA 25），中间的"SHP→MAP 转换规范"是标准空白，也是转换工厂的定位空间。

---

## 3. 中国测绘合规事实（仅政策事实+出处）

### 3.1 导航电子地图制作甲级测绘资质

- **审批机关**：自然资源部（管理最严的甲级测绘资质类别）。
- **数量级**：2021-2022 年资质复审换证前全国近 30 家，**复审后仅 19 家过审**。来源：https://m.mp.oeeee.com/a/BAAFRD0000202410171009868.html
- **代表名单**（截至 2024，二手整理，官方逐家核验未证实）：四维图新、高德、灵图、长地万方（百度）、凯立德、易图通、国家基础地理信息中心、立得空间、腾讯大地通途、江苏省测绘工程院、浙江省第一测绘院、江苏省基础地理信息中心、武汉光庭、滴图科技（滴滴）、武汉中海庭、贵州宽凳、北京初速度（Momenta）、江苏晶众、江苏智途。来源：https://blog.csdn.net/Slyvia_HD/article/details/141354596

### 3.2 高精地图管理政策脉络（2022→2025）

| 时间 | 文件 | 要点 | 来源 |
|---|---|---|---|
| 2022-08-30 | 《自然资源部关于促进智能网联汽车发展维护测绘地理信息安全的通知》（自然资规〔2022〕1号） | 智能网联汽车装载传感器采集空间坐标/影像/点云属于**测绘活动**；数据收集存储传输处理者是测绘行为主体，依法担责；向境外传输须履行审批/地图审核 | http://gk.mnr.gov.cn/zc/zxgfxwj/202208/t20220830_2757960.html |
| 2022-08 | 自然资源部办公厅《关于做好智能网联汽车高精度地图应用试点有关工作的通知》 | 在**北京、上海、广州、深圳、杭州、重庆 6 城市**开展高精地图应用试点；2022-09 广深率先落地 | https://m.bjnews.com.cn/detail/167834790714637.html |
| 2023-03 | 《智能汽车基础地图标准体系建设指南（2023版）》 | 到 2025 初步构建、2030 较完善的标准体系 | https://nr.gd.gov.cn/zwgknew/zcjd/gj/content/post_4133747.html |
| 2024-07-26 | 《自然资源部关于加强智能网联汽车有关测绘地理信息安全管理的通知》（自然资发〔2024〕139号） | 数据处理及地图制作**应由具有导航电子地图制作等测绘资质的单位承担**；地理信息数据**必须境内存储**；采集数据**直接传输至资质单位管理，其他单位或个人不得接触**；地图审核通过后方可使用 | https://www.gov.cn/zhengce/zhengceku/202407/content_6965227.htm |
| 2024-01 | 五部委《关于开展智能网联汽车"车路云一体化"应用试点工作的通知》 | 鼓励高精度地图应用、**众源采集及更新**先行先试 | https://ghzyj.sh.gov.cn/hyxw/20240118/694a84ecd4534100a6c9c36228b2eef8.html |
| 2022-12 / 2025-08 | 上海市《智能网联汽车高精度地图试点审图工作细则》；《上海市智能网联汽车测绘地理信息安全管理导则（试行）》（2025-08-25 印发） | 地方层面细化审图流程与安全管理；2025 年仍在出新文件 | https://ghzyj.sh.gov.cn/zcwj/chgl/20250825/7fb303a0931747f9a4f24b34dc4bfe60.html |
| 2026 | 部级新政 | 本次检索**未发现** 2026 年新的部级政策文件（未证实） | — |

### 3.3 GCJ-02 的官方定位与车路协同坐标现实

- **官方定位**：GCJ-02 官方名称为"地形图非线性保密处理算法"，基于 WGS-84 做非线性偏移；依据 **GB 20263-2006**（导航电子地图安全处理基本要求）及《测绘法》《地图管理条例》体系，境内公开地图必须使用 GCJ-02 或再加密坐标；**官方不提供 GCJ-02→WGS-84 逆转换**。来源：https://en.wikipedia.org/wiki/Restrictions_on_geographic_data_in_China
- **车路协同现实**：T/CITSA 25-2022 明文允许交通系统高精地图采用 **CGCS2000 或 GCJ-02，并兼容公安 P-GIS（一般为 WGS84）**——路侧场景多坐标系并存是标准认可的现实；DB11/T 2041（北京）则要求 CGCS2000。C-V2X 空口 MAP/BSM 消息坐标系在公开网页上未见权威原文（标准需付费，**未证实**；技术社区普遍按 WGS-84 经纬度处理）。
- 合规提示（事实性）：按 2024 年 139 号文，涉及测绘地理信息数据的处理环节须由资质单位承担——转换工厂若处理真实路网坐标数据，属地化合规评估不可省略。

---

## 4. SHP→OpenDRIVE / SHP→车道模型工具盘点

| 名称 | URL | 许可证/性质 | 关键结论 | 转换工厂中角色 |
|---|---|---|---|---|
| MathWorks RoadRunner | https://www.mathworks.com/help/roadrunner/ug/Vector-Data-Tool.html | 商业 | Vector Data Tool 可加载 **.shp/.geojson/.gpx/.osm** 及属性，但官方明言"**不支持把矢量数据自动转换为 RoadRunner 内部格式**"——SHP 只能当描图参考；自动建路仅支持 OSM（SD 级）和 HD 地图服务（Scene Builder：HERE/TomTom/Apollo） | 人工精修与 OpenDRIVE 导出终端；不能做 SHP 自动化流水线 |
| Trian3DBuilder | https://triangraphics.de/2025/02/24/new-trian3dbuilder-version-8-1-available-now/ ；https://triangraphics.de/2026/03/23/trian3dbuilder-3d-terrain-generation-major-release-v9-0/ | 商业（柏林） | OpenDRIVE 1.6–1.8 **双向导入/导出**；路网可导出 OpenDRIVE 或 **ESRI Shapefile**；可导入 OSM、HERE RDF 等 GIS 数据建路网；CARLA 集成 | 少数具备"GIS 数据→路网→OpenDRIVE"产线能力的商业软件 |
| Safe FME | https://community.safe.com/ideas/new-reader-writer-opendrive-28563 | 商业 ETL，300+ 格式 | **无原生 OpenDRIVE reader/writer**（Idea 自 2019 年挂起）；SHP/GPKG/Parquet 等 GIS 侧极强 | 只能承担 GIS 侧预处理，OpenDRIVE 端需自研 |
| SUMO netconvert | https://sumo.dlr.de/docs/Networks/Import/ArcView.html ；https://sumo.dlr.de/docs/Networks/Export.html | EPL-2.0（DLR/Eclipse） | **唯一查到的开源"SHP 进、OpenDRIVE 出"链路**：`--shapefile-prefix` 导入（字段映射：LINK_ID/ST_NAME/NOLANES/SPEED…），`--opendrive-output` 导出 OpenDRIVE **1.4**；但导出标注"实现中"：道路单向、无车道加宽、道路类型固定、signal 信息为空——**道路级可用，车道级保真不够** | 原型验证/兜底路径 |
| GDAL XODR 驱动 | https://gdal.org/en/stable/drivers/vector/xodr.html | MIT（DLR 贡献，基于 libOpenDRIVE） | OpenDRIVE→GIS **只读**方向（xodr→GPKG/SHP/GeoJSON） | 转换结果的逆向验证/可视化质检 |
| scenariogeneration (pyoscx) | https://github.com/pyoscx/scenariogeneration | MPL-2.0 | Python 程序化生成 .xodr/.xosc，带几何自动衔接算法 | 自研 SHP→OpenDRIVE writer 的现成构件 |
| 其他开源 | https://github.com/benediktschwab/awesome-openx ；https://github.com/JHMeusener/osm2xodr ；https://github.com/virtualcitySYSTEMS/opendriveconverter | 各异 | 全部是 OSM→xodr 或 xodr→GIS 方向；**未检索到成熟的"shapefile→OpenDRIVE"直转开源项目** | 参考实现与生态索引 |
| 图商转换服务 | （无公开产品页） | — | **未检索到任何图商公开提供"SHP→OpenDRIVE/MAP 消息"自助转换服务**——此类能力目前以项目制交付，公开市场为空白 | 竞争空白佐证 |

---

## 5. 商业授权量级（公开资料）

| 产品 | 许可模式 | 数量级 | 来源 |
|---|---|---|---|
| Safe FME | 2023-05 起简化为 FME Form + FME Flow；永久授权或订阅 | FME Form 永久授权约 **£9,500**（含维保）；平台订阅年费约 **£15,000 起** | https://fme.globema.com/fme-frequently-asked-questions/ ；https://locusglobal.com/new-fme-pricing/ |
| MathWorks RoadRunner | **独立产品，不需要 MATLAB**；Scene Builder 单独付费 add-on | 2021 年英国资料：个人永久授权约 **£8,500/席**；现价需询售（未证实） | https://itassetmanagement.net/2021/01/26/mathworks-licensing-getting-started/ |
| Trian3DBuilder | 商业授权+可选模块，报价制 | 第三方目录（时间较早）：标准包约 **€9,900**，模块另计；现价未证实 | http://vterrain.org/Packages/Com/ |

---

## 本方向调研结论（10 条要点）

1. **SHP 在中国路侧/交通高精地图交付中有"标准背书"**：T/CITSA 25-2022 明文将 shp 与 xodr、DWG 并列为交付格式——转换工厂把 SHP 作为首批输入格式方向正确。
2. **但 SHP 无字段级公开规范**：图商/示范区的 SHP 图层与字段 schema 均不公开、项目制定制（DBF 10 字符字段名迫使各家用缩写码），转换工厂必须内置**可配置图层/字段映射层**，而非硬编码 schema。
3. **SHP 硬限制与车道级需求冲突点明确**：10 字符字段名、无 DateTime/布尔/嵌套、无 NULL、无参数化曲线、2GB 分幅、编码混乱——工厂入口应立即规范化到内部模型（GeoPackage 为成熟落地格式；GeoParquet 适合内部批处理）。
4. **Python 栈完全够用且全部宽松许可**：GDAL 3.13.2（MIT）+ GeoPandas 1.1（pyogrio 默认引擎，5–20x 提速）+ Shapely 2.1（向量化）+ pyproj 3.7。
5. **"SHP→OpenDRIVE"开源直转工具不存在**，最近路径是 SUMO netconvert（道路级、1.4）；商业侧仅 Trian3DBuilder 具备 GIS→OpenDRIVE 产线，RoadRunner 的 SHP 只能当描图参考、FME 无 OpenDRIVE writer——**车道级 SHP→OpenDRIVE 自动转换是真实空白**。
6. **"SHP→MAP 消息"更是标准与工具双空白**：消息端有 T/CSAE 53/YD/T 3709，地图端有 DB11/T 2041/T/CITSA 25，但没有公开标准或工具规定/实现"用何种地图数据、如何生成 RSU MAP"——转换工厂最独特的价值位。MAP 本质是车道中心线抽稀点列+node/link/lane/movement 拓扑，与 SHP 车道中心线层结构同构。
7. **坐标系是三格式转换的核心工程问题**：路侧生态 CGCS2000/GCJ-02/WGS84(P-GIS) 并存（T/CITSA 25 明文），北京地标要求 CGCS2000，公开地图强制 GCJ-02（GB 20263-2006）且无官方逆变换——工厂需要显式坐标系声明与转换审计。
8. **合规红线清晰且趋严**：2022 年 1 号文认定智能网联数据采集属测绘活动；2024 年 139 号文要求数据处理/地图制作由甲级资质单位（全国约 19 家）承担、数据境内存储、非资质单位不得接触原始采集数据——转换工厂的商业模式应定位为"给资质单位/示范区提供工具与产线"，而非自行持图处理。
9. **试点格局**：高精地图应用试点 6 城 + 2024 年车路云一体化 20 城试点 + 第一批 63 项推荐标准——目标客户集中在这些示范区的建设方、集成商与 19 家资质图商。
10. **商业工具授权量级**（自研 ROI 对比用）：FME Form 约 £9.5k 永久，RoadRunner 约 £8.5k/席（2021 价），Trian3DBuilder 约 €9.9k 起（旧价）——三者都不能开箱解决"中国 SHP→MAP 消息"，自研 Python 产线在功能与成本上均有正当性。
