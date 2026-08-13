# V2X MAP 消息（路侧交叉口地图消息）标准与工具链调研报告

> 调研代理产出，2026-08-13。所有内容基于当日实际检索/打开的网页；无法核实处均标注"未证实"。本报告是《地图格式转换工厂-首批三格式方案》第 2.3/5/6 节的事实依据。

---

## 1. 标准盘点

### 1.1 中国：T/CSAE 53-2020（第一阶段）中的 MAP（MapData）

- T/CSAE 53-2020《合作式智能运输系统 车用通信系统应用层及应用数据交互标准（第一阶段）》2020-12-31 发布，代替 T/CSAE 53-2017；修订要点包括定位精度要求提高至 ≤1.5 m、调整应用层交互数据集（来源：https://www.scimall.org.cn/article/detail?id=5096557 ；标准官方页 http://csae.sae-china.org/portal/standardDetail?id=8023e1a471a66e3c46892eebbac9da1d ）。
- 消息集为 BSM/RSI/RSM/SPAT/MAP 五大消息，统一装入 MessageFrame（CHOICE：bsmFrame/mapFrame/rsmFrame/spatFrame/rsiFrame），ASN.1 定义 + UPER 编码（来源：https://github.com/xuanxuanblingbling/cv2x ；https://polelink.com/index.php?a=show&c=index&catid=93&id=91&m=content ）。
- **MapData 层级**（来源：https://geekdaxue.co/read/lovebetterworld@c-v2x/gqwfsv ）：
  - `MapData → msgCnt(0..127) + timeStamp + nodes[Node]`
  - `Node`：节点 id（region+id）、`refPos`（经纬度+海拔的参考位置）、`inLinks[Link]`（上游进入路段）
  - `Link`：上游节点 id、路段宽度、`points`（路段中心线点列）、`movements[Movement]`（转向 + `phaseId`，与 SPAT 相位关联的唯一纽带）、`lanes[Lane]`
  - `Lane`：`laneID`、`maneuvers`（位图，如 000000000011 表示可左转+直行）、`connectsTo[Connection]`（与下游路段车道的连接关系及对应信号相位）
- **偏移量类型**：车道/路段点列使用 `PositionOffsetLLV = SEQUENCE{ offsetLL, offsetV }`；`OffsetLL` 为 B12~B24 多档位宽的经纬度偏移，如 OffsetLL-B16 范围 ±0.0032767°、OffsetLL-B24 范围 ±0.8388607°，正值表示相对参考点向东/北偏移（来源：https://blog.csdn.net/weixin_41301603/article/details/140860774 ；https://openv2x.org/posts/2022-11-25-1/ ）。
- **Position3D 分辨率**：J2735 体系的经纬度 LSB 为 1/10 微度（1e-7°）（来源：NOCoE《Overview of MAP messages》 https://transops.s3.amazonaws.com/uploaded_files/SPaT%20Webinar%20%233%20-%20NOCoE%20-Overview%20of%20MAP%20Messages.pdf ）；中国标准 MAP 为 J2735 同源设计，多篇解读一致采用同分辨率，但 T/CSAE 53 原文数值本次未能直接打开核对（标准全文站点均不可预览），**此点按"推定一致、原文未证实"处理**。
- 配套 RSU 标准 T/CSAE 159-2020 逐条引用 YD/T 3709-2020 的 Msg_MAP 定义，并规定：MSG_MAP 必须包含 DF_NodeList（一个或多个地图节点）；Movement 的 phaseId 必须与 Msg_SPAT 中对应节点的信号灯关联；msgCnt 首条随机初始化（0–127），签名证书变化时重置、否则 +1 回绕；节点/路段名称字符串 1–63 字节（来源：T/CSAE 159-2020 全文 PDF https://static1.tianyancha.com/czd_file/standard/e0fc957ab18e120b7914ff4dab474229.pdf ，第 6 章、6.4.1）。

### 1.2 T/CSAE 157（第二阶段）与 MAP 的关系；YD/T 3709 现状；国标化

- **T/CSAE 157-2020**《…应用层及应用数据交互标准（第二阶段）》：在第一阶段基础上定义 12 个二阶段应用场景和 9 个新交互消息；星云互联牵头、30 余家单位参与（来源：https://www.sohu.com/a/432466279_620780 ；https://www.uedoc.com/view/15659775.html ）。MAP 消息本体仍由第一阶段/消息层标准定义，二阶段场景复用之（未见二阶段修改 MapData 结构的公开描述）。
- **YD/T 3709-2020**《基于LTE的车联网无线通信技术 消息层技术要求》：2020-04-16 发布、2020-07-01 实施，**现行有效、未被代替**（截至本次查询，全国标准信息公共服务平台页面）（来源：https://std.samr.gov.cn/hb/search/stdHBDetailed?id=A75176EB37D0B551E05397BE0A0A545D ）。检索未发现 2023/2024 修订版发布记录（未证实存在修订版）。
- **国标化现状**：未检索到直接定义 MapData 的"车用通信系统应用层及应用数据交互"国家标准。实际路径是"国标引用行标"：
  - GB/T 44417-2024《车路协同系统智能路侧协同控制设备技术要求和测试方法》（2024-08-23 发布、2025-03-01 实施）在通信接口中规定 LTE-V2X 直连通信各层分别符合 YD/T 3340（接入层）、YD/T 3707（网络层）、YD/T 3957（安全层）、**YD/T 3709（消息层）**，被业界称为"LTE-V2X 全栈载入国标"（来源：https://openstd.samr.gov.cn/bzgk/gb/newGbInfo?hcno=2277A58E1ECF5CD32FD2996CCF592106 ；https://www.cictci.com/index/industryHotspots/566.html ）。
  - GB/T 45315-2025《基于LTE-V2X直连通信的车载信息交互系统技术要求及试验方法》（计划号 20230390-T-339，汽标委 TC114/SC34 归口）已发布，为车端系统级国标（来源：https://std.samr.gov.cn/gb/search/gbDetailed?id=F77B015669BE422CE05397BE0A0A8AC4 ）。
  - 安全证书管理已国标化为 GB/T 45112-2024（来源：https://std.samr.gov.cn/gb/search/gbDetailed?id=E116673ECFDDA3B7E05397BE0A0AC6BF ）。

### 1.3 美国 SAE J2735

- 最新版本为 **J2735_202409**（2024-09-16 发布，current；替代 J2735_202309）（来源：https://saemobilus.sae.org/standards/j2735_202409-v2x-communications-message-set-dictionary ）。ANSI 网店另售 "SAE J2735SET-2024" 套装（https://webstore.ansi.org/standards/sae/sae2735set2024 ，页面被 403，套装构成未证实）。
- MapData 内 `GenericLane`：laneID、laneAttributes（行驶方向/共享方式/车道类型）、maneuvers、`nodeList`（定义车道中心线几何，节点为相对锚点 refPoint 的偏移编码）、`connectsTo`（内含 **signalGroup**，与 SPaT 的关联键；MAP 与 SPAT 中 signalGroupID 必须一致）（来源：https://www.sae.org/standards/content/j2735_202309/ ；https://engineering.virginia.edu/sites/default/files/Connected-Vehicle-PFS/Resources/MAP%20Guidance%20Document%20-%20Revision%202_06232023.pdf ；NOCoE PDF 同上）。
- 美国新动向：FHWA 已将 MAP 工具升级为 "MAP/**RGA**（J2945/A Road Geometry Attributes）Message Creator"，即 MAP 之后的新一代路侧几何消息（来源：FHWA 简报 https://highways.dot.gov/sites/fhwa.dot.gov/files/FHWA-HRT-26-019.pdf ）。
- 配套指南：Virginia UVA/CV-PFS《Guidance Document for MAP Message Preparation》已出到 Rev.4（2025-07）（来源：https://engineering.virginia.edu/sites/default/files/Connected-Vehicle-PFS/Projects/(MAP)%20Guidance/MAP%20Guidance%20Document%20-%20Revision%204%20FINAL.pdf ，站点 403 未能打开正文）。

### 1.4 欧洲 ETSI TS 103 301（MAPEM）与 ISO/TS 19091

- ETSI TS 103 301 定义基础设施服务消息 SPATEM/MAPEM/SREM/SSEM/IVIM/RTCMEM；**MAPEM = ETSI ItsPduHeader（protocolVersion/messageID/stationID）+ ISO/TS 19091 的 MapData**，而 ISO/TS 19091 又是 SAE J2735 的 profile；MAPEM 属 RLT（Road and Lane Topology）服务，与 SPATEM 持续共同播发，lane connection 中的 signal-group id 是与 SPATEM 的关联键（来源：https://arxiv.org/html/2407.12799v1 ；ETSI 正文 https://www.etsi.org/deliver/etsi_ts/103300_103399/103301/02.02.01_60/ts_103301v020201p.pdf ）。
- 版本线：V1.1.1(2016)→V1.2.1(2018)→V1.3.1(2020)→V2.1.1(2021)→**V2.2.1(2024-08)**（ETSI deliver 目录，上述 URL）。官方 ASN.1 在 ETSI Forge：https://forge.etsi.org/rep/ITS/asn1/is_ts103301 （含 MAPEM-PDU-Descriptions.asn，v1.3.1/v2.1.1/release2 各版本）。
- ISO/TS 19091 现行版为 **2019 版**（2024 年复审确认继续有效；未见 2022 新版）（来源：https://www.iso.org/standard/73781.html ）。

### 1.5 中/美/欧 MAP 结构差异简表

| 维度 | 中国 T/CSAE 53 / YD/T 3709 | 美国 SAE J2735_202409 | 欧洲 ETSI TS 103 301 (MAPEM) |
|---|---|---|---|
| 外层封装 | MessageFrame（五消息 CHOICE），UPER | MessageFrame（含 MapData 等），UPER | ItsPduHeader + MapData（ISO/TS 19091 profile of J2735），UPER |
| 顶层地图对象 | MapData → nodes[Node]（以"节点/路段"为主干） | MapData → intersections[IntersectionGeometry] + roadSegments（以"交叉口"为主干） | 同 J2735（经 ISO 19091 裁剪/扩展） |
| 路段/车道 | Node.inLinks[Link] → Link.lanes[Lane]；Link 带 movements | IntersectionGeometry.laneSet[GenericLane] | 同 J2735 |
| 几何编码 | refPos + PositionOffsetLLV（OffsetLL-B12~B24 经纬度偏移，B16≈±0.0032767°、B24≈±0.8388607°） | refPoint + nodeList 偏移节点（多档位宽偏移 + 64bit 完整经纬点；LSB 1/10 微度） | 同 J2735 |
| 与信号灯关联键 | Movement/Connection 中的 **phaseId** ↔ SPAT 相位 | connectsTo 中的 **signalGroup** ↔ SPaT | 同 J2735（signalGroup ↔ SPATEM） |
| 播发服务定义 | T/CSAE 159-2020（RSU）：MAP 周期 ≤1s | 部署指南（SPaT Challenge/NOCoE）：MAP 1Hz | RLT service：MAPEM 与 SPATEM 持续播发 |
| 演进方向 | 二阶段 T/CSAE 157（新增场景/消息，MAP 本体未变）；国标以引用方式落地 | J2945/A RGA（MAP 的下一代） | Release 2（V2.x） |

---

## 2. ASN.1 工具链

| 名称 | URL | 许可证 | 语言 | 最近活跃 | 成熟度/结论 | 转换工厂中的角色 |
|---|---|---|---|---|---|---|
| asn1c (vlm) | https://github.com/vlm/asn1c | BSD-2-Clause | C | 主仓陈旧（最后正式 release 0.9.28，Debian 仍打包 0.9.28+dfsg），社区活跃 fork 维护 | 支持 BASIC-UPER/CANONICAL-UPER（uper_encode/uper_decode）；1.2k star、619 fork、228 open issues；USDOT asn1_codec 即基于它 | C 侧 UPER 编解码代码生成（建议用 mouse07410 fork） |
| asn1c (mouse07410 fork) | https://github.com/mouse07410/asn1c | BSD-2-Clause | C | 持续维护（被称为"最演进的开源 ASN.1 编译器"，第三方仓从其 fork） | 事实上的 asn1c 维护线 | 同上 |
| pycrate | https://github.com/pycrate-org/pycrate | LGPL-2.1 | Python | 2024 迁移至新组织、PyPI 持续发版 | 完整 UPER（from_uper/to_uper）；提供 pycrate_asn1compile.py 编译任意 .asn；**已被证明可编译中国 YD/T 3709 的 v2x.asn**（见 cv2x 仓库） | Python 侧首选：编译 CSAE/YDT asn → MAP 编解码核心 |
| asn1tools (eerimoq) | https://github.com/eerimoq/asn1tools | 见仓库 | Python | — | 支持 UPER；曾有 issue 讨论编译 ETSI TS 103 301 v1.1.1 | 备选 Python 编解码 |
| OSS Nokalva | https://www.oss.com/asn1/products/asn1-products.html | 商业 | C/C++/Java/C#/Python | 持续 | 官方提供 its_sae_j2735 样例（因版权不含 J2735 asn 原文，需自行向 SAE 获取） | 商业兜底：一致性最好，付费 |
| Objective Systems (ASN1C/ASN1VE) | https://obj-sys.com/blog/adding-information-to-the-j2735-specification.html ；https://obj-sys.com/products/asn1ve/pricing.php | 商业 | 多语言 | 持续 | 有 J2735 专题支持与 ASN1VE 可视化编辑器 | 商业兜底/调试查看器 |

**中国标准 .asn 公开情况（GitHub 核查）**：
- 存在公开的 YD/T 3709 消息层 ASN.1 文件及 UPER 编解码示例：`xuanxuanblingbling/cv2x`（毕业设计性质，asn/v2x.asn 按 YD/T 3709，gen.py 用 pycrate 生成 v2x.py，exp.py 演示 to_uper/from_uper；69 star，无 LICENSE，26 commits）——可用作参照但许可证不明，**商用需自行重打 asn**（来源：https://github.com/xuanxuanblingbling/cv2x ）。
- 亦有用 asn1c 直接编译中国 MsgFrame.asn 的教程（`asn1c asn/*.asn -D out/`）（来源：https://gitcode.csdn.net/65acb26fb8e5f01e1e45319c.html ，原文 https://blog.csdn.net/mao834099514/article/details/109102770 ）。
- 未发现名为 "CSAE53 asn" / "YDT 3709 asn" 的官方开源仓库；T/CSAE 53 的 asn 文本官方渠道需购买标准（未证实存在官方免费 asn 发布）。

---

## 3. 开源项目逐个核查

| 名称 | URL | 许可证 | 语言 | 最近活跃 | 成熟度/结论 | 转换工厂中的角色 |
|---|---|---|---|---|---|---|
| ConnectedVCS Tools（原 ISD Message Creator） | https://github.com/usdot-fhwa-stol/connectedvcs-tools ；在线版 https://webapp.connectedvcs.com/isd/ 、开放版 https://webappopen.connectedvcs.com/isd/ | Apache-2.0 | Java + JS（含 fedgov-cv-mapencoder、ASN.1 decoder 模块） | 活跃（498 commits，CI 运行；FHWA 2026 简报仍在推介） | 浏览器上基于航拍影像画车道/进口道，导出 J2735 MAP 与 J2945/A RGA 的 UPER 十六进制；美国部署事实标准工具（来源：https://highways.dot.gov/sites/fhwa.dot.gov/files/FHWA-HRT-26-019.pdf ） | 人工制图基准参照；其 MAP 编码器可作为 J2735 侧编码参考实现 |
| usdot-jpo-ode | https://github.com/usdot-jpo-ode/jpo-ode | Apache-2.0 | Java 21 | 活跃（4,678 commits，88 star） | 实时数据路由，支持 BSM/TIM/MAP/SPaT/… 解码为 JSON | J2735 MAP 二进制 → JSON 的参考链路 |
| asn1_codec (ACM) | https://github.com/usdot-jpo-ode/asn1_codec | Apache-2.0 | C/C++（基于 asn1c 生成） | 活跃（随 ODE 生态维护） | Kafka 上的 ASN.1 编/解码微服务（UPER↔XER/XML） | 生产级 J2735 编解码服务样板 |
| jpo-geojsonconverter | https://github.com/usdot-jpo-ode/jpo-geojsonconverter | Apache-2.0 | Java 21 (Spring Boot) | 活跃（448 commits、CI/CD、Docker Hub） | 将 ODE 输出的 MAP/SPaT JSON 校验（对照 J2735 与 CTI-4501）并转 **ProcessedMap GeoJSON**（车道/连接车道要素集） | **与"MAP → GeoJSON 可视化"目标直接对口的现成参考实现** |
| V2X-Hub | https://github.com/usdot-fhwa-OPS/V2X-Hub | Apache-2.0 | C++（+JS Web） | 活跃（1,531 commits，148 star） | 插件化路侧消息处理：**MAP Plugin**（播发 J2735 MAP，输入用 ISD 工具产物）、**SPaT Plugin**（NTCIP 1202 对接信号机生成 J2735 SPaT） | 路侧播发/信号机对接架构参照 |
| vanetza | https://github.com/riebl/vanetza | LGPLv3 | C++ | 活跃（2024 起 nfiniity 赞助） | ETSI C-ITS 协议栈（GN/BTP/DCC/Security）；asn1 目录含 **TS103301v211-MAPEM.asn/SPATEM.asn** 等，具备 MAPEM 编解码 | 欧洲 MAPEM 输出通道的编解码底座 |
| Apollo modules/v2x | https://github.com/ApolloAuto/apollo/tree/master/modules/v2x | Apache-2.0 | C++/proto | 活跃（Apollo 主线） | proto 目录含 v2x_junction.proto、v2x_rsi.proto、v2x_obu_traffic_light.proto 等 **OBU→车内 proto 私有格式**；未见 J2735/CSAE MapData 的 ASN.1 编解码（RSU 侧编码不在 Apollo 开源内） | 参考"车端消费侧"数据模型；不承担 MAP 编码 |
| OpenV2X | https://github.com/open-v2x （dandelion：https://github.com/open-v2x/dandelion ） | 未证实（页面未核出） | Python 等 | 约 2022–2023（albany 版本线；技术博客 2022） | 国内开源车路协同云控平台（RSU/雷达/信号灯设备管理、边缘数据、EdgeView 可视化）；官方博客有 MAP 数据结构梳理文 https://openv2x.org/posts/2022-11-25-1/ ；MAP 制作工具未见 | 国内侧云控/可视化集成参照；活跃度偏低 |
| Mcity-J2735-MAP | https://github.com/mcity/Mcity-J2735-MAP/ | 无 LICENSE | 数据仓库（XML/ASN/UPER Hex） | 低（16 commits，4 star） | Mcity 7 个路口的真实 MAP 消息样本（2009/2016 两代 J2735，含 ISD 工具兼容的 child map 文件） | **测试样例/回归数据集** |
| cv2x（中国 asn） | https://github.com/xuanxuanblingbling/cv2x | 无 LICENSE | Python | 低 | 见第 2 节 | 中国 asn + pycrate 流程验证样例 |
| v2x-server (sbublie) | https://github.com/sbublie/v2x-server | 见仓库 | — | — | "smart intersection backend"，规模小 | 参考价值有限 |

**关键空白点核查（最重要结论）**：围绕 "opendrive to J2735"、"opendrive MAP消息"、"shp MAP 消息 生成"、"v2x map generator"、"SUMO/CARLA generate MAPEM" 等关键词的多轮 GitHub/全网检索，**未发现任何"从 OpenDRIVE 或 SHP 自动生成 V2X MAP/MAPEM 消息"的现成开源工具**。现状是两个世界各自成熟但不连通：OpenDRIVE 侧有丰富转换器（如 xodr→GeoJSON：https://github.com/virtualcitySYSTEMS/opendriveconverter ；xodr/Apollo 可视化转换：https://github.com/daohu527/imap ），MAP 侧只有"人工画图"工具（ConnectedVCS/ISD、商用 GUI）。**SHP/OpenDRIVE → MAP 自动生成确为空白，适合作为转换工厂的差异化主打能力**（同时 jpo-geojsonconverter 证明 MAP→GeoJSON 方向有可借鉴实现）。

---

## 4. 商业工具/服务（公开资料）

| 厂商 | 公开证据 | 结论 |
|---|---|---|
| 星云互联 | 官网 RSU+/全栈车路协同 https://nebula-link.com/Innovate/view/id/2 ；牵头 T/CSAE 157 二阶段标准 | 具备 MAP 播发生态与标准话语权；**专门的公开 MAP 制作工具页面未证实** |
| 万集科技 | 年报/研报：基于高精地图+数字孪生的智能网联云控平台（https://pdf.dfcfw.com/pdf/H3_AP202302011582609830_1.pdf ） | 平台含地图能力；MAP 消息制作工具未证实 |
| 金溢科技 | 《如何给金溢RSU配路网》第三方教程存在（https://blog.csdn.net/usstmiracle/article/details/100072861 ，站点 521 未能打开正文） | 其 RSU 有路网(MAP)配置工具链的间接证据；工具名/形态未证实 |
| 华砺智行 | 官网 https://www.huali-tec.com/ | V2X 方案商；MAP 工具未证实 |
| 大唐高鸿/中信科智联 | 官网标准解读（https://www.cictci.com/index/industryHotspots/566.html ） | 深度参与国标/行标；MAP 制作工具未证实 |
| 希迪智驾 | 检索无有效公开结果 | 未证实 |
| 百度 Apollo（Air） | Apollo 高精地图产线公开资料（https://www.qianzhan.com/analyst/detail/220/211122-a4b89a16.html 等） | 高精地图市占率第一（28.07%）；面向 RSU 的 MAP 消息制作工具未证实 |
| 高德/四维图新 | 车道级/场景地图数据服务公开报道（https://nev.ofweek.com/2022-03/ART-77015-8420-30555711.html ） | 车道级地图交付能力明确；MAP 消息数据服务未证实 |
| Commsignia | 产品页：Central 设备管理软件支持**在地图 GUI 上创建 MAP/TIM 消息并一键下发**（https://commsignia.com/products/infrastructure-device-managment ；佛州互操作测试报告 https://www.cflsmartroads.com/projects/CV_Testing/Seminole%20Lab%20Interoperability%20Testing%20121818%20DRAFT.pdf ） | 国外最明确的商用 MAP 编辑器 |
| Cohda Wireless | MK5 SDK 按 ETSI ASN.1 解码 MAPEM/SPATEM（https://www.researchgate.net/publication/366659390 ） | 栈内编解码，非制图工具 |
| Yunex Traffic | **Map2x** 可生成/发送 SPaT/MAP（同上论文）；SEPAC 信号机固件支持 V2I SPaT（https://us.yunextraffic.com/portfolio/intersection-control/sepac/ ） | 信号机厂商自带 MAP/SPaT 工具链 |
| Econolite | TDOT DSRC 测试：ASC/3 信号机与 RSU 联调产生 J2735 SPaT 验证通过（https://www.tn.gov/content/dam/tn/tdot/traffic-engineering/TDOTDSRC_Final%20Report_version%201.0_Nov%202018.pdf ） | SPaT 对接公开案例 |
| Danlaw | 检索未见 MAP 制作工具信息 | 未证实 |

**SPAT 对接公开方案**：美国以 NTCIP 1202 为界面（V2X-Hub SPaT Plugin）＋信号机厂商固件（Yunex SEPAC、Econolite ASC/3）；中国以公安行标 **GA/T 1743-2020《道路交通信号控制机信息发布接口规范》**（2021-03-01 实施）为信号机对 RSU/平台的信息发布接口，并由 **GA/T 2151-2024** 扩展（来源：https://www.auto-testing.net/news/show-108852.html ；https://www.crsa.net/news/article/10147 ）；T/CSAE 159-2020 第 7 章进一步要求 SPAT 与信号机实际配时一致、状态变化端到端时延受限。

---

## 5. 工程约束事实核查

以下为**已核实**的一手条款（T/CSAE 159-2020，PDF 全文 https://static1.tianyancha.com/czd_file/standard/e0fc957ab18e120b7914ff4dab474229.pdf ）：

- **播发周期（5.4）**：MAP ≤ 1 s；SPAT ≤ 500 ms；RSM 100 ms；RSI 静态 1 s / 半静态 500 ms / 动态 100 ms；周期须为 100 ms 整数倍；PDB 与周期一致。→ 中国"MAP 1Hz"的规范出处即此（≤1s）。美国部署实践同为 1 Hz（SPaT Challenge 实施指南 https://www.transportationops.org/spatchallenge/resources/Implementation-Guide ）。
- **承载与寻址（5.2/5.3）**：MAP/SPAT/RSM/RSI 经 YD/T 3707-2020 DSMP 作为专用短消息（DSM）广播；AID→L2 目的地址映射固定（MAP=3618→0x000006，SPAT=3619→0x000007 等）；接入层广播、模式 4、RLC UM。
- **优先级（5.5）**：MAP Priority=16（最高），SPAT=176，RSM=208，RSI 按静/半静/动 48/80/112。
- **几何抽点（附录 D，规范性）**：点取道路/车道中心线；Link 首点为上游节点进入路段第一点、末点为停止线中心；任意连续两点弦线与实际中心线的垂距须小于 ActualError——这是 SHP/OpenDRIVE 抽稀成 MAP 点列时的直接工程判据。
- **MAP 消息大小**：NOCoE《Overview of MAP messages》：MAP 典型数百字节至 1000+ 字节，DSRC 存在单包大小约束，大路口需按策略处理。**LTE-V2X PC5 单帧的确切字节上限数值：本次未检索到权威公开出处，未证实**（T/CSAE 159/YD/T 3707 公开摘要中亦未见显式分片机制条款）。
- **大路口拆分实践**：J2735 生态有实证——Mcity 仓库将大地图拆为 ISD 工具兼容的 "child map files"；中国侧 MapData 天然支持一帧多 Node（T/CSAE 159 要求 NodeList "一个或多个节点"），但"大路口拆分为多帧/精简节点"的公开成文规范未检索到，属工程惯例（未证实为标准条款）。

---

## 本方向调研结论（要点）

1. 中国 MAP 的规范基座是 T/CSAE 53-2020 + YD/T 3709-2020（内容同源，五消息/UPER），RSU 播发行为由 T/CSAE 159-2020 约束（MAP ≤1s、Priority 16、AID 3618、附录 D 抽点精度）——转换工厂的中国侧输出应以这三份为验收依据。
2. 消息层未单独国标化，但已被 GB/T 44417-2024（路侧）与 GB/T 45315-2025（车载）以引用方式纳入国标体系；跟踪 YD/T 3709 修订即可，暂无"新国标 MapData"风险。
3. 三大体系同根（J2735 血统）：中国用 phaseId、美欧用 signalGroup 作为 MAP↔SPAT 关联键；几何都是"参考点+偏移点列"，因此**一套内部中间模型可同时出 CSAE MAP / J2735 MAP / ETSI MAPEM 三种目标**。
4. 最新版本坐标：SAE J2735_202409（2024-09），ETSI TS 103 301 V2.2.1（2024-08，ASN.1 在 ETSI Forge 免费可得），ISO/TS 19091:2019（2024 确认现行）；美国正向 J2945/A RGA 演进，建议架构上为 RGA 预留输出插槽。
5. **核心空白点确认：全网/GitHub 未发现任何 OpenDRIVE/SHP → MAP(J2735/CSAE/MAPEM) 的自动生成开源工具**——现有 MAP 制作全靠人工 GUI（ConnectedVCS ISD、Commsignia Central、Yunex Map2x）；这正是转换工厂最有价值的切入点。
6. 反向链路已有成熟参照：usdot jpo-geojsonconverter（Apache-2.0，Java）实现 MAP/SPaT → GeoJSON（含车道与连接要素），可直接借鉴其 ProcessedMap 数据模型做我们的 MAP→GeoJSON 可视化。
7. ASN.1 工具选型：Python 线 pycrate（LGPL-2.1，UPER 全支持，已被公开项目证明能编译 YD/T 3709 的 asn）为首选；C 线 asn1c 用 mouse07410 fork；商业兜底 OSS Nokalva / Objective Systems。中国标准 asn 文本无官方开源，需购买标准后自行提取（GitHub 上的 cv2x 仓库可作校对参照但无许可证）。
8. 国内商业厂商（星云互联/万集/金溢/中信科智联等）均有 MAP 播发生态，但公开可查的独立"MAP 制作工具/数据服务"产品页几乎没有（金溢有第三方配置教程佐证）；国外 Commsignia Central 是公开证据最充分的商用 MAP 编辑器——竞品压力小，工具化空间大。
9. SPAT 对接：美国走 NTCIP 1202（V2X-Hub SPaT Plugin 开源可参照），中国走 GA/T 1743-2020（+GA/T 2151-2024）；MAP 侧 phaseId/signalGroup 分配必须与信号机配时方案协同设计，这是转换工厂输出 MAP 时唯一无法从地图数据自动推导、需要外部输入的字段。
10. 尺寸/分片：MAP 典型数百字节至 1KB+，受单包约束，大路口靠"多 Node/child map 拆分+抽稀"处理（Mcity 有公开样例）；LTE-V2X PC5 单帧确切字节上限未找到权威公开数值（未证实），建议转换工厂内置"目标字节预算 + 按 T/CSAE 159 附录 D 精度约束的自动抽稀/拆分"能力，而不是依赖具体上限数字。
