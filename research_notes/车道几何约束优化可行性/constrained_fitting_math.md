# 带硬容差与光顺约束的分段多项式/回旋线拟合：数学表述、凸性边界与非凸部分的成熟手段

> 调研日期 2026-10-10。访问限制：本会话出口代理对 arxiv.org、ar5iv、web.stanford.edu、dpapp.math.ncsu.edu、projecteuclid.org 等返回 403/DNS 失败（组织策略），只有 github.com / raw.githubusercontent.com 可直接读取。因此标注“（搜索摘要）”的条目只来自搜索引擎对原文的摘要，**未能核对原文**；GitHub 上的 LICENSE / README / 源码是直接读取的一手内容。
> 本地实测：在会话草稿区单独建的 venv（Python 3.13.16、numpy 2.5.3、scipy 1.18.1、cvxpy 1.9.3、clarabel 0.11.1、osqp 1.1.3、highspy 1.15.1，4 核 Linux 容器）上，用**合成数据**跑了 (a) 偏移曲线曲率公式的数值验证与线性化误差表，(b) 稠密节点 C2 三次样条 QP 基准。下文写作“本地实测”。它不是项目代码，也不使用项目数据；项目锁定的 Python 3.11 / uv 环境未改动。
> 结构约定：“Cited Findings”只放有出处的事实（含本地实测）；“Inferences”是对本项目（参考线固定，在 s–t 坐标拟合 laneOffset(s) 与各车道 width(s) 的逐记录三次多项式）的适用性判断。

## Q1 固定节点的约束最小二乘 / QP 样条拟合：L∞ 容差带（采样点线性约束）、C2（线性等式）、最小化 ∫(t'')² 或 ∫(t''')²

### Takeaway
节点（记录分界 s 位置）固定时，三次分段多项式对其系数是线性的：采样点容差带 |t(s_i)−y_i|≤tol 是线性不等式，C0/C1/C2 拼接是线性等式，∫(t'')²、∫(t''')² 是半正定二次型，因此整个问题是凸 QP（有唯一解当目标严格凸），这是机器人最小 snap 轨迹、P-spline、Apollo Frenet 路径 QP 的共同骨架；“整个区间而不只是采样点”满足容差带可用 Bernstein 控制点的凸包性质，仍是线性约束。本地实测：300 m 边界、600 段、约 1.8k 变量的此类 QP 用 Clarabel 0.35 s 解完，重复求解逐位相同。

### Cited Findings
- 最小 snap 轨迹（Mellinger & Kumar, ICRA 2011）的核心是一个 QP：在多项式系数上最小化二次代价，约束为途经点等式与段间连续性等式；作者对 x、y、z、yaw 分别解一个优化问题 —（搜索摘要）[Generating Minimum-Snap Quadrotor Trajectories Really Fast, arXiv 2008.00595](https://arxiv.org/pdf/2008.00595)；[Richter, Bry, Roy ISRR 2013](https://groups.csail.mit.edu/rrg/papers/Richter_ISRR13.pdf)
- Richter、Bry、Roy 把分段多项式联合优化改写为**无约束** QP，称其对高阶多项式和大量分段数值稳定；每段时长（相当于节点位置）由外层按“激进度”参数自动选择，不在 QP 内 —（搜索摘要）[Richter ISRR13](https://groups.csail.mit.edu/rrg/papers/Richter_ISRR13.pdf)
- PyPI 包 minsnap-trajectories 同时实现了 Mellinger–Kumar 原始方法与 Bry/Roy 的数值稳定无约束 QP —（搜索摘要）[minsnap-trajectories](https://pypi.org/project/minsnap-trajectories)
- P-spline（Eilers & Marx 1996）= B 样条基回归 + 系数差分罚；罚项是离散的，任意阶差分都容易使用 —（搜索摘要）[Eilers & Marx, Splines, Knots and Penalties](https://www.stat.cmu.edu/~brian/valerie/617-2022/617-2021/0%20-%20nonparametric%20methods/splines/Splines,%20Knots%20and%20Penalties%20(Eilers%20&%20Marx).pdf)
- cpsplines 把形状约束作为硬约束，问题表述为带二次目标的凸锥优化 —（搜索摘要）[cpsplines (PyPI)](https://pypi.org/project/cpsplines)
- Bertolazzi、Frego、Biral（Math. Comput. Simul. 2020）用三次多项式对有序噪声点做最小二乘 + Tikhonov 正则平滑，正则权用广义交叉验证选取，应用是从 GPS / 激光雷达重建道路边界 —（搜索摘要）[IDEAS/RePEc 记录](https://ideas.repec.org/a/eee/matcom/v176y2020icp36-56.html)
- Bernstein/Bézier 曲线位于其控制点的凸包内，约束控制点即约束整段；Gao 等（ICRA 2018，Btraj）用此把分段 Bézier 曲线**整段**限制在飞行走廊与动力学限内；Park & Kim（2019）用 SFC/RSFC 的线性约束把整个规划写成一个 QP —（搜索摘要）[Btraj 仓库](https://gitee.com/shoufei/Btraj)；[Park & Kim, arXiv 1909.10219](https://arxiv.org/pdf/1909.10219)
- Apollo 的 piecewise-jerk 路径 QP 在 Frenet 框架下沿引导线的空间参数离散，每点一个横向偏移；用 OSQP 实现；150 m 视野、0.5 m 分辨率，平均每周期约 15 ms，在合成 Dreamland 场景 100% 成功 —（搜索摘要）[Zhang et al., IV 2020, arXiv 2112.02132](https://arxiv.org/pdf/2112.02132)
- 本地实测（合成数据）：边界 t(s) = 0（s<100）、1:4 渐变到 3.5 m（100–114 m）、末 20 m 路缘外展 0.5 m（二次）、σ=2 cm 噪声，采样步长 0.25 m；均匀三次 B 样条（自动 C2）节点间距 h=0.5 m（600 段）；目标 Σ(t'')²·Δs + λ·Σ|t''' 在节点处的跳变| + 1000·Σ松弛；约束 |t''|≤0.02 /m（直参考线下的线性化曲率界）、|t−y|≤0.10 m + 松弛。规模与耗时（Clarabel，cvxpy 端到端墙钟）：h=0.5 → 1804 变量/2402 约束，0.42 s；h=0.25 → 3604/4802，1.12 s；h=0.10 → 9004/12002，5.1 s。三种规模各重复解两次，系数 SHA-256（12 位舍入后）**逐位相同**。—本地实测

### Inferences
- OpenDRIVE 映射：laneOffset 与每条车道的 width 都是“按 sOffset/ds 分段的三次多项式”，最直接的变量化是每条记录 4 个系数 (a,b,c,d) + 记录分界处 C0/C1/C2 三个线性等式（与 OpenDRIVE 记录 1:1，写出时无需转换）；B 样条形式自动 C2、变量更少，但写出前要换算成逐段幂基系数（线性、无损）。两种都是凸 QP。
- 第 k 条车道边界 t_k(s) = laneOffset(s) + Σ_{j≤k} width_j(s)（带号），对所有变量**线性**；所以“每条边界都在源边界多段线的容差带内”即使变量是宽度也仍是线性约束，可把 laneOffset 和所有车道宽度放进同一个 QP 联合求解（而不是逐条依次拟合）。
- 采样点约束与连续约束之差可定量控制：若采样点集合包含源多段线的全部顶点，则相邻采样点间源边界是直线，误差 e(s)=t(s)−y(s) 的二阶导等于 t''，线性插值误差界给出采样点间超出量 ≤ Δs²/8·max|t''|；Δs=0.25 m、|t''|≤0.02 /m 时 ≤ 0.16 mm。因此“采样点 + 源顶点”约束在工程上等价于连续约束，不必上 Bernstein 包络（这是推导，非引用）。
- 容差在 s–t 中按横向差 Δt 计量；对一条切向偏角 Δθ（tanΔθ = t'/(1−κ_r t)）的曲线，到曲线的法向距离约为 |Δt|·cosΔθ ≤ |Δt|，故横向带是保守的（|t'|=0.25 时差约 3%）。
- ∫(t'')² 约束“弯曲能量”（≈曲率平方），∫(t''')² 约束“曲率变化率”（与回旋线 dκ/ds 对应）。对 C2 三次段 t''' 段内为常数，∫(t''')² = Σ 36·d_i²·ℓ_i，同样是二次型；两者可加权组合。当前启发式里“G2 拐角按固定曲率目标定尺寸”在 QP 中对应的是 |t''| 上界（硬）或 ∫(t'')² 权（软），不再需要逐个拐角规则。
- 口部端点与连接路在位置/航向/曲率上匹配：若参考线在口部已匹配，则端点处 t、t'、t'' 的给定值就是线性等式（见 Q3 的曲率式，曲率对 t'' 仿射），是凸的。

### Gaps
- Mellinger & Kumar 原文、Richter 等原文正文未能直接阅读（arxiv/学校站点被拦），只有搜索摘要。
- 未找到专门研究“L∞ 容差带 + C2 + ∫(t''')² 的固定节点 QP”的教科书条款（de Boor、Dierckx 相关章节本次未能访问）；采样点间超出量的界是本笔记自行推导。
- 本地实测只覆盖单条边界、直参考线、合成数据；多车道联合（变量 ×(车道数+1)）与真实 SHP 数据未测。

## Q2 多项式段在区间上的非负性：Bernstein 系数充分条件 vs SOS（半定）精确条件，哪种在 QP 里实用

### Takeaway
Bernstein 系数非负是**线性**充分条件（每个三次段 4 个线性不等式），能直接进 QP；它对“在端点为零”的情况是精确的（零宽出生/消失点正好是端点），只在区间**内部**触零或接近零时保守，靠细分/升阶可收敛。SOS（Markov–Lukács）给出精确条件，但需要锥约束：对三次多项式恰好可化为两个 2×2 半正定块，即二阶锥（SOCP），OSQP 不支持，Clarabel/ECOS/SCS 支持。

### Cited Findings
- Papp & Alizadeh（JCGS）用多项式样条在形状约束下估计光滑函数，方法基于非负多项式的刻画，导出 SDP 与 SOCP 表述；也考虑了更简单的做法：用“在非负基下系数非负”的多项式段来近似非负样条 —（搜索摘要）[Papp & Alizadeh, Shape constrained estimation using nonnegative splines](https://dpapp.math.ncsu.edu/pub/shape_constrained_jcgs.pdf)
- Markov–Lukács 定理：区间 [a,b] 上非负的一元多项式，偶次可写为 s(x) + (x−a)(b−x)·t(x)，奇次可写为 (x−a)·s(x) + (b−x)·t(x)，s、t 为平方和（SOS）；每个平方和对应半正定 Gram 矩阵，由此得到半定表示 —（搜索摘要）[Parrilo, MIT 6.972 Lecture 10](https://ocw.mit.edu/courses/6-972-algebraic-techniques-and-semidefinite-optimization-spring-2006/a4337243b7dfbd98173565dfca7d6ce9_lecture_10.pdf)
- Bernstein 系数全部非负即是非负性证书；升阶后非负系数仍非负；严格正的多项式在足够高的升阶下系数全为正；系数给出值域上下界，升阶时单调收敛，细分（subdivision）改进更快 —（搜索摘要）[arXiv 1710.05735](https://arxiv.org/pdf/1710.05735)；[Interval Computations 1993(2):154–168](https://www.reliable-computing.org/archive/reliable-computing-journal/1993/interval-computations-1993-2-pp-154-168.pdf)
- 在孤立点为零的非负多项式（单纯形上，多元情形）可能**不存在** Bernstein 非负系数证书 —（搜索摘要）[Nonnegative Polynomial with no Certificate of Nonnegativity in the Simplicial Bernstein Basis, arXiv 1710.05735](https://arxiv.org/pdf/1710.05735)
- Clarabel 求解 LP、QP、SOCP、SDP 及指数锥/幂锥；OSQP 只解 `min ½xᵀPx+qᵀx s.t. l≤Ax≤u`（纯线性约束 QP）；ECOS 解 SOCP —（一手 README）[OSQP README](https://github.com/osqp/osqp)；[Clarabel.rs README](https://github.com/oxfordcontrol/Clarabel.rs)；[ECOS README](https://github.com/embotech/ecos)

### Inferences
- 三次段 w(s)，s∈[0,ℓ] 的 Bernstein 控制值（推导）：b0 = w(0)，b1 = w(0) + (ℓ/3)·w'(0)，b2 = w(ℓ) − (ℓ/3)·w'(ℓ)，b3 = w(ℓ)。约束 b0..b3 ≥ 0 对系数线性，每段 4 个不等式（与段长无关）。
- 零宽出生/消失：若某车道在出生点之前宽度恒为 0 且要求 C2，则出生段必为 w = c·u³（w(0)=w'(0)=w''(0)=0），Bernstein 系数 (0,0,0,c·ℓ³)，条件 c≥0 恰好精确，无保守性。端点为零不触发“孤立零点无证书”的问题（那是区间内部/多元单纯形情形）。
- 保守性只在宽度在**区间内部**降到接近零时出现；例：(u−½)² 在 [0,1] 上的二次 Bernstein 系数为 (¼, −¼, ¼)，升到三次仍为 (¼, −1/12, −1/12, ¼)，即非负却不被证实（本笔记手算）。车道宽度在区间内部触零而两端不为零的情形在本项目里本身就是“非法几何”，因此保守性实际影响很小；若需要，可把该段在内部再切一刀（细分）。
- 精确 SOS 版本对三次：w = (u)·σ1(u) + (ℓ−u)·σ2(u)，σ1、σ2 为一次多项式的平方和 → 各对应一个 2×2 PSD Gram 矩阵；2×2 PSD ⇔ 旋转二阶锥（x11≥0, x22≥0, x11·x22≥x12²）。所以每段 2 个小 SOC + 线性系数匹配，Clarabel 可解，但 OSQP 不行。考虑到 Bernstein 在本项目的典型情形（端点触零或远离零）已精确或近似精确，**QP 内首选 Bernstein**，SOS 仅在需要证明“内部触零”时作为校验手段。
- 采样点 w(s_j) ≥ 0 加稠密采样是第三种实用做法：不是证书，但段内违反量 ≤ Δs²/8·max|w''|（同 Q1 推导），可与 Bernstein 共存作冗余。

### Gaps
- Papp & Alizadeh 正文中 SDP 与“非负基系数”方法的数值对比、运行时间未能读到（站点被拦）。
- 一元区间情形下 Bernstein 证书在内部双重零点处“任何升阶都失败”的严格定理表述本次未找到原文；上面的例子是手算。

## Q3 Frenet 框架下偏移曲线的曲率：精确式、常用线性化、SCP/SQP，及 |t'|≈0.3 时线性化精度

### Takeaway
偏移曲线 p(s)=r(s)+t(s)N(s) 的有符号曲率精确式为 κ = [(1−κ_r t)²κ_r + (1−κ_r t)t'' + t'(κ_r' t + 2κ_r t')] / ((1−κ_r t)² + t'²)^{3/2}；它对 t'' **仿射**（固定 t、t' 时），但对 (t,t',t'') 整体非凸。直参考线下“κ≈t''”线性化把 |κ| 高估 (1+t'²)^{3/2} 倍（|t'|=0.25 时 9.5%，0.3 时 13.8%），作为上界约束是**保守**的；弯参考线上丢掉 t' 项的线性化在 |t'|≤0.3 时误差约 −8%～+13%（κ_r t 到 0.7；κ_r t≤0.35 时在 ±6.3% 内），而一阶全线性化 κ≈κ_r+κ_r²t+t'' 在 κ_r t=0.7 时低估 50% 以上，且忽略 κ_r' 会差 20% 以上，此时应采用“固定上一轮 t、t'，对 t'' 线性”的 SCP 迭代。

### Cited Findings
- Werling、Ziegler、Kammel、Thrun（ICRA 2010, pp. 987–993）是 Frenet 框架轨迹生成及全局↔Frenet 坐标换算式的常用出处 —（搜索摘要，正文未读到）[Semantic Scholar 记录](https://www.semanticscholar.org/paper/Optimal-trajectory-generation-for-dynamic-street-in-Werling-Ziegler/6bda8fc13bda8cffb3bb426a73ce5c12cc0a1760)；[MathWorks 讨论](https://in.mathworks.com/matlabcentral/answers/739037-frenet-coordinate-system-definition)
- Apollo `cartesian_frenet_conversion.cc`（一手源码，直接读取）：`one_minus_kappa_r_d = 1 - rkappa * d`；`tan_delta_theta = d' / one_minus_kappa_r_d`；`kappa = (((d'' + kappa_r_d_prime * tan_delta_theta) * cos²Δθ) / one_minus_kappa_r_d + rkappa) * cosΔθ / one_minus_kappa_r_d`，其中 `kappa_r_d_prime = rdkappa*d + rkappa*d'`；另一函数 `CalculateKappa` 的分子为 `rkappa + ddl - 2*l*rkappa² - l*ddl*rkappa + l²*rkappa³ + l*dl*rdkappa + 2*dl²*rkappa`，分母基于 `dl² + (1 - l*rkappa)²` —（一手源码）[Apollo cartesian_frenet_conversion.cc](https://github.com/ApolloAuto/apollo/blob/master/modules/common/math/cartesian_frenet_conversion.cc)
- 平面 Frenet 框架仅在 1 − κ·y > 0 时正则 —（搜索摘要）[Models and Predictive Control for Nonplanar Vehicle Navigation, arXiv 2104.08427](https://arxiv.org/pdf/2104.08427)
- 本地实测（公式验证）：上述精确式与对偏移曲线做数值差分得到的曲率在 4 组参数上一致到 6 位小数（例：κ_r=0.1、κ_r'=−0.005、t=5、t'=0.3、t''=0.02 → 0.229505 vs 0.229505）；Apollo `CalculateKappa` 的分子展开与该式逐项相同（本笔记手工展开核对）。—本地实测
- 本地实测（线性化误差，相对精确值；L0 = κ_r/(1−κ_r t) + t''/(1−κ_r t)²，即丢掉所有 t' 项；L1 = κ_r + κ_r² t + t''，即一阶小量全线性化）：
  - κ_r=0（直参考线），t''=0.01：|t'|=0.05/0.1/0.2/0.25/0.3 时 L0=L1 高估 +0.38%/+1.5%/+6.1%/+9.5%/+13.8%（即因子 (1+t'²)^{3/2}）。
  - κ_r=0.02 /m（R=50 m），t∈{1.75,3.5,7} m，|t'|≤0.3：L0 误差在 −4.5%～+3.0%；L1 在 −10.7%～−0.3%（低估，不保守）。
  - κ_r=0.1 /m（R=10 m，路口转弯量级）：t=3.5 m 时 L0 −6.3%～−0.5%，L1 −20%～−13%；t=7 m 时 L0 −8.1%～+13.1%，L1 约 −51%～−59%（完全失效）。
  - κ_r' 的影响：κ_r=0.05、t=3.5、t'=0.25、t''=0 时，κ_r'=0 → κ=0.0629；κ_r'=0.002 → 0.0656；κ_r'=0.01 → 0.0765（+22%）。—本地实测

### Inferences
- 推导（与 Apollo 一致）：设 a=1−κ_r t，则 p' = aT + t'N，p'' = (a' − κ_r t')T + (aκ_r + t'')N，κ = (p'×p'')/|p'|³ 即得上式。世界弧长 dσ = √(a²+t'²)·ds；世界曲率变化率 dκ/dσ = (dκ/ds)/√(a²+t'²)，其中 dκ/ds 含 t'''，在 (t,t',t'') 固定时对 t''' 仿射——所以“边界曲率光顺”（Q1 中 ∫(t''')²）在 SCP 里同样是二次的。
- 凸性边界：|κ|≤κ_max 在 (t,t',t'') 联合空间中非凸（分母含 t'²，分子含 t'²、t·t'、t·t''）。可凸的部分：给定 (t̄, t̄') 后 κ 是 t'' 的仿射函数 → |κ|≤κ_max 是两条线性不等式。
- 推荐的 SCP（序列凸规划）方案：第 k 轮用上一轮解的 (t̄,t̄') 在每个采样点算 A_i、B_i 使 κ_i ≈ A_i + B_i·t''_i，加信赖域 |t−t̄|≤ρ、|t'−t̄'|≤ρ'，解 QP，收敛判据为精确曲率违反量 ≤ 容差。由于直参考线下近似是保守的、弯参考线下在 |t'|≤0.3、|κ_r t|≤0.35 范围内误差 ≤ ~6%，通常 2–3 轮可收敛（这是推断，未在真实数据上验证）。
- 实用简化：道路段（κ_r 小，t' 仅在渐变处到 0.25）可直接用 L0 一次性 QP，事后用精确式复核；连接路（κ_r 可达 0.1、κ_r' 非零、外侧车道 t 大）必须用 SCP 或至少保留 κ_r' t t' 项，否则 L1 类线性化会低估 50% 以上。
- 1:4 渐变（t'=0.25，偏角约 14°）处“精确曲率 ≠ t''”的偏差约 10%：若评分板/门禁用的是世界曲率，而 QP 用 t'' 界，直线段上 QP 会稍偏保守，弯段上需 SCP 复核，二者不能混用口径。

### Gaps
- 未能读到 Werling 2010 原文附录与 Zhang 2020（Apollo 路径 QP）里曲率约束的具体线性化写法（站点被拦），只能以 Apollo 源码 + 自行推导 + 数值验证代替。
- 未找到关于“Frenet 曲率约束 SCP 收敛性/迭代次数”的专门文献；上面的“2–3 轮”是推断。

## Q4 节点位置 / 段数（“少段”要求）：自由节点非凸；稠密固定节点 + 稀疏（L1/TV）、混合整数、贪心插入/删除、动态规划分段——哪些结果确定

### Takeaway
自由节点样条问题非凸、局部极小多；成熟的凸替代是“稠密固定节点 + 对 t''' 跳变（= t'' 的全变差 / 三阶导 TV）做 L1”（趋势滤波 / 局部自适应回归样条），它让大多数节点失活、得到少段 C2 三次样条，且是凸问题、求解确定；但 L1 只是近似 L0：阈值 + 重拟合会丢掉“小而必要”的节点，需要以可行性为驱动的插入修复，且可能产生间距很近的节点簇，“最短段长”本身是组合约束。精确最少段数要靠 MIQP/L0 或带边界状态的动态规划，确定性好但代价高。

### Cited Findings
- 自由节点最小二乘样条“已知有大量远离全局最优的局部极小点”；驻点条件不充分，可能有多个全局/局部极小 —（搜索摘要）[Sukhorukova & Ugon, arXiv 1412.2323](https://ar5iv.arxiv.org/html/1412.2323)；[arXiv 2003.03847](https://www.arxiv.org/pdf/2003.03847v2)
- 一种补救是把问题近似为组合问题并用分支定界求解，作者报告可在合理时间内解出现实规模问题 —（搜索摘要）[KIT 出版物](https://publikationen.bibliothek.kit.edu/1000157075/150480567)
- Arge、Dæhlen、Lyche、Mørken（1990, Algorithms for Approximation II）有“约束下的数据驱动节点删除”工作 —（仅书目记录）[Simula 记录](https://www.simula.no/publications/constrained-approximation-functions-and-data-based-constrained-knot-removal)
- Bellman & Kotkin（RAND）用动态规划以有限条线段逼近给定函数；Gluss（CACM 1962）推广到要求段端点落在曲线上；R 包 dpseg 用动态规划做分段线性分割 —（搜索摘要）[RAND RM-2978](https://www.rand.org/pubs/research_memoranda/RM2978.html)；[Gluss, CACM 1962](https://cacmb4.acm.org/magazines/1962/8/14182-further-remarks-on-line-segment-curve-fitting-using-dynamic-programming)；[dpseg](https://raim.r-universe.dev/dpseg/doc/dpseg.html)
- ℓ1 趋势滤波（Kim, Koh, Boyd, Gorinevsky, SIAM Review 51(2):339–360, 2009）把 Hodrick–Prescott 滤波的平方和罚换成绝对值和，估计是分段线性的，拐点可解释为突变事件；内点法每次运算量随数据点数线性增长 —（搜索摘要）[l1 Trend Filtering](https://web.stanford.edu/~boyd/papers/l1_trend_filter.html)
- k 阶趋势滤波（Tibshirani, Ann. Statist. 42(1):285–323, 2014）罚 k 阶离散导数的绝对值和，估计看起来像“节点由数据自适应选择的 k 次样条”；与罚 k 阶导数全变差的局部自适应回归样条（Mammen & van de Geer 1997）高度相似；对 k 阶导数有界变差的函数达到极小极大收敛率 —（搜索摘要）[Tibshirani 2014, Project Euclid](https://projecteuclid.org/journals/annals-of-statistics/volume-42/issue-1/Adaptive-piecewise-polynomial-estimation-via-trend-filtering/10.1214/13-AOS1189.pdf)
- L0 趋势滤波（Wen, Wang, Zhang, INFORMS J. Computing 2023）把变点最大个数作为约束，用 splicing 算法求解，称在变点检测上优于其他趋势滤波 —（搜索摘要，经 CRAN 包文档）[L0TFinv (CRAN)](https://cran.ma.ic.ac.uk/web/packages/L0TFinv/refman/L0TFinv.html)
- Richter 等的分段时长（节点位置）是在 QP 之外由梯度下降类外层选择的 —（搜索摘要）[Richter ISRR13](https://groups.csail.mit.edu/rrg/papers/Richter_ISRR13.pdf)
- 本地实测（Q1 同一合成问题，h=0.5 m、600 段，λ=1 的 L1 罚 t''' 跳变）：
  - Clarabel：|跳变| > 1e-4·max 的节点 19 个（> 1e-6 绝对值 36 个），即数值上近似稀疏而非严格为零。
  - OSQP 默认设置：迭代上限终止（`user_limit`，2.1 s），同一阈值下 407 个“活跃”节点——ADMM 的中等精度破坏了稀疏模式；收紧到 eps=1e-7、polishing、max_iter=2e5 后 42.8 s 仍为 `user_limit`，但稀疏模式恢复为 19 个。
  - 两阶段（阈值选节点 → 用所选节点做显式 C2 分段三次重拟合，OpenDRIVE 记录式）：相对阈值 1e-4/1e-3/1e-2/5e-2 得 9/9/7/6 个节点簇；重拟合后最大偏差 0.62–0.63 m，松弛样本从渐变区扩散到 s=85–300 m——末端路缘外展起点（s=280）的 t''' 跳变很小，被阈值丢掉了。
  - 以“对最坏违反所在段二分插入节点”做 4 轮修复后：14 段 / 300 m，松弛样本只剩渐变冲突区（56 个，与第一阶段一致），最大偏差 0.338 m；但得到了 106.5/107.0/107.5 m 这样 0.5 m 间距的节点簇（密集短段）。“对最坏违反点插节点”的朴素贪心在同一处反复插入而卡住（12 轮无进展），改为二分违反段才收敛。—本地实测

### Inferences
- 对本项目的关键等价：C2 三次样条中，节点“有效”⇔ t''' 在该处有跳变。所以“段数少”= “t''' 跳变非零的个数少”= L0；其凸松弛就是对稠密均匀节点（如 0.25–0.5 m）上的 t''' 跳变做 L1（= t'' 的全变差，k=3 的局部自适应回归样条）。laneOffset 与每条 width 各自一组跳变；若希望各车道记录分界对齐（减少 OpenDRIVE 记录数），可对“同一位置所有函数的跳变向量”做 group-lasso（每节点一个 ℓ2 范数，SOCP，OSQP 不支持）。
- 确定性排序：(1) 动态规划分段在段代价可分时给出全局最优且完全确定，但 C2 拼接耦合相邻段，标准 DP 失效，只能把分界点上的 (t,t',t'') 离散化进状态（近似、状态爆炸）；(2) 严格凸 QP（L2 目标 + 线性约束）解唯一，任何正确求解器都收敛到同一解，配同一二进制/单线程可逐位复现（本地实测 Clarabel 如此）；(3) 含 L1 项的问题可能有非唯一最优解（LP 型退化），不同求解器/版本可能给出不同的稀疏模式——需要固定求解器与版本，或加小的 L2 正则使目标严格凸；(4) 贪心插入/删除确定但依赖平局规则与顺序，需显式规定；(5) MIP 在分支定界下可确定（单线程/确定模式），但运行时间不可预测。
- 推荐流水线（推断）：稠密节点 L1 QP（Clarabel，含硬光顺约束与软容差带）→ 合并 1 m 内节点簇（跳变加权质心）→ 显式记录式 C2 重拟合 → 违反驱动的“二分违反段”插入循环 → 最短段长检查（低于阈值则合并并重解）。全过程确定。L1 权 λ 的选取应按“段数–最大偏差”曲线固定一次而非逐路口调。
- OSQP 不适合作稀疏选择阶段（精度不足导致稀疏模式失真），适合作固定节点重拟合阶段（纯 QP、可热启动）。

### Gaps
- L0 趋势滤波原文、Lyche–Mørken 节点删除算法正文未读到。
- 未找到“C2 约束下带最小段长的最优分段”的专门 DP/MIP 文献。
- 本地两阶段实验只做了一条合成边界，λ、阈值、簇合并半径均为随手选值，未调参。

## Q5 离散事件（车道零宽出生/消失点）位置作为变量：如何保持凸（固定候选网格、凸松弛）vs 混合整数

### Takeaway
出生/消失点位置一旦作为连续变量，“该点之前 w≡0、之后 w≥0”就成了非凸（互补型）约束；保持凸的两种成熟做法是：(a) 在固定候选网格上枚举出生点、每个候选解一个凸 QP、取最优（确定、可并行、在网格上全局最优），(b) 稠密节点上对宽度本身加 L1/非负约束让宽度在一段上“自然”精确为零（凸松弛，零点位置由解决定）；精确建模需二进制变量（big-M 的 MIQP），而 HiGHS 只解 MILP（二次目标时不支持整数），需要把目标线性化或换求解器。另外 C2 + |t''|≤κ_max 本身给 1:4 渐变设了下限：最小切角偏差 ≈ (Δt')²/(8κ_max)，κ_max=0.02 时约 0.39 m，这是结构性冲突，不是算法问题。

### Cited Findings
- HiGHS 求解 LP、凸 QP 与 MIP，其中“若 Q 为零，可要求部分变量取整数”——即整数变量只支持线性目标（MILP），不支持 MIQP —（一手 README）[HiGHS README](https://github.com/ERGO-Code/HiGHS)
- ECOS 附带分支定界扩展 ECOS_BB 解混合整数/混合布尔 SOCP，README 自述为“以可接受速度解小问题、附加代码最少（约 200 行）” —（一手 README）[ECOS README](https://github.com/embotech/ecos)
- 自由节点/离散位置问题可近似为组合问题用分支定界求解 —（搜索摘要）[KIT 出版物](https://publikationen.bibliothek.kit.edu/1000157075/150480567)
- 本地实测（Q1 合成问题，tol=0.10 m、|t''|≤0.02）：硬容差带版本被 Clarabel 在 0.05 s 内判为 `infeasible`；软带版本的松弛恰好集中在渐变区 s=92.25–121.75 m（56 个样本），最大松弛 0.238 m、最大偏差 0.338 m。—本地实测

### Inferences
- C2 出生的形状约束：在零宽段之后要求 C2，则出生段 w = c·u³（Q2），斜率需经过 ≥ Δt'/κ_max 的长度才达到源渐变斜率；对 1:4 渐变（Δt'=0.25），抛物线过渡的最小切角偏差 δ_min ≈ (Δt')²/(8κ_max)：κ_max = 0.01/0.02/0.05/0.1 /m → 0.78/0.39/0.16/0.08 m，过渡长 25/12.5/5/2.5 m（本笔记推导+计算；本地实测的 0.338 m 与 0.39 m 量级一致）。因此若容差带 0.1 m，曲率界必须放到 ≈0.08 /m 以上才可能可行——“忠实 1:4 渐变”和“世界车道边缘曲率小”在源中就互相冲突，必须由软约束（Q7）显式记录。
- 固定候选网格（推荐，确定）：候选出生点 s_b ∈ {源渐变起点 ± k·Δ}（Δ 如 0.5 m，范围如 ±一个过渡长度），对每个候选：零宽段 w≡0（等式）、出生段 w=c·u³ 且 c≥0、其余同 QP；解 N 个独立 QP，按（总松弛, 光顺目标）字典序取最优，平局按 s 最小。N≈20–50、每个 0.1–1 s 级，可并行。这把当前 `lane_birth_advance`（“提前半个转角”）这类固定规则换成“在网格上搜索最优提前量”，并可把每个候选的代价写进报告。
- 凸松弛（稠密节点）：w(s_j) ≥ 0（Bernstein）+ 在源标为零宽的区域对 w 加 L1 罚或等式 w=0；出生点由解中“w 从 0 变正”的位置读出。问题：L1 会把 w 在出生点附近整体压小（偏置），需重拟合去偏；出生点位置可能落在节点之间，仍需取整到节点。
- MIQP（精确）：二进制 z_j（“在 s_j 之前已出生”）单调 z_j ≤ z_{j+1}，w(s_j) ≤ M·z_j。只有在出生点与其它离散选择（如多个车道出生顺序）相互耦合、网格枚举组合爆炸时才值得；求解器需 MIQP（HiGHS 不行；SCIP/商业求解器另行核实许可），或把 ∫(t'')² 改为 L1/L∞ 光顺度量后用 HiGHS 的 MILP。
- 项目约束提醒（推断，非研究项目本身）：“9 处已批准零宽出生来源角色只在原对象生效”这类人工批准结果在优化里应体现为对应车道的**固定等式**（出生点固定），而不是候选网格变量。

### Gaps
- 未找到把“车道出生点位置”作为优化变量的道路几何文献；上面方案为通用技巧的移植。
- SCIP（MIQP 能力、当前许可证）本次未能核实。

## Q6 回旋线 / G2 样条拟合文献（Bertolazzi & Frego “G1 fitting with clothoids”、回旋线样条、Clothoids 库许可）与按容差拟合

### Takeaway
Bertolazzi & Frego 的成果是**插值**（G1 Hermite：两点两切向一段回旋线，归结为单变量非线性方程，存在唯一、Newton 几步收敛；G2 Hermite 单段不够，需三段方案；回旋线样条可做曲率连续插值），不是“容差带内少段拟合”；按容差拟合回旋线样条是非凸 NLP（Fresnel 积分），本次未找到成熟的凸表述。对本项目，回旋线只与**参考线**（planView）有关；在固定参考线上拟合 laneOffset/width 时，三次 t(s) 在直参考线上给出 κ≈t''（线性于 s），本身就近似“回旋线型”边界。Clothoids 库为 BSD-2-Clause。

### Cited Findings
- Bertolazzi & Frego, “G1 fitting with clothoids”, Math. Methods Appl. Sci. 38(5):881–897 (2015)：G1 Hermite 插值（过两点、给定单位切向的一段回旋线）原为多解的三元非线性方程组，作者化为单变量标量函数求零，零点区间解析给出、证明解存在且唯一，简单初值下 Newton 迭代几步收敛；在直线/圆弧极限附近用渐近展开避免 Fresnel 积分精度损失 —（搜索摘要）[arXiv 1305.6644](https://arxiv.org/pdf/1305.6644)；[unibz 记录](https://bia.unibz.it/esploro/outputs/journalArticle/G1-fitting-with-clothoids/991007029161601241)
- 单段回旋线自由度不足，一般不能满足 G2 Hermite 数据；“On the G2 Hermite interpolation problem with clothoids”（J. Comput. Appl. Math. 2018）指出 G2 问题不总有一段或两段解，提出三段方案 —（搜索摘要）[arXiv 1305.6644 相关摘要](https://pdf.arxiv.org/pdf/1305.6644)；该论文亦列于 [Clothoids README](https://github.com/ebertolazzi/Clothoids)
- “Interpolating clothoid splines with curvature continuity”, Math. Methods Appl. Sci. 41(4):1723–1737 —（仅书目信息，搜索摘要）；同列于 [Clothoids README](https://github.com/ebertolazzi/Clothoids)
- Clothoids 库：实现回旋线、回旋线样条（G1 与 G2 连续）、圆弧、双圆弧的 G1/G2 拟合；C++11 编写，带 MATLAB mex 接口；含 `src_py` 目录（Python 支持细节 README 未说明） —（一手 README）[ebertolazzi/Clothoids](https://github.com/ebertolazzi/Clothoids)
- Clothoids 库许可：BSD 2-Clause，“Copyright (c) 2020, Enrico Bertolazzi and Marco Frego” —（一手 license.txt）[license.txt](https://raw.githubusercontent.com/ebertolazzi/Clothoids/master/license.txt)
- PyClothoids 许可：MIT，“Copyright (c) 2020 Phillip Dix” —（一手 LICENSE）[PyClothoids LICENSE](https://raw.githubusercontent.com/phillipd94/PyClothoids/master/LICENSE)
- 同组作者对噪声点做平滑时用的是**三次多项式** + Tikhonov 正则 + GCV（道路边界重建），而非回旋线 —（搜索摘要）[Bertolazzi, Frego, Biral 2020](https://ideas.repec.org/a/eee/matcom/v176y2020icp36-56.html)

### Inferences
- 插值 ≠ 拟合：G1/G2 Hermite 求解器适合作“给定端点状态的拼接/过渡件”（如口部端点之间的连接路参考线），不适合直接替代“容差带内少段拟合”。把它用于拟合需要外层非凸搜索（分段点位置 + 端点状态），即本项目现有“逐条择优”式启发式的同类问题。
- 对 lane 几何（本次范围）：参考线固定后，变量 t(s) 是 OpenDRIVE 规定的三次多项式，回旋线不直接出现；由 Q3，直参考线上边界曲率 ≈ t''（对三次 t 线性于 s）——三次 width/laneOffset 在小 t' 时天然给出“曲率线性变化”的回旋线型边界，所以 lane 层面不需要回旋线拟合器。
- 许可：BSD-2 / MIT 均为宽松许可，与项目“GPL 组件只能进程隔离”约束不冲突（推断，按项目规则仍应由用户确认）。

### Gaps
- 未找到“在容差带内用最少段回旋线拟合噪声多段线”的成熟算法（含 Bertolazzi/Frego 2018 论文正文）；该论文本身只拿到书目信息。
- Clothoids 库 Python 绑定的维护状态本次未核实。

## Q7 源冲突导致硬约束不可行时的保真–光顺取舍：带松弛的软约束、Pareto/ε-约束、如何报告放松了哪条约束

### Takeaway
标准做法是把“保真容差带”写成带非负松弛的软约束、对松弛用 **L1** 罚（精确罚：权重有限即可在硬约束可行时恢复硬解，且松弛稀疏、天然定位冲突点），把“光顺/曲率”保持为硬约束或另一优先级；求解后非零松弛的位置与大小就是“哪条约束在哪里放松了多少”的报告，对偶乘子给出灵敏度。L2 罚需要无穷权才精确，不宜用于判定。字典序（先最小化总松弛，再在 ε 内最小化光顺目标）可得确定的优先级解。

### Cited Findings
- ℓ1 罚可以是精确罚函数：在约束收紧问题可行时恢复硬约束解（引 Kerrigan & Maciejowski 2000）；ℓ2 二次罚需要无穷大罚参数才能精确恢复，而 ℓ1 只需有限权 —（搜索摘要）[arXiv 2403.18235](https://arxiv.org/html/2403.18235v3)
- 权重足够大时，软约束问题在硬约束问题可行时给出与之相同的解（松弛为零）；硬约束不可行时软问题仍可行，部分松弛为正 —（搜索摘要）[arXiv 2504.12036](https://arxiv.org/pdf/2504.12036)；[arXiv 1303.1090](https://arxiv.org/pdf/1303.1090)
- 把约束残差的正部定义为松弛变量，可将非光滑罚问题等价改写为光滑 QP —（搜索摘要）[arXiv 2403.18235](https://arxiv.org/html/2403.18235v3)
- Clarabel 用齐次嵌入检测不可行问题；OSQP 是首个能从迭代直接可靠检测原始/对偶不可行的算子分裂 QP 方法 —（一手 README / 搜索摘要）[Clarabel.rs README](https://github.com/oxfordcontrol/Clarabel.rs)；[OSQP 论文页](https://web.stanford.edu/~boyd/papers/osqp.html)
- 本地实测：硬带版本 0.05 s 判不可行；软带版本（L1 松弛权 1000）松弛仅出现在 s=92.25–121.75 m 的 56 个样本，最大 0.238 m；路缘外展区（280–300 m）无松弛——即冲突被精确定位到 1:4 渐变两端的两个拐角。h=0.25/0.10 时为 103/264 个样本（样本更密）、最大松弛 0.237/0.251 m，定位范围一致。—本地实测

### Inferences
- 报告格式（推断）：对每条边界输出 {s 区间, 放松的约束类型（容差带/非负/端点匹配）, 最大与积分松弛量, 该处的对偶乘子（≈每放宽 1 m 容差光顺目标下降多少）}；这正好对应项目“源冲突只记录”的口径，可替代逐条特例规则中的“冲突识别”部分。
- 优先级建议：硬 = C2、宽度非负、零宽等式、口部端点匹配（若可行）；软 = 容差带（L1 松弛，大权）；目标 = 光顺。若端点匹配本身与容差带冲突，用三级字典序（端点 > 容差 > 光顺）分两到三次求解，每级把上一级最优值 + ε 作为约束（ε-约束法），结果确定且可解释。
- L1 松弛 vs L∞ 松弛：L1 让违反集中在少数样本（易定位、可报告“哪里”），L∞（单一最大松弛变量）把违反均摊到整个冲突区（易报告“最多多少”）。两者都是线性，可同时算作报告。
- Pareto 前沿：对 (容差, κ_max) 网格逐点求解即可画出“保真–光顺”前沿；由 Q5 的 δ_min ≈ (Δt')²/(8κ_max)，渐变处前沿可解析估计，可用于事先选定草案阈值而非为候选通过而改阈值。

### Gaps
- Kerrigan & Maciejowski 2000 原文的精确罚权阈值条件（与对偶乘子 ∞-范数的关系）未读到原文，只有转述。
- Boyd & Vandenberghe 中多目标标量化/ε-约束章节本次未能访问（站点被拦），相关陈述属于标准凸优化常识而非本次引证。

## Q8 求解器生态：OSQP、Clarabel、ECOS、SCS、HiGHS（MILP）、CVXPY——许可、确定性、1–10k 变量规模的运行时间

### Takeaway
对本问题（QP / 少量 SOC、1–10k 变量）首选 Clarabel（Apache-2.0，内点法，原生二次目标、支持 SOCP/SDP，精度高、能得到近似稀疏的 L1 解，本地实测 1.8k/3.6k/9k 变量分别 0.4/1.1/5.1 s 且逐位可复现）；OSQP（Apache-2.0，ADMM）快、可热启动，但在本问题上默认精度达不到、L1 稀疏模式失真，适合作固定节点重拟合；ECOS 是 **GPL-3.0**（与项目 GPL 约束冲突，且 cvxpy 1.9.3 已不再依赖它）；SCS（MIT）是一阶锥求解器，精度低于内点法；HiGHS（MIT）有 LP/凸 QP/MILP，但不支持 MIQP，且在本地软带 QP 上 45 s 未解完；CVXPY（Apache-2.0）作建模层。

### Cited Findings
- 许可（一手 LICENSE 文件）：OSQP Apache-2.0 [LICENSE](https://raw.githubusercontent.com/osqp/osqp/master/LICENSE)；Clarabel.rs Apache-2.0 [LICENSE.md](https://raw.githubusercontent.com/oxfordcontrol/Clarabel.rs/main/LICENSE.md)；ECOS GPL-3.0（README：“其他许可可向 embotech 申请”）[COPYING](https://raw.githubusercontent.com/embotech/ecos/develop/COPYING)、[README](https://github.com/embotech/ecos)；SCS MIT [LICENSE.txt](https://raw.githubusercontent.com/cvxgrp/scs/master/LICENSE.txt)；HiGHS MIT [LICENSE.txt](https://raw.githubusercontent.com/ERGO-Code/HiGHS/master/LICENSE.txt)；CVXPY Apache-2.0 [LICENSE](https://raw.githubusercontent.com/cvxpy/cvxpy/master/LICENSE)
- HiGHS README：可选的 HiPO 组件（`highspy-extras`）为 Apache-2.0；`*-mit` 二进制包为 MIT；有 `--threads` 选项 —（一手 README）[HiGHS README](https://github.com/ERGO-Code/HiGHS)
- OSQP 问题形式 `min ½xᵀPx + qᵀx s.t. l ≤ Ax ≤ u`，P 半正定 —（一手 README）[OSQP README](https://github.com/osqp/osqp)
- OSQP 论文（Stellato, Banjac, Goulart, Bemporad, Boyd, Math. Prog. Comp. 12(4):637–672, 2020）：支持因子分解缓存与热启动；作者称通常比内点法快约 10 倍；对问题数据无正定性等要求；初次分解后可无除法运行 —（搜索摘要）[OSQP 论文页](https://web.stanford.edu/~boyd/papers/osqp.html)
- Clarabel：基于新的齐次嵌入的内点法；解 LP/QP/SOCP/SDP 与指数锥、幂锥；处理二次目标不需要上图（epigraph）改写，因而对二次目标问题可显著快于标准 HSDE 内点法 —（一手 README）[Clarabel.rs README](https://github.com/oxfordcontrol/Clarabel.rs)
- SCS：“splitting conic solver”，面向大规模凸锥问题，当前版本 3.3.1 —（一手 README）[SCS README](https://github.com/cvxgrp/scs)
- ECOS：嵌入式 SOCP 求解器；ECOS_BB 分支定界仅面向小问题 —（一手 README）[ECOS README](https://github.com/embotech/ecos)
- 本地 `pip show cvxpy`（1.9.3）：License-Expression Apache-2.0；Requires: clarabel, highspy, numpy, osqp, qdldl, scipy, scs, sparsediffpy（**无 ecos**）；`cvxpy.installed_solvers()` = CLARABEL, SCS, SCIPY, HIGHS, OSQP —本地实测
- 本地实测运行时间（合成单边界问题，见 Q1；cvxpy 端到端，含建模）：
  - Clarabel：1804 变量/2402 约束 0.42 s（求解器自报 0.38 s）；3604/4802 1.12 s；9004/12002 5.1 s；硬带不可行判定 0.05 s；重复两次逐位相同。
  - OSQP 默认：2.1 s 后 `user_limit`（解不精确）；eps=1e-7 + polishing + max_iter=2e5：42.8 s 仍 `user_limit`。
  - HiGHS（经 cvxpy 的 QP 接口）：同一 1804 变量软带 QP 45 s 内未返回（被看门狗终止）。—本地实测
- Apollo 路径 QP（150 m 视野、0.5 m 分辨率）用 OSQP 平均约 15 ms/周期 —（搜索摘要）[arXiv 2112.02132](https://arxiv.org/pdf/2112.02132)。（推断：约 300 个离散点，若每点取 (l,l',l'') 则约 900 变量量级，与本地 h=0.5 m 问题同量级；Apollo 问题无 L1 稀疏项、松弛结构更简单，故 OSQP 在那里表现好不矛盾。）

### Inferences
- 确定性：严格凸 QP 的最优解在数学上唯一；同一求解器版本、同一平台、单线程、同一输入顺序下通常逐位可复现（本地 Clarabel 如此）。跨平台/跨 BLAS/跨版本不保证逐位相同——与项目“Python 3.11 锁定、`sum()` 浮点求和差异会改证据数值”的经验同类，若要进入证据绑定体系，应锁定求解器版本并对输出做舍入（例如 1e-9）后再写出/哈希。
- 本地耗时几乎全是求解器本身：墙钟（含 cvxpy 规范化）与 Clarabel 自报求解时间之差仅 0.04/0.06/0.11 s（1.8k/3.6k/9k 变量）；耗时随规模超线性增长（0.38→1.06→5.0 s），离线批处理可接受。若需更快：降低节点密度（0.5 m 已足以得到 19 个有效节点）、或按道路分块（每条道路/每个车道段独立，口部只通过固定端点等式耦合）。
- OSQP 的问题不是“慢”而是本问题条件数差（尺度相差大的 t 与 t'' 约束、大松弛权 1000）+ ADMM 一阶方法精度有限；做变量/约束缩放或降低松弛权可能改善，但本次未验证。
- 许可层面：Clarabel / OSQP / SCS / HiGHS / CVXPY 均为宽松许可，可直接作库依赖；ECOS 为 GPL-3.0，按项目硬约束第 7 条只能进程隔离或不用（cvxpy 当前版本已不默认安装它）。
- MILP/MIQP：HiGHS 只能 MILP；若坚持 MIQP（Q4/Q5 精确最少段数或出生点），需另行评估 SCIP 等（许可与能力本次未核实），或把二次光顺目标换成 L1/L∞ 后走 HiGHS 的 MILP。

### Gaps
- OSQP 文档中 polishing 精度、确定性说明，Clarabel 的线程/确定性说明本次未能访问 osqp.org / clarabel.org（被拦）。
- HiGHS QP 在本问题上卡住的原因（active-set 对大量不等式约束不利？cvxpy 接口？）未诊断。
- 未测跨机器逐位复现；未测 Windows（项目实际运行环境为 Windows，F:\MapFactory）。
- SCIP 当前许可证与 MIQP 能力未核实。

## 综合：各具体子问题的数学表述、凸性与推荐手段（映射表）

### Takeaway
把现有“RDP 顶点 + 固定曲率目标的 G2 拐角 + 顶点值最小二乘 + 特例规则”替换为单一约束优化是可行的：除“节点数/位置”“出生点位置”“弯参考线上的精确曲率界”三处非凸外，其余全部是线性约束 + 二次目标的凸 QP；三处非凸分别有确定性的标准手段（稠密节点 L1 + 修复循环、候选网格枚举、SCP）。源冲突（1:4 渐变 vs 曲率界）在数学上不可同时满足，应由 L1 软容差带显式定位并报告，而不是靠规则规避。

### Cited Findings
- 各条目出处见 Q1–Q8；本表是对其的汇总，不引入新事实。本地实测部分见 Q1、Q4、Q5、Q7、Q8。

### Inferences
| 子问题 | 变量 / 约束 | 凸性 | 推荐表述 | 主要注意点 |
|---|---|---|---|---|
| laneOffset(s)、width_k(s) 每记录三次、记录处 C2 | 每记录 (a,b,c,d)；拼接 3 个线性等式 | 凸（线性等式） | 显式记录式或 B 样条，联合一个 QP | 节点固定时才凸 |
| 宽度 ≥ 0 | Bernstein 控制值 b0..b3 ≥ 0（每段 4 个线性不等式） | 凸（线性，充分条件） | QP 内用 Bernstein；SOS→SOC 仅作校验 | 仅区间内部触零时保守 |
| 出生/消失处宽度恰为 0 | 零宽段 w≡0 等式；出生段 w=c·u³, c≥0 | 凸（给定位置时） | 位置固定 → 线性等式 | 位置可变 → 见下 |
| 出生/消失点位置 | 离散位置 | 非凸 | 候选网格枚举 N 个 QP 取最优（确定）；或稠密节点 L1 松弛；MIQP 仅在组合耦合时 | 已批准的人工结果作固定等式 |
| 边界在源多段线容差带内 | t_k(s_i) = 线性组合；|t_k − y| ≤ tol + slack | 凸（线性） | 采样点 = 源顶点 ∪ 0.25 m 网格；L1 松弛大权 | 横向带略保守；采样间超出 ≤ Δs²/8·max|t''| |
| 世界边缘曲率 ≤ κ_max | 精确 κ(t,t',t'',κ_r,κ_r') | 联合非凸；固定 (t,t') 后对 t'' 仿射 | 直/缓参考线：L0 线性化一次 QP + 精确复核；连接路：SCP（2–3 轮、信赖域） | |t'|=0.3 时直线上保守 13.8%；κ_r=0.1 时一阶全线性化低估 >50% |
| 边缘曲率光顺 | ∫(t''')² 或 dκ/dσ 线性化 | 凸（固定 (t,t') 后） | 目标项或上界 | 三次段 t''' 为段常数 |
| 段数少、无密集短段 | t''' 在稠密节点处的跳变 | L0 非凸；L1 凸松弛 | 稠密节点 L1（Clarabel）→ 合并簇 → 记录式重拟合 → 二分违反段插入 → 最短段长检查 | L1 阈值会丢小而必要节点；可能产生节点簇；OSQP 精度不足以选稀疏模式 |
| 口部端点与连接路匹配位置/航向/曲率 | 端点 t、t'、t'' 等式 | 凸（给定参考线时，曲率对 t'' 仿射） | 硬等式；若与容差冲突则字典序 | 参考线本身的回旋线拼接不在本 QP 内（Clothoids G1/G2 Hermite 求解器可用，BSD-2） |
| 源冲突（1:4 渐变、路缘外展） | slack ≥ 0 | 凸 | L1 精确罚 + 报告 slack 位置/量 + 对偶乘子 | 1:4 渐变最小切角 ≈ (Δt')²/(8κ_max)：κ_max=0.02 → 0.39 m |
| 求解器 | — | — | Clarabel（主）、OSQP（固定节点重拟合/热启动）、HiGHS 仅 MILP、不用 ECOS（GPL） | 锁定版本、输出舍入后再写出/哈希 |

- 与项目规则的接口（推断）：任何此类重写都属“改几何”，按项目规则必须给评分板前后对比与泛化集结果；不应因候选不过而改草案阈值——Q5/Q7 的 δ_min 公式正好可用来事先说明哪些阈值组合在 1:4 渐变上不可能同时满足。
- 新功能应写进新模块（项目证据绑定规则），上述 QP 可作为独立模块与现有启发式并行输出、按评分板择优，而不是直接替换受绑定文件。

### Gaps
- 上表所有运行时间来自单条合成边界；多车道联合、真实 SHP/MAP 路口、Windows 环境下的规模与耗时未测。
- 未能找到把 OpenDRIVE laneOffset/width 作为联合约束 QP 拟合的已发表工作（可能由其他调研方向覆盖）。
