# 自动驾驶栈中的道路/车道几何平滑约束优化（Apollo / Autoware）及其向 OpenDRIVE 车道几何拟合的可迁移性

> 调研日期 2026-10-10。主要一手来源：Apollo 源码 `v7.0.0` 标签与 `master`（raw.githubusercontent.com 直接读取），Autoware Universe `main` 分支文档与参数文件，OSQP `v0.6.3` 与 `master`（1.x）源码，各求解器仓库 LICENSE 文件与 PyPI 元数据。
> 访问限制：本会话的出口代理对 arxiv.org / ar5iv / autowarefoundation.github.io 返回 403 或 DNS 失败（组织策略），因此 EM planner（arXiv 1807.08048）、DL-IAPS（arXiv 2009.11135）及若干 OpenDRIVE 生成论文的内容**只能来自搜索引擎摘要**，已在对应条目注明“搜索摘要”，未能核对原文。
> 结构约定：每节 “Cited Findings” 只放有出处的事实；“Inferences” 是对本项目（离线 SHP/MAP→OpenDRIVE，参考线固定后拟合 laneOffset + 各车道 width 三次多项式）的适用性判断。

## Q1 Apollo 的三类参考线平滑器：变量、目标、约束、求解器、运行时、失败模式、为何并存

### Takeaway
Apollo 有三种可切换的参考线平滑器：QpSpline（分段五次参数样条 x(t),y(t) + OSQP，凸 QP）、Spiral（分段五次螺线 θ(s) + IPOPT，非凸 NLP）、DiscretePoints（离散点 FEM_POS_DEVIATION 用 OSQP 的凸 QP，可选 SQP 线性化曲率约束；或 COS_THETA 用 IPOPT）。v7.0.0 与当前 master 的 `planning.conf` 实际启用的都是 DiscretePoints/FEM_POS_DEVIATION，且默认**不开曲率约束**；三者的“保真”都是硬约束（盒/圆形容差带），平滑都是加权罚项，失败时由外层回退/校验兜底。

### Cited Findings

**选择与默认值**
- 平滑器由 `smoother_config_filename` 指向的配置决定：配置里有 `qp_spline` 用 `QpSplineReferenceLineSmoother`，有 `spiral` 用 `SpiralReferenceLineSmoother`，有 `discrete_points` 用 `DiscretePointsReferenceLineSmoother` — [reference_line_provider.cc v7.0.0 L74-83](https://github.com/ApolloAuto/apollo/blob/v7.0.0/modules/planning/reference_line/reference_line_provider.cc)
- gflags 的编译期默认值仍是 `qp_spline_smoother_config.pb.txt` — [planning_gflags.cc v7.0.0 L130-132](https://github.com/ApolloAuto/apollo/blob/v7.0.0/modules/planning/common/planning_gflags.cc)；但 v7.0.0 与 master 的 `planning.conf` 都把 spiral、qp_spline 两行注释掉，实际启用 `discrete_points_smoother_config.pb.txt` — [planning.conf v7.0.0 L13-15](https://github.com/ApolloAuto/apollo/blob/v7.0.0/modules/planning/conf/planning.conf)、[planning.conf master L12-14](https://github.com/ApolloAuto/apollo/blob/master/modules/planning/planning_component/conf/planning.conf)
- master 的 discrete_points 配置：`FEM_POS_DEVIATION_SMOOTHING`，`weight_fem_pos_deviation: 1e10`、`weight_ref_deviation: 1.0`、`weight_path_length: 1.0`、`apply_curvature_constraint: false`、OSQP `max_iter: 500`、`time_limit: 0.0`、`scaled_termination: true`、`warm_start: true`；`max_constraint_interval: 0.25`（注释称离散点平滑器的输出分辨率直接由它决定）、`max_lateral_boundary_bound: 0.5`、`min_lateral_boundary_bound: 0.1`、`longitudinal_boundary_bound: 2.0` — [discrete_points_smoother_config.pb.txt master](https://github.com/ApolloAuto/apollo/blob/master/modules/planning/planning_component/conf/discrete_points_smoother_config.pb.txt)、[reference_line_smoother_config.proto v7.0.0](https://github.com/ApolloAuto/apollo/blob/v7.0.0/modules/planning/proto/reference_line_smoother_config.proto)
- 平滑后有一致性校验：每 10 m 取平滑线上的点投到原始线，|l| 超过 `smoothed_reference_line_max_diff`（默认 5.0 m）即判失败 — [reference_line_provider.cc L758-780](https://github.com/ApolloAuto/apollo/blob/v7.0.0/modules/planning/reference_line/reference_line_provider.cc)、[planning_gflags.cc L177-179](https://github.com/ApolloAuto/apollo/blob/v7.0.0/modules/planning/common/planning_gflags.cc)

**锚点（anchor points）= 走廊/容差带的来源**
- 锚点按 `max_constraint_interval` 沿原始参考线均匀切分；每个锚点的 `lateral_bound` = 车道宽度减去自车宽度（并处理路缘偏移 `curb_shift`），再 clamp 到 [min_lateral_boundary_bound, max_lateral_boundary_bound]；首末锚点 `longitudinal_bound = lateral_bound = 1e-6` 且 `enforced = true`（即端点钉死）；与上一帧参考线拼接时，第一个落在前缀线上的锚点被改成前缀线上的点，界 1e-6 — [reference_line_provider.cc L788-926](https://github.com/ApolloAuto/apollo/blob/v7.0.0/modules/planning/reference_line/reference_line_provider.cc)

**QpSplineReferenceLineSmoother（凸 QP，OSQP）**
- 变量：每段 x(t)、y(t) 各一个 `spline_order` 次多项式（配置 5 次），段数 = 长度 / `max_spline_length`（配置 25 m）四舍五入，节点取整数 t（参数已归一化，s→t 用 `scale = 长度/段数`）— [qp_spline_reference_line_smoother.cc v7.0.0](https://github.com/ApolloAuto/apollo/blob/v7.0.0/modules/planning/reference_line/qp_spline_reference_line_smoother.cc)
- 目标：`second_derivative_weight`×∫(x''²+y''²) + `third_derivative_weight`×∫(x'''²+y'''²) + 正则；配置值 200 / 1000 / 1e-5（proto 默认 0 / 100 / 0.1）— [qp_spline_smoother_config.pb.txt v7.0.0](https://github.com/ApolloAuto/apollo/blob/v7.0.0/modules/planning/conf/qp_spline_smoother_config.pb.txt)、同上源码 `AddKernel()`
- 约束：(1) 每个锚点处一个**按航向旋转的矩形**（横向 ±lateral_bound、纵向 ±longitudinal_bound，4 条线性不等式）`Add2dBoundary`；(2) 开启参考线拼接时首点航向等式；(3) 段间到二阶导连续 `AddSecondDerivativeSmoothConstraint` — 源码同上；[spline_2d_constraint.cc L63-104](https://github.com/ApolloAuto/apollo/blob/v7.0.0/modules/planning/math/smoothing_spline/spline_2d_constraint.cc)
- 文档与代码不一致：官方说明文档写的连续性约束列到三阶导 f'''、g'''，代码只调用二阶连续；文档的代价只写了三阶导积分 — [docs/specs/reference_line_smoother.md v7.0.0](https://github.com/ApolloAuto/apollo/blob/v7.0.0/docs/specs/reference_line_smoother.md)
- 失败处理缺陷：`Solve()` 返回失败时只打印 “Solve spline smoother problem failed”，**不 return false**，继续对样条采样输出（500 点）— [qp_spline_reference_line_smoother.cc](https://github.com/ApolloAuto/apollo/blob/v7.0.0/modules/planning/reference_line/qp_spline_reference_line_smoother.cc)

**SpiralReferenceLineSmoother（非凸 NLP，IPOPT）**
- 变量：每个节点 (θ, κ, κ', x, y) 5 个 + 每段弧长 Δs；每段是 `QuinticSpiralPath`（由两端 θ、κ、κ' 与 Δs 确定的五次螺线，θ(s) 五次多项式）— [spiral_problem_interface.cc v7.0.0 L33-45, L91-110](https://github.com/ApolloAuto/apollo/blob/v7.0.0/modules/planning/reference_line/spiral_problem_interface.cc)
- 约束：(a) 每段积分得到的终点 x、y 必须等于下一节点（2(N-1) 个非线性等式）；(b) 每节点 (x−x0)²+(y−y0)² ≤ max_deviation²（圆形容差带，N 个）；变量界：内部节点 θ ∈ 初值 ±0.2π，κ ∈ [−0.25, 0.25]，κ' ∈ [−0.02, 0.02]，x、y ∈ 初值 ±max_deviation，Δs ∈ [d_i − 2·dev, d_i·π/2]；有固定起/终点时 θ、κ、κ' 钉死 — 源码同上 L94-230
- 目标：Σ_段 [w_len·Δs + Σ_内部采样点 (w_κ·κ² + w_dκ·κ'²)]，配置 w = 1 / 1 / 100；`max_deviation 0.05`、`piecewise_length 10`、`max_iteration 500`、`opt_tol 1e-6`、`opt_acceptable_tol 1e-4`、`opt_acceptable_iteration 15`；输出插值分辨率 0.02 m — [spiral_problem_interface.cc L270-297](https://github.com/ApolloAuto/apollo/blob/v7.0.0/modules/planning/reference_line/spiral_problem_interface.cc)、[spiral_smoother_config.pb.txt v7.0.0](https://github.com/ApolloAuto/apollo/blob/v7.0.0/modules/planning/conf/spiral_smoother_config.pb.txt)
- 求解器设置：IPOPT，`hessian_approximation = limited-memory`（L-BFGS 近似 Hessian），`Solve_Succeeded` 或 `Solved_To_Acceptable_Level` 都算成功 — [spiral_reference_line_smoother.cc v7.0.0 L196-230](https://github.com/ApolloAuto/apollo/blob/v7.0.0/modules/planning/reference_line/spiral_reference_line_smoother.cc)

**DiscretePointsReferenceLineSmoother / FEM_POS_DEVIATION（凸 QP，OSQP）**
- 变量：N 个点的 (x_i, y_i)，共 2N 个；目标三项（源码注释原话的三项）：① `weight_fem_pos_deviation`×Σ‖p_{i−1}+p_{i+1}−2p_i‖²（“中点与有限元估计点的距离”，即二阶差分）；② `weight_path_length`×Σ‖p_{i+1}−p_i‖²；③ `weight_ref_deviation`×Σ‖p_i−p_i^ref‖²；P 矩阵为带状（每点最多与前后两点耦合）— [fem_pos_deviation_osqp_interface.cc v7.0.0 CalculateKernel](https://github.com/ApolloAuto/apollo/blob/v7.0.0/modules/planning/math/discretized_points_smoothing/fem_pos_deviation_osqp_interface.cc)
- 约束：每点 **轴对齐盒** x_ref ± b_i、y_ref ± b_i（硬约束）；调用方先把横向界乘 1/√2（注释：“box constraints on pos are used … thus shrink the bounds by 1.0 / sqrt(2.0)”），并把首末点的界设为 0（“fix front and back points to avoid end states deviate from the center of road”）— 同上 `CalculateAffineConstraint`；[discrete_points_reference_line_smoother.cc v7.0.0 L48-51, L103-108, L144-149](https://github.com/ApolloAuto/apollo/blob/v7.0.0/modules/planning/reference_line/discrete_points_reference_line_smoother.cc)
- 求解：OSQP 用原始点 warm start；Apollo 只覆盖 `max_iter / time_limit / verbose / scaled_termination / warm_start`，eps 等其余设置沿用 OSQP 默认；状态为 1（SOLVED）**或 2（SOLVED_INACCURATE）都当成功** — [fem_pos_deviation_osqp_interface.cc OptimizeWithOsqp](https://github.com/ApolloAuto/apollo/blob/v7.0.0/modules/planning/math/discretized_points_smoothing/fem_pos_deviation_osqp_interface.cc)；状态码定义见 [OSQP v0.6.3 constants.h L19-20](https://github.com/osqp/osqp/blob/v0.6.3/include/constants.h)
- 输出是离散点；航向、κ、dκ 由 `DiscretePointsMath::ComputePathProfile` 用**有限差分**事后计算 — [discrete_points_reference_line_smoother.cc L205-222](https://github.com/ApolloAuto/apollo/blob/v7.0.0/modules/planning/reference_line/discrete_points_reference_line_smoother.cc)、[discrete_points_math.cc](https://github.com/ApolloAuto/apollo/blob/v7.0.0/modules/planning/math/discrete_points_math.cc)
- 可选曲率约束（`apply_curvature_constraint`，默认 false）：`use_sqp=true` 走 SQP+OSQP，否则走 IPOPT NLP — [fem_pos_deviation_smoother.cc v7.0.0 L39-48](https://github.com/ApolloAuto/apollo/blob/v7.0.0/modules/planning/math/discretized_points_smoothing/fem_pos_deviation_smoother.cc)。proto 默认：`curvature_constraint 0.2`、`weight_curvature_constraint_slack_var 1e2`、`sqp_ftol 1e-4`、`sqp_ctol 1e-3`、`sqp_pen_max_iter 10`、`sqp_sub_max_iter 100`、IPOPT `tol 1e-8 / acceptable_tol 1e-1` — [fem_pos_deviation_smoother_config.proto master](https://github.com/ApolloAuto/apollo/blob/master/modules/planning/planning_base/proto/math/fem_pos_deviation_smoother_config.proto)
- SQP 细节：增加 N−2 个松弛变量（≥0，线性罚）；曲率约束为 ‖p_{i−1}+p_{i+1}−2p_i‖² ≤ (Δs̄²·κ_max)² + slack，其中左边在当前迭代点做**一阶 Taylor 线性化**、Δs̄ 用**平均点距**；内循环每次更新 q、A、界后 warm start 重解，直到目标相对变化 < ftol；外循环约束违反 > ctol 时松弛罚权 ×10，最多 10 轮 — [fem_pos_deviation_sqp_osqp_interface.cc v7.0.0 L53-216, L337-420](https://github.com/ApolloAuto/apollo/blob/v7.0.0/modules/planning/math/discretized_points_smoothing/fem_pos_deviation_sqp_osqp_interface.cc)

**COS_THETA（非凸 NLP，IPOPT）**
- 目标：Σ‖p_i−p_i^ref‖² − w_cos·Σ cos(相邻两段夹角)（即最大化相邻线段夹角余弦），可选自动微分；位置用盒约束（同样 ×1/√2）— [cos_theta_ipopt_interface.cc v7.0.0 eval_f](https://github.com/ApolloAuto/apollo/blob/v7.0.0/modules/planning/math/discretized_points_smoothing/cos_theta_ipopt_interface.cc)、[discrete_points_reference_line_smoother.cc L100-113](https://github.com/ApolloAuto/apollo/blob/v7.0.0/modules/planning/reference_line/discrete_points_reference_line_smoother.cc)

**同一代码库中与“车道横向偏移拟合”最相似的模块：Piecewise-Jerk 路径 QP**
- `PiecewiseJerkProblem`：沿 s 等距节点，变量为每节点 (x, x', x'')；节点间 jerk 常数；等式约束 x'_{i+1}−x'_i = ½Δs(x''_i+x''_{i+1})，x_{i+1} = x_i + Δs·x'_i + ⅓Δs²·x''_i + ⅙Δs²·x''_{i+1}；jerk 界 (x''_{i+1}−x''_i)/Δs ∈ [dddx_lo, dddx_hi]；x、x'、x'' 逐点上下界；头文件注释假设 “s(k+1) − s(k) == s(k) − s(k−1)” — [piecewise_jerk_problem.h / .cc v7.0.0](https://github.com/ApolloAuto/apollo/blob/v7.0.0/modules/planning/math/piecewise_jerk/piecewise_jerk_problem.cc)
- 路径版代价：w_x·x² + w_x_ref·(x−x_ref)²（可逐点权重）+ w_dx·x'² + w_ddx·x''² + w_dddx·((x''_{i+1}−x''_i)/Δs)² + 终点状态权重 — [piecewise_jerk_path_problem.cc v7.0.0 CalculateKernel 注释 L39-82](https://github.com/ApolloAuto/apollo/blob/v7.0.0/modules/planning/math/piecewise_jerk/piecewise_jerk_path_problem.cc)
- EM planner 论文（搜索摘要）：QP 路径在 SL 坐标中优化 l=f(s)，代价是 f'、f''、f''' 平方积分加权和 + 与 DP 路径 g(s) 的偏差；约束是在一列站点上对 f、f'、f'' 施加的边界与动力学可行性；“DP 生成可行隧道，样条 QP 在隧道内生成平滑路径”；自 Apollo 1.5（2017-09）部署，截至 2018-05-16 约 3,380 h、68,000 km 闭环测试 — [arXiv 1807.08048（搜索摘要，未能打开原文）](https://arxiv.org/abs/1807.08048v1)

**运行时与论文**
- DL-IAPS（Zhou 等，arXiv 2009.11135，IEEE RA-L，DOI 10.1109/LRA.2020.3045925）：内环以 Hybrid A* 无碰路径为参考、用序列凸规划（SCP）带曲率约束平滑；外环做碰撞检查并有条件地收缩可行域（“iterative anchoring”）；动机是 Convex Elastic Smoothing 在平滑后路径明显变短时曲率约束失效 — [arXiv 2009.11135（搜索摘要）](https://arxiv.org/pdf/2009.11135)。Apollo `planning.conf` 中确有 `--use_iterative_anchoring_smoother` — [planning.conf v7.0.0 L34](https://github.com/ApolloAuto/apollo/blob/v7.0.0/modules/planning/conf/planning.conf)
- 第三方比较（搜索摘要，出处论文未能核对）报 DL-IAPS 总耗时 1043 ms（对比 C-Planner 499 ms、该文方法 106 ms），这是整条开放空间规划用例的总时间，不是参考线平滑的 OSQP 时间 — [搜索结果，疑为 arXiv 2302.08873](https://arxiv.org/pdf/2302.08873)
- 另一篇基于 Apollo 的二次规划路径规划论文报告平均计算 20 ms（与参考线平滑器不是同一模块）— [arXiv 2112.02132（搜索摘要）](https://arxiv.org/pdf/2112.02132)

### Inferences
- 三者并存的原因 Apollo 文档没有写明；从代码可以看出的取舍是：QpSpline 输出解析的参数样条，但它罚的是对归一化参数 t 的导数而非弧长曲率，C2 只在参数意义下成立；Spiral 直接把 θ/κ/κ' 当变量，几何最“干净”（κ' 也连续），代价是非凸 NLP（IPOPT + L-BFGS），收敛与时间都更难保证；FEM 是最简单的稀疏带状 QP，最稳最快，所以被选为默认，但输出是 0.25 m 离散点，曲率靠有限差分，默认不约束曲率。
- 共同模式：硬容差带（盒或圆）+ 加权平滑罚 + 端点钉死 + 事后整体校验（5 m）+ 失败回退。Apollo 把松弛罚只用在“非凸的曲率约束”上，保真带一直是硬的；这对在线规划可以（失败了下帧再来），对离线转换不够，因为离线转换需要每条路都有可解释的结果。
- “SOLVED_INACCURATE 也接受”和“eps 用默认 1e-3”说明 Apollo 只把解当近似值用，不追求可复现到比特；本项目若需逐字节复现，不能照搬这套设置。

### Gaps
- 未找到 Apollo 官方对三种平滑器各自运行时间、失败率的公开数据，也未找到“为何默认切到 FEM”的版本说明（release notes 未核对）。
- DL-IAPS 原文的具体公式、曲率约束线性化细节和计时数据未能核对（arXiv 被代理策略拦截）。
- Apollo 用的 OSQP 版本：v7 的 `third_party/osqp` 只指向系统路径 `/opt/apollo/sysroot/include`，具体版本未查到。

## Q2 Autoware（Universe）路径/轨迹平滑器的数学形式

### Takeaway
Autoware Universe 的路径平滑分两级：Elastic Band（EB）是一个纯几何 QP（二阶差分平滑 + 很小的横向误差罚，点只能沿法向移动、移动量有硬上限 0.1 m，OSQP），目的就是让后面的 MPT 在 Frenet 系里稳定；MPT 是线性化运动学模型的 QP，道路边界与障碍物写成**带松弛变量的软约束**，唯一硬约束是自车前方轨迹点与上一帧一致。

### Cited Findings
- EB 文档：“Since the latter optimization (model predictive trajectory) is calculated on the frenet frame, path smoothing is applied here so that the latter optimization will be stable”；且明确“does not consider collision checking … the output path may have a collision with road boundaries or obstacles” — [autoware_path_smoother docs/eb.md (main)](https://github.com/autowarefoundation/autoware_universe/blob/main/planning/autoware_path_smoother/docs/eb.md)
- EB 目标：min Σ‖p_{k+1} − 2p_k + p_{k−1}‖²（文档称“菱形对角线长度”），写成 x、y 两个块的五对角 P 矩阵（1,−2,1 / −2,5,−4,1 / 1,−4,6,−4,1 …）；另有 `lat_error_weight` 横向误差项 — 同上
- EB 约束：每点**纵向移动量为 0**，横向移动量 ≤ `clearance_for_fix` / `clearance_for_joint` / `clearance_for_smooth`，写成过该点、斜率角 θ_k 的直线上的线性区间 C_k^l ≤ C_k ≤ C_k^u；起点固定，终点若为目标点也固定 — 同上
- EB 参数（main）：`num_points 100`、`delta_arc_length 1.0 m`、`clearance_for_fix 0.0`、`clearance_for_joint 0.1`、`clearance_for_smooth 0.1`、`num_joint_points 3`、`smooth_weight 1.0`、`lat_error_weight 0.001`、OSQP `max_iteration 10000`、`eps_abs 1e-7`、`eps_rel 1e-7`、`enable_warm_start true`、`enable_optimization_validation false`（开启时 `max_error 3.0 m`）；末尾补点使点数恒为 num_points，“for enabling warm start” — [elastic_band_smoother.param.yaml (main)](https://github.com/autowarefoundation/autoware_universe/blob/main/planning/autoware_path_smoother/config/elastic_band_smoother.param.yaml)、[eb.md](https://github.com/autowarefoundation/autoware_universe/blob/main/planning/autoware_path_smoother/docs/eb.md)
- MPT 状态：相对参考路径的横向误差 y_k、航向误差 θ_k，输入为转角；Frenet 系自行车模型 + 转角一阶滞后；线性化：sin θ≈θ，tan δ 在由参考曲率 κ_k 算出的 δ_ref 附近线性化，且 δ_ref 先 clamp 到转角上限 — [autoware_path_optimizer docs/mpt.md (main)](https://github.com/autowarefoundation/autoware_universe/blob/main/planning/autoware_path_optimizer/docs/mpt.md)
- MPT 目标：w_y Σy² + w_θ Σθ² + w_δ Σδ² + w_δ̇ Σδ̇² + w_δ̈ Σδ̈² + 松弛项；道路边界/障碍物：车辆轮廓用若干圆近似，圆心横向偏差 y' 是状态的线性函数，约束 b_l + r − λ ≤ y' ≤ b_u − r + λ，λ ≥ 0；可选松弛的 L2 形式或共享松弛的 L∞ 形式（`l_inf_norm`）；转角上下限为线性不等式；“the only hard constraints” 是自车前方若干点等于上一帧轨迹；优化失败或结果不无碰时输出上一帧轨迹 — 同上
- MPT 局限（README 原文）：“Computation cost is sometimes high”；“Because of the approximation such as linearization, some narrow roads cannot be run by the planner”；调参说明承认“Due to the model error for optimization, the constraint such as collision-free is not fully met”；为降低计算量只优化前方较短轨迹（默认 50 m）再拼接 — [autoware_path_optimizer README (main)](https://github.com/autowarefoundation/autoware_universe/blob/main/planning/autoware_path_optimizer/README.md)
- README 对方法的定位：轨迹规划“non-convex and high dimension”；选择优化法，并“by the preprocessing to approximate the problem to convex that almost equals to the original non-convex problem” — 同上

### Inferences
- EB 的“只许沿法向移动、移动量有硬上限”在固定参考线的 (s, t) 坐标里就是 t 的逐点上下界，本项目可以直接用。EB 的平滑项是二阶差分（近似曲率），不是三阶（曲率变化率），所以它只给“近似 G1/G2”的点列，不保证 C2。
- MPT 的“软约束 + 松弛 + L2 或 L∞”模式，以及“只有极少数点是硬约束”的做法，更适合离线转换器的“永远可解、违反之处记录下来”的需求；运动学部分与本项目无关。

### Gaps
- Autoware 的速度平滑器（`autoware_velocity_smoother`，含 OSQP 版本）只处理速度剖面，本次未调研其公式。
- MPT 使用的 QP 求解器名称在我读到的 mpt.md 文本中没有写明；参数文件中未见 OSQP 字段。
- Autoware Universe 的许可证本次未从 LICENSE 文件核实。

## Q3 硬容差带 vs 加权偏差罚；曲率约束如何线性化、凸性如何保持

### Takeaway
所调研的工业实现几乎都把“离原线多远”写成**线性（或凸二次）的硬容差带**、把“平滑”写成二次罚，从而保持凸 QP；唯一真正非凸的曲率上界，Apollo 用“一阶 Taylor 线性化 + 非负松弛 + 罚权逐轮 ×10 的 SQP”处理，默认还关掉；Autoware MPT 则把边界约束整体软化（松弛变量）以保证总有解。

### Cited Findings
- Apollo FEM：保真 = 每点轴对齐盒 ±b（硬，线性），且为把盒放进半径 b 的圆而乘 1/√2；Apollo Spiral：保真 = 圆 (x−x0)²+(y−y0)² ≤ dev²（硬，凸二次但在 IPOPT 里当一般非线性约束）；Apollo QpSpline：保真 = 按航向旋转的矩形（横向、纵向各 ±界，线性）— 见 Q1 所引 [fem_pos_deviation_osqp_interface.cc](https://github.com/ApolloAuto/apollo/blob/v7.0.0/modules/planning/math/discretized_points_smoothing/fem_pos_deviation_osqp_interface.cc)、[spiral_problem_interface.cc](https://github.com/ApolloAuto/apollo/blob/v7.0.0/modules/planning/reference_line/spiral_problem_interface.cc)、[spline_2d_constraint.cc](https://github.com/ApolloAuto/apollo/blob/v7.0.0/modules/planning/math/smoothing_spline/spline_2d_constraint.cc)
- 同时 FEM 还有一个加权的参考偏差项（`weight_ref_deviation`），即“带内再拉回原线”；默认 `weight_fem_pos_deviation` 1e10 相对 1.0 极大，平滑项占绝对主导，保真基本靠硬带 — [discrete_points_smoother_config.pb.txt](https://github.com/ApolloAuto/apollo/blob/master/modules/planning/planning_component/conf/discrete_points_smoother_config.pb.txt)
- 曲率约束线性化：‖p_{i−1}+p_{i+1}−2p_i‖² 在当前点做一阶展开（代码 `CalculateLinearizedFemPosParams` 给出各变量线性系数与常数项），右端 (Δs̄²·κ_max)² 用平均点距 Δs̄，并对每个约束加一个罚权为 `weight_curvature_constraint_slack_var` 的非负松弛；不满足 ctol 时罚权 ×10 — [fem_pos_deviation_sqp_osqp_interface.cc](https://github.com/ApolloAuto/apollo/blob/v7.0.0/modules/planning/math/discretized_points_smoothing/fem_pos_deviation_sqp_osqp_interface.cc)
- Spiral 平滑器把曲率与曲率变化率直接设为变量的**箱界**（κ ∈ ±0.25，κ' ∈ ±0.02），非凸性在“螺线积分终点 = 下一节点”的等式里 — [spiral_problem_interface.cc L158-230](https://github.com/ApolloAuto/apollo/blob/v7.0.0/modules/planning/reference_line/spiral_problem_interface.cc)
- Piecewise-Jerk QP：在 Frenet（s, l）坐标中把 l、l'、l''、l''' 的界全部写成线性约束，问题是凸 QP — [piecewise_jerk_problem.cc](https://github.com/ApolloAuto/apollo/blob/v7.0.0/modules/planning/math/piecewise_jerk/piecewise_jerk_problem.cc)
- DL-IAPS（搜索摘要）：曲率约束在“平滑后长度≈参考路径长度”的假设下近似为二次约束，用 SCP 迭代；Convex Elastic Smoothing 在平滑后路径明显变短时此近似失效 — [arXiv 2009.11135（搜索摘要）](https://arxiv.org/pdf/2009.11135)
- Autoware EB：硬的横向带（≤0.1 m）+ 二阶差分罚；Autoware MPT：边界软约束 + 松弛（L2 或 L∞）— [eb.md](https://github.com/autowarefoundation/autoware_universe/blob/main/planning/autoware_path_smoother/docs/eb.md)、[mpt.md](https://github.com/autowarefoundation/autoware_universe/blob/main/planning/autoware_path_optimizer/docs/mpt.md)

### Inferences
- 本项目的情况比它们简单：参考线固定后，车道边界的横向偏移 t(s) 对 laneOffset/width 的多项式系数是**线性**的。因此“离源边界 ≤ ε”在采样点上是线性不等式，“宽度 ≥ 0 / ≥ w_min”是线性不等式，“口部处值/斜率/二阶导等于连接路”是线性等式，“∫t'''²”“∫t''²”是凸二次 —— 整个问题天然是凸 QP，不需要 SQP。
- 世界坐标下的边界曲率不是 t 的线性函数（推导：P = R + tN，P' = (1−κt, t')，P'' = (−κ't − 2κt', (1−κt)κ + t'')，κ_b = (P'×P'')/|P'|³）。当 |t'| 小、|κt| 远小于 1 时 κ_b ≈ κ/(1−κt) + t''/(1−κt)²，可在当前解附近线性化成 t'' 的逐点界（凸）；外加一轮“拟合→算世界曲率→收紧界”的确定性外循环即可，不必像 Apollo 那样在 Cartesian 坐标里做 SQP。另由上式：参考线 κ' 在几何段接缝处跳变时，边界曲率也会跳 Δκ'·t·t'/|P'|³（只在该处 t'≠0 时才有），所以“边界 C2”也依赖参考线拼接处的 t'——这是本项目 C2 门禁要额外注意的项（自行推导，未见文献）。
- 硬带 vs 罚：离线转换建议“硬带 + 带松弛的精确罚”（L1 罚，即 MPT 的做法）——无解时自动退化为“尽量少越界”，并把 slack>0 的位置原样写入源复核报告；这和项目“源冲突只记录”口径一致，可替代目前的多条特例规则。纯加权罚（无带）会把保真/平滑的取舍藏在权重里，正是现在被诟病的“隐式权衡”。

### Gaps
- 未找到把“曲率连续 + 段数少”作为联合目标的工业实现；Apollo/Autoware 的输出都是稠密点或固定段长。

## Q4 确定性与可复现性：OSQP 等 QP 求解器跨平台/跨次运行是否一致

### Takeaway
OSQP 的计算本身是确定性的，但 **0.6.x 在默认构建（开 PROFILING）下按墙钟时间决定 ρ 的更新间隔**，同一机器同一输入都会出现不同迭代次数；1.x 已把默认改为“每 50 次迭代”的固定间隔，时间模式仍可选。版本升级会改变“可行/不可行”判定；MKL Pardiso 后端有非确定性。逐字节跨平台一致没有任何求解器保证。

### Cited Findings
- OSQP v0.6.3：`ADAPTIVE_RHO 1`、`ADAPTIVE_RHO_INTERVAL 0`（自动）、`ADAPTIVE_RHO_FRACTION 0.4`（“fraction of setup time after which we update rho”）；开 PROFILING 时，求解耗时 > 0.4×setup 时间后，按当时的迭代数（取整到 check_termination 的倍数）定下 ρ 更新间隔；未开 PROFILING 时固定为 4×check_termination（=100）— [OSQP v0.6.3 constants.h L108-113](https://github.com/osqp/osqp/blob/v0.6.3/include/constants.h)、[OSQP v0.6.3 src/osqp.c L265-279, L456-485](https://github.com/osqp/osqp/blob/v0.6.3/src/osqp.c)
- OSQP 1.x（master）：`adaptive_rho` 改成枚举（DISABLED / ITERATIONS / TIME / KKT_ERROR），**默认 `OSQP_ADAPTIVE_RHO_UPDATE_ITERATIONS`，间隔 50**；TIME 模式需要编译开 profiling，否则 setup 报错；CMake 选项 `OSQP_ENABLE_PROFILING` 默认 ON；代数后端可选 builtin / mkl / cuda；默认 `eps_abs = eps_rel = 1e-3`、`polishing = 0` — [osqp_api_constants.h (master)](https://github.com/osqp/osqp/blob/master/include/public/osqp_api_constants.h)、[osqp_api.c (master) L595, L783-793](https://github.com/osqp/osqp/blob/master/src/osqp_api.c)、[auxil.c (master) L1137-1142](https://github.com/osqp/osqp/blob/master/src/auxil.c)、[CMakeLists.txt (master) L77-99](https://github.com/osqp/osqp/blob/master/CMakeLists.txt)
- PyPI 上 osqp 最新为 1.1.3（2026-06-12），1.0.x 系列首发于 2025-04 — [PyPI osqp](https://pypi.org/project/osqp/)
- osqp-r issue #19（2020-06-01）：同一问题冷启动跑 200 次，多数 950 次迭代、少数 875 或 850 次；维护者 Paul Goulart：“the adaptive_rho feature is configured by default to update rho … based on the relative time taken between the factor phase and the iteration phase”，“The factorisation code is deterministic, but the execution time can vary”，建议改为固定迭代数更新（50–100）；提问者用 `adaptive_rho_interval = 100` 后关闭 — [osqp/osqp-r#19](https://github.com/osqp/osqp-r/issues/19)
- OSQP 讨论组（搜索摘要）：另一位维护者指出用 MKL Pardiso 时线性求解步骤有非确定性，换成 QDLDL 可改善；也提到 warm start 初值不同会导致差异 — [OSQP Google Group “Non-determinism of the solutions”（搜索摘要）](https://groups.google.com/g/osqp/c/7BphLGOuqsg)。注意：该摘要称“设 adaptive_rho_interval=0 以关闭自动间隔”与 v0.6.3 源码（0 即自动）相反，以源码为准。
- OSQP.jl issue #94（2021-02-28 起）：OSQP 二进制从 0.6.1 升到 0.6.2 后，同一 JuMP 模型由可解变为 “infeasible”；原因是不可行判据的容差比较方式改变，维护者最终未定论 — [osqp/OSQP.jl#94](https://github.com/osqp/OSQP.jl/issues/94)
- Apollo FEM 默认开 warm start 并接受 SOLVED_INACCURATE；Autoware EB 开 warm start 且为此保持点数恒定 — 见 Q1/Q2 所引源码与参数文件。

### Inferences
- 对本项目：若用 OSQP，必须固定版本（项目已有 uv.lock 机制）、`adaptive_rho` 用固定迭代间隔（1.x 默认即是）、不用 warm start（或从确定的初值开始）、不用 MKL 后端、打开 polishing 并收紧 eps，使解精度远小于输出写出精度。ADMM 类求解器即使确定，其解也只精确到 eps 量级，若 eps 与写出精度接近，Windows/Linux 浮点细差会在四舍五入边界处翻转。
- 本项目规模很小（单条路几百个变量），可以考虑对 KKT 系统做稠密/稀疏直接求解的**主动集**或**内点法**（Clarabel、HiGHS QP、piqp、proxsuite），收敛到 1e-10 量级后再按写出精度舍入；或者在等式约束 + 罚（无不等式）的子情形下直接解线性方程。这比 ADMM 更接近“同输入同输出”。
- 跨平台差异的其余来源（未核实，凭一般经验）：NumPy/SciPy 的 BLAS（OpenBLAS 按 CPU 选内核、多线程归约顺序）、编译器 FMA 收缩、libm 超越函数实现。现在 Windows 与 Linux 输出不同的主因很可能是“离散择优”在近似相等时翻转；把离散选择改成连续优化，本身就会显著减少这类分叉。

### Gaps
- 未找到 Clarabel、HiGHS、piqp、proxsuite 关于跨平台位级可复现的官方声明。
- 未核实 osqp Python wheel 是否以 PROFILING=ON 构建（对 1.x 默认 ITERATIONS 模式无影响，对 0.6.x 有影响）。

## Q5 求解器/建模库许可证（GPL 只能进程隔离；LGPL 只引用不改源码）

### Takeaway
主流凸 QP 求解器 OSQP、Clarabel（Apache-2.0）、HiGHS（MIT）、piqp、proxsuite（BSD-2）、SciPy（BSD）都没有 copyleft 问题；qpOASES（LGPL-2.1）、CasADi（LGPL-3.0）、qpsolvers 封装层（LGPL-3.0）按项目规则只能原样引用；IPOPT/cyipopt 为 EPL-2.0（弱 copyleft，非 GPL）。所列库中没有 GPL。

### Cited Findings
- OSQP：Apache License 2.0 — [osqp/osqp LICENSE](https://github.com/osqp/osqp/blob/master/LICENSE)；PyPI `osqp` 1.1.3，Apache-2.0 — [PyPI](https://pypi.org/project/osqp/)
- Clarabel（Rust 与 C++ 版）：Apache License 2.0 — [Clarabel.rs LICENSE.md](https://github.com/oxfordcontrol/Clarabel.rs/blob/master/LICENSE.md)、[Clarabel.cpp LICENSE.md](https://github.com/oxfordcontrol/Clarabel.cpp/blob/master/LICENSE.md)；PyPI `clarabel` 0.11.1（2025-06-11），Apache-2.0 — [PyPI](https://pypi.org/project/clarabel/)
- HiGHS：MIT — [HiGHS LICENSE.txt](https://github.com/ERGO-Code/HiGHS/blob/master/LICENSE.txt)；PyPI `highspy` 1.15.1（2026-07-02），MIT — [PyPI](https://pypi.org/project/highspy/)
- qpOASES：GNU LGPL 2.1 — [qpOASES LICENSE](https://github.com/coin-or/qpOASES/blob/master/LICENSE)
- Ipopt：Eclipse Public License 2.0 — [Ipopt LICENSE](https://github.com/coin-or/Ipopt/blob/master/LICENSE)；PyPI `cyipopt` 1.7.0，EPL-2.0 — [PyPI](https://pypi.org/project/cyipopt/)
- CasADi：GNU LGPL 3.0（PyPI 标注 LGPLv3+），PyPI 3.8.1（2026-09-16）— [casadi LICENSE.txt](https://github.com/casadi/casadi/blob/master/LICENSE.txt)、[PyPI](https://pypi.org/project/casadi/)
- qpsolvers（多求解器统一封装）：LGPL-3.0，PyPI 4.13.0 — [qpsolvers LICENSE](https://github.com/qpsolvers/qpsolvers/blob/master/LICENSE)、[PyPI](https://pypi.org/project/qpsolvers/)
- CVXPY：Apache-2.0，PyPI 1.9.3 — [cvxpy LICENSE](https://github.com/cvxpy/cvxpy/blob/master/LICENSE)；piqp 0.6.4：BSD 2-Clause；proxsuite 0.7.3：BSD-2-Clause — [PyPI piqp](https://pypi.org/project/piqp/)、[PyPI proxsuite](https://pypi.org/project/proxsuite/)；SciPy 1.18.1：BSD — [scipy LICENSE.txt](https://github.com/scipy/scipy/blob/master/LICENSE.txt)
- Apollo 源码文件头均为 Apache-2.0 声明 — 见 Q1 所引任一源文件

### Inferences
- 推荐优先级（许可证维度）：SciPy（已在依赖中）/ OSQP / Clarabel / HiGHS / piqp 均可直接进程内使用。CasADi、qpOASES、qpsolvers 属 LGPL，按项目第 7 条“库引用不改源码”可以用，但没必要。IPOPT 只在走非凸 NLP（Spiral 类）时才需要，本项目推荐的凸形式用不到。
- 需要另外核对 Python 3.11 + 现有锁定版本下各 wheel 的可用性（项目锁 3.11，不升 3.12）。

### Gaps
- Ipopt 实际使用的线性求解器（MUMPS / HSL MA27 等）各自的许可证未核实；HSL 不是开源许可证，若采用 IPOPT 需单独确认。
- HiGHS 的 QP 求解能力（仅凸 QP、算法类型）本次未从官方文档核实。

## Q6 从单条中心线推广到多条耦合车道边界（共享边界、宽度非负、车道出生/消失）需要改什么

### Takeaway
所调研的工业平滑器都只处理**一条曲线**（参考线或自车路径），没有“多条边界联合拟合”的现成实现；但在固定参考线、OpenDRIVE“laneOffset + 逐车道 width”的参数化下，多边界耦合只是在同一个 QP 里增加线性等式/不等式，问题依旧是凸的。真正非凸的只剩离散决策（分段点位置、车道段划分、出生点位置、源线到车道的对应），需要在 QP 之外用确定性规则先定好。

### Cited Findings
- 现有 OpenDRIVE 生成工作的做法都是逐边界/逐车道的三次多项式最小二乘，未见联合约束优化：点云生成 OpenDRIVE 的工作把参考线按 100 m 分段（“the amount of information that can be displayed in one curve is limited by their cubic nature”），用前视/后视窗口让相邻段平滑衔接，并给首末车道线更大权重 — [arXiv 2405.07544（搜索摘要）](https://arxiv.org/pdf/2405.07544)
- 另一工作假设参考线曲率连续，把每条车道宽度建模为 s 的三次多项式 w_i(s) = as³ + bs² + cs + d，并分 lane section 各自有效 — [arXiv 2006.03403（搜索摘要）](https://arxiv.org/pdf/2006.03403)
- LiDAR+OSM 数字孪生工作：按横向偏移聚类后，把各车道边界在参考线坐标下用最小二乘拟合三次函数 — [arXiv 2606.16570（搜索摘要）](https://arxiv.org/pdf/2606.16570)
- 一件中国专利描述 Shapefile→OpenDRIVE：带边界约束的 ParamPoly 拟合，误差超限时重新分段拟合（专利，非论文）— [CN 121997573 A（搜索摘要）](https://www.goveda.com/patent/CN-121997573-A)
- CommonRoad 的 OpenDRIVE 转换代码把车道边界（border）表示为沿整个 lane section 的路径，宽度系数按 ds 计 w = a + b·ds + c·ds² + d·ds³ — [commonroad-scenario-designer border 源码文档](https://commonroad-scenario-designer.readthedocs.io/en/latest/_modules/opendrive/opendrive_conversion/plane_elements/border/)
- Apollo Piecewise-Jerk QP 已经是“沿 s 的 C2 分段三次 + 逐点线性界 + 参考值追踪罚”的单曲线形式（见 Q1）— [piecewise_jerk_problem.cc](https://github.com/ApolloAuto/apollo/blob/v7.0.0/modules/planning/math/piecewise_jerk/piecewise_jerk_problem.cc)

### Inferences
- **参数化**：在一条路上取一套共享的 s 节点（每个 lane section 内各车道共享），把每条边界的横向偏移 t_k(s) 表示为 C2 分段三次（用 Piecewise-Jerk 的节点状态 (t, t', t'')，把它推广到非等距 Δs_i）；laneOffset = t_0，右侧 w_k = t_{k−1} − t_k，左侧对称。同一节点网格上的三次多项式相减仍是三次，所以换算到 OpenDRIVE `<laneOffset>`/`<width>` 记录是精确的，记录数 = 节点区间数。共享边界由这种参数化自动保证，不需要额外约束。
- **约束**（全为线性）：① 保真带 |t_k(s_ij) − t̂_kj| ≤ ε_kj + σ_kj，σ ≥ 0（ε 可按区域分档：普通段、口部、外展区）；② 宽度 w_k(s) ≥ w_min 在稠密采样点上（或用“每段 Bernstein 系数 ≥ w_min”作为整段成立的充分条件——这是多项式的一般性质，本次未查文献）；③ 口部：t_k、t_k'、t_k'' 在路端等于连接路对应值（或连接路与道路一起入同一个 QP，以等式耦合）；④ 车道出生/消失：出生点 s_b 处 w = 0、w' = 0（要 C2 再加 w'' = 0），s_b 之前该车道不存在；⑤ lane section 之间的连续性需要显式等式（OpenDRIVE 本身不强制）；⑥ 可选的曲率近似界 |t_k''| ≤ c(s)（由 Q3 的线性化给出）。
- **目标**：Σ ρ·(t − t̂)²（或 Huber/L1 以抗源噪声，L1 用松弛表示仍是 QP）+ λ₃Σ∫(t_k''')² + λ₂Σ∫(t_k'')² + M·Σσ。这样“保真/平滑”的取舍只由少数显式参数（ε、λ、M）决定。
- **非凸的部分仍需外部决定**：节点数与位置（“段少”与“误差小”的权衡）、出生点 s_b、lane section 划分、源线与车道的对应、口部停止线位置。可行做法：先粗节点（如 20–30 m），对 slack>0 或残差超带的区间按确定性规则加节点（类似 DL-IAPS 的外环、2405.07544 的分段窗口），迭代次数设上限；s_b 先取源数据给出的事件点。L1 惩罚 t''' 的跳变（ℓ1 trend filtering 一类思路）可以在凸框架内自动得到“少节点”的解，但本次未调研其在地图领域的应用。
- **规模**：一条 200 m、5 条边界、10 个节点的路约 150 个变量、上千个采样约束，任何 QP 求解器都是毫秒级；把一个路口的所有进出口道路与连接路放进一个 QP 也只有几千个变量。

### Gaps
- 未找到任何把多条车道边界联合放进一个约束优化（共享边界 + 宽度非负 + 出生/消失）的论文或开源实现；OpenDRIVE 生成论文的细节只来自搜索摘要。
- OpenDRIVE `<border>` 元素（直接给出外边界多项式，与 `<width>` 互斥）的规范条文与 esmini 支持情况未核实。

## Q7 适用性评估：哪些形式可改造用于“固定参考线 + laneOffset/width 三次多项式”的拟合

### Takeaway
最值得借鉴的是 **Apollo Piecewise-Jerk 路径 QP 的结构**（Frenet 坐标、节点状态 (l, l', l'')、常 jerk 即 C2 分段三次、逐点线性界、参考追踪罚），加上 **EB/MPT 的“硬横向带 + 松弛罚”**；把它从单条 l(s) 推广到共享节点的多条边界，就能一次性得到 C2、段数可控、可直接写成 OpenDRIVE 记录的车道几何。Apollo 的 FEM、COS_THETA、Spiral、QpSpline 本身都工作在 Cartesian 坐标、优化的是参考线本身，不适合用来拟合车道偏移；其中 Spiral 的思路只对“参考线 planView 的回旋线链拟合”有参考价值。

### Cited Findings
- 各形式的变量空间与输出：QpSpline 为 x(t)、y(t) 五次参数样条（25 m/段）；Spiral 为五次螺线（θ 五次、κ 四次）；FEM/COS_THETA 为 0.25 m 离散点；Piecewise-Jerk 为 Frenet 坐标下 C2 分段三次（常 jerk）— 见 Q1 所引源码与配置
- Autoware EB 只沿法向移动、纵向位移为 0，横向硬界 0.1 m；MPT 用松弛软化边界 — 见 Q2 所引文档

### Inferences（判断，不是事实）

| 形式 | 能否直接用于 laneOffset/width 拟合 | 主要理由 |
|---|---|---|
| Apollo Piecewise-Jerk 路径 QP（OSQP） | **可以，改动最小** | 已是 Frenet 下的 C2 分段三次；需要改的只是：非等距节点、多条边界共享节点、加入宽度非负/出生/口部等式、把 x_ref 罚改成到源点的偏差并加硬带+松弛。每个节点区间对应一条 OpenDRIVE 三次记录，节点可稀疏，避免“密集短段”。 |
| Apollo QpSpline（OSQP） | 思路可用，形式不对 | “分段多项式 + 连续性等式 + 锚点矩形界 + 三阶导能量”这套 QP 构造可借鉴；但它拟合的是 Cartesian 参数曲线、五次，OpenDRIVE paramPoly3 只有三次，且对参数 t 求导不是对弧长；应改成 t(s) 的三次样条。 |
| Apollo FEM_POS_DEVIATION（OSQP） | 不直接适用 | 优化参考线本身；输出稠密离散点，曲率靠差分，默认不约束曲率；若用于“先平滑再拟合”又回到两阶段近似。可作为“稠密离散版 t(s) 预平滑”的对照基线。 |
| Apollo FEM + SQP 曲率约束 | 不需要 | 本项目在 Frenet 下曲率近似界已是线性，无需 SQP；其“松弛罚权逐轮 ×10”可借鉴为外循环。 |
| Apollo COS_THETA / Spiral（IPOPT） | 不适用于车道；Spiral 思路可用于参考线 | 非凸 NLP，迭代路径依赖初值与容差，确定性与稳定性都更差；五次螺线不是 OpenDRIVE 原生几何（OpenDRIVE spiral 为线性曲率变化的回旋线，需另行核实条文）。若将来要把参考线拟合也改成优化，可借鉴“节点 (θ, κ) + 段长为变量 + 位置闭合等式 + 圆形容差带”，换成回旋线段。 |
| Autoware EB（OSQP） | 约束形式可用 | “只许沿法向移动 + 硬横向带”在 (s,t) 中就是 t 的上下界；目标仅二阶差分，不足以保证 C2。 |
| Autoware MPT（QP） | 只借鉴松弛模式 | 运动学模型无关；“软边界 + 松弛 + L2/L∞ 选项、极少硬约束”适合离线转换器“总有解、越界即记录”的需求。 |

- **预期收益**：用一个凸 QP（每条路或每个路口一个）替代“laneOffset → 内侧到外侧逐边界 → RDP 顶点 + 固定曲率目标 0.04 /m 的 G2 圆角 + 顶点值最小二乘 → 口部/出生/外展特例 → 连接路事后对接”的顺序链。共享边界、宽度非负、口部对齐、出生零宽这些现在靠特例保证的性质变成显式约束；保真/平滑的取舍集中到 ε、λ、M 几个参数；越界与源冲突由松弛量直接给出位置和大小。
- **仍然需要启发式的部分**：节点放置/段数、lane section 与出生点、源线配对、口部位置、外展区的 ε 分档。这些应当用无平局的确定性规则（固定排序、明确的打破平局规则）先定好，QP 只负责连续部分。
- **风险**：① 内弯处 |t|·|κ_r| 接近 1（小半径连接路的外侧车道）时，源点向参考线投影求 (s, t) 不唯一或失真，需要沿法线求交并检测；② t 空间平滑不等于世界坐标边界平滑，拟合后必须在世界坐标复核曲率与 C2（项目已有 g2-k04-c2 门禁可复用）；③ 硬带与 C2/口部等式可能互相矛盾，必须带松弛，否则会出现大面积不可行；④ 改用新内核后评分板与冻结证据的数值必然变化，按项目规则须新写模块并给出评分板与泛化集前后对比；⑤ 求解器精度与舍入（见 Q4）。
- **确定性建议**：首选内点法/主动集（Clarabel、HiGHS、piqp、proxsuite）或在无不等式子情形下直接解 KKT；若用 OSQP，用 1.x、固定 ρ 更新间隔、关 warm start、开 polishing、收紧 eps，并在写出时按固定精度舍入；不要保留“多候选择优”的离散分支。

### Gaps
- 没有找到任何工业或开源项目公开把这种“Frenet 下多边界联合 QP”用于 HD 地图/OpenDRIVE 生成的实证数据（精度、段数、运行时），以上收益与风险均为推断，需要在评分板与泛化集上验证。
