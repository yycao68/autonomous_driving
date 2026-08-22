# 基于常状态转移双积分器骨架的统一预测式运动规划

> 本文为 `motion_planning_double_integrator_revised.md` 的中文版，与英文修订版保持同步。公式、表格、图片路径与参考文献保持一致；正文、小节标题与图注为中文译文。

---

## 摘要

传统机器人运动规划通常将几何寻路与时间参数化分离，这限制了机器人在不进入昂贵重规划循环的情况下响应动态障碍的能力。本文提出一种统一的实时局部预测式运动规划器，它直接在时间域中构造，并继承此前用于物理人机交互（pHRI）柔顺控制的两层架构。核心观察是：虚拟双积分器规划器给出一个与机器人构型无关的常状态转移矩阵 $A_d$。因此，动力学预测矩阵可离线预计算，在线规划退化为一个具有固定动力学结构的小型凸二次规划（QP）。解耦盒约束情形可轻松以 100 Hz 或更高频率运行；耦合障碍约束行仍保持凸性，并在固定稀疏更新的 Python 测试程序中达到 100 Hz 的 p95 时序，但若不进一步优化实现，最坏单周期时间可能超过 10 ms。

第一层将多变量关节空间映射为一组与构型无关的双积分器预测模型。第二层以高频求解凸 QP，同时施加运动学限幅（速度、加速度）与线性化障碍约束。该框架输出光滑连续的位置/速度曲线 $(q_d, \dot{q}_d)$，并具有硬有界、分段常值的加速度 $\ddot{q}_d$；同时在杂乱空间中展示了有界时延的反应式偏转，无结构性重规划停顿。当需要连续加速度时，同一幂零结构论证可扩展到分段加加速度（三积分器）骨架。常 $A_d$ 结构带来了批处理规划流水线在结构上不具备的有界单周期反应时延。

---

## I. 引言

部署在非结构化或人机共享环境中的现代机器人系统，必须在高精度跟踪规划轨迹的同时，对动态工作空间变化保持响应。经典运动规划流水线按顺序处理这些需求：几何规划器（如 OMPL [1]）首先在构型空间中搜索无碰撞路径；独立的时间参数化阶段（如时间最优轨迹生成，TOTG [2]）随后通过从弧长到时间的变换 $s \rightarrow t$，将空间路径映射为带时间的轨迹。这种解耦带来根本局限：若执行过程中障碍移动或有人介入，整条流水线必须中断并完整重规划，其时延与实时反应行为不相容。

近期关于 pHRI 预测式阻抗控制的工作表明，建立在构型无关双积分器模型上的两层架构，可在严格施加执行器约束的同时实现亚毫秒级 QP 求解 [3]。其关键结构性洞见是：剩余被控对象可化为一个线性系统，其状态转移矩阵 $A_d$ 在所有构型下恒定。这使自由响应预测矩阵 $\Phi$ 可完全离线预计算，运行时只需轻量矩阵—向量运算。

**本文的核心观察是：虚拟双积分器规划器给出与机器人构型无关的常状态转移矩阵。因此，所有动力学预测矩阵均可离线预计算，在线规划被化简为具有固定动力学结构的小型凸 QP。** 本文将 pHRI 架构原则转用于统一的时间域预测式运动规划器。我们并非发明一种全新规划范式，而是说明：此前的 pHRI MPC 架构在纯软件规划语境中自然形成一个高效的预测式局部运动规划器，并具备有界时延避障能力。

该框架应理解为**局部预测式运动规划器（滚动时域轨迹优化器）**，而非完整全局规划器。它不做全局搜索；避障是局部的，依赖线性化与人工势场。其区别性性质包括：

- 受固定优化结构约束，结构规模随活动障碍约束数增长。
- 同时施加运动学限幅与障碍约束。
- 无空间到时间的变换阶段。
- 借助常 $A_d$ 性质离线预计算所有动力学预测矩阵。
- 输出 $C^1$ 轨迹且加速度硬有界；可选分段加加速度扩展以获得连续加速度。

本文组织如下。第 II 节给出数学骨架。第 III 节描述滚动时域 QP。第 IV 节介绍两类互补避障策略。第 V 节分析关键架构性质。第 VI 节报告实验基准。第 VII–IX 节给出实现说明、讨论与结论。

---

## II. 数学框架与系统结构

运动规划的目标是构造一条可行轨迹，使机器人通过一系列任务空间或关节空间路径点，同时满足运动学与环境约束。不同于为物理对象计算执行器命令的运动控制，运动规划工作在轨迹层，输出供下游控制器跟踪的参考轨迹。

设机器人构型为 $q \in \mathbb{R}^n$，任务空间位置由正运动学确定：

$$
p=f(q).
$$

规划问题是生成轨迹

$$
q(t),\dot q(t),\ddot q(t)
$$

使其满足路径点要求、关节界、速度界、加速度界与避障约束。

为参数化可行轨迹空间，规划器采用与构型无关的双积分器骨架。该骨架可理解为：(i) 如 pHRI 架构中，经前馈对消机器人动力学后得到的剩余系统；或 (ii) 仅用于规划的虚拟轨迹生成模型。两种解释得到相同且恒定的状态转移矩阵。

$$
\ddot q_i=u_i.
$$

在规划中，双积分器骨架不用于建模执行器级动力学。它只是轨迹生成骨架，其状态包含规划关节位置与速度，输出为下游跟踪控制器使用的参考轨迹。因此，优化是在一族运动学可行轨迹上进行，而非直接在执行器命令上进行。

该参数化有吸引力，因为它保证位置与速度连续，并允许把加速度限幅直接作为线性约束施加。更重要的是，所得状态转移矩阵与构型无关，因此所有动力学预测矩阵均可离线预计算。

### A. 来自动力学对消的双积分器骨架

考虑标准刚体机械臂动力学：

$$ M(q)\ddot q + C(q,\dot q)\dot q + g(q)=\tau . $$

在动力学对消解释中，规划器在由双积分器骨架参数化的关节坐标上工作。沿用此前 pHRI 两层架构，使用前馈补偿项对消与构型相关的非线性动力学。控制输入分解为

$$ \tau = M(q)u + C(q,\dot q)\dot q + g(q), $$

其中 $u$ 是预测层生成的虚拟加速度命令。代入机械臂动力学得

$$ \ddot q = u. $$

定义状态 $x = \begin{bmatrix} q ^\top, \dot q ^\top \end{bmatrix} ^\top$，剩余系统为

$$\dot x = \begin{bmatrix} 0&I\\ 0&0 \end{bmatrix}x +
\begin{bmatrix} 0\\ I \end{bmatrix}u, $$

即与构型无关的双积分器。

关键结果是系统矩阵恒定且幂零。离散化后，规划模型的状态转移矩阵 $A_d$ 不依赖机器人构型或工作点。因此，滚动时域优化器所需的动力学预测矩阵可一次性离线计算，并在执行过程中不变复用。

该双积分器骨架与 pHRI 阻抗 MPC 架构中的骨架相同；区别在于，本文用它做预测式运动规划，而非抗扰阻抗调节。

### B. 精确 ZOH 离散化与常 $A_d$ 性质

连续系统矩阵 $A_c$ 幂零：$A_c^2 = 0$。因此，对规划周期 $\Delta t$，矩阵指数精确截断：

$$e^{A_c \Delta t} = I + A_c \Delta t$$

精确零阶保持（ZOH）离散化为：

$$A_d = \begin{bmatrix} 1 & \Delta t \\ 0 & 1 \end{bmatrix}, \qquad B_d = \begin{bmatrix} \frac{\Delta t^2}{2} \\ \Delta t \end{bmatrix}$$

关键结构性结论是：$A_d$ **对每个关节、每个构型都恒定且相同**。这与物理对象离散化（如线性化机械臂动力学）形成鲜明对比，后者的 $A_d$ 依赖于随机器人状态变化的 $M(q)$、$C(q,\dot{q})$ 与雅可比项。

由于 $A_d$ 不变，有限时域 $N$ 上的轨迹预测矩阵

$$\Phi = \begin{bmatrix} A_d \\ A_d^2 \\ \vdots \\ A_d^N \end{bmatrix}, \qquad \Gamma = \begin{bmatrix} B_d & 0 & \cdots & 0 \\ A_d B_d & B_d & \cdots & 0 \\ \vdots & & \ddots & \vdots \\ A_d^{N-1} B_d & \cdots & A_d B_d & B_d \end{bmatrix}$$

可**完全离线预计算并作为常量存储**。运行时，时域轨迹推演退化为一次矩阵—向量乘法，从而支撑高频规划循环。这是核心结构性洞见：轨迹预测的计算负担与机器人构型完全解耦。

---

## III. 滚动时域优化（第二层 QP 规划器）

### A. 优化问题结构

每个规划时间步，规划器在预测时域上求解最优加速度序列：

$$U = \begin{bmatrix} u(0) \\ u(1) \\ \vdots \\ u(N-1) \end{bmatrix} \in \mathbb{R}^{nN}$$

其中 $n$ 为关节数，$N$ 为时域长度（例如 7 自由度 FR3 且 $N=20$ 时，决策变量为 $nN = 7\times 20 = 140$）。预测状态轨迹为

$$X = \Phi x(k) + \Gamma U$$

给定当前规划状态 $x(k)$，它对 $U$ 仿射。该线性性使所有下游约束（运动学界、避障）对决策变量保持凸性。若无障碍耦合，Hessian 为分块对角，问题可分解为 $n$ 个规模为 $N$ 的独立单关节 QP。

### B. 代价函数：跟踪与光滑性

目标函数最小化相对目标状态 $x_{\text{goal}} = [q_{\text{goal}}, 0]^\top$ 的跟踪偏差与控制努力的加权和。按常规 $\tfrac{1}{2}$ 缩放写为

$$J(U)=\frac{1}{2}\left(\Phi x(k)+\Gamma U-\mathbf{1}_N\otimes x_{\text{goal}}\right)^\top
\bar Q\left(\Phi x(k)+\Gamma U-\mathbf{1}_N\otimes x_{\text{goal}}\right)
+\frac{1}{2}U^\top\bar R U,$$

收集与 $U$ 有关的项得到

$$\min_{U} \; \frac{1}{2} U^\top H U + h^\top U$$

其中

$$H = \Gamma^\top \bar{Q} \Gamma + \bar{R}, \qquad h = \Gamma^\top \bar{Q} \, x_{\text{free,err}}$$

$x_{\text{free,err}} = \Phi x(k) - \mathbf{1}_N \otimes x_{\text{goal}}$ 是相对目标的无控偏差。若同一最小二乘代价写成无前置 $\tfrac{1}{2}$ 的形式，则 $H$ 与 $h$ 均乘以 $2$；由于整体目标只被正数缩放，优化器不变。所有 OSQP 调用均采用上式约定。其余定义为：

- 完整目标状态 $x_{\text{goal}}=[q_{\text{goal}},0]^\top$ 复制到全部 $N$ 个时域步；
- 阶段权重 $Q = \text{blkdiag}(K_q, D_q)$ 惩罚位置与速度误差；
- 控制代价矩阵 $R$ 惩罚大加速度以正则化轨迹；
- 终端权重 $Q_f = \gamma Q$（$\gamma \approx 5$–$10$）施加在最后一个时域步以促进收敛；
- 堆叠权重 $\bar Q = \mathrm{blkdiag}(Q,\dots,Q,Q_f)$ 与 $\bar R = I_N \otimes R$。

由于 $H$ 是半正定矩阵（来自 $\bar{Q}$）与正定矩阵（来自 $\bar{R}$ 且 $R \succ 0$）之和，QP 在所有构型与时间步上均**严格凸**。唯一全局最小值存在，并可由 OSQP [4] 等算子分裂求解器配合热启动高效求得，通常在 0.5 ms 内收敛。

**权重整定建议。** 各权重的作用直观，便于初始整定。$K_q$（位置误差权重）设置收敛激进程度：增大它会更快拉向目标，但可能触碰加速度限幅。$D_q$（速度误差权重）提供阻尼并抑制超调；较好的初始比例为 $D_q \approx 2\sqrt{K_q}$（临界阻尼二阶响应）。$R$（加速度正则）在光滑性与速度之间折中：更大的 $R$ 产生更温和曲线，但增加到达时间。终端乘子 $\gamma$ 补偿有限时域；对 $N \geq 20$，$\gamma = 5$–$10$ 通常足够。本文 FR3 实验使用 $K_q = 100$、$D_q = 20$、$R = 0.1\,I$、$\gamma = 8$（逐关节标量）。实用整定顺序为：(1) 令 $R$ 较小并增大 $K_q$，直到规划器能在运动学限幅内舒适到达目标；(2) 增大 $D_q$ 直到超调消失；(3) 若加加速度对应用不可接受，则增大 $R$。

### C. 运动学硬约束

物理关节限制作为预测状态与输入序列上的硬不等式约束施加。对每个关节 $i$ 与时域步 $k$：

**速度界：**
$$-\dot{q}_{i,\max} \leq \dot{q}_{i}(k) \leq \dot{q}_{i,\max} \quad \forall k \in \{1,\dots,N\}$$

**加速度（输入）界：**
$$-u_{i,\max} \leq u_i(k) \leq u_{i,\max} \quad \forall k \in \{0,\dots,N-1\}$$

由于预测状态经 $\Gamma$ 对 $U$ 仿射，这些约束转化为线性不等式 $C_{\text{kin}} U \leq d_{\text{kin}}$，由预计算 $\Gamma$ 和固定在线项组装。关节位置界 $q_{i,\min} \leq q_i(k) \leq q_{i,\max}$ 也可追加，且不引入额外结构性开销。

---

## IV. 引入工作空间避障

预测关节位置对 $U$ 的线性性，使避障约束可在不破坏凸性的情况下直接加入。下面给出两种互补方法。

### A. 凸走廊多面体约束（硬约束）

由感知系统检测到的笛卡尔障碍表示为任务空间半空间不等式：

$$A_{\text{obs}} \, p(k) \leq b_{\text{obs}}$$

其中 $p(k) \in \mathbb{R}^3$ 是第 $k$ 步预测的末端执行器（或连杆）位置。利用当前工作点处的平动雅可比 $J_v(q_0)$ 对正运动学线性化：

$$p(k) \approx p_0 + J_v(q_0)\bigl(q(k) - q_0\bigr)$$

工作空间约束变为

$$A_{\text{obs}} J_v(q_0) \, \Delta q(k) \leq b_{\text{obs}} - A_{\text{obs}} p_0$$

其中 $\Delta q(k) = q(k) - q_0$ 经由 $\Gamma$ 对 $U$ 线性。为显式写出 QP 行，令 $E_{q,k}$ 选择第 $k$ 个时域步的预测关节位置块，则

$$q(k)=E_{q,k}\bigl(\Phi x(k_0)+\Gamma U\bigr).$$

于是每条障碍行具有仿射形式

$$
\underbrace{A_{\text{obs}}J_v(q_0)E_{q,k}\Gamma}_{C_{\text{obs},k}}\,U
\le
\underbrace{b_{\text{obs}}-A_{\text{obs}}p_0
-A_{\text{obs}}J_v(q_0)\bigl(E_{q,k}\Phi x(k_0)-q_0\bigr)}_{d_{\text{obs},k}} .
$$

因此，自由响应项被吸收到右端项中，障碍约束化为关于 $U$ 的仿射不等式，完全兼容凸 QP。

**近似质量。** 由于 $J_v(q_0)$ 在单周期的整个时域推演中保持不变，这些行构成安全走廊的一阶局部近似：在 $q_0$ 附近精确，但随着预测运动离开 $q_0$ 邻域而误差增大。该线性化并不自动保守。若要得到保守半空间，应按正运动学线性化误差界收紧每条障碍行。每周期重新线性化可刷新该近似。理论上，二阶 Taylor 余项给出上界：令 $H_p(q_0)$ 为位置 Hessian（$p$ 对 $q$ 的二阶偏导张量，在 $q_0$ 处求值），真实末端位置满足

$$\|p(q) - (p_0 + J_v(q_0)\Delta q)\| \leq \tfrac{1}{2} \|H_p(q_0)\|_F \|\Delta q\|^2$$

其中 $\|\cdot\|_F$ 为 Frobenius 范数。对障碍半空间行 $a^\top p\le b$，保守收紧行为

$$a^\top\bigl(p_0+J_v(q_0)\Delta q(k)\bigr)\le b-\|a\|\,\varepsilon_p,$$

其中 $\varepsilon_p \geq \tfrac{1}{2}\|H_p\|_{F}\|\Delta q_{\max}\|^2$。FR3 的 $\|H_p\|_F$ 通过在关节范围内均匀采样 5000 个构型，并对雅可比做中心差分数值求导得到；分布均值为 $0.91$ m/rad²，95 分位为 $1.43$ m/rad²，对应下文引用的约 $0.5$–$1.5$ m/rad² 范围。配套测试报告的代表性末端时域线性化误差为：25% 关节速度、0.1 s 时 7.1 mm；25%、0.2 s 时 28.0 mm；50%、0.1 s 时 28.0 mm；50%、0.2 s 时 106.4 mm；全速、0.4 s 时 1.03 m。因此，在推荐的 $N=20$（0.2 s）下，中等速度可能只需厘米级裕度，而测试 FR3 构型中 50% 关节速度运动已需约 10 cm 裕度。对快速运动，应缩短时域或由上述界设置裕度，最好使用工作空间特定分位（如 $\|H_p\|_{F,95}$）而非单一名义值。

**约束保证的准确含义。** QP 保证满足的是*线性化*安全走廊约束。真正的非线性碰撞间隙保证还需要用 $\varepsilon_p$ 保守收紧、对接受的 rollout 做非线性碰撞检查，或两者兼用。比笼统宣称无碰撞更安全的表述是：QP 保证预测轨迹不穿入声明的线性化障碍多面体；该结果如何转化到真实非线性几何，由上述线性化误差裕度决定。

对非凸环境，可在上游采用标准安全走廊分解技术 [5]。

**多个障碍。** 当同时存在 $M$ 个障碍时，方法 A 将 $M$ 组独立半空间行追加到 QP 不等式系统；问题仍为凸，求解器成本随 $M$ 温和增长（实践中 OSQP 迭代数对约束数呈次线性增长）。若障碍行冲突（如两个障碍将预测轨迹推向相反半空间），QP 会不可行。标准处理是引入逐障碍松弛变量 $s_m \geq 0$，并在代价中加入大惩罚 $\mu \|s\|^2$，将硬约束转为软约束，优先级由 $\mu$ 控制。方法 B 通过叠加各 APF 排斥场处理多个障碍；若需要单位方向，则叠加后归一化。这避免 QP 不可行，但继承标量 APF 的局部极小风险。

### B. 人工势场目标偏转（软约束）

对于追求最高求解吞吐的场景，可采用基于人工势场（APF）[6] 的软约束方法。位于 $p_{\text{obs}}$、影响半径为 $\rho_0$、排斥增益为 $\eta$ 的障碍产生任务空间排斥势：

$$\mathcal{U}_{\text{rep}}(p) = \begin{cases} \frac{1}{2} \eta \left(\frac{1}{\|p - p_{\text{obs}}\|} - \frac{1}{\rho_0}\right)^2 & \|p - p_{\text{obs}}\| \leq \rho_0 \\ 0 & \text{otherwise} \end{cases}$$

负梯度 $g_{\text{rep}}(p)=-\nabla \mathcal{U}_{\text{rep}}(p)$ 给出任务空间排斥方向。我们不将其解释为物理力（否则单位为牛顿，雅可比映射会在量纲上不一致），而是将其视为合成方向场。在归一化版本中，

$$\hat{g}_{\text{rep}}=\frac{g_{\text{rep}}}{\|g_{\text{rep}}\|+\epsilon},$$

启发式关节空间目标偏移由雅可比伪逆 $J_v^+$ 计算：

$$\delta q_{\text{obs}} = K \, J_v^+(q) \, \hat{g}_{\text{rep}}$$

其中 $K > 0$ 为经验整定的标量增益，用于设置偏转幅度。这是启发式目标偏移，不是物理力矩计算；$K$ 吸收所有量纲缩放，无机械意义。下文仿真中，同一 APF 思路被实现为速度/参考偏转，并可加入切向分量以避免停滞；但这只是实现层差异：两种情况下 APF 都改变供给 QP 的参考，而不是添加硬安全行。目标位置在线更新为

$$q_{\text{target}}(k) = q_{\text{goal}} + \delta q_{\text{obs}}$$

该方法完全绕过不等式行，保留最小 QP 结构并最大化求解速度。代价是避障通过代价函数软施加，而非由硬约束保证；APF 避障在复杂环境中也易陷入局部极小（见第 VIII 节）。

**局部极小缓解。** 当 APF 梯度消失（目标吸引与障碍排斥平衡）时，规划器会停滞。可用三类实用逃逸策略，复杂度递增：(1) *路径点注入*——若规划器超过可配置超时（如 0.5 s）未向目标取得进展，则在障碍侧向插入中间路径点，打破对称；(2) *随机扰动*——对 $q_\text{target}$ 添加一个小的随机关节空间位移，持续一个或数个周期以脱离平衡；(3) *混合切换*——检测停滞后启用方法 A（硬多面体约束）并带松弛变量，即使 APF 参考无信息，也迫使 QP 解离开不可行区域。实践中优先使用 (1)，因为它不增加求解器开销。两种方法互补：静态基础设施障碍（墙、桌面）使用硬多面体约束，动态障碍（人、移动物体）使用 APF 偏转。

---

## V. 架构性质

### A. 常结构 MPC 与有界重规划时延

核心架构性质是所有**动力学**预测矩阵（$\Phi$、$\Gamma$ 与 Hessian $H$）均离线预计算，在线从不重建。障碍约束行随障碍几何变化每周期重组，但其系数结构（$J_v$ 经固定 $\Gamma$ 投影）仍利用预计算 $\Gamma$。每个控制周期中，规划器求解一个动力学结构在初始化时即完全确定的 QP；只有障碍行变化。这意味着重规划时延——从检测到障碍到更新轨迹的时间——由一次 QP 求解界定，其成本随障碍约束数温和增长（实践中 OSQP 迭代数对约束数次线性增长），但与轨迹长度或机器人构型无关。

这不同于构型相关 MPC，后者必须在每个线性化点重算预测矩阵；也不同于批处理规划（TOTG、TOPPRA），后者在任意工作空间变化后都触发整条轨迹重建。常 $A_d$ 性质正是凸 formulation 中实现有界单周期反应性的结构性原因。

### B. 消除空间到时间的变换

传统规划流水线必须求解时间参数化问题：给定以弧长 $s \in [0,L]$ 参数化的几何路径 $\sigma(s)$，求 $s(t)$ 使关节速度与加速度约束满足。该空间—时间变换作为独立优化，与避障解耦，并且路径一旦改变就必须重新执行。

在本文框架中，没有几何路径。规划器直接在时间域工作；时域上的关节位置、速度与加速度都在单一 QP 中同时生成。$s \leftrightarrow t$ 转换阶段及其相关奇异性（第 VI.A 节）按构造被消除。

### C. $C^1$ 输出与硬有界加速度

由于本文规划器直接以关节加速度 $u_i$ 为优化变量，并将加速度界作为硬约束施加，输出 $(q_d, \dot{q}_d)$ 为 $C^1$，且 $\ddot{q}_d$ 始终硬有界。由于 $u_i$ 在每个控制周期内零阶保持，$\ddot{q}_d$ 分段常值并在周期之间跳变；若要完整 $C^2$ 连续性，需要在更高阶规划骨架中把加加速度作为优化变量。

### D. 用于连续加速度的分段加加速度三积分器扩展

当需要连续加速度时，例如降低力矩变化率激励、满足舒适性限制或约束车辆转向率，同一常 $A_d$ 原理可直接扩展到三积分器骨架。规划器不再优化加速度，而是优化加加速度：

$$\dddot q_i = j_i.$$

对每个坐标，定义高阶规划状态

$$z_i=\begin{bmatrix}q_i & \dot q_i & \ddot q_i\end{bmatrix}^\top,$$

连续时间动力学为

$$
\dot z_i =
\begin{bmatrix}
0&1&0\\
0&0&1\\
0&0&0
\end{bmatrix} z_i+
\begin{bmatrix}0\\0\\1\end{bmatrix}j_i .
$$

连续系统矩阵仍幂零，此时 $A_c^3=0$。精确 ZOH 离散化为

$$
A_d^{(3)} =
\begin{bmatrix}
1&\Delta t&\tfrac{1}{2}\Delta t^2\\
0&1&\Delta t\\
0&0&1
\end{bmatrix},
\qquad
B_d^{(3)} =
\begin{bmatrix}
\tfrac{1}{6}\Delta t^3\\
\tfrac{1}{2}\Delta t^2\\
\Delta t
\end{bmatrix}.
$$

因此预测矩阵仍与构型无关，仍可离线预计算。若每个控制间隔内加加速度保持常值，

$$
\ddot q(t+\tau)=\ddot q(t)+\tau j(t),\qquad 0\leq \tau\leq \Delta t,
$$

则在精确传播状态时，加速度跨采样点连续，而加加速度分段常值。输出光滑性从 $C^1$ 提升到 $C^2$：

$$q_d\in C^2,\qquad \dot q_d\in C^1,\qquad \ddot q_d\in C^0,$$

并可在线性形式下硬约束速度、加速度与加加速度：

$$|\dot q_i(k)|\leq \dot q_{i,\max},\qquad
|\ddot q_i(k)|\leq \ddot q_{i,\max},\qquad
|j_i(k)|\leq j_{i,\max}.$$

代价是状态维度增加，预测状态约束行增加；在相同坐标数与时域长度下，输入决策序列长度仍为 $nN$。决策向量仍是线性凸的，但其解释从加速度序列 $U=[\ddot q(0),\dots,\ddot q(N-1)]$ 变为加加速度序列 $J=[j(0),\dots,j(N-1)]$。第 VI.C 节的自动驾驶转向率基准正是对侧向通道使用该思想：转向率依赖侧向加加速度，因此三积分器侧向骨架是合适的光滑性类别。

本文其余部分以双积分器骨架作为主要机械臂规划器，因为它是直接施加速度与加速度限幅的最小模型，并给出最快 QP。当连续加速度或显式加加速度约束对应用关键时，三积分器形式可作为直接替换扩展。

### E. 双积分器骨架的解析 LQR 表征

双积分器骨架的另一个优势是，无约束无限时域二次跟踪问题可通过代数 Riccati 方程（ARE）得到解析的线性二次调节器（LQR）表征。实践中 ARE 可用代数或数值方法求解，但所得反馈律为下文有限时域 QP formulation 提供了一个有用的参考点。

考虑与构型无关的规划动力学

$$\dot x = \begin{bmatrix} 0&I \\ 0&0 \end{bmatrix} x +
\begin{bmatrix} 0 \\ I \end{bmatrix} u,$$

其中 $x= [q^\top, \dot q ^\top ]^\top$ 包含规划关节位置与速度，$u=\ddot q$ 为规划关节加速度。对参考轨迹 $x_r=[q_r^\top,\dot q_r^\top]^\top$，定义跟踪误差 $e=x-x_r$。无限时域二次跟踪代价为

$$ J = \int_0^\infty \left ( e^\top Q e + (u-u_{ff})^\top R (u-u_{ff}) \right )dt,$$

其中 $Q \succeq 0, R \succ 0$。无约束最优反馈律由连续时间代数 Riccati 方程给出：

$$ A^\top P + PA - PB R^{-1} B^\top P + Q = 0, $$

其中

$$ A= \begin{bmatrix} 0&I\\ 0&0 \end{bmatrix},\qquad
B= \begin{bmatrix} 0\\ I \end{bmatrix}.$$

反馈增益为 $K = R^{-1}B^\top P.$ 前馈项由

$$ \dot x_r = A x_r + B u_{ff} $$

定义。对双积分器骨架，这给出

$$u_{ff} = \ddot q_r. $$

最优跟踪律为

$$u = u_{ff} - K(x-x_r).$$

闭环误差动力学为
$$\dot e = (A-BK)e.$$

对固定目标 $x_r=[q_{goal}^\top,0^\top]^\top$ 的调节，前馈项为零：
$$u_{ff}=0.$$

LQR/ARE 解提供无约束候选轨迹。若该轨迹满足运动学与工作空间约束，则接受为有效运动计划；否则调用有限时域有约束 QP 规划器。所得轨迹

$$ \mathcal T = \{q_d(t),\dot q_d(t),\ddot q_d(t)\}$$

是所选参考与权重下无约束无限时域二次跟踪问题的全局最优解。

#### 与运动规划的关系

若 LQR/ARE 生成的轨迹满足所有运动学与工作空间约束，

$$ q_{\min} \le q_d(t) \le q_{\max}, $$

$$ |\dot q_d(t)| \le \dot q_{\max}, $$

$$|\ddot q_d(t)| \le \ddot q_{\max},$$

以及所有声明的避障要求（包括非线性几何所需的线性化裕度），则该无约束反馈律本身构成有效运动规划解。

然而在实践中，机器人经常工作在执行器限幅附近或杂乱环境中，此时障碍约束变为活动约束，无约束 LQR/ARE 轨迹可能违反可行性要求。可用同一双积分器骨架构造一个有限时域有约束对应问题，即滚动时域 QP：

$$ \min_U \frac12 U^\top H U + h^\top U $$

并满足

$$ CU \le d. $$

因此，本文预测式运动规划器可理解为无约束 LQR 表征的有限时域有约束对应物。无约束反馈律在约束不活动时给出最优无限时域轨迹；当运动学或线性化障碍约束活动时，QP 形式施加声明线性约束的可行性。

这种联系是双积分器骨架的关键优势之一：它在同一数学框架内同时拥有清晰的无约束 LQR 表征与凸有约束实现。

本文框架与线性 MPC 在数学上相似，因为它使用线性预测模型求解滚动时域二次规划 [9]。但其在机器人软件栈中的角色不同。传统 MPC 将预测模型视为物理对象近似，并计算控制作用来调节对象状态。相反，本文 formulation 使用虚拟规划状态，其目的在于轨迹生成而非对象预测。机器人模型仍由运动学/操作空间映射 $p=f(q)$ 及其工作空间和关节约束给定 [8]。

从这个角度看，双积分器骨架应被视为一种轨迹参数化，它定义了可接受运动的光滑性类别。优化器在该轨迹空间中搜索可行运动计划。规划器输出因此是参考轨迹 $(q_d,\dot q_d,\ddot q_d)$，随后可由任意合适的底层控制器跟踪。

$$\mathcal T = \{q_d(t),\dot q_d(t),\ddot q_d(t)\} $$

该轨迹表示与标准机器人执行流水线（包括样条生成器、TOTG 与 TOPPRA）期望的输出相同。差别在于轨迹如何生成，而非所得参考形式。

本文框架的新颖性不在于双积分器预测模型本身，后者早已成熟；而在于利用一种与构型无关的轨迹参数化，使预测矩阵可完全离线构造，同时保留对运动学与障碍约束的直接施加能力。

### F. 与传统流水线对比

```
TRADITIONAL PIPELINE (MoveIt + OMPL + TOTG):
[Perception] → [3D Map] → [OMPL Geometric Search] → [TOTG: s → t] → [Controller]
                                                       (Batch; full replan on obstacle change)

PROPOSED UNIFIED BACKBONE:
+----------------------------------------------------------+
| Layer 2: Receding-Horizon Time-Domain QP (100 Hz+)       |
|   • Precomputed constant Ad matrix (fast rollouts)       |
|   • Kinematic bounds as hard constraints                 |
|   • Obstacle avoidance (hard polytope or APF)            |
|   • Outputs: q_d, q_dot_d, q_ddot_d (C¹, q_ddot bounded)|
+----------------------------------------------------------+
                ↑ Perception constraints fed online
                           ↓
+----------------------------------------------------------+
| Layer 1: Low-Level Tracking Control Loop (1 kHz)         |
|   • Computed torque / impedance / admittance             |
+----------------------------------------------------------+
```

---

## VI. 实验评估

我们在 7 自由度 Franka FR3 机械臂与自动驾驶转向任务上，将所提规划器与路径优先的时间最优轨迹生成流水线进行对比。两个规划器接收相同路径点与运动学限幅，并以相同 $\Delta t$ 输出 $(q_d, \dot q_d, \ddot q_d)$；下文所有数值均由配套开源测试程序（`benchmarks/`）实测得到。TOTG 参考使用数值积分 TOPP 内核：圆弧过渡路径（Kunz–Stilman 几何 [10]）、对 $\dot s^2$ 做时间最优前向/后向扫掠 [7]，再执行 $s\!\to\!t$ 反演与重采样。

**对比范围。** 本文对比聚焦路径优先家族（TOTG、TOPPRA）以及一个 resolved-rate + APF 反应式基线。更完整评估还应包括在线 MPC、CHOMP/TrajOpt 或 MPPI 类方法；本文规划器的吸引力恰在于其线性凸结构带来常 $A_d$ 预计算与有界单周期时延，而更丰富的非线性 formulation 通常放弃这些性质。相关比较留作未来工作。

### A. 路径优先 $t\!\leftrightarrow\!s$ 转换伪影

$s\!\to\!t$ 重构会对几何路径求导：

$$\ddot q(t) = q'(s)\,\ddot s \;+\; \underbrace{q''(s)}_{\text{curvature vector}}\,\dot s^2 .$$

标准圆弧过渡路径表示带来两类病态。第一，$q''(s)$ 在每个过渡接缝处跳变（直线段为 $0$，圆弧段为 $1/r$），因此重构的 $\ddot q$ 不连续，且跳变随 $\dot s^2$ 放大。第二，$t(s)=\int ds/\dot s$ 在任何 $\dot s \to 0$ 处奇异（停驻点、紧近反向过渡），使均匀时间重采样病态。这些伪影属于圆弧过渡路径表示，而非时间参数化本身。如下文验证，用 $C^2$ 路径（如三次样条）替代圆弧可消除它们。时间优先规划器从不构造 $s$；其 $\ddot q$ 即为有界决策变量，故不会产生这两类病态。

$(s,\dot s)$ 相平面（图 1）使奇异性可视化：最大速度曲线与最优曲线 $\dot s^*(s)$ 在紧过渡处向零下探；近反向时尤为严重，$\dot s\to0$ 使 $t(s)$ 奇异。

![Phase-plane figure](benchmarks/topp_phaseplane.png)
***图 1.** $(s,\dot s)$ 相平面中的时间最优路径参数化（TOPP-RA 风格）。最优曲线在紧过渡处下探；在近反向情形中跌至 $\dot s\approx 0$，正是 $s\!\to\!t$ 映射变得奇异之处。*

### B. FR3 机械臂结果

为便于解读，应力集中在两个关节，但全部七个关节均参与规划。关键指标是峰值加速度比 $\lVert\ddot q\rVert_\infty/\ddot q_{\max}$：大于 $1$ 表示输出轨迹违反声明的加速度限幅。DI 规划器将该指标作为 QP 硬约束施加，并在所有场景中钉在 $1.000$ 且零违反。

| Scenario | DI $T$ [s] | TOTG $T$ [s] | DI accel ratio | TOTG accel ratio | TOTG accel viol. | DI jerk RMS | TOTG jerk RMS |
|---|---|---|---|---|---|---|---|
| B1 point-to-point | 2.02 | **0.59** | 1.000 | 1.000 | 0 | **22** | 143 |
| B2 acute corner (tight blend) | 1.92 | **1.23** | 1.000 | **1.047** | 2 | 36 | 133 |
| B3 near-reversal | 1.99 | 1.07 | 1.000 | up to **3.13** | 1–4 | 29 | 166 |
| B4 dense (24 waypoints) | 13.24 | **3.83** | 1.000 | **1.445** | 49 | **54** | 333 |

**计时表。** 为避免混淆解耦与耦合情形，下面列出配套单线程 Python 测试程序中的测量值（OSQP 绝对/相对容差 $10^{-6}$，热启动开启，耦合障碍更新路径的 KKT 稀疏结构固定）：

| Case | Variables | Mean solve [ms] | Worst-case solve [ms] |
|---|---|---|---|
| Decoupled box constraints (7 DOF, N=20) | 140 | 0.05–0.09 | ~0.4 |
| Coupled obstacle QP (Method A, fixed-sparsity update) | 141 | 2.1 | 9.8 |
| Decoupled goal reaction (warm-started) | 140 | 0.03 | — |

**时域长度扫掠。** 同一测试程序在 FR3 动态障碍问题上对 $N=\{10,20,30,40\}$ 做扫掠，使用固定稀疏耦合 QP（`sweep_N.py`）。该障碍场景使用速度律/APF 参考，因此完成时间与间隙对 $N$ 不特别敏感；主导趋势是计算成本。

| Horizon $N$ | Look-ahead [s] | Completion [s] | Min EE clearance [m] | Jerk RMS [rad/s³] | Solve p95 [ms] | Full cycle p95 [ms] |
|---:|---:|---:|---:|---:|---:|---:|
| 10 | 0.10 | 1.86 | 0.297 | 275.5 | 1.09 | 2.55 |
| 20 | 0.20 | 1.83 | 0.296 | 277.3 | 4.45 | 6.12 |
| 30 | 0.30 | 1.84 | 0.298 | 292.7 | 20.85 | 23.04 |
| 40 | 0.40 | 1.86 | 0.296 | 289.2 | 56.90 | 59.40 |

同一脚本中的静态路径点扫掠说明了为何 $N=20$ 仍为默认值：纯位置跟踪任务中 $N=10$ 可能显著放大到达时间，而 $N\geq30$ 会提高耦合 QP 时延，却不改善动态障碍测试中的间隙。

**对比范围说明。** 加速度违反指标比较的是各规划器原样输出的轨迹，也就是直接送入控制器的命令序列，不对任一规划器输出施加下游平滑或再插值。实际系统中有时会对 TOTG 输出再做最终样条处理，这会降低或消除 B2 与 B4 中由重采样诱发的违反；若加入此类后处理，DI 在这些场景上的零违反优势会缩小。B3（近反向奇异）与反应式场景（第 VI.D–F 节）的优势则不依赖下游平滑。

图 2 绘制各场景中的加速度比：DI 曲线从不越过 $1$，而 TOTG 曲线在过渡接缝处越界。

![FR3 acceleration-ratio comparison](benchmarks/fr3_accel_compare.png)
***图 2.** FR3 输出轨迹的加速度比 $\max_j|\ddot q_j|/\ddot q_{j,\max}$ 随时间变化，DI-QP（蓝）对 TOTG（红），B1–B4。虚线为限幅（$=1$）。DI-QP 由硬约束钉在限幅内；TOTG 在接缝处越界：B2 为 $1.047\times$，B3 最高达 $3.13\times$，B4 为 $1.445\times$。*

在 **B3（近反向）** 中，过渡半径塌缩，$\dot s\to 0$，参数化变为奇异：$\max(1/\dot s)\to\infty$，$s\!\to\!t$ 转换失败（由测试程序标记并截断）。在 **B4** 中，路径优先命令在 49 个样本处超过加速度限幅 45%。DI 规划器全程保持可行且光滑。

**对参数化器的稳健性（TOPP-RA）。** 为排除弱基线伪影，我们用基于可达性的 **TOPP-RA** [7] 重跑。结构性结论仍成立：锐角拐角处重采样命令仍超出加速度限幅（TOPP-RA 为 $1.020\times$，而上方 FR3 主表中使用的数值积分基线为 $1.047\times$）；稠密路径仍违反（$1.12\times$，33 个样本）；近反向情形对两种参数化器都仍奇异（$\min\dot s = 0$）。TOPP-RA 确实降低了幅值，B3 的 $3.13\times$ 尖峰在 TOPP-RA 下变为 $1.000\times$，这区分了依赖实现的尖峰大小与路径优先固有的曲率不连续重构和 $\dot s\to0$ 奇异。注意，用 $C^2$ 样条替代圆弧过渡可完全消除 B2 与 B4 的加速度违反；这说明违反来自过渡几何，也说明在这些指标上，基于 $C^2$ 路径的 TOPP-RA 是 DI 规划器不能宣称严格优越的竞争基线。

**时间最优性。** TOTG 按构造给出下界，而二次代价 DI 规划器以时间换取光滑性（B1 慢 $3.4\times$，B4 慢 $3.5\times$）。提供解析时间最优速度律参考后，可将差距恢复到 $1.06$–$1.32\times$，同时 DI-QP 仍保持零限幅违反。因此，DI 规划器相对于声明的时间域约束按构造保持可行；而路径优先轨迹在圆弧过渡重构与均匀时间重采样后可能丢失可行性，除非使用更光滑的路径表示或下游后处理。

### C. 基于双积分器骨架的自动驾驶运动规划

同一架构原则可自然扩展到自动驾驶运动规划。与机械臂情形一样，规划器目标不是直接计算执行器命令，而是生成满足运动学约束且能响应环境变化的可行时间参数化轨迹。

Apollo 等量产自动驾驶栈将规划与控制分离。规划器输出包含位置、航向、曲率、速度与加速度信息的轨迹点序列；下游横纵向控制器生成跟踪该轨迹所需的转向、油门与制动命令。规划器输出形式为

$$
\mathcal T = \{ x(t), y(t),\theta(t),\kappa(t),v(t),a(t) \},
$$

类似机械臂轨迹

$$ \mathcal T = \left \{ q_d(t), \dot q_d(t), \ddot q_d(t) \right \}. $$

#### 车辆运动学模型

考虑标准运动学自行车模型：

$$ \dot x = v \cos\theta, $$

$$ \dot y = v \sin\theta, $$

$$\dot\theta = \frac{v}{L}\tan\delta,$$

$$ \dot v = a, $$

其中 $L$ 为轴距，$\delta$ 为转向角，$a$ 为纵向加速度。

车辆的支配方程仍是这些非线性运动学关系。但如机械臂 formulation 一样，可用虚拟笛卡尔双积分器规划参数化作为时间域轨迹生成器。它本身并不消除车辆可行性约束；它产生候选笛卡尔位置、速度、加速度曲线，再由这些量检查或约束航向、曲率、转向与加速度限制。规划坐标作为虚拟双积分器演化：

$$ \ddot p = u, $$

其中

$$ p = \begin{bmatrix} x\\ y \end{bmatrix},
u = \begin{bmatrix} u_x \\ u_y \end{bmatrix}$$

为虚拟规划加速度。定义规划状态

$$ X = \begin{bmatrix} p\\ \dot p \end{bmatrix},$$

则预测模型为

$$ \dot X = \begin{bmatrix} 0&I\\ 0&0\end{bmatrix} X + \begin{bmatrix} 0\\ I \end{bmatrix} u. $$

该模型与机械臂规划的双积分器骨架相同，因此继承同一常状态转移矩阵 $A_d$、离线预测矩阵构造与固定结构 QP。车辆特定可行性条件通过附加代数约束或事后检查进入，而非通过构型相关预测矩阵进入。

#### 轨迹生成

规划器直接在时间域优化轨迹，而非先构造几何路径再执行独立时间参数化。位置、速度与加速度曲线在单一滚动时域优化中同时生成。

每个周期中，优化器输出一段预测笛卡尔状态

$$ \{x(k),y(k),\dot x(k),\dot y(k),\ddot x(k),\ddot y(k)\} _{k=1}^{N},$$

再由车辆运动学恢复航向与曲率：

$$ \theta(k)=\operatorname{atan2}(\dot y(k),\dot x(k)),$$

$$ \kappa(k) = \frac{\dot x(k)\ddot y(k)-\dot y(k)\ddot x(k)}
{\left(\dot x(k)^2+\dot y(k)^2\right)^{3/2}}. $$

仅当 $v=\sqrt{\dot x^2+\dot y^2}>v_{\min}$ 时计算曲率；低于阈值时保留上一曲率估计。相对自行车模型的可行性至少要求

$$|\kappa(k)| \leq \kappa_{\max}=\frac{\tan\delta_{\max}}{L},$$

$$v(k)^2|\kappa(k)| \leq a_{y,\max},$$

以及速度与纵向加速度界。这些约束只有在线性化后才能追加到 QP，或者作为接受检查并触发重规划。最终规划器输出为标准自动驾驶轨迹：

$$ (x,y,\theta,\kappa,v,a) $$

并按离散时间间隔采样。

#### 规划与控制分离

规划器不直接生成转向或油门命令。转向角、油门与制动由下游跟踪控制器根据规划轨迹计算。

因此，自动驾驶 formulation 与机械臂 formulation 遵循相同的规划/控制分离：

$$ \text{Vehicle Feasibility Checks / Linearized Constraints} \rightarrow
\text{Double-Integrator Trajectory Backbone} \rightarrow
\text{Predictive Motion Planner} \rightarrow
(x,y,\theta,\kappa,v,a) $$

随后在控制层中

$$ (x,y,\theta,\kappa,v,a) \rightarrow (\delta, T,B).$$

这保留了统一解释：双积分器骨架是用于生成候选轨迹的构型无关规划模型；车辆特定可行性与执行仍由附加约束和独立控制系统负责。

#### 自动驾驶转向基准

配套 AV 测试程序评估从规划笛卡尔轨迹恢复的转向量。该基准是诊断性的：若未约束曲率，纯笛卡尔双积分器轨迹仍可能超过转向率限制。为展示补充缺失转向率光滑性的效果，测试程序也包含固定巡航速度下的加加速度有界侧向扩展（三积分器侧向骨架）。该扩展不是本文主要双积分器贡献，但展示了同一时间域原则：优化必须被约束的导数量，再从所得轨迹恢复车辆命令。

| Scenario | Planner | Peak steering [deg] | Peak steering rate [rad/s] | Steering jump [rad] | Peak lateral accel [m/s²] |
|---|---|---:|---:|---:|---:|
| Single lane change | jerk-bounded time-first | 3.6 | **0.129** | **0.0065** | 3.99 |
| Single lane change | TOTG path-first | 1.5 | 0.254 | 0.0254 | 2.47 |
| Double lane change | jerk-bounded time-first | 3.6 | **0.129** | **0.0065** | 4.00 |
| Double lane change | TOTG path-first | 3.4 | 0.594 | 0.0594 | 4.05 |

在代表性转向率上限 0.5 rad/s 下，加加速度有界的时间优先规划器在两种巡航机动中均保持在执行器速率包络内；路径优先命令在双变道中由于圆弧过渡曲率在路径接缝处跳变而越过该上限。测试程序也报告纯笛卡尔双积分器版本；本文有意不宣称它在未加入曲率/速率约束或更高阶侧向状态时满足转向率约束。

### D. 有界时延反应式重规划

反应性优势是结构性的，而非依赖具体实现。一个平面质点机器人需要到达目标，同时圆盘障碍在执行中途横穿其直线路径。DI 规划器每个 100 Hz 周期都从障碍当前位置重新计算 APF 速度参考，并重解盒约束 QP，持续偏转。路径优先规划器必须停车、重建绕行路径，并从静止重新运行 TOPP。

二者都无碰撞到达目标，但代价结构不同。DI 规划器的**最坏反应时延为 $0.71$ ms 且与场景无关**：它是单个固定尺寸 QP。我们将重规划时延从 $0$ 扫到 $400$ ms：

| Added path-search latency | TOTG completion [s] | TOTG time halted [s] | DI completion [s] |
|---|---|---|---|
| 0 (TOPP only) | 6.28 | 0.08 | **6.53 (flat)** |
| 100 ms | 6.35 | 0.36 | 6.53 |
| 200 ms | 6.63 | 0.66 | 6.53 |
| 400 ms | 7.18 | 1.26 | 6.53 |

该结构性优势受固定优化结构界定，结构规模随活动障碍约束数增长：DI 在一个周期内偏转且从不停车，而 TOTG 必须停车并重建。当重规划昂贵（完整几何搜索）或障碍快速时，反应优势是决定性的；当重建廉价时则较微弱。DI 规划器并不展示更低总吞吐；在零重规划时延下，TOTG 与 DI 完成时间相当。

![Dynamic obstacle, planar](benchmarks/dynamic_obstacle.png)
***图 4.** 平面动态障碍测试：DI 规划器（实线）连续偏转；路径优先规划器（虚线）停车并重建。DI 反应时延界为 $0.71$ ms，且与场景复杂度无关。*

### E. APF 局部极小失败与路径点注入恢复

APF 方法被有意作为软性、高吞吐启发式方法提出，而非碰撞证书方法。为暴露其失败模式，我们将平面双积分器机器人、目标与圆形障碍放在同一直线上。纯径向 APF 场中，吸引项与排斥项在障碍前平衡，规划器停滞。简单的路径点注入规则检测到缺乏进展后，在障碍侧向插入绕行序列；同一盒约束 QP 随后在保持速度与加速度界的同时到达目标（`apf_local_minimum.py`）。

| Metric | Pure APF | Waypoint injection |
|---|---:|---:|
| Reached goal | no | yes |
| Completion / final time [s] | 17.99 | 15.56 |
| Final distance to goal [m] | 6.07 | 0.12 |
| Min obstacle clearance [m] | 0.22 | 0.22 |
| Max speed [m/s] | 1.50 | 1.50 |
| Max acceleration [m/s²] | 3.00 | 3.00 |
| Mean QP solve [ms] | 0.092 | 0.135 |

该基准支持本文的混合建议：APF 参考偏转适合快速反应式转向；当进展停滞或间隙必须被认证时，应启用硬多面体约束或显式路径点/安全走廊逻辑。

### F. 扩展到带动态障碍的 7 自由度机械臂

我们在 7 自由度 FR3 上运行完整第 IV 节机制，此时障碍约束通过平动雅可比耦合各关节。机械臂到达关节空间目标，同时球形障碍穿过末端执行器路径。对比基线为无硬限幅的 resolved-rate + APF 反应式控制器。

| Metric | DI-QP | Reactive baseline |
|---|---|---|
| Reached goal | yes | yes |
| Min EE clearance [m] | 0.296 | 0.339 |
| Joint velocity violations | **0** | 42 |
| Joint acceleration violations | **0** | 346 |
| Worst-case solve [ms] | 9.8 | — |
| Max slack used [m] | 0.000 | — |

二者都避开障碍并到达目标。反应式基线缺少硬约束，在 42 个样本处超过关节速度界、在 346 个样本处超过加速度界；QP 则精确守住所有关节界（零违反），并全程保持线性化半空间可行（零松弛）。将原先每周期重建的原型替换为固定稀疏 OSQP 更新路径后，测试程序中的名义耦合 QP 求解时间为均值 2.1 ms、p95 4.4 ms、最坏 9.8 ms（`fixed_sparsity_timing.py`）。若计入 Python 侧组装与更新开销，完整单周期时间为均值 3.8 ms、p95 6.1 ms、最坏 11.7 ms；因此该 Python 测试程序在 p95 意义上满足 100 Hz，但最坏周期若无进一步实现优化会超过 10 ms。

![FR3 dynamic-obstacle signals](benchmarks/fr3_dynamic.png)
***图 5.** 7 自由度 FR3 动态障碍。左上：DI-QP 关节速度，全部在限幅内。右上：resolved-rate+APF 基线，超过速度界。左下：末端间隙。右下：DI-QP 单周期求解时间。*

![FR3 obstacle-avoidance render](benchmarks/fr3_motion.png)
***图 6.** FR3 执行规划轨迹的 MuJoCo 渲染，其间球形障碍（红）穿过工作空间。*

### G. 随机化 FR3 动态障碍鲁棒性

为检查动态障碍结果并非绑定于单一手选构型，我们在 20 次随机试验中扰动起点与目标关节状态、障碍侧向偏移、初始高度与穿越速度（`fr3_dynamic_randomized.py`，seed 7）。两个规划器在所有试验中均到达目标并避开障碍。区别在于约束忠实度：QP 在每次试验中保持硬关节限幅，而 resolved-rate 基线反复越界。

| Metric | DI-QP | Reactive baseline |
|---|---:|---:|
| Success rate | 100% | 100% |
| Min EE clearance, mean ± std [m] | 0.293 ± 0.018 | 0.328 ± 0.021 |
| Worst min EE clearance [m] | 0.261 | 0.266 |
| Total joint velocity violations | **0** | 618 |
| Total joint acceleration violations | **0** | 6889 |
| Max slack used [m] | 0.0000 | — |
| QP solve p95 / max [ms] | ≈5.3 / ≈13.6 | — |

基线通常保持略大间隙，因为它无约束，可命令任意激进的关节速度与加速度。QP 接受较小但仍为正的间隙，同时守住所有声明的关节界。

### H. MuJoCo 中的物理执行

为验证规划器层面的差异在刚体动力学中仍存在，我们在 MuJoCo（3.8.1；完整质量矩阵与重力）中以力矩控制的 FR3 执行两条参考。一个 500 Hz 计算力矩控制器在锐角拐角机动上跟踪各自的 100 Hz 参考。

| Metric (joint 1 / worst) | DI-QP | TOTG |
|---|---|---|
| Peak demanded torque [Nm] | 58.0 | 56.7 |
| Torque-rate RMS [Nm/s] | **181** | 383 |
| Samples over torque limit | 0 | 0 |
| Tracking RMSE [mrad] | 1.9 | 2.0 |

二者都保持在力矩包络内，并跟踪至约 2 mrad RMS。TOTG 在过渡接缝处的加速度不连续在物理上表现为阶梯式力矩命令（力矩变化率 RMS 为 DI 参考的 $2.1\times$），而 DI 参考产生光滑力矩曲线。TOTG 更快完成固定路径；DI 命令对传动系统更温和。这把光滑性与时间之间的折中从规划器输出搬到了闭环物理中。

![MuJoCo computed-torque comparison](benchmarks/mujoco_compare.png)
***图 7.** FR3 在 MuJoCo 中的计算力矩执行。DI 参考产生光滑加速度与力矩；TOTG 在接缝处产生阶梯命令。二者都保持在力矩限幅内，跟踪 RMSE 相近。*

---

## VII. 实现说明

### A. 求解器配置

QP 由 OSQP [4] 求解，配置如下：

- **热启动：** 以上一解 $U^*$ 作为下一次求解初值，在基准测试中将解耦盒约束 QP 的冷启动时延从约 5 ms 降至 0.5 ms 以下。
- **利用稀疏结构：** Hessian $H$ 离线组装一次。解耦基准中，OSQP 对象设置一次，在线仅更新向量。耦合障碍基准中，障碍半空间使用固定稀疏模式，并通过 OSQP 的 `Ax` 更新就地修改数值；这避免每周期重建符号分解。
- **时域长度：** 在 100 Hz 规划频率（$\Delta t = 10$ ms）下，$N = 10$（0.1 s 前瞻）对纯位置跟踪路径点任务过短，到达时间可能膨胀数倍。**推荐默认值为 $N = 20$（0.2 s）**：它在解耦路径点基准中恢复大部分可达速度，同时使固定稀疏耦合障碍 QP 的 p95 求解时间保持在数毫秒内。增大到 $N \approx 30$ 可在部分场景中缩小静态时间最优性差距，但会增加加加速度，并显著提高耦合障碍约束下的求解时间。

### B. 目标序列与路径点跟踪

对多路径点任务，当路径点被到达（落入阈值 $\epsilon_q$）时，目标状态 $x_{\text{goal}}$ 在线更新。由于规划器在时间域中以滚动时域 formulation 工作，路径点切换平滑，无需显式路径拼接或重初始化。

### C. 计算复杂度

对 $n$ 个关节与时域 $N$，决策变量数为 $nN$。7 自由度机器人在 $N=20$ 下，盒约束情形有 140 个决策变量；加入一个共享障碍松弛变量时有 141 个。热启动 OSQP 在普通 CPU 上对解耦盒约束 QP 远低于 1 ms 收敛，对固定稀疏耦合障碍 QP 在数毫秒内收敛。耦合障碍约束增加行数与稀疏线性代数成本，但符号结构可保持固定，仅更新数值。预计算的 $\Phi$ 与 $\Gamma$ 矩阵在初始化时构造一次且从不重建；在线计算负担主要由 QP 求解本身（OSQP 内部 ADMM 迭代与稀疏线性求解）主导，而非预测矩阵构造。后者已通过常 $A_d$ 性质完全离线化，这是该性质的实际计算收益。

---

## VIII. 讨论

本文框架实现了一种具体且清晰的运动规划架构转变：不再先在空间中规划再在时间中参数化，而是将轨迹生成与时间域约束施加统一到一个持续运行的优化循环中。常 $A_d$ 性质源自双积分器系统矩阵的幂零性，是使这一点成立的结构性洞见。它将轨迹预测的计算负担与机器人构型解耦，使标准硬件上的 100 Hz 实时重规划可行。

### 范围与局限

该框架应理解为**局部预测式运动规划器**，而非完整全局规划器。当前 formulation 有若干局限：

**(i) 默认非时间最优。** 使用纯位置跟踪代价时，规划器像阻尼二阶系统那样调节到目标，在用尽全部执行器能力之前就提前减弱，导致到达时间膨胀至 TOTG 的 $3$–$3.5\times$。这很大程度上可由解析时间最优速度律参考恢复，把差距缩小到 $1.06$–$1.32\times$，同时保持零限幅违反。因此速度参考可视为可整定的速度—光滑性选择器。下表总结主要工作点上的折中与推荐应用：

| Mode | $R$ (accel. weight) | Velocity reference | Time vs. TOTG | Jerk RMS | Recommended for |
|---|---|---|---|---|---|
| Maximum smoothness | large ($\geq 1.0$) | position-tracking only | $3$–$3.5\times$ | low (~22) | Human collaboration, compliant tasks |
| Balanced | medium ($0.1$) | position-tracking only | $2$–$2.5\times$ | medium | General manipulation |
| Near-time-optimal | small ($0.01$) | analytic velocity law | $1.06$–$1.32\times$ | high (~bang-bang) | High-throughput assembly, no humans |

近时间最优模式接近 bang-bang 加速度；当下游执行器可承受快速力矩变化时可以接受。若有人附近或关心传动系统寿命，光滑模式更合适。

**(ii) 线性化精度。** 雅可比线性化（方法 A）只在当前构型邻域内准确；误差随关节速度 × 时域大致二次增长。FR3 测试中，50% 关节速度、0.2 s 时域下的末端时域误差已约 10 cm。对快速运动或需要大绕行的大障碍，应缩短时域、用 Hessian/余项界增大安全裕度，或对接受的 rollout 做非线性几何检查。

**(iii) 局部极小。** APF 避障（方法 B）在复杂环境中易陷入局部极小。有限 QP 时域下的持续可行性可通过在时域末端附加终端安全集（如最大制动到静止曲线）改善。

**(iv) 有限前瞻。** 100 Hz 下 $N = 20$ 只提供 $0.2$ s 前瞻，对非常快速移动的障碍可能不足。

### 与路径优先流水线的关系

路径优先基线（TOTG、TOPP-RA）按构造时间最优并一次性规划整条路径，因此在静态任务上拥有本文规划器不匹配的优势。B2 与 B4 的加速度违反来自圆弧过渡几何，而非时间参数化本身；用 $C^2$ 样条替代圆弧可完全消除这些违反。同样，当路径具备有界曲率时，近反向奇异可被缓解。这些观察支持在任何时间参数化器中使用高质量路径表示。

路径优先结构内在且不通过修补即可消除的是：(a) 缺少有界单周期反应性，任何变化都会触发完整批处理重建；(b) 必须预先计算几何路径。这正是本文时间域架构要解决的局限。两种方法互补：本文规划器以全局时间最优性与全路径前瞻为代价，换取可行性、光滑性、几何退化处的良态性与有界时延反应性。

### 验证空白

当前证据有三处边界。第一，验证仅限仿真；最重要的下一步是在物理 FR3 上演示移动障碍，证明所声称反应性在真实硬件上无需安全急停也成立。第二，对比覆盖路径优先方法与 resolved-rate+APF 反应式基线。第 VI.D 节动态障碍测试使用 full-stop-and-rebuild TOTG 流水线；更强基线应是增量几何重规划器（如 RRT* 或带前一路径热启动的 OMPL）配合 TOPP-RA，这会降低重规划时延并缩小反应性差距。该比较留作未来工作，但若要支撑相对先进在线重规划方法的反应性主张，需要明确开展。与非线性 MPC、微分动态规划、MPPI 或学习型反应策略正面对比，也将更好定位该方法。第三，当前只有 FR3 动态障碍实验包含随机鲁棒性研究；其余表格是单场景或小场景组评估。未来工作应对每类基准至少报告 20–50 次随机试验的均值与标准差，以建立不依赖特定构型的鲁棒性主张。

---

## IX. 结论

本文提出一种建立在常状态转移双积分器骨架上的统一时间域局部预测式运动规划器。核心观察是：虚拟双积分器规划器给出与机器人构型无关的常 $A_d$，从而可离线预计算所有动力学预测矩阵，将在线计算降为固定活动约束结构下具有有界单周期时延的凸 QP。所得规划器输出 $C^1$ 轨迹 $(q_d, \dot{q}_d, \ddot{q}_d)$ 且加速度硬有界，并通过滚动时域 QP 更新响应工作空间变化，达成批处理“先规划后执行”流水线在结构上不具备的有界反应行为。对需要连续加速度的应用，同一构造可扩展到具有常预测矩阵的分段加加速度三积分器骨架。该框架受 pHRI 架构启发，但仅将所得双积分器骨架作为规划模型使用，不依赖任何特定底层对消策略。

未来工作将把该框架验证于物理 7 自由度机械臂，通过安全走廊分解将避障扩展到非凸环境，并与在线 MPC 和轨迹优化方法进行基准对比。

---

## 参考文献

[1] I. A. Şucan, M. Moll, and L. E. Kavraki, "The Open Motion Planning Library," *IEEE Robot. Autom. Mag.*, vol. 19, no. 4, pp. 72–82, 2012.

[2] D. Lertkultanon and Q.-C. Pham, "Time-optimal path parameterization for redundantly actuated robots," *IEEE/ASME Trans. Mechatronics*, vol. 21, no. 4, pp. 1643–1651, 2016.

[3] Anonymous Author(s), "Impedance MPC for Physical Human–Robot Interaction: Predictive Disturbance Rejection with Joint-Limit Safety," *submitted for review*, 2024.

[4] B. Stellato, G. Banjac, P. Goulart, A. Bemporad, and S. Boyd, "OSQP: An operator splitting solver for quadratic programs," *Math. Program. Comput.*, vol. 12, no. 4, pp. 637–672, 2020.

[5] S. Liu *et al.*, "Planning dynamically feasible trajectories for quadrotors using safe flight corridors in 3-D complex environments," *IEEE Robot. Autom. Lett.*, vol. 2, no. 3, pp. 1688–1695, 2017.

[6] O. Khatib, "Real-time obstacle avoidance for manipulators and mobile robots," *Int. J. Robot. Res.*, vol. 5, no. 1, pp. 90–98, 1986.

[7] H. Pham and Q.-C. Pham, "A new approach to time-optimal path parameterization based on reachability analysis," *IEEE Trans. Robot.*, vol. 34, no. 3, pp. 645–659, 2018.

[8] O. Khatib, "A unified approach for motion and force control of robot manipulators: The operational space formulation," *IEEE J. Robot. Autom.*, vol. 3, no. 1, pp. 43–53, 1987.

[9] D. Q. Mayne *et al.*, "Constrained model predictive control: Stability and optimality," *Automatica*, vol. 36, no. 6, pp. 789–814, 2000.

[10] T. Kunz and M. Stilman, "Time-optimal trajectory generation for path following with bounded acceleration and velocity," in *Robotics: Science and Systems*, 2012.


# 附录 A. 双积分器规划骨架的无约束 ARE 解

本附录说明双积分器规划骨架对应的无约束 LQR 解。该结果提供一个有用的参考轨迹生成器，并澄清解析线性二次调节器（LQR）表征与正文中有约束预测式运动规划器之间的关系。

## A.1 双积分器规划模型

规划器采用与构型无关的双积分器骨架：

$$ \dot x = Ax + Bu,$$

其中
$$ A= \begin{bmatrix} 0&I\\ 0&0\end{bmatrix}, \qquad B=\begin{bmatrix}0\\I\end{bmatrix},$$

$x=[q^\top, \dot q ^\top ]^\top$ 包含规划关节位置与速度，$u=\ddot q$ 为规划关节加速度。

对 7 自由度机械臂，
$$
q\in\mathbb{R}^7, \qquad
x\in\mathbb{R}^{14}, \qquad
u\in\mathbb{R}^{7}. $$

规划器输出为时间索引轨迹

$$\mathcal T = \{q_d(t),\dot q_d(t),\ddot q_d(t)\}.$$

## A.2 由末端目标生成参考

假设末端执行器需要从

$$p_0= \begin{bmatrix} 0\\ 0\\ 0 \end{bmatrix} $$

移动到

$$p_g=\begin{bmatrix} 1\\ 1\\1 \end{bmatrix}. $$

令 $p=f(q)$ 表示正运动学。期望关节空间目标可由逆运动学得到：
$$q_g = IK(p_g), $$

或在局部由雅可比线性化近似：

$$q_g \approx q_0 + J(q_0)^\dagger \bigl(p_g-p_0\bigr). $$

对应参考状态为

$$x_r= \begin{bmatrix} q_g \\0 \end{bmatrix},$$

表示期望最终构型与零速度。

## A.3 代数 Riccati 方程

对非零参考的调节，定义跟踪误差 $e=x-x_r$。考虑无限时域二次代价

$$J = \int_0^\infty \left(e^\top Qe + u^\top Ru \right) dt,$$

其中 $Q\succeq0, R\succ0.$ 连续时间代数 Riccati 方程为

$$A^\top P + PA - PBR^{-1}B^\top P + Q = 0.$$

最优反馈增益为 $K =R^{-1}B^\top P.$ 对参考状态调节，最优反馈律为 $u=-K(x-x_r).$ 等价地，
$$ \ddot q = -K_p(q-q_g) - K_d\dot q,$$

其中 $K_p$ 与 $K_d$ 是 $K$ 中的位置与速度反馈分量。

## A.4 闭环轨迹生成

定义跟踪误差 $e=x-x_r.$ 将最优控制律代入规划动力学，得到 $\dot e = (A-BK)e.$ 由于闭环矩阵 $(A-BK)$ Hurwitz，跟踪误差渐近收敛到零：$e(t)\rightarrow0.$ 所得关节加速度 $\ddot q(t)$ 生成速度轨迹
$$\dot q(t) = \dot q(0) + \int_0^t \ddot q(\tau),d\tau,$$

以及位置轨迹
$$q(t) =q(0) +\int_0^t \dot q(\tau),d\tau.$$

因此，ARE 解生成完整轨迹
$$\mathcal T = \{q_d(t),\dot q_d(t),\ddot q_d(t)\}$$

并以最优方式收敛到期望构型。

## A.5 可行性检查

ARE 解表示双积分器规划问题的无约束最优解。因此，生成的轨迹可作为候选运动计划。

若轨迹满足所有规划约束，

$$q_{\min} \le q_d(t) \le q_{\max},$$

$$|\dot q_d(t)| \le \dot q_{\max},$$

$$|\ddot q_d(t)| \le \ddot q_{\max},$$

以及所有工作空间障碍约束，则 ARE 轨迹本身构成有效运动计划。

若任一约束被违反，无约束解对无约束问题仍然最优，但对运动规划而言不再可行。此时调用正文提出的有约束预测式规划器。由此，MPC/QP formulation 可视为无约束 LQR/ARE 解的有限时域有约束对应物：它在同一双积分器骨架上运行，同时显式施加运动学与环境约束。
