# Equation Reference and Verification

Complete listing of every equation used in `motion_planning_double_integrator.md`
and in the benchmark harness, cross-referenced to the code and to the numerical
checks in `verify_math.py`. Status column: **PASS** = verified to the residual shown
by `python3 verify_math.py` (14/14 pass).

Notation: $\Delta t$ time step, $N$ horizon, $n$ axes (joints/DOF), $q$ position,
$\dot q$ velocity, $\ddot q$ acceleration, $u$ control.

---

## 1. Backbone dynamics

### 1.1 Double integrator (manipulator; paper §II.A)

Continuous, per axis:
$$\dot x = A_c x + B_c u,\quad
x=\begin{bmatrix}q\\\dot q\end{bmatrix},\;
A_c=\begin{bmatrix}0&1\\0&0\end{bmatrix},\;
B_c=\begin{bmatrix}0\\1\end{bmatrix},\;
u=\ddot q. \tag{1}$$

$A_c$ is nilpotent ($A_c^2=0$), so the matrix exponential truncates and the **exact
ZOH** discretization is (paper §II.B):
$$e^{A_c\Delta t}=I+A_c\Delta t,\qquad
A_d=\begin{bmatrix}1&\Delta t\\0&1\end{bmatrix},\qquad
B_d=\int_0^{\Delta t}\! e^{A_c\tau}B_c\,d\tau=\begin{bmatrix}\tfrac{\Delta t^2}{2}\\[2pt]\Delta t\end{bmatrix}. \tag{2}$$
*Code:* `di_matrices`, step update in `plan_di`. *Check:* **E1** (residual $1.7\times10^{-18}$).

### 1.2 Triple integrator (AV lateral channel; `di_lateral.py`)

$$z=\begin{bmatrix}y\\\dot y\\\ddot y\end{bmatrix},\;
A_c=\begin{bmatrix}0&1&0\\0&0&1\\0&0&0\end{bmatrix},\;
B_c=\begin{bmatrix}0\\0\\1\end{bmatrix},\; u=\dddot y\ (\text{jerk}),\quad A_c^3=0. \tag{3}$$
$$A_d=\begin{bmatrix}1&\Delta t&\tfrac{\Delta t^2}{2}\\0&1&\Delta t\\0&0&1\end{bmatrix},\qquad
B_d=\begin{bmatrix}\tfrac{\Delta t^3}{6}\\[2pt]\tfrac{\Delta t^2}{2}\\[2pt]\Delta t\end{bmatrix}. \tag{4}$$
*Code:* `_tri_matrices`, step in `plan_lateral_jerk`. *Check:* **E2** ($4.6\times10^{-16}$).

---

## 2. Finite-horizon prediction

State rollout: $x_k=A_d^{\,k}x_0+\sum_{j=0}^{k-1}A_d^{\,k-1-j}B_d\,u_j$, stacked over
$k=1..N$ (paper §II.B):
$$X=\Phi\,x_0+\Gamma\,U,\qquad
\Phi=\begin{bmatrix}A_d\\A_d^2\\\vdots\\A_d^N\end{bmatrix},\quad
\Gamma=\begin{bmatrix}B_d&&\\A_dB_d&B_d&\\\vdots&&\ddots\\A_d^{N-1}B_d&\cdots&B_d\end{bmatrix}. \tag{5}$$

### 2.1 Double-integrator entries (split into position/velocity rows)

With $m=k-1-j$:
$$\Phi_p[k]=\begin{bmatrix}1&k\Delta t\end{bmatrix},\quad
\Phi_v[k]=\begin{bmatrix}0&1\end{bmatrix},\quad
\Gamma_p[k,j]=\Delta t^2\!\left(m+\tfrac12\right),\quad
\Gamma_v[k,j]=\Delta t. \tag{6}$$
*Code:* `di_matrices`. *Check:* **E3** ($4.2\times10^{-17}$, vs brute-force rollout).

### 2.2 Triple-integrator entries

$$\Gamma_p[k,j]=\Delta t^3\!\left(\tfrac16+\tfrac{m}{2}+\tfrac{m^2}{2}\right),\quad
\Gamma_v[k,j]=\Delta t^2\!\left(m+\tfrac12\right),\quad
\Gamma_a[k,j]=\Delta t, \tag{7}$$
$$\Phi_p[k]=\begin{bmatrix}1&k\Delta t&\tfrac{(k\Delta t)^2}{2}\end{bmatrix}. \tag{8}$$
*Code:* `_tri_matrices`. *Check:* **E4** ($2.2\times10^{-16}$).

---

## 3. Receding-horizon QP (paper §III)

Decision variable $U=[u_0,\dots,u_{N-1}]$. Objective (paper §III.B):
$$\min_U\ \tfrac12 U^\top H U + h^\top U,\qquad
H=\Gamma^\top\bar Q\,\Gamma+\bar R,\quad
h=\Gamma^\top\bar Q\,x_{\text{free,err}}, \tag{9}$$
$$x_{\text{free,err}}=\Phi x_0-\mathbf 1_N\!\otimes x_{\text{goal}}. \tag{10}$$

Implemented per axis with explicit position/velocity weights $W_q,W_v$ (terminal
multiplier $\gamma$ on the last step) and control weight $w_r$:
$$H=\Gamma_p^\top W_q\Gamma_p+\Gamma_v^\top W_v\Gamma_v+w_rI, \tag{11}$$
$$h=\Gamma_p^\top W_q(\Phi_p x_0-q_g\mathbf1)+\Gamma_v^\top W_v(\Phi_v x_0-v_g\mathbf1). \tag{12}$$
This minimizes the explicit weighted least-squares cost
$J(U)=\sum_k W_{q,k}(q_k-q_g)^2+W_{v,k}(\dot q_k-v_g)^2+w_r\sum_k u_k^2$ (the OSQP
$\tfrac12 H$ convention scales $H,h$ by $\tfrac12$ vs $J$; the minimizer is
identical). $H\succ0$ since $w_r>0$ ⇒ strictly convex.
*Code:* `JointQP`. *Check:* **E5** ($\nabla J = 2HU+2h$ matches, $4.4\times10^{-16}$).

### 3.1 Hard kinematic constraints (paper §III.C)

For $k=1..N$ (velocity) and $k=0..N-1$ (acceleration), affine in $U$:
$$-\dot q_{\max}-\Phi_v x_0\le \Gamma_v U\le \dot q_{\max}-\Phi_v x_0, \tag{13}$$
$$-u_{\max}\le U\le u_{\max}, \tag{14}$$
optional $\;-q_{\max}-\Phi_p x_0\le\Gamma_p U\le q_{\max}-\Phi_p x_0$. *Code:* `JointQP.solve`.

### 3.2 Triple-integrator (AV) constraints (`di_lateral.py`)

$$|\ddot y|\le a_{\text{lat,max}}\ \text{(lateral accel)},\quad
|\dddot y|\le j_{\max}\ \text{(lateral jerk)},\quad |\dot y|\le \dot y_{\max}. \tag{15}$$

### 3.3 Exact state advance (ZOH, one step)

$$q_{k+1}=q_k+\Delta t\,\dot q_k+\tfrac12\Delta t^2u_k,\qquad
\dot q_{k+1}=\dot q_k+\Delta t\,u_k, \tag{16}$$
and the triple-integrator analogue
$y_{k+1}=y_k+\Delta t\dot y_k+\tfrac12\Delta t^2\ddot y_k+\tfrac{\Delta t^3}{6}\dddot y_k$, etc.
*Checks:* **E10** (double, residual $0$), **E11** (triple, residual $0$).

---

## 4. Obstacle constraints (paper §IV; not exercised by the planning benchmarks)

Linearized polytope (hard, §IV.A): with translational Jacobian $J_v(q_0)$ and
$p(k)\approx p_0+J_v(q_0)\Delta q(k)$,
$$A_{\text{obs}}J_v(q_0)\,\Delta q(k)\le b_{\text{obs}}-A_{\text{obs}}p_0. \tag{17}$$
APF target deflection (soft, §IV.B):
$$U_{\text{rep}}(p)=\tfrac12\eta\!\left(\tfrac{1}{\|p-p_{\text{obs}}\|}-\tfrac{1}{\rho_0}\right)^2,\;
\|p-p_{\text{obs}}\|\le\rho_0;\quad
\delta q_{\text{obs}}=J_v^{+}(q)\,F_{\text{rep}},\ F_{\text{rep}}=-\nabla U_{\text{rep}}. \tag{18}$$

---

## 5. Path-first reference (TOTG / TOPP; `di_totg.py`)

### 5.1 Circular-blend geometry (Kunz–Stilman [10])

For interior waypoint $W_i$ with unit leg directions $u_1,u_2$:
$$\varphi=\arccos(u_1\!\cdot u_2),\quad
r=\frac{\delta_{\max}}{\sec(\varphi/2)-1},\quad
\ell=r\tan(\varphi/2), \tag{19}$$
($\ell$ capped to $\le\tfrac12\min(L_1,L_2)$, then $r=\ell/\tan(\varphi/2)$). Tangent
points and arc center:
$$P_s=W_i-\ell u_1,\quad P_e=W_i+\ell u_2,\quad
C=W_i+r\sec(\varphi/2)\,\frac{u_2-u_1}{\|u_2-u_1\|}. \tag{20}$$
Max deviation from the corner: $\delta=r(\sec(\varphi/2)-1)$. *Check:* **E8** (closest
approach $=0.100=\delta_{\max}$).

### 5.2 Arc-length parameterization

With orthonormal $e_1=(P_s-C)/r$, $e_2\perp e_1$ toward $P_e$, and arc angle
$a=(s-s_0)/r\in[0,\varphi]$:
$$q(s)=C+r\big(\cos a\,e_1+\sin a\,e_2\big),\quad
q'(s)=-\sin a\,e_1+\cos a\,e_2,\quad
q''(s)=-\tfrac1r(\cos a\,e_1+\sin a\,e_2). \tag{21}$$
On straight segments $q(s)=P_0+(s-s_0)d$, $q'=d$, $q''=0$. Arc length $L_{\text{arc}}=r\varphi$.
Thus $\|q'(s)\|=1$ everywhere (arc-length), and $\|q''(s)\|=0$ on lines, $1/r$ on
arcs — **discontinuous at seams**. *Checks:* **E6** ($\|q'\|=1$, $2.2\times10^{-16}$),
**E7** ($C^1$ tangent + curvature jump $0\!\to\!1/r$).

### 5.3 Velocity/acceleration along the path

$$\dot q=q'(s)\,\dot s,\qquad
\ddot q=q'(s)\,\ddot s+q''(s)\,\dot s^2. \tag{22}$$
The second (centripetal) term carries the $q''$ discontinuity and scales with $\dot s^2$.

### 5.4 Time-optimal parameterization (numerical-integration TOPP)

Let $x=\dot s^2$. Maximum-velocity curve from velocity limits:
$$\dot s_{\max}(s)=\min_j\frac{\dot q_{\max,j}}{|q'_j(s)|},\qquad x_{\text{vel}}=\dot s_{\max}^2. \tag{23}$$
Acceleration limits give, per axis, an admissible $\ddot s$ interval (affine in $x$):
$$-a_{\max,j}\le q'_j(s)\,\ddot s+q''_j(s)\,x\le a_{\max,j}
\;\Rightarrow\;\ddot s\in[L(s,x),\,U(s,x)], \tag{24}$$
and the accel-MVC is the largest $x$ with $L\le U$ (found by bisection). Forward/
backward sweep with $x'=\mathrm dx/\mathrm ds=2\ddot s$:
$$x^{f}_{k+1}=\min\!\big(x^f_k+2U\Delta s,\;\text{MVC}_{k+1}\big),\quad
x^{b}_{k-1}=\min\!\big(x^b_k-2L\Delta s,\;\text{MVC}_{k-1}\big), \tag{25}$$
$$x=\min(x^f,\,x^b,\,\text{MVC}). \tag{26}$$
*Code:* `topp`, `_u_interval`. *Checks:* **E9** (path-domain $|\dot q|\le\dot q_{\max}$, residual $0$), **E9b** (admissible $\ddot s$ interval exists at every $s$).

### 5.5 The $s\!\to\!t$ conversion (where it breaks)

$$\ddot s=\tfrac12\frac{\mathrm dx}{\mathrm ds},\qquad
\Delta t_k=\frac{2\Delta s}{\dot s_k+\dot s_{k+1}},\qquad
t(s)=\sum_k\Delta t_k. \tag{27}$$
$$\boxed{\,t(s)=\int\frac{\mathrm ds}{\dot s}\ \text{ is singular as }\dot s\to0\,}\quad\Rightarrow\quad
\text{near-reversal }(r\to0)\ \Rightarrow\ T\to\infty\ (\text{B3 STALL}). \tag{28}$$
Reconstruction at uniform time: $s(t)=\text{interp}$, then $q=q(s(t))$ and $\ddot q$
from (22) — the $q''$ jump makes $\ddot q$ discontinuous and can exceed $a_{\max}$.
*Code:* `plan_totg`. *Check:* **E13** ($\ddot s=\tfrac12\,\mathrm dx/\mathrm ds$, $6\times10^{-5}$).

---

## 6. Autonomous-vehicle kinematics (steering; `benchmark_av.py`)

Kinematic bicycle, wheelbase $L$, planar velocity $v=(\dot x,\dot y)$, accel
$(\ddot x,\ddot y)$:
$$\text{speed}=\|v\|,\quad
\kappa=\frac{\dot x\ddot y-\dot y\ddot x}{\|v\|^3},\quad
\delta=\operatorname{atan}(L\kappa),\quad
a_{\text{lat}}=\frac{\dot x\ddot y-\dot y\ddot x}{\|v\|}=\|v\|^2\kappa. \tag{29}$$
Steering rate $\dot\delta=\mathrm d\delta/\mathrm dt$. *Code:* `steering`. *Check:*
**E12** (constant-curvature circle: $\kappa=1/R$, $\delta=\operatorname{atan}(L/R)$, residual $5.6\times10^{-17}$).

### 6.1 Integrator-order principle (AV)

At cruise speed $V$ ($x=Vt$, $\dot y\ll V$): $\kappa\approx\ddot y/V^2$, so
$$\delta\approx\operatorname{atan}\!\big(L\ddot y/V^2\big),\qquad
\dot\delta\approx \frac{L\,\dddot y}{V^2}\ \text{(small angle)}. \tag{30}$$
Bounding lateral jerk $|\dddot y|\le j_{\max}$ (15) therefore bounds steering rate by
construction — motivating the **triple** integrator (one order above the
manipulator's double integrator).

---

## 7. Benchmark metrics

$$\text{accel ratio}=\frac{\|\ddot q\|_\infty}{\ddot q_{\max}}\ (>1\Rightarrow\text{limit broken}),\quad
\text{jerk}_k=\frac{\ddot q_{k+1}-\ddot q_k}{\Delta t}, \tag{31}$$
$$\text{accel jump}=\max_k\|\ddot q_{k+1}-\ddot q_k\|_\infty,\quad
\text{conditioning}=\max_s\frac{1}{\dot s(s)}\ \text{(TOTG only)}. \tag{32}$$

---

## 8. Verification summary

`python3 verify_math.py` → **14/14 PASS**:

| Check | Equation(s) | Residual |
|---|---|---|
| E1 | (2) double-int ZOH | 1.7e-18 |
| E2 | (4) triple-int ZOH | 4.6e-16 |
| E3 | (6) DI prediction $\Phi,\Gamma$ | 4.2e-17 |
| E4 | (7,8) triple-int prediction | 2.2e-16 |
| E5 | (9–12) QP cost algebra | 4.4e-16 |
| E6 | (21) endpoints + $\|q'\|=1$ | 2.2e-16 |
| E7 | (21) $C^1$ + curvature jump | 1.4e-05 |
| E8 | (19,20) blend deviation $\le\delta_{\max}$ | 0.10 (=cap) |
| E9 / E9b | (23–26) TOPP feasibility | 0 / 0 |
| E10 | (16) DI time consistency | 0 |
| E11 | (16) triple-int time consistency | 0 |
| E12 | (29) steering formula | 5.6e-17 |
| E13 | (27) $\ddot s=\tfrac12\,\mathrm dx/\mathrm ds$ | 6.0e-05 |

Non-machine-precision residuals (E7, E8, E13) are finite-difference/grid artifacts
of the *check itself*, not of the underlying equation, and are within the stated
tolerances.
