# Cover Letter — IEEE Transactions on Robotics (T-RO)

**Date:** June 29, 2026
**To:** Editor-in-Chief, IEEE Transactions on Robotics
**Manuscript:** Reactive Time-Domain Motion Planning via a Configuration-Independent Double-Integrator Backbone
**Submission type:** Regular Paper
**Author:** Yongyan Cao (Voryx Robotics LLC, San Jose, CA, USA)

---

Dear Editor,

We submit the enclosed manuscript, *"Reactive Time-Domain Motion Planning via a Configuration-Independent Double-Integrator Backbone,"* for consideration as a Regular Paper in the IEEE Transactions on Robotics.

**Problem and contribution.** Most motion-planning pipelines separate geometric path generation from time parameterization, which makes reactive updates costly: when an obstacle moves or the task objective changes, a path-first pipeline must re-search and re-time from scratch. We present a local predictive motion planner formulated directly in the time domain. The contribution is architectural rather than a property of the double integrator itself: by planning on a *virtual* double-integrator backbone instead of a plant linearization, the state-transition matrix $A_d$ stays constant across all robot configurations, whereas a configuration-dependent MPC must rebuild its prediction matrices every cycle. All horizon prediction matrices are therefore precomputed once offline, and online planning reduces to a convex Quadratic Program with fixed dynamics structure in which only the current state, reference terms, and active constraint rows change. In effect, the geometric path-to-time ($s\rightarrow t$) interface is replaced by a single fixed-structure time-domain QP.

**Why a full-length treatment, and why T-RO.** The architecture warrants the comprehensive development a T-RO paper allows. The manuscript contributes: (i) a *configuration-independent* time-domain parameterization whose prediction structure is fixed and grows only with horizon, dimension, and active constraint rows—not with the robot configuration; (ii) direct optimization of time-indexed joint positions, velocities, and accelerations on a fixed grid, with velocity, acceleration, and linearized workspace-obstacle constraints in a single QP, eliminating the separate spatial-to-temporal stage and its stop-and-rebuild behavior, with $C^1$ output under hard-bounded piecewise-constant acceleration and a piecewise-jerk triple-integrator extension when continuous acceleration is required; and (iii) two complementary local obstacle-avoidance mechanisms (linearized safe-corridor rows and artificial-potential-field reference deflection), evaluated on Franka FR3 manipulation benchmarks against a path-first TOTG pipeline, with an autonomous-vehicle steering example demonstrating the same backbone applied to steering-rate smoothness. This breadth—architecture, formulation, two avoidance strategies, and cross-domain (manipulator and vehicle) evaluation—is well suited to the in-depth scope of the Transactions.

We report timing transparently: in a single-thread Python harness, decoupled box-constrained QPs solve well within a 100 Hz budget and coupled obstacle rows meet it at p95 with fixed-sparsity updates, while worst-case cycles can exceed 10 ms. We present these as diagnostics of the fixed-structure advantage rather than a hard real-time guarantee, and we delineate the planner's scope as a *local* predictive planner that relies on a downstream controller and local avoidance, not a global search method.

**Originality and disclosure.** This manuscript is original, has not been published previously, and is not under review at any other journal or conference. For transparency we disclose a related, non-overlapping manuscript by the author—an impedance-MPC architecture for physical human–robot interaction (cited as `[phri]`, arXiv:2606.08281, 2026)—which inspired the constant double-integrator residual used here as a *planning backbone*; the present paper concerns motion planning and does not duplicate that control-layer contribution. The author declares no conflicts of interest.

We thank you and the reviewers for their time and consideration, and we would be glad to suggest qualified reviewers or an appropriate Associate Editor upon request.

Sincerely,

Yongyan Cao
Voryx Robotics LLC, San Jose, CA, USA
yongyancao@gmail.com
