## Title
Energy-Efficient Stable Cooperative Grasping of a Heavy Object by Multiple Robotic Manipulators

## Technical field
This disclosure relates to robotic manipulation, multi-arm cooperative grasping, force/torque control, optimal control, and real-time constrained optimization for stable and energy-efficient object handling.

## Background
Cooperative manipulation of heavy objects using multiple robotic manipulators is constrained by limited actuator power, uncertainty in object mass/inertia and contact parameters, frictional slip, and the need to maintain stable grasp (wrench closure) under disturbances. Conventional approaches often prioritize either stability (large safety margins and high normal forces) or tracking (high stiffness), which can increase energy usage and lead to saturations and heat, or degrade robustness. There is a need for architectures that explicitly optimize stability and energy usage under contact and actuator constraints, while ensuring closed-loop safety and passivity.

## Summary
Disclosed is a system and method for cooperative grasping and manipulation of a heavy object by \(N\) robotic manipulators. The system computes, in real time, contact force distributions and joint torques that (i) satisfy equilibrium and friction constraints, (ii) preserve stability margins against slip and loss of contact, (iii) regulate internal forces, and (iv) minimize energy usage. The disclosed architecture combines:

- a perception/estimation layer that estimates object pose, object wrench, and contact states;
- a neural grasp proposal layer that generates candidate grasps for each manipulator using one or more trained neural networks;
- a grasp/stability layer that maintains and maximizes an explicit grasp stability margin;
- an energy estimation layer that estimates energy consumption of candidate grasps and planned motions;
- an optimal cooperative force distribution layer formulated as a constrained optimization (typically a convex QP or SOCP) over contact forces and/or joint torques;
- a passivity-aware execution layer (e.g., impedance/admittance control with an energy-tank or passivity observer/controller) that preserves stability despite model mismatch and delays.

In one embodiment, a neural network generates multiple grasp candidates per manipulator, each candidate is evaluated for stability margin and energy, and poorly scoring candidates are replaced by regenerated candidates until a cooperative set meets thresholds. A constrained optimizer then uses the resulting grasp matrix \(G\), manipulator Jacobians \(J_i\), and friction cone constraints to compute a minimum-energy feasible force distribution that maintains a target grasp wrench and internal force regulation. In another embodiment, the method uses a multi-rate architecture where a high-level grasp selection loop runs at lower rate and a low-level passivity-stabilized controller runs at higher rate.

## Brief description of drawings (conceptual)
The disclosure contemplates block diagrams illustrating:

- multi-arm sensing and object state estimation;
- neural grasp candidate generation for each manipulator;
- stability margin computation (wrench closure, slip margin);
- energy estimation and candidate regeneration;
- constrained optimal force distribution (QP/SOCP);
- execution with impedance control and passivity enforcement;
- monitoring and safe fallback (regrasp, reduce motion, increase friction margin).

## Definitions (selected)
- **Manipulator**: A robot arm with joints and an end-effector capable of contact forces at one or multiple contact points.
- **Contact wrench/force**: Wrench applied at a contact point, represented as force (and possibly torque) in a local contact frame.
- **Grasp matrix \(G\)**: Linear map from stacked contact wrenches \(f\) to net object wrench \(w\): \(w = G f\).
- **Internal forces**: Contact forces that produce zero net object wrench (lie in the null space of \(G\)).
- **Stability margin**: A quantitative margin to slip and/or loss of wrench closure (e.g., minimum distance to friction cone boundary, wrench-closure quality metric, or disturbance wrench margin).
- **Energy**: Electrical or mechanical energy proxies such as squared joint torques, instantaneous power, or integral of power.

## Detailed description

### System overview
An exemplary system includes:

- \(N \ge 2\) robotic manipulators each with joint sensors (position, velocity, torque) and optionally joint torque control capability.
- End-effector force/torque sensors and/or tactile sensors to estimate contact forces and detect slip.
- One or more exteroceptive sensors (vision, depth, motion capture) and/or proprioceptive estimation to determine object pose and motion.
- A real-time compute unit executing neural grasp generation, cooperative optimization, and control laws.

In one embodiment, the compute unit includes a learned grasp generation model configured to output grasp candidates conditioned on an object representation (e.g., point cloud, mesh, or signed-distance field), manipulator kinematic limits, and task context (e.g., lift, carry, place).

### Modeling primitives
Let the object pose and twist be \((x_o, v_o)\), and let the desired object wrench to achieve a task (hold, lift, move) be \(w_d\). For each manipulator \(i \in \{1,\dots,N\}\), the end-effector contact kinematics are described by Jacobian \(J_i(q_i)\), and contact forces (stacked) \(f_i\) mapped to joint torques \(\tau_i\) via:

\[
\tau_i \approx J_i(q_i)^\top f_i + \tau_{i,\text{bias}}
\]

where \(\tau_{i,\text{bias}}\) may include gravity compensation and known biases.

The object wrench induced by all contacts is:

\[
w = G f,\quad f = [f_1^\top\; f_2^\top\;\dots\; f_N^\top]^\top.
\]

External wrench disturbances (gravity, inertia, unknown interactions) are denoted \(w_\text{ext}\).

### Neural grasp proposal and regeneration (example embodiment)
For each manipulator \(i\), a neural network generates \(K\) candidate grasps \(\mathcal{G}_i = \{g_{i,1},\dots,g_{i,K}\}\). Each grasp candidate may comprise:

- an end-effector pose relative to the object,
- one or more anticipated contact points/patches and local contact frames,
- an approach direction and finger configuration,
- an optional predicted friction coefficient and contact compliance.

The system evaluates candidate grasp sets \(\mathbf{g} = (g_1,\dots,g_N)\) using a stability estimator and an energy estimator. If a grasp set fails to meet stability or energy criteria, the system regenerates candidates by re-sampling from the neural network, conditioning on failure modes (e.g., predicted slip at a contact, excessive torque) and excluding previously rejected regions on the object. The regeneration may be repeated until a feasible set is found or a timeout occurs, in which case a fallback strategy is used (e.g., reduce motion or request human intervention).

### Stability constraints and margins
Each contact force \(f_{i,k}\) (for contact \(k\) on manipulator \(i\)) must satisfy friction constraints. A common polyhedral friction cone approximation yields:

\[
A_{i,k} f_{i,k} \le b_{i,k},
\]

including a non-negativity constraint on normal force, and tangential components bounded by \(\mu\) times normal force. Additional constraints may include:

- normal force lower bound for contact maintenance: \(f_n \ge f_{n,\min}\),
- upper bound to avoid damage: \(f_n \le f_{n,\max}\),
- center-of-pressure limits for soft fingertips,
- torque limits if torsional friction is modeled.

The disclosed method computes and tracks an explicit stability margin \(m\), e.g.:

- **slip margin**: minimum distance of each contact force to the friction cone boundary;
- **disturbance wrench margin**: largest admissible disturbance wrench \(\Delta w\) such that \(w+\Delta w\) remains realizable within contact constraints;
- **wrench closure metric**: quality metric \(Q(G,\mathcal{C})\) for grasp cone \(\mathcal{C}\).

The controller enforces \(m \ge m_\text{min}\) and optionally maximizes \(m\) subject to energy objectives.

### Energy objective
Energy efficiency is achieved by minimizing one or more proxies:

- squared joint torques: \(\sum_i \|\tau_i\|_{W_{\tau,i}}^2\),
- squared contact forces (regularization): \(\|f\|_{W_f}^2\),
- instantaneous mechanical power: \(\sum_i \|\tau_i \odot \dot q_i\|_{W_P}^2\) or \(\sum_i \tau_i^\top \dot q_i\) subject to sign-safe constraints,
- force/torque rate penalties to reduce chatter: \(\|\Delta f\|^2\), \(\|\Delta\tau\|^2\).

In one embodiment, an energy estimator evaluates candidate grasps by predicting expected joint torques and/or power along a planned cooperative motion under gravitational loading and estimated object inertia.

### Constrained optimization (example embodiment)
At each control cycle, solve:

\[
\begin{aligned}
\min_{f,\,s}\quad & \|J^\top f\|_{W_\tau}^2 + \lambda_f \|f\|_{W_f}^2 + \lambda_{\Delta}\|f - f^{\text{prev}}\|^2 + \lambda_s \|s\|^2 \\
\text{s.t.}\quad & G f + w_\text{ext} = w_d + s \\
& A f \le b \\
& f_{n} \in [f_{n,\min},\,f_{n,\max}] \\
& \tau = J^\top f + \tau_\text{bias},\quad \tau \in [\tau_{\min},\,\tau_{\max}] \\
& m(f) \ge m_\text{min}
\end{aligned}
\]

where \(J = \mathrm{blkdiag}(J_1,\dots,J_N)\), slack \(s\) allows graceful degradation, and \(m(f)\) is implemented via linear or convex constraints consistent with the chosen margin definition (for example, by enforcing a minimum distance to polyhedral friction cone facets).

In another embodiment, internal forces are explicitly parameterized. Let \(N_G\) span the nullspace of \(G\). Write:

\[
f = f_p + N_G \alpha
\]

where \(f_p\) is a particular solution matching the net wrench, and \(\alpha\) are internal force coefficients chosen to maximize margin and minimize energy.

In another embodiment, a two-stage optimization is performed:

- **Outer grasp selection**: select a cooperative set of grasps \(\mathbf{g}\) from neural candidates to maximize stability and minimize predicted energy subject to kinematic reachability and collision constraints.
- **Inner force/torque optimization**: given \(\mathbf{g}\) (hence \(G\) and contact frames), solve a convex program to compute forces/torques satisfying equilibrium and friction constraints while minimizing energy.

### Passivity-aware execution and robustness
To ensure stability under uncertainty and delays, the disclosed architecture executes the optimized references using impedance/admittance control and passivity enforcement:

- compute desired end-effector wrench \(f^*\) and object wrench \(w^*\),
- track them with an impedance controller \(F = K(x-x_d)+D(v-v_d)\) blended with \(f^*\),
- maintain a passivity observer/controller or energy-tank that limits commanded power when the estimated dissipated energy budget is exceeded.

This structure prevents destabilizing energy injection when contact models are inaccurate or when communications are delayed.

### Monitoring and fallback
The system continuously monitors constraint margins and a set of safety indicators:

- minimum slip margin across contacts,
- actuator saturation proximity,
- object acceleration deviations from expected,
- contact loss indicators (tactile/force discontinuities),
- solver health (feasibility, slack usage).

Upon degraded conditions, the system reduces object motion, increases margin via force redistribution, changes grasp mode, or triggers regrasp.

## Example method steps (high-level)
1. Estimate object state and external wrenches from sensors and models.
2. Generate candidate grasps for each manipulator using one or more neural networks.
3. Estimate stability margins and energy consumption for candidate grasps and candidate cooperative grasp sets.
4. Regenerate one or more grasps if stability and/or energy criteria are not met.
5. Compute/estimate contact frames, friction parameters, and current stability margins for the selected cooperative grasp set.
6. Formulate a constrained optimization to compute contact forces and/or joint torques minimizing energy while maintaining stability.
7. Solve the optimization in real time and generate reference wrenches/forces.
8. Execute references via passivity-aware impedance/admittance control.
9. Monitor margins; adapt weights/constraints; fallback on constraint violations.

## Claims (non-limiting, draft)
1. **A method** for cooperative grasping of an object by a plurality of robotic manipulators, comprising: generating candidate grasps for each manipulator using one or more trained neural networks; estimating an object state and contact states; evaluating candidate grasps for a stability margin and an energy estimate; regenerating one or more grasps based on the evaluating; solving, at each control cycle, a constrained optimization that minimizes an energy proxy subject to equilibrium and friction constraints and a minimum stability margin; and commanding the manipulators using a passivity-aware controller to track a solution of the constrained optimization.

2. **The method** of claim 1, wherein the constrained optimization is a convex quadratic program over contact forces with constraints comprising polyhedral friction cones and actuator torque limits.

3. **The method** of claim 1, wherein the optimization includes explicit internal force variables spanning the nullspace of a grasp matrix, and the internal forces are selected to increase the stability margin while minimizing energy.

4. **The method** of claim 1, wherein the energy proxy includes a weighted sum of squared joint torques and a penalty on changes in contact forces between cycles.

5. **The method** of claim 1, wherein a passivity observer/controller or energy-tank limits commanded power to ensure stability under contact uncertainty and communication delays.

6. **The method** of claim 1, wherein the grasp stability margin is defined as a minimum distance to the boundary of a friction cone for each contact force.

7. **A system** comprising a plurality of robotic manipulators, sensors configured to estimate object state and contact forces, and one or more processors configured to execute the method of any of claims 1–6.

8. **The system** of claim 7, wherein the processors operate in a multi-rate architecture with a slower optimization loop and a faster passivity-stabilized tracking loop.

## Notes
This document is a technical draft to support a patent-style disclosure. Jurisdiction-specific legal formatting, inventor lists, and prior art review are not included.

