## Cooperative grasping pipeline (NN grasp generation → stability/energy estimation → regeneration → optimal cooperative grasp)

This document describes a practical architecture to achieve **stable** and **energy-efficient** cooperative grasping of a heavy object by \(N\) robotic manipulators using:

1) **neural grasp generation** per hand,  
2) **stability and energy estimation** for candidate cooperative grasps,  
3) **regeneration** of poor candidates, and  
4) a final **optimization problem** that enforces stability constraints while minimizing energy.

---

## Inputs / outputs

### Inputs
- **Object representation**: point cloud, mesh, SDF, or multi-view depth.
- **Task context**: hold/lift/carry/rotate; desired object trajectory \(x_o(t)\) or desired wrench \(w_d(t)\).
- **Robot models**: kinematics, joint limits, torque limits, end-effector geometry.
- **Contact model**: friction coefficients \(\mu\) (estimated), compliance, allowable normal force ranges.
- **Safety settings**: minimum stability margin \(m_\text{min}\), max allowed power/torque, timeouts.

### Outputs
- **Selected cooperative grasp**: per-hand grasp parameters \(g_i\) (pose, finger config, intended contact frames/patches).
- **Real-time commands**: per-arm joint torques \(\tau_i\) and/or end-effector wrench targets \(f_i\).
- **Monitoring signals**: slip margin, saturation margin, feasibility/slack, passivity/energy tank state.

---

## Architecture (block diagram)

1. **Perception & state estimation**
   - Estimate object pose/twist \((x_o, v_o)\), optionally mass/inertia.
   - Estimate external wrenches \(w_\text{ext}\) (gravity + inertial + disturbances).
   - Detect contact/slip from F/T + tactile (optional).

2. **Neural grasp proposal (per hand)**
   - For each hand \(i\), generate \(K\) candidates \(\mathcal{G}_i = \{g_{i,1},\dots,g_{i,K}\}\).
   - Candidates can include: grasp pose relative to object, approach direction, finger configuration, predicted contact patches/frames, predicted friction/compliance.

3. **Candidate set formation (cooperative)**
   - Form cooperative sets \(\mathbf{g} = (g_1,\dots,g_N)\) by combining per-hand candidates.
   - Apply hard filters early: reachability, collision, self-collision, object occlusion, minimum separation, feasible approach paths.

4. **Stability estimation**
   - For each \(\mathbf{g}\), compute/estimate:
     - grasp matrix \(G(\mathbf{g})\),
     - friction constraints \(\mathcal{C}(\mathbf{g})\),
     - stability margin \(m(\mathbf{g})\) (e.g., slip margin or disturbance wrench margin).

5. **Energy estimation**
   - Predict energy/power/torque usage for holding and/or planned motion, e.g.:
     - \(\sum_i \|\tau_i\|^2\) under gravity load,
     - predicted mechanical work \(\int \sum_i \tau_i^\top \dot q_i \, dt\),
     - rate/jerk penalties if relevant.

6. **Regeneration loop**
   - If top candidates do not satisfy thresholds \(m(\mathbf{g}) \ge m_\text{min}\) and \(E(\mathbf{g}) \le E_\text{max}\):
     - regenerate one or more hands’ candidates conditioned on failure (slip-prone contacts, torque-limited arms, collisions),
     - bias proposals away from rejected regions,
     - repeat until feasible or timeout.

7. **Final optimization (stable + minimal energy cooperative grasp)**
   - Choose a cooperative grasp \(\mathbf{g}^\*\) and compute real-time force/torque commands via an inner convex optimization (QP/SOCP).

8. **Execution + passivity & safety**
   - Track wrench/torque references with impedance/admittance control.
   - Enforce passivity/energy tank limits to avoid destabilizing energy injection.
   - If constraints degrade: slow motion, redistribute forces, regrasp.

---

## Stability and energy metrics (typical choices)

### Stability metrics
- **Slip margin (polyhedral friction cone)**: for each contact, distance to the closest friction facet; overall margin is min across contacts.
- **Disturbance wrench margin**: max admissible \(\Delta w\) such that there exists feasible contact forces satisfying friction and actuator limits.
- **Wrench closure quality**: grasp quality metrics based on convex hull of achievable wrenches.

### Energy metrics
- **Torque-squared** (electrical/mechanical proxy): \(\sum_i \|\tau_i\|_{W_{\tau,i}}^2\).
- **Force regularization**: \(\|f\|_{W_f}^2\) to avoid excessive normal force.
- **Power proxy**: \(\sum_i \|\tau_i \odot \dot q_i\|^2\) or bounded \(\tau^\top \dot q\).
- **Chatter reduction**: \(\|f - f^\text{prev}\|^2\) or \(\|\tau-\tau^\text{prev}\|^2\).

---

## Optimization problems

### A) Outer problem (select / regenerate grasps)

Let \(g_i\) denote the grasp parameters for hand \(i\). Let \(\mathbf{g}=(g_1,\dots,g_N)\). The outer loop can be posed as:

\[
\begin{aligned}
\min_{\mathbf{g} \in \mathcal{G}_1 \times \dots \times \mathcal{G}_N}\quad
& \lambda_E \, \widehat{E}(\mathbf{g}) - \lambda_m \, \widehat{m}(\mathbf{g}) + \lambda_c \, \text{Cost}_\text{collision}(\mathbf{g}) \\
\text{s.t.}\quad
& \text{Reachable}_i(g_i)=\text{true}\ \forall i, \\
& \text{ApproachFeasible}_i(g_i)=\text{true}\ \forall i, \\
& \widehat{m}(\mathbf{g}) \ge m_\text{min}.
\end{aligned}
\]

Notes:
- \(\widehat{m}\) and \(\widehat{E}\) may be fast analytic estimates, learned predictors, or short-horizon simulation results.
- Because \(\mathbf{g}\) is discrete (pick from candidates), solve by scoring + beam search, cross-entropy method (CEM), or iterative regeneration.

### B) Inner problem (stable, minimal-energy cooperative force/torque distribution)

Given a selected cooperative grasp \(\mathbf{g}\) (fixing contact frames and \(G\)), solve at each control cycle:

Decision variables:
- stacked contact wrenches/forces \(f\),
- optional slack \(s\) for robustness,
- optional internal-force coefficients \(\alpha\).

Model:
- net object wrench: \(w = G(\mathbf{g}) f\),
- joint torques: \(\tau = J(q)^\top f + \tau_\text{bias}\).

One practical convex QP:

\[
\begin{aligned}
\min_{f,\,s}\quad &
\|J^\top f\|_{W_\tau}^2
+ \lambda_f \|f\|_{W_f}^2
+ \lambda_\Delta \|f - f^\text{prev}\|^2
+ \lambda_s \|s\|^2 \\
\text{s.t.}\quad &
G f + w_\text{ext} = w_d + s \\
& A(\mathbf{g}) f \le b(\mathbf{g}) \quad \text{(polyhedral friction cones + normal bounds)}\\
& \tau_{\min} \le J^\top f + \tau_\text{bias} \le \tau_{\max} \\
& m_\text{lin}(f;\mathbf{g}) \ge m_\text{min}
\end{aligned}
\]

where \(m_\text{lin}\) enforces a minimum distance to the friction cone facets (a linear margin constraint for polyhedral cones).

#### Internal force parameterization (optional)
Let \(N_G\) span the nullspace of \(G\). Write \(f = f_p + N_G \alpha\). Then solve for \(\alpha\) to increase stability margin while keeping energy small:

\[
\min_{\alpha}\ \|J^\top (f_p + N_G \alpha)\|_{W_\tau}^2 + \lambda_\alpha \|\alpha\|^2
\quad \text{s.t. friction/torque/margin constraints.}
\]

---

## Regeneration strategy (practical)

When a candidate cooperative grasp \(\mathbf{g}\) fails:
- **Stability failure (slip/low margin)**:
  - regenerate the hand(s) contributing the lowest margin;
  - bias toward higher-normal feasible contacts, larger moment arms, or more opposed contacts;
  - add constraints to avoid contact frames with low predicted friction.
- **Energy failure (high torque/power)**:
  - regenerate the hand(s) near torque limits;
  - bias toward grasps reducing required moment about gravity, improving leverage, or moving contacts closer to torque-favorable configurations.
- **Collision or reach failure**:
  - regenerate with geometric constraints / approach corridor constraints.

The loop terminates when a feasible set is found or a timeout occurs (fallback: slow down, request regrasp, increase safety margin, or abort).

---

## Real-time rates (suggested)
- **Perception + estimation**: 30–200 Hz (depends on sensors).
- **Neural grasp generation/regeneration**: event-driven / 1–10 Hz (or on failures).
- **Outer grasp selection**: 1–20 Hz (multi-rate).
- **Inner force QP**: 200–1000 Hz (controller-rate), warm-started.
- **Passivity observer/controller**: same as inner loop.

