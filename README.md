<h1 align="center">
Hello, I'm Denis Ryabko
	<a href="https://github.com/Bouaskaoun" target="_self">
		<img src="https://media.giphy.com/media/hvRJCLFzcasrR4ia7z/giphy.gif" width="30">
	</a>
</h1>

<hr>

<pre>
💻 I am mainly a Frontend Developer
📚 I have a Bachelors in Gas Reservoir Engineering from the Gubkin University
📝 I have a strong interest in Frontend and geomechanical modeling 
🛠️ Currently working on a Gazprom Neft
🌟 Main language: JavaScript
🚩 Interested in learning more about Microfrontend Architectures.
😃 I look forward to collaborate on impactful projects
</pre>
<hr>

## 🤝 Connect with me

<p align="center">
 <a href="www.linkedin.com/in/denis-ryabko-a02543298">
    <img src="https://img.shields.io/badge/LinkedIn-blue?style=for-the-badge&logo=linkedin&logoColor=white" alt="LinkedIn Badge"/>
  </a>
</p>

## 💻 My Tech Stack

<div align="center">
  <img src="https://github.com/devicons/devicon/blob/master/icons/react/react-original-wordmark.svg" title="React" alt="React" width="40" height="40"/>&nbsp;
    <img src="https://github.com/devicons/devicon/blob/master/icons/redux/redux-original.svg" title="Redux" alt="Redux " width="40" height="40"/>&nbsp;
  <img src="https://github.com/devicons/devicon/blob/master/icons/angular/angular-original.svg" title="Angular" alt="Angular" width="40" height="40"/>&nbsp;
  <img src="https://github.com/devicons/devicon/blob/master/icons/materialui/materialui-original.svg" title="Material UI" alt="Material UI" width="40" height="40"/>&nbsp;
  <img src="https://github.com/devicons/devicon/blob/master/icons/nodejs/nodejs-original-wordmark.svg" title="NodeJS" alt="NodeJS" width="40" height="40"/>&nbsp;
   <img src="https://github.com/devicons/devicon/blob/master/icons/go/go-original.svg" title="Golang" alt="Golang" width="40" height="40"/>&nbsp;
   <img src="https://github.com/devicons/devicon/blob/master/icons/python/python-original.svg" title="Python" alt="Python" width="40" height="40"/>&nbsp;
  <img src="https://github.com/devicons/devicon/blob/master/icons/git/git-original-wordmark.svg" title="Git" alt="Git" width="40" height="40"/>
</div>

---

## 🧪 How to use simulation for planning/optimization (robotics + MIP)

Simulation is the “truth model” that helps you **stress-test** plans (schedules, assignments, routes, timings) before deploying them. In these problems, simulation is useful for three things:

- **Feasibility checking**: catch collisions, deadlocks, buffer overflows, missed precedence, and timing violations that a simplified optimizer model may miss.
- **Model calibration**: estimate travel-time models, service-time distributions, congestion effects, and failure rates from realistic dynamics.
- **Policy evaluation**: compare heuristics vs MILP/MIQP solutions on throughput, WIP, lateness, energy, and robustness under randomness.

### 1) Factory/warehouse scheduling for heterogeneous robot fleets (job-shop + AGV/AMR)

**What to simulate (minimum viable “digital twin”)**

- **Discrete-event simulation (DES)** of workstations, precedence constraints, buffers (finite capacity), and resource constraints (machines, operators, chargers).
- **Fleet/traffic layer** for AGV/AMR travel and contention: graph-based routes, intersections, one-way aisles, blocking, and reservation/priority rules.
- **Stochasticity**: travel time variability, pick/place time variability, breakdowns, battery/charging delays, blocked aisles, and rush-hour demand spikes.

**How to connect simulation ↔ optimization**

- **Inputs to the optimizer** (estimated from sim or logs): travel-time matrix by robot type & time-of-day, service-time distributions, buffer capacities, station calendars, robot availability, charging constraints.
- **Optimizer outputs**: job-to-station schedule, robot assignment, pickup/delivery pairing, time windows, and (optionally) coarse routes on a graph.
- **Simulation as a validator**: replay the optimizer plan in the simulator to measure congestion, blocking, buffer violations, and missed deadlines.
- **Iterate**: update travel-time / congestion penalties and add constraints (e.g., intersection capacity, minimum headways) until sim and solver agree.

**Metrics to track**

- **Throughput / makespan**, **WIP**, **order lateness**, **robot utilization**, **queue lengths**, **buffer overflow rate**, **deadlock/near-deadlock events**, **travel distance/energy**, **charger contention**.

**Practical pattern**

- Start with a **graph-level** traffic sim (fast, scalable) and only add higher-fidelity physics for critical zones (tight aisles, docking).
- Use simulation to produce **scenario sets** (peak load, partial outages, layout changes) and require the optimizer to perform well across them.

### 2) Integrated task + motion planning with MILP/MIQP (assignment + collision-free timing)

**Where simulation helps most**

- Mixed-integer models often rely on **simplified geometry** (convex obstacles, time discretization, inflated safety margins). Simulation exposes when those simplifications cause “planner success” but **execution failure** (contacts, narrow passages, timing slips).

**What to simulate**

- **Kinematic/physics simulation** (as needed): robot dynamics, actuation limits, acceleration bounds, payload effects, sensing noise, and contact.
- **Multi-robot interaction**: timing uncertainty, priority rules at shared regions, and recovery behaviors (stop-and-wait, reroute, replanning).

**How to connect simulation ↔ MILP/MIQP**

- **From sim to model**:
  - Learn/tune **motion envelopes** (max accel/decel, turning constraints, stopping distance).
  - Fit **time-parameterization** and tracking error bounds for each robot/payload.
  - Identify “hard” regions (narrow aisles, shared workcells) and represent them as **capacity/time-window constraints** in the MILP/MIQP.
- **From model to sim**:
  - Export a plan as \((\text{task assignment}, \text{sequence}, \text{timestamps}, \text{path/waypoints})\).
  - Simulate execution with noise/disturbances; log collision margins, tracking error, and task completion times.
- **Close the loop**:
  - If sim shows collisions or frequent delays, increase safety margins or add constraints (e.g., **separating hyperplanes**, **conflict regions**, **min headway**) and re-solve.
  - Keep the MIP solving the **high-level discrete decisions**, and let a local planner handle last-meter refinements—simulation tells you where the boundary should be.

**Metrics to track**

- **Success rate** (no collisions, all tasks complete), **throughput**, **mean delay vs planned**, **replan frequency**, **near-miss margin**, **constraint violations** (speed/accel), **deadlock rate** in shared regions.

### Common “simulation workflow” that scales

- **Build**: Start with a fast, simplified sim that matches the optimizer’s abstractions (graph + DES).
- **Calibrate**: Tune with logs (or high-fidelity sim) so that travel/service times and variability are realistic.
- **Optimize**: Solve MILP/MIQP on the calibrated model.
- **Validate**: Replay in simulation under randomness + scenario stress tests.
- **Deploy**: Roll out as an “optimization layer” above WMS/MES/robot fleet manager, with periodic re-optimization and monitored drift (sim vs reality).
