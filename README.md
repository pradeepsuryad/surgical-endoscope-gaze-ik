# Surgical Endoscope Tracking via SE(3) Newton-Raphson IK
### 7-DOF Franka Emika Panda | MuJoCo | Graduate Robotics Portfolio

[![ci](https://github.com/pradeepsuryad/surgical-endoscope-gaze-ik/actions/workflows/ci.yml/badge.svg)](https://github.com/pradeepsuryad/surgical-endoscope-gaze-ik/actions/workflows/ci.yml)

> **Summary:** A custom SE(3) Inverse Kinematics solver drives a simulated
> 7-DOF manipulator along a circular trajectory while re-orienting the
> tool's camera axis toward a fixed surgical target — all without using any
> built-in IK solver. In the committed run, tracking is sub-millimetre for
> roughly the first 80% of the lap (after a brief start-up transient), then
> diverges — see the tracking-error plot below.

---

## Demo

### MuJoCo Simulation Video

<video src="results/simulation.mp4" autoplay loop muted controls width="700"></video>

> Recorded with `python main.py --no-render --record` using MuJoCo's offscreen renderer.

---

### Summary Dashboard
![Summary Dashboard](results/00_summary_dashboard.png)

### 3D End-Effector Trajectory — Desired vs Actual
![3D Trajectory](results/01_3d_trajectory.png)

### SE(3) Tracking Error over Time
![Tracking Error](results/02_error_over_time.png)

> Apart from the start-up transient, position and orientation error stay near
> zero until t ≈ 1.5 s, then grow to ≈ 107 mm and ≈ 1 rad by the end of the
> lap without recovering. The rise coincides with q1 flattening at ≈ 166° in
> the joint-angle plot below — its +2.8973 rad limit in `src/ik_solver.py`.
> The boxed annotation is fixed text placed at the peak (`src/visualizer.py`),
> not derived from the log: the joint at its limit is q1, not q4, and neither
> λ nor σ_min(J) is logged.

### Analytical FK vs MuJoCo FK Verification
![FK Comparison](results/03_fk_comparison.png)

> Analytical DH FK includes the 103.4 mm flange → `attachment_site` offset (`_T_FLANGE_EE`), bringing discrepancy to **< 10⁻¹² mm** (floating-point noise only).

### Joint Angles
![Joint Angles](results/04_joint_angles.png)

### Newton-Raphson IK Convergence per Iteration
![IK Convergence](results/05_ik_convergence.png)

> **Top panel:** mean ± σ residual ‖e_k‖ across all converged steps, with
> individual step traces shown behind (thin lines).
> **Bottom panel:** per-iteration reduction ratio e_k / e_{k-1}. Converged
> steps stop as soon as ‖e‖ < tol and are padded with their last residual
> (`src/visualizer.py`), so the bars at k = 2–5 are exactly 1 by construction;
> all of the decay shown happens in the first iteration.
> Null-space joint-limit avoidance (`N = I − J⁺J`, gain = 0.5) is applied
> at every iteration as a secondary task on the 1-DOF redundancy.

### IK Computation Time per Step
![Computation Time](results/06_computation_time.png)

---

## Architecture Overview

```
surgical_endoscope_tracking/
│
├── src/
│   ├── kinematics.py    — DH-based analytical FK + SE(3) spatial-error math
│   ├── ik_solver.py     — Newton-Raphson loop + damped-least-squares (DLS/LM)
│   ├── trajectory.py    — Circular trajectory + gaze-aligned (look-at) orientation
│   ├── simulation.py    — MuJoCo environment, step loop, data logging
│   └── visualizer.py    — 7 Matplotlib/Seaborn analysis plots
│
├── models/
│   └── franka_emika_panda/   ← place MuJoCo Menagerie XML here
│
├── results/             — generated PNG plots
├── main.py              — CLI entry point
└── requirements.txt
```

### Data flow

```
CircularTrajectory
       │  T_desired ∈ SE(3)
       ▼
NewtonRaphsonIK ←── J(q) from mj_jacSite  ─── MuJoCo Model
       │  q*               ←── T_cur from mj_kinematics
       ▼
EndoscopeSimulation
  data.qpos[:7] = q*     (set directly: the arm is teleported to q* each step)
  data.ctrl[:7] = q*  →  mj_step()
       │
       ├── log: MuJoCo FK pose
       ├── log: Analytical DH FK pose      ← FK verification
       └── log: ‖e_p‖, ‖e_o‖, solve time
```

---

## Mathematical Formulation

### 1. Denavit-Hartenberg Forward Kinematics

Each joint frame uses the **standard DH convention**:

```
T_{i-1}^{i} = Rot_z(θᵢ) · Trans_z(dᵢ) · Trans_x(aᵢ) · Rot_x(αᵢ)
```

The full analytical forward kinematics are:

```
T_0^{EE}(q) = T_0^1(q₁) · T_1^2(q₂) · … · T_6^7(q₇) · T_{flange}^{EE}
```

Panda DH parameters (metres / radians):

| i | aᵢ      | dᵢ     | αᵢ       |
|---|---------|--------|----------|
| 1 | 0.0000  | 0.3330 | 0        |
| 2 | 0.0000  | 0.0000 | −π/2     |
| 3 | 0.0000  | 0.3160 | +π/2     |
| 4 | 0.0825  | 0.0000 | +π/2     |
| 5 |−0.0825  | 0.3840 | −π/2     |
| 6 | 0.0000  | 0.0000 | +π/2     |
| 7 | 0.0880  | 0.1070 | +π/2     |

### 2. SE(3) Spatial Error

The 6-DOF task-space error vector **e ∈ ℝ⁶** driving the NR update:

```
e = [eₚ; eₒ]

eₚ = p_d − p_c                           (position error, metres)

R_err = R_d · R_cᵀ
eₒ = log_{SO(3)}(R_err) = θ · k̂          (axis-angle, radians)
```

where `log_{SO(3)}` is computed via the Rodrigues inverse formula:

```
θ = arccos((tr(R) − 1) / 2)
k̂ = [R₃₂−R₂₃, R₁₃−R₃₁, R₂₁−R₁₂] / (2 sin θ)
```

### 3. Newton-Raphson IK Update

```
q_{k+1} = q_k + J⁺ · e_k
```

**Damped Moore-Penrose pseudoinverse (Levenberg-Marquardt):**

```
J⁺ = Jᵀ(JJᵀ + λ²I)⁻¹

λ = λ_min   if σ_min(J) ≥ σ_thresh   (standard pinv near regular configs)
λ = λ_max   if σ_min(J) < σ_thresh   (DLS damping near singularities)
```

| Hyperparameter | Default | Purpose |
|---|---|---|
| `max_iter` | 5 | Fixed NR budget per step (real-time constraint) |
| `λ_min` | 1 × 10⁻⁶ | Near-zero damping in regular configs |
| `λ_max` | 0.05 | Damping near kinematic singularities |
| `σ_thresh` | 0.05 | Min singular value threshold for DLS activation |
| `tol` | 1 × 10⁻⁴ | Early-stop threshold on ‖e‖ |

### 4. Gaze (Look-At) Constraint / Endoscope Active Vision

At each waypoint `p(θ) = [R·cos θ, R·sin θ, z_height]` on the circle,
the desired rotation matrix is constructed so that the **local X-axis acts
as the camera gaze direction** pointing at the fixed target `p_target`:

```
x̂_d = (p_target − p(θ)) / ‖p_target − p(θ)‖

ẑ_d = (x̂_d × ẑ_world) / ‖…‖
ŷ_d = ẑ_d × x̂_d

R_d(θ) = [x̂_d | ŷ_d | ẑ_d]   ∈ SO(3)
```

This points the desired camera axis at the tissue target at every waypoint
as the end-effector orbits — a gaze (look-at) constraint. There is no remote
centre of motion: no fulcrum point is constrained or measured.

---

## Dependencies

| Package | Version | Role |
|---|---|---|
| `mujoco` | ≥ 3.1.0 | Physics simulation, Jacobian extraction |
| `numpy`  | ≥ 1.24.0 | Linear algebra core |
| `matplotlib` | ≥ 3.7.0 | Analysis plots |
| `seaborn` | ≥ 0.12.0 | Publication-quality plot styling |
| `scipy` | ≥ 1.10.0 | Auxiliary scientific utilities |
| `imageio[ffmpeg]` | ≥ 2.28.0 | MP4 video recording (`--record` flag) |

---

## Setup & How to Run

### 1. Install dependencies

```bash
pip install -r requirements.txt
```

### 2. Download the MuJoCo Menagerie Panda model

```bash
git clone https://github.com/google-deepmind/mujoco_menagerie.git
cp -r mujoco_menagerie/franka_emika_panda models/
```

The simulation expects the XML at `models/franka_emika_panda/panda.xml`.

### 3. Run the simulation

```bash
# Default: 1 lap, 1 mm steps, 5 NR iterations, live viewer
python main.py

# Headless (no viewer — useful for servers / CI)
python main.py --no-render

# Record a video to results/simulation.mp4
python main.py --no-render --record

# 2 laps, 0.5 mm steps, 10 NR iterations
python main.py --laps 2 --step-mm 0.5 --max-iter 10

# Custom circle geometry
python main.py --radius 0.12 --height 0.45
```

### 4. View results

All plots are saved to `results/`:

| File | Description |
|---|---|
| `simulation.mp4` | Full MuJoCo simulation video (offscreen render) |
| `00_summary_dashboard.png` | One-page summary for quick inspection |
| `01_3d_trajectory.png` | Desired vs actual 3D path |
| `02_error_over_time.png` | Position (mm) & orientation (rad) error; peak error marked |
| `03_fk_comparison.png` | Analytical FK vs MuJoCo FK verification |
| `04_joint_angles.png` | All 7 joint angles over the run |
| `05_ik_convergence.png` | NR residual decay + per-iteration reduction ratio (two-panel) |
| `06_computation_time.png` | Per-step IK solve time |

---

## Key Design Decisions

- **No built-in IK solvers.** The Newton-Raphson loop in `ik_solver.py` is
  implemented from scratch using only NumPy linear algebra.

- **MuJoCo Jacobian + Analytical FK.** The geometric Jacobian is extracted
  from MuJoCo's `mj_jacSite` for physical accuracy, while the analytical
  DH-based FK is computed independently to verify model agreement. A fixed
  103.4 mm flange → `attachment_site` offset (`_T_FLANGE_EE`) is appended
  after joint 7 so both FK methods resolve to the same physical point.

- **Fixed-iteration budget (N = 5).** Real surgical robots operate under
  hard real-time constraints.  Locking `max_iter = 5` deliberately models
  this; the residual plots quantify accuracy within that budget.

- **1 mm waypoint spacing.** `CircularTrajectory` computes
  `n = ceil(2πR / 0.001)` waypoints, guaranteeing sub-millimetre arc
  discretisation error.

---

## Author

**Pradeep Surya Dadi** — Graduate Robotics Student, Northeastern University  
`dadi.pr@northeastern.edu`

---

*Built as a graduate robotics portfolio project demonstrating SE(3) IK,
kinematically redundant manipulation, and surgical robotics task constraints.*
