# Build a small NMPC, then out-benchmark it

The design that fits two weeks is deliberately modest. Use a **3-state unicycle NMPC** (Δt = 0.1 s, N ≈ 20, RK4 or midpoint discretization) written with **CasADi multiple shooting, solved by IPOPT, and warm-started** from the shifted previous solution. Pass LiDAR obstacles to it as a fixed-size list of inflated circles, and give it **slack-softened discrete-time CBF constraints**, with plain distance constraints as the comparison arm. Run everything in the `odom` frame, compensate latency by forward-simulating the robot model, and use a hand-written LiDAR clustering node. Spend most of the remaining effort on what the grading rewards: a paired Monte Carlo benchmark against Kanayama, Samson/backstepping and kinematic sliding-mode baselines.

The work is mostly integration and evaluation, not solver research. The only published IPOPT timings for a comparable problem are 28–62 ms per solve ([Zeng et al. 2021](https://arxiv.org/pdf/2007.11718)), which fits a 100 ms budget. acados is therefore a stretch goal, not a requirement.

The real robot sets the constraints. It is slow (0.22 m/s, 2.84 rad/s), its LiDAR scans at about 5 Hz, it talks over Wi-Fi DDS, and its driver has **no `cmd_vel` timeout** ([turtlebot3.cpp](https://raw.githubusercontent.com/ROBOTIS-GIT/turtlebot3/humble/turtlebot3_node/src/turtlebot3.cpp)). Safety layers and delay handling are therefore requirements, not polish.

No published study compares NMPC against Kanayama and SMC on a TurtleBot3 in one benchmark. A reproducible, statistically tested comparison is a legitimate contribution and the strongest thing to defend in the viva.

## A 3-state multiple-shooting NMPC meets the 100 ms budget with IPOPT

### Model and limits

**Model.** Use the unicycle with state x = [p_x, p_y, θ] and input u = [v, ω]. The Burger's published limits are **v ≤ 0.22 m/s and ω ≤ 2.84 rad/s** ([ROBOTIS e-manual](https://emanual.robotis.com/docs/en/platform/turtlebot3/features/)). The Humble driver config gives **wheel separation 0.160 m and wheel radius 0.033 m** and applies no acceleration profile (`profile_acceleration: 0.0`) ([burger.yaml](https://raw.githubusercontent.com/ROBOTIS-GIT/turtlebot3/humble/turtlebot3_node/param/burger.yaml)).

**Coupled limits.** Both limits are wheel-speed limits, so v and ω cannot reach their maxima together. The constraint that actually binds is the wheel "diamond" |v| + 0.08|ω| ≤ 0.22. At v = 0.15 m/s, for example, it allows only |ω| ≤ 0.875 rad/s. Put the diamond in the NMPC constraint set and in the shared saturation block that every controller passes through.

**Acceleration limits.** No official Burger acceleration limit was found. Treat a_max ≈ 0.5 m/s² and α_max ≈ 3 rad/s² as stated engineering choices. Enforce them as Δu constraints, because a zero acceleration profile means step commands cause wheel slip.

### Discretization and transcription

**Integrator.** Forward Euler is adequate at Δt = 0.1 s, since the robot moves only about 2.2 cm per step. RK4, or the midpoint rule with θ_mid = θ + ωΔt/2, is more accurate at high ω and costs almost nothing after SX expansion. The exact zero-order-hold arc map is singular as ω → 0, so avoid it inside the NLP.

**Multiple shooting.** Use multiple shooting, with X (3×(N+1)) and U (2×N) as decision variables plus continuity constraints. CasADi's docs describe it as "includ[ing] the state at certain shooting nodes as decision variables" ([CasADi docs](https://web.casadi.org/docs/)). It suits obstacles because the constraints become direct functions of the decision variables instead of nested rollouts. It also warm-starts naturally: shift X and U by one step and repeat the last column.

### Solver setup and timing

**Build and compile.** Write the problem in the Opti stack for readability, then compile it once with `opti.to_function`. The docs say this is "quite a bit faster than opti.set_initial + opti.solve + sol.value" ([CasADi docs](https://web.casadi.org/docs/)). Set `expand: True` to convert MX to SX.

**Warm start.** Warm starting matters: "consecutively calling solve will not help the convergence," while seeding from the previous solution cut iterations from 15 to 4 in the docs example ([CasADi docs](https://web.casadi.org/docs/)). Pass the previous `lam_g` as well.

**IPOPT options.** A real-time starting point is `max_iter` 100, `tol` 1e-4, `acceptable_tol` 1e-3, `max_wall_time` 0.08, `print_level` 0, and the MUMPS linear solver. These names are standard but were not checked against the IPOPT options page, which could not be fetched. The tolerances popularized by the Mehrez workshop (2000 iterations, 1e-8) suit offline use, not a deadline.

**Failure handling.** Always read `return_status`. On failure, apply the second control of the previous plan or brake. `opti.debug.show_infeasibilities()` is the tool to use when obstacle constraints make the problem infeasible.

**Expected timing.** No published CasADi/IPOPT benchmark exists for a unicycle at N ≈ 20. The nearest evidence is Zeng et al.'s 4-state, 1-obstacle problem: **0.028 s at N = 5, 0.033 s at N = 7, 0.048 s at N = 15 and 0.062 s at N = 30** with distance constraints ([Zeng et al. 2021, Table I](https://arxiv.org/pdf/2007.11718)). Extrapolating, a warm-started 3-state problem with N = 20 and about 8 obstacles should take roughly 10–50 ms on a laptop. That is an inference, so measure it in the first days.

**acados.** acados (SQP-RTI with HPIPM) is built "for real-time applications and embedded systems" ([acados GitHub](https://github.com/acados/acados)). However, building it from source with CMake costs about half a day, and it pays off only if IPOPT misses the deadline or you want a solver ablation. The GMPC TurtleBot3 study adds a useful data point: a convex reformulation solved about **1.5× faster than IPOPT NMPC with much lower variance** ([Jin et al. 2024](https://arxiv.org/pdf/2403.07317)). An LTV-MPC QP via OSQP is therefore a cheaper ablation arm than acados.

### Cost design

**Tracking error.** Penalize tracking error either in the robot-aligned Kanayama frame, where lateral error e_y and along-track error e_x can carry different weights, or as world-frame position error. Never use a raw (θ − θ_r)². Use 2(1 − cos(θ − θ_r)) instead, or unwrap the reference so it is continuous with the current heading.

**Effort.** Penalize u − u_r, the deviation from the flatness-derived feedforward (v_r, ω_r), not raw u. Otherwise the controller fights the motion it is supposed to track. Add a Δu term, with the previously applied control passed in as a parameter, for smoothness.

**Starting weights.** Reasonable starting points at TB3 scale are Q = diag(10, 10, 1–5), R = diag(0.5, 0.05), S = diag(1, 0.1) and terminal weight P = 5–10 × Q. Zeng et al. used Q = 10I, R = I and P = 100I ([Zeng et al. 2021](https://arxiv.org/pdf/2007.11718)), the same tenfold terminal convention.

**Stability argument.** For the report, cite Gu & Hu: stability of receding-horizon tracking for a nonholonomic robot is "guaranteed by adding a terminal-state penalty to the cost function and constraining the terminal state to a terminal-state region" ([Gu & Hu 2006](https://www.bohrium.com/paper-details/receding-horizon-tracking-control-of-wheeled-mobile-robots/812073680136830977-2483)). Then implement only the terminal cost. A terminal set hurts feasibility near obstacles. Dropping terminal ingredients has precedent in nonholonomic NMPC that omits them "to reduce computational complexity" ([Salimi Lafmejani et al. 2021](https://faculty.engineering.asu.edu/acs/wp-content/uploads/sites/33/2021/03/Salimi-Lafmejani-RAS-2021-Nonlinear-MPC-for-Collision-Free-and-Deadlock-Free-Navigation-of-Multiple-Nonholonomic-Mobile-Robots.pdf)). Making that gap between theory and implementation explicit is a good viva move.

| Design element | Recommended choice | Basis |
|---|---|---|
| State / input | [x, y, θ] / [v, ω] | Unicycle kinematics |
| Δt, N | 0.1 s, 15–25 (1.5–2.5 s, ≈0.44 m lookahead at 0.22 m/s) | Matches the 10 Hz loop and 5 Hz scans |
| Integrator | RK4 or midpoint, one step per interval | Accurate at high ω, cheap after `expand` |
| Transcription | Multiple shooting, Opti → `to_function` | [CasADi docs](https://web.casadi.org/docs/) |
| Input limits | 0 ≤ v ≤ 0.22, \|ω\| ≤ 2.84, \|v\| + 0.08\|ω\| ≤ 0.22, Δu rate limits | [ROBOTIS](https://emanual.robotis.com/docs/en/platform/turtlebot3/features/), [burger.yaml](https://raw.githubusercontent.com/ROBOTIS-GIT/turtlebot3/humble/turtlebot3_node/param/burger.yaml) |
| Cost | Q on position, 1 − cos on heading, R on u − u_r, S on Δu, P ≈ 10Q | [Zeng et al.](https://arxiv.org/pdf/2007.11718) convention |
| Terminal | Cost only; cite the terminal-region theory | [Gu & Hu 2006](https://www.bohrium.com/paper-details/receding-horizon-tracking-control-of-wheeled-mobile-robots/812073680136830977-2483) |
| Solver | IPOPT + MUMPS, warm start, wall-time cap, fallback on failure | Measure p95 solve time against Δt |

## CBF constraints with slacks beat raw distance constraints at short horizons

### Distance versus CBF constraints

Zeng, Zhang and Sreenath's MPC-CBF paper is the core reference for the obstacle-constraint choice, and its numbers frame the ablation.

**Two formulations.** Both use the barrier h = (x − x_obs)² + (y − y_obs)² − r². The distance-constraint (DC) form imposes h(x_k) ≥ 0. The discrete-time CBF form imposes **h(x_{k+1}) ≥ (1 − γ) h(x_k)** along the whole horizon, so the safety margin can shrink by at most a factor (1 − γ) per step ([Zeng et al. 2021](https://arxiv.org/pdf/2007.11718)).

**What the table shows.** DC constraints stay inactive until the predicted trajectory touches the obstacle, so **"MPC-DC always reaches to the boundary of the obstacle"** with zero minimum clearance, and at N = 5 it was outright infeasible. CBF constraints act earlier and set their own margin. At N = 5, γ = 0.1 kept 1.48 m of clearance and γ = 0.5 kept 0.11 m, and MPC-CBF at N = 5 with γ = 0.25 behaved like MPC-DC at N = 7 ([Zeng et al. 2021](https://arxiv.org/pdf/2007.11718)).

**Feasibility trade-off.** The cost is feasibility. A small γ is safer "but this might also make the optimization infeasible," and persistent feasibility "is still an open problem" ([Zeng et al. 2021](https://arxiv.org/pdf/2007.11718)).

**What to build.** Both arms should be slack-softened. A constraint of the form h(x_{k+1}) ≥ (1 − γ)h(x_k) − s, with s ≥ 0 and an L1 penalty ρ ≈ 10³–10⁴, behaves as a hard constraint whenever a feasible solution exists. It also survives the sudden obstacle appearances and sensor noise that would otherwise crash IPOPT.

### Unicycle details that matter

**Look-ahead point.** The barrier depends only on position, so ω does not enter h(x_{k+1}) under one Euler step. A small γ can then make the robot brake rather than steer. Evaluating h at a look-ahead point p + ℓ[cos θ, sin θ], with ℓ ≈ 0.05–0.1 m, puts heading into the constraint. This is standard practice rather than something from a fetched source, and worth a one-line ablation.

**Constraint count.** With M = 8 obstacle slots and N = 20, the problem has 160 inequality constraints. That is comfortable for IPOPT, whose runtime is dominated by N. Applying CBF constraints only for the first 5–10 steps is a further cost saving.

**Potential fields.** Penalizing 1/(h + ε) in the cost is always feasible but gives no guarantee and creates local minima. It is a useful third, "naive" arm, not the main method.

### LiDAR-to-circle pipeline

**Write it yourself.** No maintained Humble port of the popular `obstacle_detector` exists; the only ROS 2 repo ports just its scan merger ([romb-technologies fork](https://github.com/romb-technologies/obstacle_detector/blob/master/README.md)). `laser_segmentation` is released for Humble and does jump-distance clustering with a 0.3 m default threshold ([index.ros.org](https://index.ros.org/p/laser_segmentation/)). Still, an rclpy node of about 80 lines is simpler to own and explain.

**Pipeline steps.** Mask invalid beams, handling both `inf` (Gazebo) and `0.0` (likely on the real driver; verify on your unit). Convert to Cartesian in `base_scan`. Segment with an adaptive breakpoint detector, and merge the wrap-around segment. Fit a Kåsa least-squares circle per segment, clamping the radius to 0.05–0.5 m and splitting walls into chains of small circles. Inflate each radius by the robot radius (≈0.105 m) plus a 0.05–0.1 m margin. Transform the centres to `odom` with tf2 **at the scan's own timestamp**. Keep the K nearest, and pad with far-away dummy circles so the NMPC parameter vector never changes size and nothing is rebuilt.

**Compute cost.** The simulated LDS gives 360 beams at 5 Hz over 0.12–3.5 m with σ = 1 cm Gaussian noise ([model.sdf](https://raw.githubusercontent.com/ROBOTIS-GIT/turtlebot3_simulations/humble/turtlebot3_gazebo/models/turtlebot3_burger/model.sdf)), so compute is negligible.

**Between scans.** Static obstacles stay fixed in `odom`, so the MPC reuses the last list while the pose updates at the odometry rate. Hold obstacles for up to about 0.5 s if a scan is missed.

## Real-hardware failures come from Wi-Fi, 5 Hz scans and missing timeouts

### Rates and bringup

**Rates.** On the Pi, bring the robot up with `TURTLEBOT3_MODEL=burger`, `LDS_MODEL=LDS-02` and a `ROS_DOMAIN_ID` unique within the lab ([ROBOTIS bringup](https://emanual.robotis.com/docs/en/platform/turtlebot3/bringup/)). The driver publishes odometry and IMU data from a **50 ms timer (≈20 Hz)** ([turtlebot3.cpp](https://raw.githubusercontent.com/ROBOTIS-GIT/turtlebot3/humble/turtlebot3_node/src/turtlebot3.cpp)). The odometry synchronizer's 75 ms joint-state bound could pull that closer to 13 Hz ([odometry.cpp](https://raw.githubusercontent.com/ROBOTIS-GIT/turtlebot3/humble/turtlebot3_node/src/odometry.cpp)), so measure it with `ros2 topic hz /odom`. The LDS-02 spins at "5 Hz or above," with a 160–8000 mm range and ±3% accuracy beyond 0.3 m ([LDS-02 appendix](https://emanual.robotis.com/docs/en/platform/turtlebot3/appendix_lds_02/)). A 10 Hz MPC loop with 5 Hz obstacle updates is the natural fit.

**cmd_vel message type.** The manual presents stamped `cmd_vel` as a Jazzy-era change. Yet the current Humble-branch `burger.yaml` contains `enable_stamped_cmd_vel: true` ([burger.yaml](https://raw.githubusercontent.com/ROBOTIS-GIT/turtlebot3/humble/turtlebot3_bringup/param/burger.yaml)), while the node source subscribes to both types ([turtlebot3.cpp](https://raw.githubusercontent.com/ROBOTIS-GIT/turtlebot3/humble/turtlebot3_node/src/turtlebot3.cpp)). Run `ros2 topic info -v /cmd_vel` on your own robot before the first test. A type mismatch makes the robot ignore commands silently.

### Networking and clocks

**DDS over Wi-Fi.** Wi-Fi DDS is the main source of unpredictable latency. Users report reliability stalls over Wi-Fi with both FastDDS and CycloneDDS on Humble ([ROS Discourse](https://discourse.ros.org/t/ros2-qos-reliability-issue/31700)). CycloneDDS's default message size above the MTU breaks large topics over Wi-Fi ([hbrobotics thread](https://groups.google.com/g/hbrobotics/c/1NZXnFtMjmQ)). Use a dedicated router or a phone hotspot with multicast enabled, not campus Wi-Fi with client isolation. Verify with `ros2 multicast send`/`receive`.

**Clock sync.** Run chrony with the laptop as the server. Without it, TF lookups at Pi-stamped times fail with extrapolation errors, and every latency you measure is wrong. The ROBOTIS setup pages do not cover time sync at all ([SBC setup](https://emanual.robotis.com/docs/en/platform/turtlebot3/sbc_setup/)).

### Latency compensation and QoS

**Measure, don't assume.** No published measurement of TB3 Humble Wi-Fi latency exists. Log `now() − header.stamp` for `/odom` and `/scan` after syncing clocks. Then run a velocity step test to fit dead time plus a first-order lag.

**Compensate in the controller.** Keep a deque of applied commands. Each tick, integrate the unicycle model from the latest odometry stamp to t_now + τ using the commands actually sent, and solve from that predicted state. With τ ≈ 0.1–0.15 s and Δt = 0.1 s, also fix the first one or two moves of the plan to the commands already committed. Report NMPC with and without compensation as two variants; the delay sweep will show the gap.

**Node structure and QoS.** Use a timer-driven control callback in its own MutuallyExclusive callback group, and subscribers in other groups that only copy the latest message under a lock. Spin with a `MultiThreadedExecutor`, so a slow solve never blocks sensor callbacks. Subscribe to `/scan` with `qos_profile_sensor_data`: a reliable subscriber to Gazebo's best-effort publisher receives nothing. Publish `/cmd_vel` as reliable with depth 1, so stale commands do not queue.

### Safety layers

**Why they are needed.** `turtlebot3_node` has only a 100 ms heartbeat to OpenCR and **no `cmd_vel` timeout** ([turtlebot3.cpp](https://raw.githubusercontent.com/ROBOTIS-GIT/turtlebot3/humble/turtlebot3_node/src/turtlebot3.cpp)). Whether the firmware stops on heartbeat loss is unverified. ROS Discourse threads document "ghost commands" that persist on bad Wi-Fi ([ROS Discourse](https://discourse.openrobotics.org/t/stop-ghost-commands-on-bad-wi-fi-ros2-kinematic-guard-for-cmd-vel-safety/54681)).

**What to build.**

- A tiny relay on the Pi that forwards `/cmd_vel_safe` and publishes zero if no command arrives within about 0.3 s. Only a robot-side node can catch a Wi-Fi drop.
- A stale-data watchdog in the controller: odometry older than about 0.3 s, or a scan older than about 0.5 s, means publish zero.
- A forward-sector proximity stop at 0.15–0.2 m.
- An `/estop` topic, with teleop kept ready.
- Zero-velocity publishes in a `finally` block before `rclpy` shuts down. On Humble, SIGINT can invalidate the context first, which is another reason the Pi relay matters.

### Odometry and ground truth

**What the built-in odometry does.** The TB3 odometry already fuses the IMU, but for heading only. Translation comes from wheel encoders with midpoint integration, heading increments come from the IMU quaternion when `use_imu: true`, and covariances are left unset ([odometry.cpp](https://raw.githubusercontent.com/ROBOTIS-GIT/turtlebot3/humble/turtlebot3_node/src/odometry.cpp)). A robot_localization EKF on top would double-count the IMU. Document the built-in fusion as adequate, keep the robot still during gyro calibration at boot, and calibrate wheel radius with a 2 m tape-measure drive.

**Ground truth.** Without mocap, hardware ground truth comes from taped start and end poses. Pseudo-ground-truth can come from running `slam_toolbox` on recorded scans. In Gazebo, add a `gazebo_ros_p3d` plugin remapped to `ground_truth` so it does not collide with `/odom` ([gazebo_ros_pkgs wiki](https://github.com/ros-simulation/gazebo_ros_pkgs/wiki/ROS-2-Migration:-P3D)). Report both the error against odometry and the error against ground truth.

**Sim-to-real gap.** One TurtleBot3 Burger CBF-MPC study reported final errors of roughly 0.13/0.06 m on hardware versus 0.08/0.07 m in simulation ([BEEI](https://www.beei.org/index.php/EEI/article/download/11309/4638)). That figure was seen via a summary, so check it against the full text before quoting it.

### Package layout and logging

**Package.** Put everything in a single `ament_python` package. Keep a ROS-free `core/` directory (model, NMPC builder, obstacle fitting, reference generators, baselines) that pytest can exercise. Add thin nodes for the controller, obstacle extractor, safety relay and Gazebo obstacle spawner, and use `sim` and `real` launch files with per-platform YAML. Everything states in `odom`, per REP-105 ([REP-105](https://www.ros.org/reps/rep-0105.html)).

**Logging.** Log a CSV row per tick (time, pose, reference, command, solve time, clearance). Also bag `/odom`, `/scan`, `/cmd_vel`, `/tf` and the MPC diagnostics, so the obstacle node can be retuned offline on replayed scans.

## Kanayama, Samson/backstepping and kinematic SMC make an honest baseline set

### Kanayama

Kanayama et al. (ICRA 1990) is the canonical baseline. Its Lyapunov-proved law was tested on real hardware with a velocity/acceleration limiter ([researchr](https://researchr.org/publication/KanayamaKMN90)):

v = v_r cos e_θ + k_x e_x, ω = ω_r + v_r(k_y e_y + k_θ sin e_θ)

This form is reproduced as the "Kanayama controller" baseline in [Ramaiah, Honc & Dušek 2017](https://skoge.folk.ntnu.no/prost/proceedings/process-control-slovakia-2017/data/papers/028.pdf).

**Lyapunov argument.** With V = ½(e_x² + e_y²) + (1 − cos e_θ)/k_y, the derivative works out to V̇ = −k_x e_x² − (v_r k_θ/k_y) sin² e_θ. This is a standard derivation; the original PDF could not be retrieved. Convergence therefore needs v_r to stay away from zero. That is a real weakness at stop points and turn-in-place references, and a legitimate point in NMPC's favour.

**Gains.** Linearizing about straight-line motion gives s² + v_r k_θ s + v_r² k_y. Critical damping then means **k_θ = 2√k_y**. For TB3 speeds, start with k_y ≈ 25–100 m⁻², k_θ ≈ 10–20 m⁻¹ and k_x ≈ 1–3 s⁻¹. These values are derived, not taken from the paper.

### Backstepping and Samson

**Same law at kinematic level.** Kinematic backstepping collapses into the Samson law, so present them as one baseline. Jiang & Nijmeijer's backstepping design gives local tracking, and global tracking under extra conditions on the desired velocities ([U. Twente record](https://research.utwente.nl/en/publications/tracking-control-of-mobile-robots-a-case-study-in-backstepping/)). At the kinematic level it reduces to ω = ω_r + c_y v_r sinc(e_θ) e_y + c_θ e_θ. That expression is reconstructed from the Lyapunov argument because the paper is paywalled, so verify the constants before quoting them.

**Gain scheduling.** The expression is algebraically the Samson law with De Luca-style gains **k1 = k3 = 2ζ√(ω_r² + b v_r²), k2 = b|v_r|**. Global asymptotic stability holds by Lyapunov analysis ([Ramaiah et al. 2017](https://skoge.folk.ntnu.no/prost/proceedings/process-control-slovakia-2017/data/papers/028.pdf); [De Luca, Oriolo & Vendittelli](https://link.springer.com/doi/10.1007/3-540-45000-9_8)). The practical benefit is that only ζ ∈ [0.5, 0.9] and b need tuning. The same two numbers work across circle, figure-8 and random references, which makes the "fair tuning" argument easy.

**Torque level is unnecessary.** The torque-level backstepping of Fierro & Lewis is not needed for a robot driven through `cmd_vel`. Instead, add an optional first-order actuator lag to the simulated plant for all controllers.

### Kinematic SMC

**Why not Yang & Kim directly.** Yang & Kim's classic SMC is torque-level, combining computed-torque linearization with polar coordinates ([KAIST repository](https://koasas.kaist.ac.kr/handle/10203/12394)). A kinematic SMC is therefore a justified simplification, and the report should say so explicitly.

**Surfaces.** Use El Youssef et al.'s verified surfaces on the Kanayama error, **σ_v = k1 e_x and σ_w = k2 e_y + k3 sin(e_θ/2)**. Their sliding dynamics are stable for v_r > 0 ([El Youssef et al., IFAC 2014](https://skoge.folk.ntnu.no/prost/proceedings/ifac2014/media/files/2210.pdf)).

**Boundary layer.** Replace sgn(σ) with sat(σ/φ). A larger boundary layer reduces chattering but weakens robustness ([PMC11086255](https://pmc.ncbi.nlm.nih.gov/articles/PMC11086255)). On a TurtleBot3, Sawarkar et al. used switching gain K = 0.3 against a disturbance bound of 0.1 ([Sawarkar et al. 2026](https://arxiv.org/pdf/2604.24518)).

**Implementation.** The resulting control law (my derivation; verify it numerically) computes ω first and then v, and needs a guarded denominator. A φ sweep, measuring chattering as the number of sign changes of Δω per second, is a cheap and impressive ablation.

| Baseline | Law (kinematic) | Tunables | Known weak spot |
|---|---|---|---|
| Kanayama | v = v_r cos e_θ + k_x e_x; ω = ω_r + v_r(k_y e_y + k_θ sin e_θ) | k_x, k_y (k_θ = 2√k_y) | Loses convergence as v_r → 0 |
| Samson / backstepping | ω = ω_r + b v_r sinc(e_θ) e_y + k3 e_θ, gain-scheduled | ζ, b | Same v_r → 0 issue; global result needs persistent excitation |
| Kinematic SMC | Surfaces σ_v = k1 e_x, σ_w = k2 e_y + k3 sin(e_θ/2), sat boundary layer | k1–k3, K, φ | Chattering versus robustness trade-off |
| NMPC (proposed) | Constrained optimal control over horizon N | Q, R, S, P, N, γ | Compute, delay and model-mismatch sensitivity |

### Fairness rules

**Same conditions for everyone.** Every controller passes through the same saturation block (box limits plus wheel diamond). Every controller also runs at the same rate and receives the same state estimate.

**Same tuning budget.** Tune each controller with the same budget, for example a grid search or Optuna, on a held-out tuning trajectory set, then freeze the gains. That pre-empts the most common viva objection, "your baselines were under-tuned."

**Preview is inherent.** NMPC sees the reference over its horizon and the baselines see only the current sample. That difference is part of the methods, so state it rather than "fix" it.

**Obstacles are a separate experiment family.** Classical trackers have no notion of obstacles. Comparing NMPC-with-constraints against a baseline that must collide proves nothing. Instead, wrap each baseline in the same CBF-QP safety filter at a look-ahead point. That mirrors the SMC + collision-cone-CBF filter validated on TurtleBot3 hardware, whose output equals the nominal command whenever the constraint is inactive ([Sawarkar et al. 2026](https://arxiv.org/pdf/2604.24518)). Count deadlocks in front of symmetric obstacles as failures. Say plainly that NMPC's advantage there comes from prediction, not tracking quality.

## Paired Monte Carlo in a kinematic simulator, confirmed in Gazebo and on hardware

### Reference trajectories

**Flatness.** References come almost for free from flatness. The unicycle's (x, y) is a flat output ([Oriolo, De Luca & Vendittelli 2002](https://www.diag.uniroma1.it/%7Elabrob/pub/papers/CST02.pdf)). So any C²-smooth path yields θ_r = atan2(ẏ, ẋ), v_r = √(ẋ² + ẏ²) and ω_r = (ẋÿ − ẏẍ)/(ẋ² + ẏ²).

**Fixed families.** Use four:

- A circle: R = 1 m, ω₀ = 0.15 rad/s, giving v_r = 0.15 m/s.
- A Gerono figure-8: x = A sin ω₀t, y = (A/2) sin 2ω₀t. Peak speed is Aω₀√2 at the crossing, so A = 1.2 m and ω₀ = 0.1 rad/s give about 0.17 m/s.
- A sinusoid.
- A Lissajous curve: the TurtleBot3 Lissajous [2.2 + 1.8 sin(0.23πt), 1.5 + 0.95 cos(0.15πt)] has |v| ≤ 0.2 m/s ([Sawarkar et al. 2026](https://arxiv.org/pdf/2604.24518)).

These connect your results to the eight-shaped and butterfly tests in the literature ([Jin et al. 2024](https://arxiv.org/pdf/2403.07317)).

**Random trajectories.** Generate them reproducibly per seed:

1. Draw 5–8 waypoints in a 3×3 m arena, at least 0.4 m apart, with turns ≤ 120°.
2. Fit a quintic spline on chord length.
3. Set v(s) = min(0.8 v_max, 0.8 ω_max/|κ|) and apply a forward-backward acceleration pass.
4. Reject paths with excessive curvature or near self-intersection.

The 0.8 factor leaves feedback headroom that every controller needs.

**Point-to-point references.** These start and end at v = 0, where Kanayama and Samson lose convergence. Either report that as a finding or apply the same posture-regulation switch to all controllers.

### Where each experiment runs

**The arithmetic.** Gazebo runs at real-time factor 1. If you keep it at 1 (and you should, so solver latency counts), 6,000 runs of 60 s would take about 100 hours, which is impossible in two weeks. A pure-Python kinematic simulator built from the same `core/` code runs as fast as the solver allows. At about 20 ms per solve and 600 steps per run, a 60 s NMPC run takes about 12 s, so roughly 1,500 NMPC runs fit in about 5 hours overnight. The baselines cost almost nothing.

**Three tiers.** Use the kinematic simulator for the statistical bulk. Use Gazebo, headless `gzserver` with p3d ground truth, for a confirmatory subset of about 10–20 seeds per family plus all obstacle scenarios. Use hardware for a handful of demonstration runs with taped ground truth. These timings are my estimates and need confirming. The tiered design, though, is what makes "Monte Carlo quantification" achievable for a solo student.

### Protocol and statistics

**Paired seeds.** The design is paired: seed s fixes the random trajectory, initial offset and noise sequence, and every controller sees the same seed. Use M = 30–50 per cell. No standard prescribes this number; it is a defensible convention.

**Precedent.** GMPC's TurtleBot3 Monte Carlo randomized the initial position in [−0.2, 0]² m and heading in [−π/6, 0] ([Jin et al. 2024](https://arxiv.org/pdf/2403.07317)). Extend this symmetrically to ±0.2 m and ±π/6.

**Statistics.** Report median and IQR with 95% bootstrap CIs (`scipy.stats.bootstrap`). Test NMPC against each baseline with a paired Wilcoxon signed-rank test (`scipy.stats.wilcoxon`) and Holm correction across comparisons.

### Robustness sweeps

Apply all perturbations to the plant only, so controllers keep their nominal models:

- **Input delay:** 0, 1, 2 and 4 steps.
- **State noise:** σ_p up to 2 cm and σ_θ up to 0.05 rad, optionally with random-walk drift.
- **Model mismatch:** symmetric wheel-radius error of ±5–10% (a speed-scale error) and asymmetric ±2–5% (a turning bias, the more revealing case). Oriolo et al. calibrated wheel radii and axle length precisely because these dominate differential-drive mismatch ([CST 2002](https://www.diag.uniroma1.it/%7Elabrob/pub/papers/CST02.pdf)).
- **Initial offsets:** up to 0.4 m and 2π/3.

Plot median RMSE with an IQR band against perturbation level, together with success rate (failure means max error above 0.5 m). These "degradation curves" are the most persuasive figure in the report. They show where NMPC's model dependence hurts and where SMC's matched-disturbance robustness ([El Youssef et al. 2014](https://skoge.folk.ntnu.no/prost/proceedings/ifac2014/media/files/2210.pdf)) pays off.

### Metrics

| Category | Metrics |
|---|---|
| Tracking | RMSE position, max position error, RMSE heading (wrapped), lateral e_y and along-track e_x separately, IAE/ITAE, settling time to 2 cm |
| Effort / smoothness | Σ(v² + λω²)Δt, Σ‖Δu‖², total variation, chattering index (Δω sign changes/s) |
| Constraints | % of steps saturated (pre-clip ≠ post-clip), wheel-diamond violations |
| Safety (obstacle family) | Min clearance, collisions, success rate, deadlocks, path-length ratio, tracking error outside obstacle neighbourhoods |
| Computation | Solve time mean/median/p95/p99/max, deadline misses (t_solve > Δt), iterations, failure statuses, CPU model, first solve reported separately |

**Figures.** Use one XY overlay per trajectory family; time series of error and inputs with the limits dashed; box or violin plots per condition; an empirical CDF of solve time with a vertical line at Δt; and a summary table of median [IQR].

**Obstacle ablation.** For the obstacle family, run Zeng et al.'s own ablation on your robot: DC-hard, DC-soft, CBF with γ ∈ {0.05, 0.1, 0.2, 0.3, 0.5, 1.0}, and potential field, each at N = 10 and N = 20 ([Zeng et al. 2021](https://arxiv.org/pdf/2007.11718)). Make the Gazebo obstacles cylinders so they match the circle model exactly, and spawn them per seed with `/spawn_entity`.

### Two-week schedule

| Days | Deliverable | Exit criterion |
|---|---|---|
| 1–2 | `core/`: unicycle model, reference generators, the three baselines, kinematic simulator, metrics module | Baselines track the figure-8 in the Python sim; unit tests pass |
| 3–4 | NMPC (Opti → `to_function`, warm start, fallback); obstacle-free N sweep | p95 solve time < 100 ms at N = 20 |
| 5 | DC/CBF/slack obstacle constraints; CBF-QP filter for baselines | γ sweep runs on synthetic circles |
| 6–7 | ROS 2 nodes, LiDAR circle extractor, Gazebo world and spawner, p3d, CSV logging | Closed loop in Gazebo with obstacles |
| 8 | Overnight kinematic-sim Monte Carlo (tracking + robustness sweeps) | Full results table generated by script |
| 9–10 | Hardware: network, chrony, cmd_vel type check, safety relay, latency step test, delay compensation | Safe figure-8 runs on the real robot |
| 11 | Gazebo confirmatory subset + obstacle Monte Carlo; hardware demo runs filmed | Sim/hardware comparison table |
| 12–14 | Plots, statistics, report, video edit, viva prep | Every claim traceable to a scripted figure |

## Conclusion

The central decision in this project is how to allocate the two weeks, not which algorithm to use. NMPC on a 0.22 m/s robot is computationally easy. The things that separate a strong submission from a weak one are the fairness rules, the paired seeds, the degradation curves and the honest separation of tracking from obstacle avoidance. Two observations make the comparison sharper than a naive "NMPC wins" story. Kinematic backstepping and the Samson law are the same controller, so claiming four distinct baselines would invite a sharp viva question. The obstacle advantage, in turn, comes from prediction and constraints, not tracking accuracy. Expect the baselines to match or beat NMPC on smooth, obstacle-free references with small offsets. That is consistent with the simulation comparison of Ramaiah et al., although their garbled results table makes the reading uncertain. NMPC should pull ahead under large offsets, input saturation, stop-and-go references and obstacles. If it does not, that is itself a defensible result.

The biggest residual uncertainties are empirical: actual IPOPT solve times on your laptop (and on a Pi 4, if you try onboard solving), the real `/odom` rate, Wi-Fi delay distributions, and the LDS-02 invalid-range encoding. None of these has a published TB3 Humble figure. Treat measuring them in the first days of hardware work (days 9–10 in the schedule) as a deliverable in its own right. Each becomes a small, citable measurement in your report, and each turns a "should work" assumption into evidence an examiner can probe.
