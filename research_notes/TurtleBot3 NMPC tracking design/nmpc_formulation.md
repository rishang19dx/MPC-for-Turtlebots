# NMPC Formulation and Solver Choices for TurtleBot3 Burger Trajectory Tracking with Obstacle Avoidance (Python + CasADi)

Scope note: about 17 tool calls were used. Primary sources fetched: the ROBOTIS e-manual features page, the TB3 `burger.yaml` (Humble branch), the CasADi user guide (Opti stack and shooting sections), the acados GitHub README, and the full text of Zeng/Zhang/Sreenath MPC-CBF (arXiv 2007.11718, which has tables and parameters). Gu & Hu 2006 and Mehrez et al. 2013 were confirmed only from their abstracts and metadata. The IPOPT options page and the acados docs site could not be fetched, because the connection failed. Items marked **[background, not re-verified this session]** are standard textbook material that I did not re-check against a fetched source. Treat them as reliable starting points, but confirm them before citing in a report.

---

## 1. Model, discretization, horizon, shooting method, warm start, solver config (IPOPT/acados)

### Takeaway
Use the 3-state unicycle model x=[p_x, p_y, θ], u=[v, ω], discretized with RK4 (or the exact unicycle map) at Δt = 0.1 s, with N ≈ 15–25 (1.5–2.5 s lookahead). Solve it with CasADi multiple shooting through the Opti stack, compiled to a `casadi.Function` with `opti.to_function` and warm-started with the previous primal and dual solution. IPOPT is enough at 10 Hz for this small problem. acados (SQP-RTI + HPIPM) is faster but probably not worth the integration cost for a 2-week solo project, unless solve times turn out above about 50 ms.

### Cited Findings
- TB3 Burger spec: max translational velocity "0.22 m/s", max rotational velocity "2.84 rad/s (162.72 deg/s)". Size is 138 × 178 × 192 mm, weight 1 kg, and the LiDAR is the "360 Laser Distance Sensor LDS-02". — [ROBOTIS e-manual, Features](https://emanual.robotis.com/docs/en/platform/turtlebot3/features/)
- The TB3 Burger driver parameters in ROS 2 Humble give `wheels: separation: 0.160, radius: 0.033`. The file has no velocity limits. The motor acceleration entries are `profile_acceleration_constant: 214.577` and `profile_acceleration: 0.0` [rev/min²], so the firmware applies no explicit acceleration profile by default. — [turtlebot3 burger.yaml (humble)](https://raw.githubusercontent.com/ROBOTIS-GIT/turtlebot3/humble/turtlebot3_node/param/burger.yaml)
- CasADi Opti stack API: `opti.variable(n,m)`, `opti.parameter()` + `opti.set_value(p, val)` (the value can be overwritten at any time), `opti.set_initial(x, val)` (when no initial guess is given, "numerical zero is assumed"), and `opti.solver("ipopt", p_opts, s_opts)`, where p_opts are CasADi plugin options and s_opts are IPOPT options such as `{"max_iter": 100}`. — [CasADi docs](https://web.casadi.org/docs/)
- Warm starting in CasADi: "Consecutively calling solve will not help the convergence." To warm start, use `opti.set_initial(sol1.value_variables())`. In the docs example this cuts iterations from 15 to 4. Duals can be warm-started with `lam_g0 = sol.value(opti.lam_g); opti.set_initial(opti.lam_g, lam_g0)`. — [CasADi docs](https://web.casadi.org/docs/)
- `opti.to_function("f", [inputs], [outputs])` produces a CasADi Function. Its variable inputs act as initial guesses, and the docs say it "will be quite a bit faster than opti.set_initial + opti.solve + sol.value". The `error_on_fail` solver option makes the function throw when the solve fails. — [CasADi docs](https://web.casadi.org/docs/)
- Debugging tools: `opti.debug.value(x)` returns the non-converged iterate, and `opti.debug.show_infeasibilities()` lists the offending constraints. These are useful when obstacle constraints make the problem infeasible. — [CasADi docs](https://web.casadi.org/docs/)
- Multiple shooting "includes the state at certain shooting nodes as decision variables in the NLP and includes equality constraints to ensure continuity of the trajectory". CasADi's single-shooting example uses "a simple fixed-step Runge-Kutta 4 (RK4) integrator ... implemented using CasADi symbolics". Reference examples are `direct_multiple_shooting.py` and `direct_single_shooting.py` in the CasADi examples pack. — [CasADi docs](https://web.casadi.org/docs/)
- `mx_function.expand()` converts MX to SX, which "might speed up the calculations significantly" at some memory cost. In nlpsol/Opti this is the plugin option `{"expand": True}`. CasADi can also generate C code (`f.generate('gen.c')`) and JIT-compile it. — [CasADi docs](https://web.casadi.org/docs/)
- acados is "specifically designed for real-time applications and embedded systems". It offers multiple shooting, integrators with sensitivities, "fast SQP-type solvers tailored for optimal control problems", structure-exploiting QP solvers such as HPIPM, a Python interface (`AcadosOcp`), and models written in CasADi symbolics. The README gives no quantitative timing claims. — [acados GitHub](https://github.com/acados/acados)
- Mehrez, Mann & Gosine (ICAR 2013) ran NMPC for point stabilization and trajectory tracking of nonholonomic wheeled robots in real time, using the open-source ACADO toolkit (C++) with stabilizing terminal equality constraints in full-scale experiments. — [Mehrez et al. 2013 (ResearchGate PDF)](https://researchgate.net/profile/Mohamed-Mehrez/publication/271549212_Stabilizing_NMPC_of_wheeled_mobile_robots_using_open-source_real-time_software/links/56e945ee08aef53a12bd8369/Stabilizing-NMPC-of-wheeled-mobile-robots-using-open-source-real-time-software.pdf); [researchr entry](https://researchr.org/publication/MehrezMG13)
- Mehrez's CasADi workshop repository implements MPC and MHE "in MATLAB using CasADi" with "a non-holonomic mobile robot" as the example system. I could not retrieve the exact per-file parameter values (T, N, Q, R, IPOPT options) from the landing page, and the raw file path returned 404. — [MPC-and-MHE CasADi workshop](https://github.com/MMehrez/MPC-and-MHE-implementation-in-MATLAB-using-Casadi)
- Salimi Lafmejani et al. (RAS 2021) formulate nonholonomic multi-robot NMPC "without stabilizing terminal constraints or terminal costs to reduce computational complexity". They use multiple shooting and open-source fast nonlinear optimization tools. — [Salimi Lafmejani et al. 2021](https://faculty.engineering.asu.edu/acs/wp-content/uploads/sites/33/2021/03/Salimi-Lafmejani-RAS-2021-Nonlinear-MPC-for-Collision-Free-and-Deadlock-Free-Navigation-of-Multiple-Nonholonomic-Mobile-Robots.pdf)
- Solve-time evidence is sparse. The only reported IPOPT timing found was in MPC-CBF: a 2D double integrator (4 states, 2 inputs, 1 obstacle, Δt = 0.2 s) solved in 0.028 ± 0.011 s at N = 5, and with distance constraints in 0.033 s at N = 7, 0.048 s at N = 15 and 0.062 s at N = 30. — [Zeng et al. 2021, Table I](https://arxiv.org/pdf/2007.11718)
- A generic tutorial claims 5–20 ms for CasADi/IPOPT MPC, but it names no model, horizon or hardware, so treat the figure as unverified. PYROBOCOP's benchmark gives CasADi+IPOPT times of 0.058 s (pendulum), 0.353 s (quadrotor) and 1.175 s (acrobot), but these are full trajectory optimizations, not warm-started MPC steps. — [kindatechnical tutorial](https://kindatechnical.com/autonomous-systems/trajectory-optimization-and-model-predictive-control.html); [PYROBOCOP arXiv 2106.03220](https://arxiv.org/pdf/2106.03220)

### Inferences
- **Model** [background, not re-verified this session]: continuous dynamics ṗ_x = v cos θ, ṗ_y = v sin θ, θ̇ = ω.
  - Wheel mapping with L = 0.160 m and r = 0.033 m: v = r(ω_R + ω_L)/2 and ω = r(ω_R − ω_L)/L.
  - Coupling check: with a wheel rim speed limit v_w,max = 0.22 m/s, pure rotation gives ω_max = 2·0.22/0.160 = 2.75 rad/s. That is close to the 2.84 rad/s spec. Both limits are wheel-speed limits, so v and ω cannot be at their maxima at the same time.
  - Optional polytopic constraint for physical consistency: |v| + (L/2)|ω| ≤ 0.22–0.227 m/s. In practice, many students use box limits v ∈ [0, 0.22] (or [−0.22, 0.22]) and |ω| ≤ 2.84. A conservative choice is ω ∈ [−1.5, 1.5] for smoother tracking.
- **Discretization**:
  - Forward Euler (x+ = x + Δt f(x,u)) is what the Mehrez workshop popularized. It is adequate at Δt = 0.1 s because the TB3 is slow: at 0.22 m/s it covers 2.2 cm per step.
  - RK4 costs about 4 times as many symbolic evaluations but stays cheap after SX expansion, and it is more accurate at high ω.
  - The exact zero-order-hold unicycle map, x+ = x + (v/ω)[sin(θ+ωΔt) − sin θ], y+ = y − (v/ω)[cos(θ+ωΔt) − cos θ], θ+ = θ + ωΔt, is singular at ω → 0 and needs a branch or Taylor patch, which complicates the NLP.
  - Recommendation: use RK4 with one step per interval, or the midpoint rule θ_mid = θ + ωΔt/2 (x+ = x + vΔt cos θ_mid, ...). The midpoint rule is exact in heading and very close to exact in position.
- **Δt and N**:
  - Δt = 0.1 s matches 10 Hz control and the LDS-02 scan rate (about 5–10 Hz). This should be confirmed.
  - Horizon T = NΔt should cover roughly the stopping and avoidance distance plus the reference curvature preview. At 0.22 m/s, N = 20 gives 2 s, or 0.44 m of lookahead. Obstacle reaction requires the horizon to "see" an obstacle about 0.3–0.5 m ahead, so N = 15–30 is sensible.
  - Per the MPC-CBF table, the cost of distance-constraint MPC grows with N, which is the motivation for testing CBF constraints at shorter N.
- **Single vs multiple shooting**: Use multiple shooting, with decision variables X (3×(N+1)) and U (2×N) plus continuity constraints. It is better conditioned for the nonlinear unicycle, makes state constraints (obstacles) direct functions of decision variables, and warm-starts naturally by shifting X and U. Single shooting has fewer variables but nests nonlinearity through the rollout, so the obstacle constraints become highly nonlinear in U.
- **Opti vs nlpsol**: Opti is easiest to write and debug. For runtime, either call `opti.to_function` once or build the problem with `nlpsol` directly, as in the Mehrez style. The nlpsol approach stacks everything into a single vector w with lbx/ubx/lbg/ubg and passes p = [x0; x_ref(0..N); u_ref; obstacles] as one parameter vector. Both give identical IPOPT problems. Opti's per-solve Python overhead (set_value, set_initial, solve) can add a few ms, which `to_function` avoids.
- **Warm start** (shift strategy):
  - X_init = [X*(:,1:N), X*(:,N)] and U_init = [U*(:,1:N−1), U*(:,N−1)].
  - Also pass the previous lam_g and lam_x, and set IPOPT `warm_start_init_point: "yes"`. With interior-point methods, dual warm starts help less than with SQP, so lower `mu_init` (for example 1e-3) and the warm-start bound pushes (for example 1e-6) to get a real benefit. These option names are standard IPOPT, but the IPOPT options page could not be fetched to confirm defaults.
- **IPOPT options** (suggested starting point [background, not re-verified this session]): `{"ipopt.max_iter": 100, "ipopt.tol": 1e-4, "ipopt.acceptable_tol": 1e-3, "ipopt.acceptable_iter": 5, "ipopt.print_level": 0, "print_time": 0, "ipopt.sb": "yes", "ipopt.max_wall_time": 0.08, "expand": True, "ipopt.linear_solver": "mumps"}`.
  - The workshop-style settings are max_iter 2000, print_level 0, acceptable_tol 1e-8, and acceptable_obj_change_tol 1e-6. Those are fine offline but too generous for a real-time guarantee.
  - Always check `solver.stats()["return_status"]`. On failure, apply the shifted previous solution, i.e. the second control of the previous plan, or brake.
- **Expected timing**: from the MPC-CBF table (about 28–62 ms for a 4-state problem with 1 obstacle on unspecified hardware), a 3-state unicycle with N = 20 and up to about 10 obstacles × N constraints should run on the order of 10–50 ms per warm-started solve on a modern laptop. This is extrapolated, so benchmark it yourself.
- **acados**: SQP-RTI with HPIPM and generated C code typically runs in sub-ms to a few ms for this size [background, not re-verified this session]. It is worth it only if IPOPT misses the 100 ms budget or you want a "solver ablation" section. Its Python template (`acados_template`) requires building acados from source with CMake and setting `ACADOS_SOURCE_DIR`, which is about half a day of setup. For a 2-week project, implement in CasADi+IPOPT first and treat acados as an optional stretch.

### Gaps
- I could not fetch the IPOPT options documentation (connection failed), so the defaults for `tol`, `acceptable_tol`, `mu_init` and the `warm_start_*` options are not verified here. Check https://coin-or.github.io/Ipopt/OPTIONS.html.
- No published solve-time benchmark was found for CasADi/IPOPT on a unicycle with N ≈ 20 at 10 Hz. Benchmark it yourself.
- I did not verify the LDS-02 range and scan frequency or the acceleration limits from ROBOTIS, because the specifications page returned 404. No official acceleration limit for the Burger was found; `burger.yaml` sets no profile acceleration.
- I could not retrieve the exact Mehrez workshop parameters (T, N, Q, R).

---

## 2. Cost design (world-frame vs error-frame, angle wrapping, Δu, terminal ingredients, feedforward)

### Takeaway
Penalize tracking error in the robot-aligned (Kanayama) error frame, or the world-frame position error plus a wrap-safe heading error such as 1 − cos(e_θ) or the atan2(sin, cos) form. Penalize deviation from the reference feedforward (v_r, ω_r) instead of raw u, and add a Δu term for smoothness. For terminal ingredients, a terminal cost (scaled Q or a Lyapunov-based weight) is enough in practice. Gu & Hu 2006 is the citable stability result, using a terminal penalty plus a terminal region.

### Cited Findings
- Gu & Hu (IEEE TCST 14(4):743–749, 2006, DOI 10.1109/TCST.2006.872512) prove that for receding-horizon tracking of a nonholonomic robot, "control stability is guaranteed by adding a terminal-state penalty to the cost function and constraining the terminal state to a terminal-state region". Inside that region they find a "virtual controller", and the scheme has "simultaneous tracking and regulation capability". A related result by the same authors (IEEE T-RO 2005) says a suboptimal solution is sufficient for stability. — [Gu & Hu 2006 abstract via Bohrium](https://www.bohrium.com/paper-details/receding-horizon-tracking-control-of-wheeled-mobile-robots/812073680136830977-2483); full text not accessed
- Mehrez et al. 2013 used terminal equality constraints for stabilizing NMPC on a real wheeled robot. — [Mehrez et al. 2013](https://researchgate.net/profile/Mohamed-Mehrez/publication/271549212_Stabilizing_NMPC_of_wheeled_mobile_robots_using_open-source_real-time_software/links/56e945ee08aef53a12bd8369/Stabilizing-NMPC-of-wheeled-mobile-robots-using-open-source-real-time-software.pdf)
- Practical alternative: drop terminal constraints and terminal costs altogether to save computation, as Salimi Lafmejani et al. 2021 do. There is also a theory line on MPC without terminal conditions for differential-drive robots (Rußwurm et al.). — [Salimi Lafmejani 2021](https://faculty.engineering.asu.edu/acs/wp-content/uploads/sites/33/2021/03/Salimi-Lafmejani-RAS-2021-Nonlinear-MPC-for-Collision-Free-and-Deadlock-Free-Navigation-of-Multiple-Nonholonomic-Mobile-Robots.pdf); [Rußwurm et al. entry](https://www.bohrium.com/paper-details/on-mpc-without-terminal-conditions-for-dynamic-non-holonomic-robots/811832367118286848-40126)
- MPC-CBF uses quadratic stage and terminal costs, q = xᵀQx + uᵀRu and p = x_Nᵀ P x_N, with Q = 10·I, R = I and P = 100·I. This is a 10× terminal-weight convention you can reuse. In the racing example, the cost tracks a reference state, (x − x_r)ᵀQ(x − x_r) + uᵀRu, with Q = diag(10, 0, 0, 0, 0, 10) and R = diag(1, 1). — [Zeng et al. 2021](https://arxiv.org/pdf/2007.11718)

### Inferences
- **Kanayama error frame** [background: Kanayama et al., "A stable tracking control method for an autonomous mobile robot", ICRA 1990; not re-verified this session]: [e_x; e_y; e_θ] = R(θ)ᵀ [x_r − x; y_r − y; θ_r − θ].
  - In expanded form: e_x = cos θ (x_r − x) + sin θ (y_r − y), e_y = −sin θ (x_r − x) + cos θ (y_r − y), e_θ = θ_r − θ.
  - e_x is the along-track (longitudinal) error and e_y the lateral error. They can carry different weights, for example a higher weight on lateral error, which world-frame errors cannot express.
  - The Kanayama control law v = v_r cos e_θ + K_x e_x, ω = ω_r + v_r(K_y e_y + K_θ sin e_θ) is a good **baseline controller** to compare against NMPC.
- **Angle wrapping**:
  - Never use (θ − θ_r)² directly with unwrapped odometry, because a jump at ±π gives a huge cost spike.
  - Option A: e_θ = atan2(sin(θ_r − θ), cos(θ_r − θ)), which CasADi supports (smooth except at ±π).
  - Option B: penalize 2(1 − cos(θ − θ_r)), which is smooth everywhere and ≈ e_θ² for small errors.
  - Option C: keep θ as a continuous, unwrapped state inside the MPC and unwrap the reference sequence to be continuous with the current θ, shifting it by 2πk before passing it in.
  - B or C is recommended. C keeps the cost quadratic, and B is robust.
- **Recommended stage cost**:
  - ℓ = e_pᵀQ_p e_p + q_θ(1 − cos e_θ)·2 + (u − u_r)ᵀR(u − u_r) + Δuᵀ S Δu, with Δu_k = u_k − u_{k−1} and u_{−1} = the previously applied control, passed as a parameter.
  - Starting weights for the TB3 scale (errors around 0.1 m, v around 0.2): Q = diag(10, 10, 1–5) [x, y, θ], R = diag(0.5, 0.05) [v, ω], S = diag(1, 0.1).
  - Terminal weight P = 5–10 × Q. All of these are tuning starting points, not sourced values.
- **Feedforward**: build the reference (x_r, y_r, θ_r, v_r, ω_r) from the desired trajectory (for example a figure-8 or circle) using v_r = √(ẋ_r² + ẏ_r²) and ω_r = (ẋ_r ÿ_r − ẏ_r ẍ_r)/v_r². Penalizing u − u_r rather than u stops the controller from fighting the reference motion, since nonzero velocity is required to track a moving reference.
  - Keep the reference within limits: v_r < 0.22 and ω_r < 2.84, ideally at about 70% of the limits to leave margin for correction. For example, a figure-8 with peak v_r of 0.15 m/s.
- **Terminal ingredients for a course report**: present Gu & Hu-style theory (terminal penalty plus terminal region with a local stabilizing "virtual" controller) as the stability argument.
  - Implement a terminal cost P only, with no terminal set constraint, which would hurt feasibility around obstacles. Justify this through the long-horizon / no-terminal-conditions literature.
  - Optional ablation: add a terminal set ‖p_N − p_r,N‖ ≤ ε and compare feasibility rates.
- Reference text for the theory [background, not re-verified this session]: Rawlings, Mayne & Diehl, *Model Predictive Control: Theory, Computation, and Design*, 2nd ed., Nob Hill, 2017 (free PDF from the authors), Ch. 2 (terminal cost/set stability) and Ch. 8 (numerical optimal control, multiple shooting). Also Mayne, Rawlings, Rao & Scokaert, "Constrained model predictive control: Stability and optimality", Automatica 36(6), 2000.

### Gaps
- I did not access the full text of Gu & Hu 2006, so the exact terminal-region shape, the weight conditions and the virtual controller form are unverified.
- I did not fetch Kanayama 1990 directly. The error-frame equations above are standard but come from background knowledge.

---

## 3. Hard constraints: TB3 Burger limits

### Takeaway
The ROBOTIS e-manual confirms v_max = 0.22 m/s and ω_max = 2.84 rad/s. No official acceleration limit is published for the Burger, so pick conservative acceleration and rate limits yourself and enforce them through Δu constraints.

### Cited Findings
- Max translational velocity is 0.22 m/s and max rotational velocity is 2.84 rad/s. — [ROBOTIS e-manual](https://emanual.robotis.com/docs/en/platform/turtlebot3/features/)
- Wheel separation is 0.160 m and wheel radius 0.033 m. The default config sets no profile acceleration (`profile_acceleration: 0.0`). — [burger.yaml](https://raw.githubusercontent.com/ROBOTIS-GIT/turtlebot3/humble/turtlebot3_node/param/burger.yaml)

### Inferences
- Constraints to implement:
  - 0 ≤ v_k ≤ 0.22. Allow v ≥ −0.22 only if reversing is acceptable; forward-only simplifies obstacle reasoning because the LiDAR covers 360° anyway.
  - |ω_k| ≤ 2.84.
  - |v_k − v_{k−1}| ≤ a_max Δt with a_max ≈ 0.5 m/s².
  - |ω_k − ω_{k−1}| ≤ α_max Δt with α_max ≈ 3 rad/s². These acceleration values are engineering choices, not sourced.
  - Optional wheel-speed polytope: |v| + 0.08|ω| ≤ 0.22.
- State bounds: an arena box, for example x, y ∈ [−2, 2] m in Gazebo. θ stays unbounded.

### Gaps
- There is no official TB3 Burger linear or angular acceleration limit. The DYNAMIXEL XL430 profile acceleration semantics were not investigated.

---

## 4. Obstacle avoidance inside MPC: distance constraints, MPC-CBF, potential-field costs, LiDAR circles as parameters

### Takeaway
Represent obstacles as up to M circles (c_x, c_y, r) passed as a fixed-size parameter vector, with unused slots filled by dummy far-away obstacles. Implement (a) MPC-DC hard distance constraints, (b) the same with slack variables (soft, using an L1 penalty), and (c) discrete-time CBF constraints h(x_{k+1}) ≥ (1 − γ)h(x_k). The comparison between (a), (b) and (c) can be the main ablation. Per Zeng et al., CBF constraints give earlier, smoother avoidance at shorter horizons, but small γ risks infeasibility, so pair them with slacks.

### Cited Findings
- **Discrete-time CBF condition**: Δh(x_k, u_k) ≥ −γ h(x_k) with 0 < γ ≤ 1, equivalent to h(x_{k+1}) ≥ (1 − γ) h(x_k). The lower bound of h decreases exponentially at rate 1 − γ. — [Zeng et al. 2021](https://arxiv.org/pdf/2007.11718)
- **MPC-CBF problem**: the standard MPC problem plus the constraint Δh(x_{t+k|t}, u_{t+k|t}) ≥ −γ h(x_{t+k|t}) for k = 0, …, N − 1, i.e. along the whole horizon. These "constraints ... are generally non-convex", so they used IPOPT. — [Zeng et al. 2021](https://arxiv.org/pdf/2007.11718)
- **Barrier for a circular obstacle**: h = (x − x_obs)² + (y − y_obs)² − r_obs². The MPC-DC baseline uses the same expression as a distance constraint g(x_k) ≥ 0. — [Zeng et al. 2021](https://arxiv.org/pdf/2007.11718)
- **Relation between the two**: as γ → 1, the CBF constraint becomes h(x_{k+1}) ≥ 0, i.e. MPC-DC shifted by one step. MPC-CBF's feasible set R_k ∩ S_cbf,k is a subset of MPC-DC's R_k ∩ C, which makes MPC-CBF "safer". Distance constraints are inactive until the reachable set touches the obstacle, whereas CBF constraints act earlier. — [Zeng et al. 2021](https://arxiv.org/pdf/2007.11718)
- **Feasibility tradeoff**: "When γ becomes relatively small ... [it is] more safer, but this might also make the optimization infeasible. A larger γ will make the optimization more likely to be feasible, but the CBF constraints might not be active." For fixed γ "we generally only have pointwise feasibility and a persistently feasible formulation is still an open problem". Recursive feasibility is not guaranteed for MPC-DC either. — [Zeng et al. 2021](https://arxiv.org/pdf/2007.11718)
- **Table I results** (double integrator, Δt = 0.2 s, Q = 10I, R = I, P = 100I, obstacle r = 1.5 m):

  | Controller | N | γ | Status | Solve time | Min distance |
  |---|---|---|---|---|---|
  | MPC-CBF | 5 | 0.1 | solved | 0.028 s | 1.483 |
  | MPC-CBF | 5 | 0.2 | solved | 0.028 s | 0.791 |
  | MPC-CBF | 5 | 0.3 | solved | 0.028 s | 0.441 |
  | MPC-CBF | 5 | 0.4 | solved | 0.028 s | 0.288 |
  | MPC-CBF | 5 | 0.5 | solved | 0.028 s | 0.110 |
  | MPC-DC | 5 | – | **infeasible** | – | – |
  | MPC-DC | 7 | – | solved | 0.033 s | 0.000 |
  | MPC-DC | 15 | – | solved | 0.048 s | 0.000 |
  | MPC-DC | 30 | – | solved | 0.062 s | 0.000 |

  "MPC-DC always reaches to the boundary of the obstacle", while MPC-CBF "could automatically set a safety margin depending on ... γ". MPC-CBF with N = 5 and γ = 0.25 behaves like MPC-DC with N = 7. — [Zeng et al. 2021](https://arxiv.org/pdf/2007.11718)
- In the racing example, MPC-CBF ran with N = 12 at 10 Hz (Δt = 0.1 s) on linearized dynamics, using a quartic (superellipse) CBF for rectangular cars. — [Zeng et al. 2021](https://arxiv.org/pdf/2007.11718)
- The γ = 0.1, 0.2, 0.3 and 1.0 sweep and the N sweep are the paper's own ablation design. — [Zeng et al. 2021](https://arxiv.org/pdf/2007.11718)
- **Follow-up work**: a discrete-time high-order CBF with iterative convex optimization (Liu, Zeng, Sreenath, Belta) linearizes the CBF constraints into a sequence of QPs, which is relevant for an LTV/QP variant. — [arXiv 2210.04361](https://arxiv.org/pdf/2210.04361v1)
- **Ellipsoidal obstacles**: robot and obstacles modeled as ellipsoids with an overlap test integrated into MPC, run in real time on hardware. This is an option for elongated obstacles. — [arXiv 2412.11552](https://arxiv.org/abs/2412.11552)

### Inferences
- **Barrier for the TB3**: h_j(x) = (p_x − c_{x,j})² + (p_y − c_{y,j})² − (r_j + r_robot + d_safe)², with r_robot ≈ 0.105 m (half the 178 mm width, padded) and d_safe ≈ 0.05–0.10 m.
  - The squared form is smooth. The non-squared ‖p − c‖ − r has a gradient singularity at the center, but that point is never reached.
  - The constraint depends only on position, so for the unicycle it has relative degree 1 with respect to v, which is fine in discrete time because it is just a constraint on x_{k+1}. However, ω does not appear in h(x_{k+1}) after one Euler step, only after two. With small γ the robot may slow down rather than steer. A common fix is to evaluate h at a point offset ahead of the axle, p + ℓ[cos θ, sin θ] with ℓ ≈ 0.05–0.1 m, so that heading enters h directly. This is standard practice and worth an ablation.
- **Soft constraints with slacks**: h_j(x_{k+1}) ≥ (1 − γ) h_j(x_k) − s_{j,k}, with s ≥ 0 and cost ρ·Σs (L1, ρ ≈ 1e3–1e4) plus optionally a quadratic term.
  - An exact L1 penalty keeps the constraint hard whenever a feasible solution exists, while avoiding IPOPT failures when the obstacle appears suddenly or the robot is already inside the inflated radius due to sensor noise. Exact penalty theory is [background, not re-verified this session].
  - A cheaper variant uses one slack per obstacle instead of per (j, k).
- **CBF horizon truncation**: apply CBF constraints only for k < N_cbf (for example 5–10) to cut the constraint count. Use plain distance constraints or none beyond that. This is an inference, not stated in Zeng et al.
- **Potential-field cost**: J_obs = Σ_k Σ_j w / (h_j(x_k) + ε), or w·exp(−β h_j). It is easy and always feasible but gives no guarantee, needs tuning, can create local minima and oscillation in corridors, and still collides if the tracking weight dominates. It is a good third arm of the ablation (soft-only vs hard DC vs CBF).
- **LiDAR to circles**:
  - Pipeline: /scan (LaserScan) → points in the odom/map frame via TF → Euclidean clustering by jump distance (gap > 0.1 m starts a new cluster) → minimum enclosing circle per cluster, or split long clusters (walls) into chains of small circles of radius about 0.1 m → keep the M nearest (for example M = 8–10) within the horizon reach of about 1–1.5 m.
  - Pass them as a parameter of shape 3 × M, padded with dummy obstacles (c = (1e3, 1e3), r = 0.1) so the NLP structure stays fixed and no rebuild is needed.
  - Also pass an "active" mask, or just rely on the far dummies, whose constraints are trivially inactive.
  - Static obstacles in world frame are simplest. For a known moving obstacle, pass a velocity and propagate c_j(k) = c_j + k Δt v_j.
- **Constraint count**: M·N, for example 10 × 20 = 200 inequality constraints. That is fine for IPOPT, and per the MPC-CBF table runtime is dominated by N.

### Gaps
- No unicycle-specific MPC-CBF results (γ, N, timing) were found; the paper uses a double integrator and a car. The offset-point CBF trick, the LiDAR clustering parameters and the slack weights are engineering inferences without a fetched source.
- I did not fetch a primary source on potential-field-in-MPC cost pros and cons.

---

## 5. Linear/LTV MPC (QP via OSQP) as a cheaper alternative and ablation

### Takeaway
Linearizing the error dynamics about the reference trajectory gives an LTV QP. You can solve it with OSQP, CasADi's `conic`/`qpsol`, or Opti created with `'conic'`, at roughly an order of magnitude lower cost. It degrades with large initial heading errors and needs linearized (half-space) obstacle constraints, which makes it a good "NMPC vs LTV-MPC" ablation.

### Cited Findings
- CasADi's Opti stack assumes an NLP by default, and passing `'conic'` to the constructor selects quadratic programs, so the same code structure can host the QP. — [CasADi docs](https://web.casadi.org/docs/)
- The MPC-CBF racing example deliberately "use[s] the linearized dynamics along the centerline to formulate our control design" at N = 12 and 10 Hz, which shows that linearized-model MPC with CBFs is a standard choice. — [Zeng et al. 2021](https://arxiv.org/pdf/2007.11718)
- Iterative convex (QP) handling of discrete-time CBFs is in Liu, Zeng, Sreenath & Belta. — [arXiv 2210.04361](https://arxiv.org/pdf/2210.04361v1)

### Inferences
- **LTV model** [background, not re-verified this session]: linearize the Euler map about (x_r,k, u_r,k), giving x̃_{k+1} = A_k x̃_k + B_k ũ_k.
  - A_k = [[1, 0, −v_r Δt sin θ_r], [0, 1, v_r Δt cos θ_r], [0, 0, 1]].
  - B_k = [[Δt cos θ_r, 0], [Δt sin θ_r, 0], [0, Δt]].
  - Controllability is lost when v_r = 0, so LTV-MPC cannot do point stabilization. NMPC also struggles there, per Brockett's condition.
- **Obstacles in the QP**: linearize h about the previous predicted trajectory to get a half-space n_jᵀ p_k ≥ n_jᵀ c_j + r_j, with n_j the unit vector from c_j to the predicted p_k. Apply it as a DC constraint or in linearized CBF form, with slacks.
- **Solver**: OSQP via the `osqp` Python package, or `ca.conic('S', 'osqp', ...)` / `qpOASES` through CasADi. Warm start with `prob.update(q=..., l=..., u=..., Ax=...)`. Expect solve times of sub-ms to a few ms. This is unverified, so measure it.
- **Ablation metrics**: RMS position and heading error, max lateral error, solve time (mean / p95 / max), number of infeasible or failed solves, minimum obstacle clearance, and control effort Σ‖u‖²Δt (the same metric as MPC-CBF's "cost integral").

### Gaps
- No direct source was fetched on OSQP timing for unicycle LTV-MPC, or on a published head-to-head NMPC vs LTV-MPC comparison for a differential drive.

---

## 6. Recommended ablation studies

### Takeaway
Run three axes, mirroring Zeng et al.'s Table I design: horizon N, obstacle formulation (DC vs CBF-γ vs soft potential), and weights (Q/R/S). Add solver and model axes (IPOPT NMPC vs LTV-QP, Euler vs RK4) if time allows. Report solve-time statistics alongside tracking and safety metrics.

### Cited Findings
- Zeng et al. benchmark the following:
  - MPC-CBF at fixed N = 5 with γ ∈ {0.1, 0.2, 0.3, 0.4, 0.5}.
  - MPC-DC with N ∈ {5, 7, 15, 30}.
  - Reported metrics are solver status, mean/std computation time, minimum distance to the obstacle, and cost integral Σ uᵀuΔt.
  - They also compare N = 1 MPC-CBF against the greedy DCLF-DCBF and show they are near-equivalent.

  — [Zeng et al. 2021](https://arxiv.org/pdf/2007.11718)
- The tradeoff is that smaller γ gives earlier avoidance and larger clearance, but risks infeasibility, while larger N in MPC-DC gives earlier avoidance at higher compute. — [Zeng et al. 2021](https://arxiv.org/pdf/2007.11718)

### Inferences
- A concrete ablation plan for 2 weeks, in Gazebo with `turtlebot3_gazebo` on ROS 2 Humble:
  1. **N sweep**: {5, 10, 15, 20, 30} with no obstacles, on a figure-8 reference. Record RMS tracking error and solve time.
  2. **Obstacle formulation**: DC-hard, DC-soft (slack), CBF with γ ∈ {0.05, 0.1, 0.2, 0.3, 0.5, 1.0}, and a potential-field cost only. Run each at N = 10 and N = 20 with 1–3 static circular obstacles on the path. Record min clearance, infeasibility count, path-length overhead and solve time.
  3. **Weights**: Q_θ ∈ {0.1, 1, 10}, R ∈ {0.01, 0.1, 1}, S (Δu) on/off. Record tracking error vs control smoothness (Σ‖Δu‖²).
  4. **Terminal cost**: P ∈ {0, Q, 10Q}, plus an optional terminal set. Report the effect at short N.
  5. **Baselines**: Kanayama feedback law, LTV-MPC with OSQP, and NMPC with IPOPT (optionally acados RTI).
  6. **Robustness**: odometry noise or initial offset (for example 0.3 m and 45°), and an obstacle appearing suddenly.
- Use the same random seeds and initial conditions across arms, and report mean ± std over 5 or more runs.

### Gaps
- I found no survey that standardizes MPC ablation protocols for mobile robots. The plan above extends Zeng et al.'s design and is not drawn from a specific source.
