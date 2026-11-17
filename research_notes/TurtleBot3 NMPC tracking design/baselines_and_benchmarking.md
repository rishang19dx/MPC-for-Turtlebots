# Baseline Controllers and Benchmarking Methodology for NMPC Trajectory Tracking on TurtleBot3 Burger

Notation used throughout (common to all controllers). Robot pose q = (x, y, θ), unicycle model ẋ = v cos θ, ẏ = v sin θ, θ̇ = ω. Reference q_r = (x_r, y_r, θ_r) with feedforward (v_r, ω_r). Error in the **robot frame** (Kanayama convention):

```
[e_x]   [ cos θ   sin θ  0] [x_r − x]
[e_y] = [−sin θ   cos θ  0] [y_r − y]
[e_θ]   [   0       0    1] [wrap(θ_r − θ)]
```

Error dynamics: ė_x = ω e_y − v + v_r cos e_θ, ė_y = −ω e_x + v_r sin e_θ, ė_θ = ω_r − ω. This exact form is reproduced in [El Youssef et al., IFAC 2014, eq. 9](https://skoge.folk.ntnu.no/prost/proceedings/ifac2014/media/files/2210.pdf), citing Kanayama 1990.

Session note on sourcing: the Kanayama 1990 PDF host (calhoun.nps.edu) failed to resolve, and the Jiang & Nijmeijer and Yang & Kim full texts are paywalled. So the equations below come from secondary papers whose text I read, or they are marked as standard textbook forms that I did not check against the original paper in this session.

---

## 1. Baseline control laws: equations, stability, and gain tuning

### Takeaway
Use three baselines, all of which fit in about 30 lines of Python each. (1) Kanayama 1990 is the canonical baseline. (2) A linear or nonlinear "gain-scheduled" variant of it (De Luca/Oriolo/Vendittelli, Samson, Klancar), with gains tied to (v_r, ω_r) through one damping ratio ζ and one extra parameter b. (3) A kinematic sliding-mode controller (SMC) with a boundary layer. At the kinematic level, backstepping (Jiang & Nijmeijer 1997) gives a law almost identical to Kanayama plus a linear e_θ term. It is only meaningfully different if you also model wheel/torque dynamics (Fierro & Lewis), which is unnecessary for a TurtleBot driven through `cmd_vel`.

### Cited Findings

**Kanayama et al. 1990 (canonical baseline)**
- Citation: Y. Kanayama, Y. Kimura, F. Miyazaki, T. Noguchi, "A Stable Tracking Control Method for an Autonomous Mobile Robot," Proc. IEEE ICRA 1990, Cincinnati, pp. 384–389. Stability is proved with a Lyapunov function. The method was tested on the Yamabico-11 robot, with and without a velocity/acceleration limiter that reduces wheel slip. Linearizing the error dynamics gives a gain choice for critical damping under small disturbances — [researchr record](https://researchr.org/publication/KanayamaKMN90); [NPS Calhoun full text (could not be fetched this session)](https://calhoun.nps.edu/bitstream/handle/10945/40155/Kanamaya_A_stable_tracking_control.pdf?sequence=1)
- A follow-up, "A Stable Tracking Control Method for a Non-Holonomic Mobile Robot," appeared at IROS 1991, pp. 1236–1241 — [researchr](https://researchr.org/publication/KanayamaKMN91)
- The Kanayama controller was reused by Fierro & Lewis (1995) to generate reference velocities for a dynamic (torque-level) backstepping controller — [IFAC 2008 paper citing it](https://skoge.folk.ntnu.no/prost/proceedings/ifac2008/data/papers/2408.pdf)
- Control law (standard form, consistent with the task statement): v = v_r cos e_θ + k_x e_x, ω = ω_r + v_r (k_y e_y + k_θ sin e_θ), with k_x, k_y, k_θ > 0. The law is reproduced as the "Kanayama controller" baseline in [Ramaiah/Honc/Dušek, Process Control 2017 (NTNU proceedings)](https://skoge.folk.ntnu.no/prost/proceedings/process-control-slovakia-2017/data/papers/028.pdf)

**Gain-scheduled linear and nonlinear variants (De Luca–Oriolo–Vendittelli, Samson)**
- The NTNU 2017 paper benchmarks NMPC against two classical laws. The first is a **linear state-tracking controller ("Kanayama controller")**: u_fb = −K(t)e with K = [[−k1, 0, 0],[0, −k2·sign(v_r)·…, −k3]]. Its gains come from pole placement on the closed-loop polynomial (s + 2ζa)(s² + 2ζa s + a²), with one real pole at −2ζa and a complex pair with natural frequency a > 0 and damping ζ ∈ (0,1). The resulting gains are k1 = k3 = 2ζa and k2 = (a² − ω_r²)/v_r. Because k2 → ∞ as v_r → 0, they schedule **a = √(ω_r² + b v_r²)**, which gives **k1 = k3 = 2ζ√(ω_r² + b v_r²), k2 = b|v_r|**, with b > 0 as an extra degree of freedom. They note that for the linear law, asymptotic stability is **not** guaranteed, because the closed loop is still nonlinear and time-varying — [NTNU 2017](https://skoge.folk.ntnu.no/prost/proceedings/process-control-slovakia-2017/data/papers/028.pdf)
- The second is a **nonlinear (Samson) controller**: ω = ω_r + k̄2 v_r (sin e_θ / e_θ) e_y + k3 e_θ, with k1 = k3 = 2ζ√(ω_r² + b v_r²) and k̄2 = b. **Global asymptotic stability** can be shown by Lyapunov analysis, citing De Luca, Oriolo & Vendittelli, "Control of wheeled mobile robots: an experimental overview," Ramsete, LNCIS 270, 2001 — [NTNU 2017](https://skoge.folk.ntnu.no/prost/proceedings/process-control-slovakia-2017/data/papers/028.pdf); [Springer chapter](https://link.springer.com/doi/10.1007/3-540-45000-9_8)
- NTNU 2017 results, as sum of squared errors [SSE_xy, SSE_θ], in simulation: unconstrained NMPC2 [0.0192, 0.0004]; Kanayama controller [0.0013, 0.0067]; Samson [0.0013, 0.0067]. With an initial offset of [0.1, −0.1, 0], NMPC2 scored [0.1736, 57.37], KC [0.2837, 67.78] and SC [0.3188, 74.66]. Their NMPC1 (successive-linearization model) had the largest SSE in all cases. *Caveat: the table extracted from the PDF lost its column alignment, so the row-to-controller mapping is my best reading* — [NTNU 2017](https://skoge.folk.ntnu.no/prost/proceedings/process-control-slovakia-2017/data/papers/028.pdf)
- The companion IEEE TCST paper by Oriolo, De Luca & Vendittelli (2002) validates tracking on an **eight-shaped trajectory** on the SuperMARIO differential-drive robot. It compares dynamic feedback linearization against other tracking laws and calibrates the actual wheel radii and axle length — [CST 2002 PDF](https://www.diag.uniroma1.it/%7Elabrob/pub/papers/CST02.pdf)

**Backstepping (Jiang & Nijmeijer 1997; Fierro & Lewis)**
- Z.-P. Jiang, H. Nijmeijer, "Tracking control of mobile robots: a case study in backstepping," *Automatica* 33(7):1393–1399, 1997 (DOI 10.1016/S0005-1098(97)00055-1). It designs time-varying state feedback by backstepping for both the kinematic and a simplified dynamic model, gives local tracking results and then global tracking under extra conditions on the desired velocities, and checks them in simulation — [U. Twente record](https://research.utwente.nl/en/publications/tracking-control-of-mobile-robots-a-case-study-in-backstepping/)
- Later literature calls it a global tracking result obtained with integrator backstepping — [search summary of citing works, IFAC 2008](https://skoge.folk.ntnu.no/prost/proceedings/ifac2008/data/papers/2457.pdf)

**Sliding mode control**
- J.-M. Yang, J.-H. Kim, "Sliding mode control for trajectory tracking of nonholonomic wheeled mobile robots," *IEEE Trans. Robotics & Automation* 15(3):578–587, 1999 (DOI 10.1109/70.768190). It uses a **polar-coordinate posture representation** and computed-torque feedback linearization of the dynamics, plus a sliding mode law that asymptotically stabilizes the robot to the desired trajectory. It is robust to bounded disturbances and was validated experimentally — [KAIST repository](https://koasas.kaist.ac.kr/handle/10203/12394); [KNU record](https://research.knu.ac.kr/en/publications/sliding-mode-control-for-trajectory-tracking-of-nonholonomic-whee/). Caution: a 1999 *IEEE Control Systems* article by the same authors has a similar title, so check the DOI when citing.
- Yang & Kim combine inverse dynamics with a sliding law that uses a **boundary layer to reduce chattering**. First-order SMC with a boundary layer attenuates chattering but with poor robustness. The authors propose a PD + super-twisting (2nd-order SMC) alternative — [El Youssef et al., IFAC 2014](https://skoge.folk.ntnu.no/prost/proceedings/ifac2014/media/files/2210.pdf)
- **A kinematic SMC on Kanayama's error that can be implemented directly**: sliding surfaces **σ_v = k1 e_x** and **σ_w = k2 e_y + k3 sin(e_θ/2)**. The sin(e_θ/2) term is zero only at e_θ = 0 on [−π, π]. On the manifold, the sliding dynamics are **ė_θ = −2 k2 v_r sin(e_θ) / (k3 cos(e_θ/2))**, which is stable for v_r > 0 — [El Youssef et al., IFAC 2014, eqs. 12–13](https://skoge.folk.ntnu.no/prost/proceedings/ifac2014/media/files/2210.pdf)
- Boundary-layer practice: replace sgn(S) with sat(S/λ), which gives finite-time convergence to a band around the manifold. Choose the switching gain K > ‖h‖∞‖d‖∞ + η. On a TurtleBot3 they used K = 0.3 with disturbance bound d̄ = 0.1, tracking a Lissajous reference p_ref = [2.2 + 1.8 sin(0.23πt), 1.5 + 0.95 cos(0.15πt)] m with |v| ∈ [0.01, 0.2] m/s and |turn rate| ≤ 2 rad/s — [Sawarkar et al., arXiv 2604.24518 (2026)](https://arxiv.org/pdf/2604.24518)
- A larger boundary layer reduces chattering but can weaken robustness — [PMC11086255](https://pmc.ncbi.nlm.nih.gov/articles/PMC11086255)
- The Solea/Nunes/Filipescu SMC for WMR tracking uses "two nonlinear sliding surfaces ensuring the tracking of the three output variables" — [Solea et al. 2009, Annals Univ. Galati](https://www.gup.ugal.ro/ugaljournals/index.php/eeaci/article/view/638); thesis at [UC Coimbra](https://baes.uc.pt/handle/10316/10776?locale=pt)

### Inferences

**(a) Kanayama: Lyapunov sketch.** This is the standard textbook form, not checked against the 1990 PDF in this session.
- V = ½(e_x² + e_y²) + (1 − cos e_θ)/k_y ≥ 0.
- Substituting the law: V̇ = e_x(ω e_y − v + v_r cos e_θ) + e_y(−ω e_x + v_r sin e_θ) + (sin e_θ/k_y)(ω_r − ω).
- The ω e_x e_y terms cancel. With v − v_r cos e_θ = k_x e_x and ω_r − ω = −v_r(k_y e_y + k_θ sin e_θ), this gives **V̇ = −k_x e_x² − (v_r k_θ / k_y) sin² e_θ ≤ 0** for v_r > 0.
- Asymptotic convergence then follows from Barbalat/LaSalle, provided v_r and ω_r are bounded and v_r does not tend to 0. So **the controller loses convergence when the robot stops** (v_r = 0): point-to-point endpoints and turn-in-place references are weak spots, which is a fair point to highlight against NMPC.

**(b) Kanayama: gain tuning.**
- Linearize about e = 0 with constant v_r, ω_r: ė_x = ω_r e_y − k_x e_x, ė_y = −ω_r e_x + v_r e_θ, ė_θ = −v_r k_y e_y − v_r k_θ e_θ.
- When ω_r = 0, the (e_y, e_θ) subsystem has characteristic polynomial s² + v_r k_θ s + v_r² k_y. Critical damping gives **k_θ = 2√k_y**, natural frequency ω_n = v_r√k_y, and the e_x pole is at −k_x.
- Units: k_x in s⁻¹, k_y in m⁻², k_θ in m⁻¹.
- Practical starting values for the TurtleBot3 (v_r ≈ 0.1–0.2 m/s, target ω_n ≈ 1–2 rad/s): k_y ≈ 25–100 m⁻², k_θ = 2√k_y ≈ 10–20 m⁻¹, k_x ≈ 1–3 s⁻¹. Check that the commands stay within the limits for the largest initial offset you test.
- These are my derived values. The original paper reports gains for Yamabico in its own units, which I could not retrieve (see Gaps).

**(c) Gain-scheduled linear and Samson variants.**
- Laws:
  - v = v_r cos e_θ + k1 e_x
  - ω_lin = ω_r + k2 sign(v_r) e_y + k3 e_θ
  - ω_Samson = ω_r + b v_r sinc(e_θ) e_y + k3 e_θ
- Gains: k1 = k3 = 2ζ√(ω_r² + b v_r²), k2 = b|v_r|.
- Typical tuning ranges: ζ ∈ [0.5, 0.9], b ∈ [10, 100] m⁻² for TurtleBot speeds. This is my estimate; b plays the same role as Kanayama's k_y.
- Advantage: the gains adapt to the reference speed, so you only tune 2 numbers (ζ, b) and the same values work for circle, figure-8 and random trajectories.
- Implement sinc safely: `np.sinc(e_th/np.pi)`.

**(d) Backstepping (kinematic).**
- Law: v = v_r cos e_θ + c_x e_x, ω = ω_r + c_y v_r e_y · sinc(e_θ) + c_θ e_θ.
- Lyapunov function: V = ½(e_x² + e_y²) + e_θ²/(2c_y), which gives V̇ = −c_x e_x² − (c_θ/c_y) e_θ².
- Note: the e_y·sinc(e_θ) term is the backstepping "virtual control" for e_θ that cancels the cross term v_r e_y sin e_θ. This is the commonly quoted Jiang–Nijmeijer kinematic form. I reconstructed it from the Lyapunov argument rather than from the paper, which is paywalled, so verify before citing the exact constants.
- This law is **algebraically the Samson law above** with b = c_y, k3 = c_θ, k1 = c_x. So in the report, present "Kanayama" and "Backstepping/Samson" as two baselines, and note the second gives exponential e_θ convergence (linear e_θ term) instead of sin e_θ.
- Global convergence needs v_r(t) or ω_r(t) to not converge to zero (persistent excitation). This matches the "extra conditions on desired velocities" in the [abstract](https://research.utwente.nl/en/publications/tracking-control-of-mobile-robots-a-case-study-in-backstepping/).
- Fierro & Lewis add a torque/velocity-loop backstep, which is not needed for a `cmd_vel`-driven TurtleBot3. Optionally, add a first-order actuator lag v̇ = (v_cmd − v)/τ, with τ ≈ 0.1 s as a guess, to the simulator plant for all controllers.

**(e) Kinematic SMC, derived from the El Youssef surfaces (my derivation; check it numerically).**
- Reaching law σ̇ = −K sat(σ/φ):
  - **ω = [k2 v_r sin e_θ + (k3/2) cos(e_θ/2) ω_r + K_w sat(σ_w/φ_w)] / [k2 e_x + (k3/2) cos(e_θ/2)]**
  - then **v = v_r cos e_θ + ω e_y + (K_v/k1) sat(σ_v/φ_v)**
- Compute ω first, because v depends on it. Guard the denominator: it stays positive while |e_x| is small and |e_θ| < π. Clip it at ≥ 0.05·k3, for example.
- Starting gains: k1 = 1, k2 = 10–30 m⁻¹ (sic, relative to k3), k3 = 1; K_v ≈ 0.1–0.2 m/s; K_w ≈ 0.5–1 rad/s·(units of σ_w); φ (boundary layer) ≈ 0.02–0.05.
- Report chattering through Δu metrics (Section 3) for φ → 0 versus the chosen φ. That makes a nice graduate-level ablation.
- Alternative "Yang & Kim / Solea style" surfaces often quoted are s1 = ė_x + k1 e_x and s2 = ė_y + k2 e_y + k0 sgn(e_y) e_θ, with reaching law ṡ = −Q sat(s) − P s. These need v̇ as the control (integrate v). I could not verify these surfaces from full text this session (see Gaps), so prefer the El Youssef surfaces, which are verified.

**(f) Common implementation rules for fairness.**
- Every controller (NMPC included) goes through the **same saturation block**: |v| ≤ 0.22, |ω| ≤ 2.84, and optionally the wheel-speed "diamond" |v| + (L/2)|ω| ≤ v_wheel,max. It also gets the same sampling period (e.g., 10–20 Hz, which is typical for TB3 `cmd_vel`) and the same state estimate.
- Tune each baseline with the **same budget**, e.g., a grid search or Optuna run on a held-out "tuning" trajectory set. Then freeze the gains for the test set. That stops reviewers from saying the baselines were under-tuned.

### Gaps
- Kanayama 1990's exact numeric gains (Yamabico, mm/rad units) and its exact Lyapunov function were not retrieved, because the NPS PDF host did not resolve. The form above is standard but not verified against the original text in this session.
- Exact Jiang & Nijmeijer 1997 and Yang & Kim 1999 equations: paywalled, not read. Yang & Kim is torque-level (computed torque + SMC in polar coordinates), so a *kinematic* SMC baseline is a justified simplification. Say so explicitly in the report.
- I found no source for the Klancar "Wheeled Mobile Robotics" (Elsevier 2017) exact gain formula. The gain-scheduled formula above is sourced from NTNU 2017 / De Luca et al. instead.

---

## 2. Obstacles: how to compare a constraint-aware NMPC against tracking-only baselines fairly

### Takeaway
The classical laws have no notion of obstacles. Report two separate experiment families. **(A) Pure tracking on obstacle-free references:** all controllers, identical saturation. This is the core comparison. **(B) Obstacle scenarios:** NMPC with obstacle constraints versus each baseline wrapped in the *same* add-on safety layer (a CBF-QP filter or a simple reactive layer), or versus baselines tracking a reference that was pre-planned around the obstacles. Do not compare NMPC-with-obstacles against a raw baseline that has to collide.

### Cited Findings
- A 2026 paper pairs a sliding-mode tracker with a **Collision-Cone CBF (C3BF) QP safety filter**. The QP minimizes deviation from the nominal SMC command subject to the CBF constraint, and when the constraints are inactive the output equals u_smc. It was validated on TurtleBot3 hardware and an F1Tenth car with moving obstacles — [Sawarkar et al., arXiv 2604.24518](https://arxiv.org/pdf/2604.24518)
- CBF constraints have been integrated into MPC on a TurtleBot3 Burger. Real-run final tracking errors were [e_x, e_y] = [0.1286, 0.0626] m and e_θ = 0.021 rad, versus [0.0824, 0.0698] m and 0.0883 rad in simulation. This is a useful sim-to-real gap reference — [BEEI paper](https://www.beei.org/index.php/EEI/article/download/11309/4638) (numbers seen via a search summary; check against the full text)
- Sliding-mode control with obstacle avoidance on TurtleBot3 has also been reported — [MDPI Eng. Proc. 77(1):7](https://www.mdpi.com/2673-4591/77/1/7)
- NTNU 2017 assumes that an obstacle-free, feasible reference is supplied by a separate trajectory planner. That is the classical division of labour: the tracker does not handle obstacles — [NTNU 2017](https://skoge.folk.ntnu.no/prost/proceedings/process-control-slovakia-2017/data/papers/028.pdf)

### Inferences
- **Recommended protocol (implementable in a few days):**
  - **A1. Tracking only.** Obstacle-free references: circle, figure-8, sinusoid, random splines. All 3–4 controllers.
  - **A2. Obstacles.** Use the same reference as A1, which now passes through 1–3 circular obstacles. Compare:
    - NMPC with hard or soft distance constraints ‖p_k − o_j‖ ≥ r_j + r_robot + margin (TB3 Burger footprint radius ≈ 0.105 m; I did not check this ROBOTIS figure this session);
    - Kanayama + CBF-QP safety filter, with h(p) = ‖p − o‖² − d_safe² and the constraint ḣ ≥ −α h solved via a tiny QP in cvxpy/OSQP, or in closed form for one obstacle;
    - optionally Kanayama + artificial potential field as the "naive reactive" baseline.
  - Report collisions, minimum clearance, success rate, and tracking error *outside* the obstacle neighbourhood.
- The CBF filter needs the unicycle handled at a look-ahead point p_l = p + l[cos θ, sin θ] (l ≈ 0.05–0.1 m) so that the relative degree is 1 in (v, ω). This is a standard trick (my inference).
- Frame the A2 results honestly. The NMPC advantage there comes from *prediction plus constraints*, not tracking quality. The CBF filter is myopic and can deadlock in front of obstacles that are symmetric about the path, so count deadlocks as failures.

### Gaps
- I found no published paper that benchmarks NMPC versus Kanayama/SMC+CBF *with obstacles* on a TurtleBot3. The protocol above is a recommended design, not a replication.

---

## 3. Reference trajectory generation, including random trajectories, using flatness

### Takeaway
The unicycle is differentially flat with flat output (x, y). Any C²-smooth (x(t), y(t)) gives θ_r = atan2(ẏ, ẋ), v_r = √(ẋ² + ẏ²), ω_r = (ẋÿ − ẏẍ)/(ẋ² + ẏ²). Generate geometric paths (circle, lemniscate of Gerono, sinusoid, B-splines through random waypoints), then time-scale them so that |v_r| ≤ ~0.8·v_max and |ω_r| ≤ ~0.8·ω_max. That leaves feedback headroom, which every controller needs.

### Cited Findings
- TurtleBot3 limits used in a published TurtleBot3 benchmark: v ∈ [−0.22, 0.22] m/s, ω ∈ [−2.84, 2.84] rad/s, control rate 50 Hz — [GMPC, Jin et al., arXiv 2403.07317, Table I](https://arxiv.org/pdf/2403.07317)
- In the unicycle, (x, y) is a flat output — [Oriolo, De Luca, Vendittelli, IEEE TCST 2002](https://www.diag.uniroma1.it/%7Elabrob/pub/papers/CST02.pdf) (footnote: "in this case, [the output] is also called a flat output")
- Published reference sets for comparison:
  - circle and "butterfly-shaped" trajectories on TurtleBot3 (GMPC) — [arXiv 2403.07317](https://arxiv.org/pdf/2403.07317)
  - eight-shaped trajectory (CST 2002) — [CST02](https://www.diag.uniroma1.it/%7Elabrob/pub/papers/CST02.pdf)
  - Lissajous p = [2.2 + 1.8 sin(0.23πt), 1.5 + 0.95 cos(0.15πt)] with |v| ≤ 0.2 m/s on TurtleBot3 — [arXiv 2604.24518](https://arxiv.org/pdf/2604.24518)
  - circle p = [1.58 + cos(0.2πt), 1.78 + sin(0.2πt)] (F1Tenth) — [arXiv 2604.24518](https://arxiv.org/pdf/2604.24518)
- NTNU 2017 generates references by spline-interpolating time-parameterized reference points — [NTNU 2017](https://skoge.folk.ntnu.no/prost/proceedings/process-control-slovakia-2017/data/papers/028.pdf)

### Inferences (implementation recipe)
- **Closed-form references (s = phase, t = time, ω_0 = angular rate):**
  - Circle: x = R cos(ω_0 t), y = R sin(ω_0 t), so v_r = Rω_0 and ω_r = ω_0. Example: R = 1 m, ω_0 = 0.15 rad/s gives v_r = 0.15 m/s.
  - Lemniscate of Gerono (figure-8): x = A sin(ω_0 t), y = A sin(ω_0 t) cos(ω_0 t) = (A/2) sin(2ω_0 t). With A = 1.0–1.5 m and ω_0 ≈ 0.1 rad/s, the peak v ≈ Aω_0√(1 + 1) ≲ 0.21 m/s. Verify numerically, since the peak occurs at the crossing.
  - Sinusoid: x = v_x t, y = A sin(2π t/T_p), with v_x = 0.12 m/s, A = 0.3 m, T_p = 15 s.
  - Compute θ_r, v_r and ω_r analytically from the derivatives, or with `np.gradient` on a fine grid (dt ≤ 0.01) and then subsample. Unwrap θ_r with `np.unwrap`.
- **Random smooth trajectories, reproducible per seed:**
  1. Draw N = 5–8 waypoints in a W×W m arena (e.g., 3×3 m), enforcing a minimum spacing ≥ 0.4 m and a maximum turning angle ≤ 120° between consecutive segments, so you avoid hairpins and cusps.
  2. Fit `scipy.interpolate.make_interp_spline(s_chord, P, k=5)` (quintic, so ω_r is C¹) or `splprep` with smoothing. Use cumulative chord length as the parameter.
  3. Resample densely and compute curvature κ(s) = (x′y″ − y′x″)/(x′² + y′²)^{3/2}.
  4. Speed profile: v(s) = min(α v_max, α ω_max/|κ(s)|, √(a_lat,max/|κ|)) with α = 0.8. Then run a forward-backward pass for the longitudinal acceleration limit (e.g., a_max = 0.5 m/s², an assumed value), and integrate dt = ds/v to get t(s).
  5. Output (x_r, y_r, θ_r, v_r = v, ω_r = κv) on a uniform time grid.
  6. Reject and redraw if any |κ| > κ_max (e.g., 10 m⁻¹) or the path self-intersects too close (< 0.3 m), so that no trajectory is infeasible for the baselines.
  7. Store the seed with each trajectory.
- For the TB3 Burger, v_max binds almost everywhere: ω_max/v_max = 2.84/0.22 ≈ 12.9 m⁻¹, so the turn-rate limit only binds for radii below ~7.7 cm. The wheel-level constraint |v| + (L/2)|ω| ≤ v_max (L ≈ 0.160 m, from the ROBOTIS spec, not checked this session) is what really couples v and ω. Use it in both the NMPC constraint set and the shared saturation block.
- **Point-to-point with variable start and goal:**
  - Sample start and goal poses, connect them with a cubic Hermite / quintic polynomial in (x, y) whose endpoint tangents match the start and goal headings, and use a trapezoidal speed profile that starts and ends at v = 0.
  - Expect Kanayama and Samson to stall or perform poorly near v_r → 0. Either report this as a known limitation or stop the evaluation when the reference reaches the goal and switch to a posture regulator. Use the same rule for every controller.
- Give NMPC the reference over its horizon (look-ahead), and give the baselines the reference only at time t. That is inherent to the methods, so state it rather than "fixing" it.

### Gaps
- I found no paper that standardizes "random trajectory" generation for unicycle tracking benchmarks. The recipe above is engineering practice, not taken from a source.
- The ROBOTIS TB3 Burger wheel separation (0.160 m), wheel radius (0.033 m) and footprint were not verified this session. Take them from the ROBOTIS e-manual / `turtlebot3_description` URDF.

---

## 4. Quantitative metrics, Monte Carlo protocol, and result presentation

### Takeaway
Report tracking (RMS/max position error, RMS heading error, IAE/ISE), effort and smoothness (Σu², ΣΔu², jerk), constraint behaviour (saturation time fraction, violations), safety (minimum clearance, collisions, success), and computation (NMPC solve-time mean/p95/max, deadline misses). Run a paired Monte Carlo study: same seeds across controllers, ≥ 30–50 trials per condition. Present box/violin plots, a summary table (median [IQR]), and CDFs of solve time and error.

### Cited Findings
- GMPC's TurtleBot3 metrics are position error e_p(t) = ‖p(t) − p_d(t)‖₂ and orientation error e_R(t) = ‖Log(R_d(t)⁻¹ R(t))‖₂, which is the Riemannian metric and equals |wrap(θ − θ_d)| in SE(2). Their **Monte Carlo** circle tests randomize the initial position in [−0.2, 0]×[−0.2, 0] m and the heading in [−π/6, 0]. The settings were MPC horizon T = 10 at 50 Hz, NMPC on IPOPT, and convex MPC on QP solvers. The convex GMPC solved about **1.5× faster** than NMPC with much lower solve-time variance. Its baselines were NMPC and a feedback-linearization controller (FBC). FBC with a fixed gain showed large errors and non-smooth velocities on the butterfly trajectory, plus a singularity issue around certain θ — [arXiv 2403.07317](https://arxiv.org/pdf/2403.07317)
- NTNU 2017 uses sum of squared errors: SSE_xy = Σ(|x − x_r| + |y − y_r|)², SSE_θ = Σ|θ − θ_r|² (as best I can read the garbled equation). It ran 12 scenarios over unconstrained, constrained (|u| ≤ 1) and offset-initial-condition settings — [NTNU 2017](https://skoge.folk.ntnu.no/prost/proceedings/process-control-slovakia-2017/data/papers/028.pdf)
- MPC versus PID on a TurtleBot3 reported a 32% reduction in distance tracking error — [EJUOW paper](https://ejuow.uowasit.edu.iq/index.php/ejuow/article/download/864/375/4900) (seen via a search summary)

### Inferences (metric definitions to implement)
Use samples k = 1..N at period Δt, excluding an optional transient window.
- **Tracking:**
  - RMSE_pos = √(mean e_p²)
  - max e_p
  - RMSE_θ = √(mean wrap(e_θ)²)
  - IAE = Σ e_p Δt, ISE = Σ e_p² Δt
  - ITAE = Σ t·e_p Δt, which penalizes slow convergence and is good for initial-offset tests
  - Lateral (cross-track) error e_y separately from along-track error e_x. MPC and Kanayama trade these off differently.
  - Settling time: the first t after which e_p < 2 cm for good.
- **Effort and smoothness:**
  - E_u = Σ(v² + λω²)Δt, with λ = (L/2)² ≈ 0.0064 to put ω in wheel-speed units, or simply report v and ω separately.
  - Smoothness S = Σ‖u_k − u_{k−1}‖²
  - Total variation TV = Σ|Δv| + |Δω|
  - RMS jerk of the *actual* trajectory: third derivative of the position from the simulator, low-pass filtered.
  - Chattering index: number of sign changes of Δω per second. Useful for SMC φ ablations.
- **Constraints:**
  - % of time steps where the commanded u hit saturation (pre-clip ≠ post-clip)
  - Count and magnitude of violations, if any controller outputs beyond limits without clipping
  - Wheel-speed limit violations under the diamond constraint
- **Safety (obstacle runs):** minimum clearance min_k(‖p_k − o_j‖ − r_j − r_robot), collision count (clearance < 0), success rate (goal reached within T_max with no collision), time-to-goal, path length ratio.
- **Computation (NMPC; baselines too, for completeness):** per-step wall-clock solve time, reported as mean, median, p95, p99 and max. Also report **deadline misses**: the fraction with t_solve > Δt (e.g., 50–100 ms). Report solver iterations and failure/"max_iter" status counts, and the CPU model. Use `time.perf_counter()` around the solve only. Warm-start statistics: report the first solve separately.
- **Monte Carlo design:**
  - Factors: controller × trajectory family (circle, eight, sinusoid, random, P2P) × perturbation condition.
  - For each condition, draw seeds s = 0..M−1 that fix the random trajectory, initial offset, and noise realizations. **Use the same seeds for every controller** (paired design).
  - M = 50 per cell is a sensible default. With 4 controllers × 5 trajectory families × ~6 robustness conditions that is about 6,000 simulated runs; a 60 s run at 20 Hz is 1,200 NMPC solves, which is fine on a laptop overnight with CasADi/acados. Use M = 30 if time-constrained.
  - Statistics: per-cell median and IQR, and a 95% bootstrap CI of the median (`scipy.stats.bootstrap`). Use a paired Wilcoxon signed-rank test (`scipy.stats.wilcoxon`) for NMPC versus each baseline, with Holm correction across comparisons.
  - Initial offsets: sample uniformly e.g. Δx, Δy ∈ [−0.2, 0.2] m, Δθ ∈ [−π/6, π/6], which mirrors GMPC's ranges and extends them symmetrically.
- **Presentation:**
  1. One representative XY plot per trajectory family with all controllers overlaid.
  2. Time series of e_p, e_θ, v and ω, with the limits drawn as dashed lines.
  3. Box/violin plots of RMSE_pos, max e_p, and E_u per controller per condition.
  4. An empirical CDF of NMPC solve times with a vertical line at Δt (makes deadline misses obvious), and a CDF of per-run RMSE.
  5. A summary table: rows = controllers; columns = median [IQR] RMSE_pos, RMSE_θ, max e_p, E_u, S, % saturated, success %, solve-time mean/p95/max.
  6. A "degradation" line plot: RMSE versus perturbation magnitude (delay, noise σ, wheel-radius error %) for each controller.

### Gaps
- I found no authoritative source that prescribes the number of Monte Carlo trials for this kind of controller benchmark. M = 30–50 with paired non-parametric tests is a common-sense recommendation, not a cited standard.
- GMPC did not publish numeric RMSE tables in the text I extracted (results are in figures), so no direct numeric TB3 baseline is available from it.

---

## 5. Robustness tests

### Takeaway
Run four perturbation sweeps, applied to the *plant* (simulator) only while controllers keep their nominal models: input delay, odometry/state noise, model mismatch via wheel-radius and track-width error, and initial-pose offset. Optionally add an actuator lag. These expose the classical trade-offs: NMPC is sensitive to delay and model mismatch unless compensated, while SMC claims robustness to matched disturbances at the cost of chattering.

### Cited Findings
- SMC with the error dynamics forced onto a sliding manifold is ideally invariant to *matched* uncertainties h(ξ, t) added to the error dynamics. In practice, neglected dynamics and sampling limits cause chattering — [El Youssef et al., IFAC 2014](https://skoge.folk.ntnu.no/prost/proceedings/ifac2014/media/files/2210.pdf)
- Oriolo et al. explicitly calibrated *actual wheel radii and axle length* for odometry, which identifies these parameters as dominant mismatch sources on differential-drive robots — [CST 2002](https://www.diag.uniroma1.it/%7Elabrob/pub/papers/CST02.pdf)
- Kanayama 1990 used a velocity/acceleration limiter to avoid wheel slippage — [researchr summary](https://researchr.org/publication/KanayamaKMN90)
- On hardware, a fixed-gain feedback-linearization controller cannot adapt to model mismatch on a different platform, according to the GMPC TurtleBot3/Scout comparison — [arXiv 2403.07317](https://arxiv.org/pdf/2403.07317)
- A sim-to-real gap on TurtleBot3 with CBF-MPC: final position errors of ~0.13/0.06 m on hardware versus ~0.08/0.07 m in simulation — [BEEI](https://www.beei.org/index.php/EEI/article/download/11309/4638)

### Inferences (concrete perturbation models and suggested levels)
- **Input delay:** buffer the commands so that u_applied(k) = u_cmd(k − d), with d ∈ {0, 1, 2, 4} steps at 20 Hz (0–200 ms). Real ROS 2 + TB3 OpenCR latency is plausibly in the tens of ms, but I did not verify that this session. NMPC can compensate by forward-predicting the state d steps with its model before solving. Report NMPC with and without compensation as an extra variant.
- **Odometry / state noise:** the controller sees q̂ = q + n, with n ~ N(0, diag(σ_p², σ_p², σ_θ²)) and σ_p ∈ {0, 0.005, 0.01, 0.02} m, σ_θ ∈ {0, 0.01, 0.02, 0.05} rad. Optionally add a slowly drifting bias, a random walk, to mimic odometry drift. Use the same noise sequence per seed for every controller.
- **Model mismatch:** the true wheel radii are r_R = r(1 + δ_R) and r_L = r(1 + δ_L), and the true track width is L(1 + δ_L′). Convert u_cmd to wheel speeds with the nominal r and L, then back to (v, ω) with the true values.
  - Symmetric δ ∈ {−10%, −5%, 0, +5%, +10%} produces a pure speed-scale error.
  - Asymmetric δ_R = −δ_L = ±2–5% produces turning bias, which is the more revealing test.
- **Initial-pose offset:** a sweep of ‖Δp‖ ∈ {0, 0.1, 0.2, 0.4} m and Δθ ∈ {0, π/6, π/3, 2π/3}. The larger offsets stress the saturation-plus-nonlinearity regime, where Kanayama's sin e_θ and the denominator guard in the SMC matter.
- **Optional extras:** first-order actuator lag τ ∈ {0, 0.1, 0.2} s; additive velocity disturbance (wheel slip); reduced control rate (50 → 10 Hz).
- For each sweep, plot median RMSE (with IQR band) versus perturbation level per controller, plus the success rate. Define "failure" as max e_p > 0.5 m or divergence.

### Gaps
- I found no measured value for end-to-end TurtleBot3 `cmd_vel` latency. Measure it, or cite the ROBOTIS docs or a ROS 2 latency study, if you need a "realistic" delay level.

---

## 6. Similar academic and student work to cite

### Takeaway
No paper found compares NMPC against Kanayama **and** SMC on a TurtleBot3 in one study, so the project fills a small, legitimate niche. The closest anchors are below: GMPC (TurtleBot3, NMPC versus FBC with Monte Carlo initial offsets), NTNU 2017 (NMPC versus Kanayama and Samson, simulation only), SMC + C3BF on TurtleBot3 (2026), CBF-MPC on TurtleBot3 Burger, and MPC versus PID on TurtleBot3.

### Cited Findings
- Jin et al., "GMPC: Geometric Model Predictive Control for Wheeled Mobile Robot Trajectory Tracking" (2024). TurtleBot3 + Scout mini, PyBullet. Compares GMPC versus NMPC (IPOPT) versus feedback linearization; circle and butterfly trajectories; Monte Carlo initial-pose tests; solve-time comparison — [arXiv 2403.07317](https://arxiv.org/pdf/2403.07317)
- Ramaiah (Kumar R.), Honc, Dušek, "Trajectory tracking of non-holonomic mobile robot with NMPC" (Process Control 2017). Compares two LTV-model NMPCs against the Kanayama (linear) and Samson (nonlinear) controllers with De Luca-style gain scheduling, using SSE metrics — [PDF](https://skoge.folk.ntnu.no/prost/proceedings/process-control-slovakia-2017/data/papers/028.pdf)
- Sawarkar, Sangeerth, Saharsh, Jagtap, "Sliding Mode Control for Safe Trajectory Tracking with Moving Obstacles Avoidance: Experimental Validation on Planar Robots" (arXiv, Apr 2026). SMC + C3BF-QP on TurtleBot3 hardware with sat(S/λ) boundary layer — [arXiv 2604.24518](https://arxiv.org/pdf/2604.24518)
- CBF-integrated MPC on TurtleBot3 Burger, with hardware versus simulation error figures — [BEEI](https://www.beei.org/index.php/EEI/article/download/11309/4638)
- MPC versus PID on TurtleBot3 (32% lower tracking error) — [EJUOW](https://ejuow.uowasit.edu.iq/index.php/ejuow/article/download/864/375/4900); hybrid MPC-PID on TurtleBot3 from a system-identified model — [Bull. Pol. Acad. Sci.](https://wydawnictwo.pan.pl/index.php/bpasts/article/download/1222/959/4741)
- Pure-pursuit look-ahead tuning on TurtleBot3 Burger in Gazebo, a possible extra geometric baseline — [DergiPark](https://dergipark.org.tr/en/download/article-file/1927194)
- Open-source TurtleBot3 MPC implementation (student-style repo): lacie-life/TurtleBot3-MPC — [relatedrepos listing](https://relatedrepos.com/gh/lacie-life/TurtleBot3-MPC)
- De Luca, Oriolo, Vendittelli, "Control of wheeled mobile robots: an experimental overview," the classic experimental comparison of tracking laws on one robot and the template for this kind of study — [Springer](https://link.springer.com/doi/10.1007/3-540-45000-9_8); companion [TCST 2002](https://www.diag.uniroma1.it/%7Elabrob/pub/papers/CST02.pdf)

### Inferences
- In the report, position the contribution as follows. A reproducible, paired Monte Carlo benchmark of NMPC versus Kanayama, Samson/backstepping and kinematic SMC on TB3-Burger-constrained references, including random trajectories and four robustness sweeps. Obstacle handling is evaluated separately with a shared safety-filter for the baselines.

### Gaps
- Several numbers (BEEI, EJUOW) were seen only via search summaries. Read the full texts before quoting them in the final report.
- Fierro & Lewis (1997, J. Robotic Systems) and the Klancar et al. *Wheeled Mobile Robotics* (2017) textbook chapter on tracking were not retrieved this session.
