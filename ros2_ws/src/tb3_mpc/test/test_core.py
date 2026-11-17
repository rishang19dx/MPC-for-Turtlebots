"""Unit + closed-loop tests for the ROS-free core (run: python3 -m pytest test/)."""
import numpy as np
import pytest

from tb3_mpc.core import make_controller
from tb3_mpc.core import lidar
from tb3_mpc.core.metrics import episode_metrics
from tb3_mpc.core.model import BURGER, predict_forward, rk4_step, saturate, wrap_angle
from tb3_mpc.core.references import (circle, figure8, lissajous, point_to_point, random_spline,
                                     sinusoid)
from tb3_mpc.core.safety_filter import CBFSafetyFilter
from tb3_mpc.core.simulator import EpisodeConfig, PlantConfig, run_episode


# ----------------------------------------------------------------- model
def test_rk4_exact_on_circle():
    x = np.zeros(3)
    u = np.array([0.2, 0.5])  # radius 0.4
    for _ in range(int(2 * np.pi / 0.5 / 0.01)):
        x = rk4_step(x, u, 0.01)
    assert np.hypot(x[0], x[1]) < 2e-3


def test_saturate_preserves_curvature_and_limits():
    u = saturate([0.22, 2.0])
    assert abs(u[0]) + BURGER.half_track * abs(u[1]) <= BURGER.v_max + 1e-12
    assert np.isclose(u[0] / u[1], 0.22 / 2.0)


def test_saturate_rate_limit():
    u = saturate([0.22, 0.0], u_prev=np.zeros(2), dt=0.1)
    assert np.isclose(u[0], BURGER.a_max * 0.1)


def test_predict_forward_matches_rollout():
    cmds = [np.array([0.1, 0.2])] * 3
    x = predict_forward(np.zeros(3), cmds, 0.1)
    y = np.zeros(3)
    for c in cmds:
        y = rk4_step(y, c, 0.1)
    assert np.allclose(x, y)


# ------------------------------------------------------------ references
@pytest.mark.parametrize("ref", [circle(), figure8(), sinusoid(), lissajous(),
                                 random_spline(0), random_spline(7),
                                 point_to_point([0, 0, 0], [1.5, 1.0, np.pi / 2])])
def test_reference_is_dynamically_consistent_and_feasible(ref):
    # integrating the flat feed-forward reproduces the path (open loop, short window)
    x = np.array([ref.x[0], ref.y[0], ref.theta[0]])
    n = min(len(ref.t) - 1, 1000)
    for i in range(n):
        x = rk4_step(x, np.array([ref.v[i], ref.omega[i]]), ref.t[i + 1] - ref.t[i])
    assert np.hypot(x[0] - ref.x[n], x[1] - ref.y[n]) < 0.03
    # within actuator limits (incl. wheel diamond)
    load = np.abs(ref.v) + BURGER.half_track * np.abs(ref.omega)
    assert np.percentile(load, 99) <= BURGER.v_max + 1e-6


def test_random_spline_reproducible():
    a, b = random_spline(3), random_spline(3)
    assert np.allclose(a.x, b.x) and np.allclose(a.y, b.y)


def test_point_to_point_rest_to_rest():
    ref = point_to_point([0, 0, 0], [1.0, 0.5, 0.0])
    # within one rate-limit step (a_max * dt = 0.05 m/s) of rest
    assert ref.v[0] < 0.05 and ref.v[-1] < 0.05
    assert np.allclose(ref.end_pose[:2], [1.0, 0.5], atol=1e-3)


def test_anchor_reference():
    ref = circle().anchored_at(np.array([1.0, 2.0, np.pi / 2]))
    assert np.allclose(ref.start_pose, [1.0, 2.0, np.pi / 2], atol=1e-6)


# ------------------------------------------------------------ controllers
@pytest.mark.parametrize("name", ["kanayama", "samson", "smc", "nmpc"])
def test_controllers_converge_from_offset(name):
    ref = figure8()
    x0 = ref.start_pose + np.array([0.2, -0.15, 0.4])
    log = run_episode(make_controller(name), ref, x0)
    m = episode_metrics(log, transient=10.0)
    assert m["success"]
    assert m["rmse_pos"] < 0.02, m


def test_zero_error_gives_feedforward():
    ref = circle()
    xr, ur = ref.sample(5.0)
    for name in ["kanayama", "samson", "smc"]:
        u, _ = make_controller(name).compute(5.0, xr[:, 0], ref)
        assert np.allclose(u, ur[:, 0], atol=1e-6), name


def test_nmpc_respects_input_constraints():
    ref = circle(radius=0.5, speed=0.2)
    log = run_episode(make_controller("nmpc"), ref, ref.start_pose + np.array([0.3, 0, 1.0]))
    u = log["u_raw"]
    assert np.all(np.abs(u[:, 0]) + BURGER.half_track * np.abs(u[:, 1]) <= BURGER.v_max + 1e-4)


# ------------------------------------------------------------ obstacles
@pytest.mark.parametrize("mode", ["cbf", "dc", "pf"])
def test_nmpc_avoids_obstacle(mode):
    ref = point_to_point([0, 0, 0], [2.0, 0.5, 0.0])
    obs = np.array([[1.0, 0.25, 0.15]])
    c = make_controller("nmpc", {"obstacle_mode": mode, "max_wall_time": 1.0})
    log = run_episode(c, ref, ref.start_pose, obstacles=obs, ep=EpisodeConfig(t_extra=10))
    m = episode_metrics(log)
    assert not m["collided"] and m["final_pos_error"] < 0.15, m


def test_cbf_filter_passthrough_and_blocking():
    f = CBFSafetyFilter(alpha=1.0)
    x = np.zeros(3)
    u_free, info = f.filter(x, np.array([0.2, 0.0]), np.array([[5.0, 5.0, 0.3]]))
    assert np.allclose(u_free, [0.2, 0.0], atol=1e-4)
    u_blk, info = f.filter(x, np.array([0.2, 0.0]), np.array([[0.35, 0.0, 0.3]]))
    assert u_blk[0] < 0.2 and info["active"]


# ------------------------------------------------------------ lidar
def test_lidar_recovers_circle():
    true = np.array([[1.0, 0.5, 0.15]])
    scan = lidar.simulate_scan(np.zeros(3), true, noise=0.005, rng=np.random.default_rng(0))
    circ = lidar.extract_circles(scan, 0.0, 2 * np.pi / 360, 0.12, 3.5)
    assert len(circ) == 1
    assert np.hypot(*(circ[0, :2] - true[0, :2])) < 0.05
    assert abs(circ[0, 2] - 0.15) < 0.05


def test_lidar_handles_zero_and_inf_ranges():
    r = np.full(360, np.inf)
    r[::2] = 0.0
    assert len(lidar.extract_circles(r, 0.0, 2 * np.pi / 360, 0.12, 3.5)) == 0


def test_lidar_wraparound_merge():
    true = np.array([[1.0, 0.0, 0.15]])  # straddles beam 0 / beam 359
    scan = lidar.simulate_scan(np.zeros(3), true)
    circ = lidar.extract_circles(scan, 0.0, 2 * np.pi / 360, 0.12, 3.5)
    assert len(circ) == 1


def test_delay_compensation_helps():
    ref = figure8()
    plant = PlantConfig(delay_steps=3)
    x0 = ref.start_pose
    res = {}
    for comp in (False, True):
        log = run_episode(make_controller("nmpc"), ref, x0, plant=plant,
                          ep=EpisodeConfig(compensate_delay=comp))
        res[comp] = episode_metrics(log, transient=5)["rmse_pos"]
    assert res[True] < res[False]


def test_wrap_angle():
    assert np.isclose(wrap_angle(3 * np.pi), -np.pi) or np.isclose(wrap_angle(3 * np.pi), np.pi)
