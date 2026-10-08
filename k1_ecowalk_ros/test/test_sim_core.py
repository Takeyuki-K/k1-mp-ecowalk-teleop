# SPDX-License-Identifier: Apache-2.0
"""Regression of the ROS wrapper against the numbers in k1_mp_turn/REPORT_TURN.md.
If these fail, the wrapper is feeding the policy something different from training.
Needs ECOWALK_ROOT (default ~/k1-mp-ecowalk-public). Run: python3 -m pytest test/test_sim_core.py -q
"""
import numpy as np
import pytest

from k1_ecowalk_ros.sim_core import EcoWalkSim


@pytest.fixture(scope='module')
def sim():
    return EcoWalkSim()


def run(sim, v, w, T=12.0, t0=4.0):
    sim.reset()
    sim.set_command(v, w)
    sim.start()
    yaws, xs, alive = [], [], True
    for k in range(int(T * 50)):
        alive &= sim.step()
        st = sim.status()
        yaws.append(st['yaw']); xs.append((st['x'], st['y']))
    k0 = int(t0 * 50)
    yaw = np.unwrap(yaws)
    w_meas = (yaw[-1] - yaw[k0]) / ((len(yaw) - 1 - k0) / 50)
    return alive, w_meas, np.array(xs), sim


@pytest.mark.parametrize('v,w', [(0.6, 0.5), (0.6, -0.5), (0.0, 0.6)])
def test_turn_matches_report(sim, v, w):
    alive, w_meas, _, _ = run(sim, v, w)
    assert alive
    assert abs(w_meas - w) / abs(w) < 0.03          # report: <= 2 % yaw-rate error


def test_straight_speed(sim):
    sim.reset(); sim.set_command(0.9, 0.0); sim.start()
    vs = []
    for k in range(600):
        assert sim.step()
        if k >= 200:
            vs.append(sim.status()['v_meas'])
    assert abs(np.mean(vs) - 0.9) / 0.9 < 0.08      # report v3: 4.6 % at 0.9 m/s


def test_safe_stop(sim):
    sim.reset(); sim.set_command(1.2, 0.0); sim.start()
    for _ in range(300):
        assert sim.step()
    sim.stop()
    for _ in range(300):
        assert sim.step()
    st = sim.status()
    assert st['alpha'] == 0.0 and abs(st['v_meas']) < 0.05


def test_envelope_flags(sim):
    sim.set_command(-0.5, 0.0)
    assert sim.applied == (0.0, 0.0) and 'backward_not_trained' in sim.envelope
    sim.set_command(2.0, 3.0)
    assert sim.applied == (1.35, 1.0)
    sim.set_command(0.15, 0.0)
    assert 'v_below_trained_0.30' in sim.envelope
