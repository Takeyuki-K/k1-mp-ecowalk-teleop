# SPDX-License-Identifier: Apache-2.0
"""ROS-free core for v5.5 / v5.6.3: walking <-> running with turning and hard braking
(k1_mp_gait55/gait55.py, k1_mp_gait56/gait56.py; v5.6 = level-ish pelvis walking, arm residuals, feet re-placed).

Same interface as sim_core.EcoWalkSim (v3), plus brake(). The gait manager and both training environments are used
as-is (n = 1); this module only maps teleop commands and reports state.
"""
import os
import sys
import numpy as np

from .sim_core import default_root, DEADBAND_V, DEADBAND_W

V_MAX_55 = 5.0        # v5 measured top speed inside the motor envelope: 4.9 m/s (commands up to 5.5 were trained)
W_MAX_55 = 1.0
A_LAT_RUN = 3.0


class EcoWalk55Sim:
    has_brake = True
    A_LAT = A_LAT_RUN           # running governor v |w| (v5.5: 3.0; v5.6.3 deploys 2.5, gait56.py)
    name = 'v5.5 walk+run'

    FOLDER, MODULE, CLASS, VERSION = 'k1_mp_gait55', 'gait55', 'Gait55', 'v5.5'

    def __init__(self, root=None, walk=None, run=None, seed=5):
        root = os.path.abspath(os.path.expanduser(root or default_root()))
        walk = walk or f'{self.FOLDER}/runs/final/walk.pt'
        run = run or f'{self.FOLDER}/runs/final/run.pt'
        pkg = os.path.join(root, self.FOLDER)
        if not os.path.isfile(os.path.join(pkg, self.MODULE + '.py')):
            raise FileNotFoundError(f'{self.FOLDER} not found under {root}')
        # the gait folders share module names (k1env, ...): make sure this folder wins
        sys.path = [p for p in sys.path if not os.path.basename(p.rstrip('/')).startswith(('k1_mp_turn', 'k1_mp_gait'))]
        sys.path.insert(0, pkg)
        import importlib
        import mujoco
        Gait55 = getattr(importlib.import_module(self.MODULE), self.CLASS)
        from k1env import ACT_JOINTS
        self._mujoco = mujoco
        p = lambda x: x if os.path.isabs(x) else os.path.join(root, x)
        self.root = root
        self.policy_path = f'{p(walk)} + {p(run)}'
        self.g = Gait55(p(walk), p(run), n=1, seed=seed)
        self.m = self.g.W.m
        self.joint_names = list(ACT_JOINTS) + ['left_mp_joint', 'right_mp_joint']
        self.dt = 1.0 / self.g.W.CTRL_HZ
        self.reset()

    # ------------------------------------------------------------------ control
    def reset(self):
        self.g.reset()
        self.fallen = False
        self.sim_time = 0.0
        self.req = (0.0, 0.0); self.applied = (0.0, 0.0)
        self.envelope = []
        self._walking = False
        self.info = {}
        self.last_switch = ''
        self._wz_f = 0.0; self._lean_f = 0.0

    @property
    def env(self):
        return self.g.env

    @property
    def walking(self):
        return self._walking

    def start(self):
        if not self.fallen:
            self._walking = True
            self.g.command(*self.applied)

    def stop(self):
        self._walking = False
        self.g.stop()

    def brake(self):
        self._walking = False
        self.g.brake()

    def set_command(self, v, w):
        v, w = float(v), float(w)
        self.req = (v, w)
        notes = []
        if abs(v) < DEADBAND_V:
            v = 0.0
        if abs(w) < DEADBAND_W:
            w = 0.0
        if v < 0:
            notes.append('backward_not_trained'); v = 0.0
        if v > V_MAX_55:
            notes.append('v_clamped_5.0'); v = V_MAX_55
        if abs(w) > W_MAX_55:
            notes.append('w_clamped_1.0'); w = float(np.sign(w)) * W_MAX_55
        if 0 < v < 0.3:
            notes.append('v_below_trained_0.30')
        if v > 1.65 and w != 0 and v * abs(w) > self.A_LAT:
            notes.append('governor_slows_for_turn')        # information, not an error: trained behaviour
        self.envelope = notes
        self.applied = (v, w)
        if self._walking:
            self.g.command(v, w)

    def step(self):
        if self.fallen:
            return False
        n_sw = len(self.g.switches)
        term, info = self.g.step()
        self.info = info
        if len(self.g.switches) > n_sw:
            self.last_switch = self.g.switches[-1][1]
        self.sim_time += self.dt
        # 0.3 s low-pass for display: a running pelvis oscillates in yaw/roll every stride
        _, gv = self.env.base_frame()
        k = self.dt / 0.3
        self._wz_f += k * (float(self.qvel()[5]) - self._wz_f)
        self._lean_f += k * (float(np.degrees(np.arcsin(np.clip(gv[0, 1], -1, 1)))) - self._lean_f)
        if term[0]:
            self.fallen = True
            self._walking = False
        return not self.fallen

    # ------------------------------------------------------------------ state
    def qpos(self):
        return self.env.qpos()[0]

    def qvel(self):
        return self.env.qvel()[0]

    def sensor(self, name):
        e = self.env
        a, d = e.sens[name]
        return e.sdata[0, a:a + d].copy()

    def base_pose(self):
        q = self.qpos()
        return q[0:3].copy(), q[3:7].copy()

    def base_twist_local(self):
        return self.sensor('base_linvel'), self.qvel()[3:6].copy()

    def imu(self):
        return self.sensor('imu_quat'), self.sensor('imu_gyro'), self.sensor('imu_acc')

    def joints(self):
        m = self.env.m
        qa = [m.jnt_qposadr[m.joint(j).id] for j in self.joint_names]
        da = [m.jnt_dofadr[m.joint(j).id] for j in self.joint_names]
        return self.qpos()[qa].copy(), self.qvel()[da].copy()

    def status(self):
        g, e = self.g, self.env
        run = g.mode[0] == 1
        q = self.qpos()
        yaw = float(e.yaw()[0])
        vw = self.qvel()[0:3]
        vx = float(np.cos(yaw) * vw[0] + np.sin(yaw) * vw[1])
        alpha = float(e.alpha[0])
        if self.fallen:
            mode = 'fallen'
        elif run:
            mode = 'brake' if e.brake[0] > 0.5 else 'run'
        elif getattr(e, 'rs', None) is not None and e.rs[0] > 0:
            mode = 'restance'          # v5.6: re-placing the feet 1 s after stopping
        elif alpha == 0:
            mode = 'stand'
        else:
            mode = 'walk'
        return dict(
            policy=self.VERSION, mode=mode, t=round(self.sim_time, 3), walking=self._walking, fallen=self.fallen,
            v_req=round(self.req[0], 3), w_req=round(self.req[1], 3),
            v_cmd=round(self.applied[0], 3), w_cmd=round(self.applied[1], 3),
            v_gov=round(float(getattr(e, 'v_gov', e.v_cmd)[0]), 3),
            v_ref=round(float(e.v_ref[0]), 3), w_ref=round(float(e.w_ref[0]), 3),
            v_meas=round(vx, 3), w_meas=round(self._wz_f, 3),
            lean_deg=round(self._lean_f, 1), lean_target_deg=round(float(np.degrees(getattr(e, 'phi', np.zeros(1))[0])), 1),
            alpha=round(alpha, 3), phase=round(float(e.ph[0]), 3),
            p_leg_w=round(float(e.P_elec[0]), 1),
            kp_scale=round(float(e.kp_scale[0].mean()), 3),
            foot_force_bw=round(float(self.info.get('impact', [0])[0] if run else e.impact[0] / (35.7 * 9.81)), 3),
            tau_sat=round(float(getattr(e, 'tau_sat', np.zeros(1))[0]) if run else 0.0, 3),
            z=round(float(q[2]), 3), x=round(float(q[0]), 3), y=round(float(q[1]), 3), yaw=round(yaw, 3),
            last_switch=self.last_switch, envelope=self.envelope,
        )


class EcoWalk56Sim(EcoWalk55Sim):
    """v5.6.3 (main of k1-mp-ecowalk-public since 2026-10-08): symmetric walking and running policies, strict static
    friction, re-stance after stopping, running governor v |w| <= 2.5 m/s^2 (k1_mp_gait56)."""
    FOLDER, MODULE, CLASS, VERSION = 'k1_mp_gait56', 'gait56', 'Gait56', 'v5.6.3'
    name = 'v5.6.3 walk+run'
    A_LAT = 2.5
