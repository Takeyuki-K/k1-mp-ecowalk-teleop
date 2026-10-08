# SPDX-License-Identifier: Apache-2.0
"""ROS-free core: runs the *trained* k1-mp-ecowalk turn policy (v3) in MuJoCo, one robot, 50 Hz.

Design rule: no re-implementation of physics / observation / action handling.
The training environment class (K1TurnBatch) is imported from the ecowalk repo and used as-is with n=1,
so the policy sees exactly the observation it was trained on (same dt, same PD law, same reference library,
same safe-stop logic). This module only:
  * maps a teleop command (v, w) onto the env's command variables (cmd / v_cmd / w_cmd),
  * reports when the command is outside the distribution the policy was trained on (no silent cheating),
  * exposes state for ROS publishing.
"""
import os
import sys
import numpy as np

# ---- trained command distribution (k1_mp_turn/k1env_turn.py _new_motion) ----
V_MAX = 1.35          # straight walking 0.30 .. 1.35 m/s
V_MIN_WALK = 0.30
V_TURN_MAX = 1.0      # walking turns were trained at v 0.3 .. 1.0
W_MAX = 1.0           # |yaw rate| <= 1.0 rad/s
DEADBAND_V = 0.05
DEADBAND_W = 0.05


def default_root():
    return os.environ.get('ECOWALK_ROOT', os.path.expanduser('~/k1-mp-ecowalk-public'))


class EcoWalkSim:
    def __init__(self, root=None, policy='k1_mp_turn/runs/final/model.pt', seed=11, torch_threads=1):
        root = os.path.abspath(os.path.expanduser(root or default_root()))
        pkg = os.path.join(root, 'k1_mp_turn')
        if not os.path.isfile(os.path.join(pkg, 'k1env_turn.py')):
            raise FileNotFoundError(f'k1-mp-ecowalk-public not found at {root} (set ECOWALK_ROOT or ecowalk_root)')
        if pkg not in sys.path:
            sys.path.insert(0, pkg)
        import torch
        import mujoco
        from k1env import DEFAULT_POSE, ACT_JOINTS
        from k1env_turn import K1TurnBatch
        from ppo_turn import ACEco
        torch.set_num_threads(torch_threads)
        self._torch, self._mujoco = torch, mujoco
        self.DEFAULT_POSE = DEFAULT_POSE
        self.root = root
        self.policy_path = policy if os.path.isabs(policy) else os.path.join(root, policy)

        # identical to eval_turn.make_env: no randomisation, no pushes, scripted commands
        env = K1TurnBatch(1, stage=2, randomize=False, seed=seed)
        env.pushes = False
        env.scripted = True
        self.env = env
        self.m = env.m
        oa, oc = env.obs()
        self.net = ACEco(oa.shape[1], oc.shape[1], env.nact)
        self.net.load_state_dict(torch.load(self.policy_path, map_location='cpu')['model'])
        self.net.eval()

        m = self.m
        self.joint_names = list(ACT_JOINTS) + ['left_mp_joint', 'right_mp_joint']
        self._qadr = np.array([m.jnt_qposadr[m.joint(j).id] for j in self.joint_names])
        self._dadr = np.array([m.jnt_dofadr[m.joint(j).id] for j in self.joint_names])
        self.dt = 1.0 / env.CTRL_HZ
        self.reset()

    # ------------------------------------------------------------------ control
    def reset(self):
        """Standing start, identical to eval_turn.stand_all (without the per-robot pose noise)."""
        env, m, mj = self.env, self.m, self._mujoco
        d = mj.MjData(m)
        mj.mj_resetData(m, d)
        d.qpos[:] = m.key_qpos[0]
        d.qpos[env.qadr] = self.DEFAULT_POSE
        d.qpos[2] = env.ref.z_stand + 0.002
        mj.mj_forward(m, d)
        st = np.zeros(env.nstate)
        mj.mj_getState(m, d, st, env.spec_state)
        env.state[0] = st
        env.sdata[0] = d.sensordata
        for k in ('ph', 'alpha', 'cmd', 'v_cmd', 'v_ref', 'w_cmd', 'w_ref', 'yaw_t'):
            getattr(env, k)[:] = 0
        env.yaw_t[:] = env.yaw()
        env.t[:] = 0
        env.last_a[:] = 0
        env.last_a2[:] = 0
        env.next_evt[:] = 10 ** 9
        self.fallen = False
        self.sim_time = 0.0
        self.req = (0.0, 0.0)
        self.applied = (0.0, 0.0)
        self.envelope = []
        self._oa, _ = env.obs()
        self.info = {}
        self.action = np.zeros(env.nact)

    def start(self):
        if not self.fallen:
            self.env.cmd[:] = 1.0

    def stop(self):
        """Trained safe stop: decelerate to stepping in place, then bring the feet together."""
        self.env.cmd[:] = 0.0

    @property
    def walking(self):
        return bool(self.env.cmd[0] > 0.5)

    def set_command(self, v, w):
        """Map a teleop request onto the trained command envelope.

        Hard limits (never sent to the policy): backward walking (not trained), |w| > 1.0.
        Soft envelope (sent, but flagged): 0 < v < 0.30 (only seen during ramps) and v > 1.0 while turning.
        """
        v, w = float(v), float(w)
        self.req = (v, w)
        notes = []
        if abs(v) < DEADBAND_V:
            v = 0.0
        if abs(w) < DEADBAND_W:
            w = 0.0
        if v < 0.0:
            notes.append('backward_not_trained')
            v = 0.0
        if v > V_MAX:
            notes.append('v_clamped_1.35')
            v = V_MAX
        if abs(w) > W_MAX:
            notes.append('w_clamped_1.0')
            w = float(np.sign(w)) * W_MAX
        if 0.0 < v < V_MIN_WALK:
            notes.append('v_below_trained_0.30')
        if v > V_TURN_MAX and w != 0.0:
            notes.append('turn_above_trained_1.0mps')
        self.envelope = notes
        self.applied = (v, w)
        self.env.v_cmd[:] = v
        self.env.w_cmd[:] = w

    def step(self):
        """One 20 ms control step (4 x 5 ms physics). Returns False if the robot is down."""
        if self.fallen:
            return False
        env, torch = self.env, self._torch
        with torch.no_grad():
            a = self.net.dist(torch.from_numpy(self._oa)).mean.numpy().astype(np.float64)   # deterministic
        _, term, _, info = env.step(a)
        self.action = a[0]
        self.info = info
        self._oa, _ = env.obs()
        self.sim_time += self.dt
        if term[0]:
            self.fallen = True
            env.cmd[:] = 0.0
        return not self.fallen

    # ------------------------------------------------------------------ state
    def qpos(self):
        return self.env.qpos()[0]

    def qvel(self):
        return self.env.qvel()[0]

    def sensor(self, name):
        a, d = self.env.sens[name]
        return self.env.sdata[0, a:a + d].copy()

    def base_pose(self):
        """pelvis position (world) and quaternion (w, x, y, z)."""
        q = self.qpos()
        return q[0:3].copy(), q[3:7].copy()

    def base_twist_local(self):
        """linear velocity in pelvis frame (velocimeter at the imu site) and angular velocity (local)."""
        return self.sensor('base_linvel'), self.qvel()[3:6].copy()

    def imu(self):
        """MuJoCo sensors at the 'imu' site: quat (w,x,y,z), gyro [rad/s], accelerometer [m/s^2, incl. gravity]."""
        return self.sensor('imu_quat'), self.sensor('imu_gyro'), self.sensor('imu_acc')

    def joints(self):
        return self.qpos()[self._qadr].copy(), self.qvel()[self._dadr].copy()

    def status(self):
        e = self.env
        q = self.qpos()
        yaw = float(e.yaw()[0])
        vw = self.qvel()[0:3]
        vx = float(np.cos(yaw) * vw[0] + np.sin(yaw) * vw[1])
        return dict(
            t=round(self.sim_time, 3), walking=self.walking, fallen=self.fallen,
            v_req=round(self.req[0], 3), w_req=round(self.req[1], 3),
            v_cmd=round(self.applied[0], 3), w_cmd=round(self.applied[1], 3),
            v_ref=round(float(e.v_ref[0]), 3), w_ref=round(float(e.w_ref[0]), 3),
            v_meas=round(vx, 3), w_meas=round(float(self.qvel()[5]), 3),
            alpha=round(float(e.alpha[0]), 3), phase=round(float(e.ph[0]), 3),
            p_leg_w=round(float(e.P_elec[0]), 1),
            kp_scale=round(float(e.kp_scale[0].mean()), 3),
            foot_force_bw=round(float(e.impact[0] / (35.706 * 9.81)), 3),
            z=round(float(q[2]), 3), x=round(float(q[0]), 3), y=round(float(q[1]), 3), yaw=round(yaw, 3),
            envelope=self.envelope,
        )
