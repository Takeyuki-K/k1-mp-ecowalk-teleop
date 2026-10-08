# SPDX-License-Identifier: Apache-2.0
"""ROS 2 node: K1 + MP toes in MuJoCo, driven by the trained ecowalk turn policy.

Sub  : /cmd_vel (geometry_msgs/Twist)  linear.x [m/s], angular.z [rad/s]   (linear.y ignored: not trained)
Srv  : /k1/start, /k1/stop (trained safe stop), /k1/brake (v5.5: hard braking), /k1/reset  (std_srvs/Trigger)
Policy set (param policy_set): 'v56' (default) / 'v55' walking<->running + turning + braking, 'v3' walking turn policy
Pub  : /clock, /joint_states, /odom (ground truth), TF odom->pelvis, /imu/data, /k1/status (JSON)
Time : simulation time (use_sim_time:=true for consumers). Physics is paced to wall clock * real_time_factor.
"""
import json
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from builtin_interfaces.msg import Time
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState, Imu
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster

from .sim_core import EcoWalkSim, default_root


def stamp(t):
    s = int(t)
    return Time(sec=s, nanosec=int(round((t - s) * 1e9)) % 1_000_000_000)


class K1SimNode(Node):
    def __init__(self):
        super().__init__('k1_ecowalk_sim')
        p = self.declare_parameter
        root = p('ecowalk_root', default_root()).value
        self.policy_set = p('policy_set', 'v56').value
        policy = p('policy', 'k1_mp_turn/runs/final/model.pt').value                 # v3
        walk55 = p('walk_policy', '').value or None                                    # v5.5 / v5.6
        run55 = p('run_policy', '').value or None
        self.rtf = float(p('real_time_factor', 1.0).value)
        self.cmd_timeout = float(p('cmd_timeout', 0.5).value)          # no cmd_vel -> v=w=0 (step in place)
        self.idle_stop = float(p('idle_stop_after', 5.0).value)        # zero command this long -> safe stop (<=0: off)
        self.auto_start = bool(p('auto_start', True).value)            # non-zero cmd_vel starts walking
        self.odom_frame = p('odom_frame', 'odom').value
        self.base_frame = p('base_frame', 'pelvis').value
        self.use_viewer = bool(p('viewer', False).value)

        if self.policy_set in ('v55', 'v56'):
            from .sim_core55 import EcoWalk55Sim, EcoWalk56Sim
            self.sim = (EcoWalk56Sim if self.policy_set == 'v56' else EcoWalk55Sim)(root, walk55, run55)
        else:
            self.sim = EcoWalkSim(root, policy)
        self.get_logger().info(f'policy set {self.policy_set}: {self.sim.policy_path}')
        self.lock = threading.Lock()
        self.last_cmd_wall = -1e9
        self.clock_t = 0.0      # ROS sim clock: monotonic across /k1/reset (time must never jump back)
        self.zero_since = time.monotonic()

        self.pub_clock = self.create_publisher(Clock, '/clock', 10)
        self.pub_js = self.create_publisher(JointState, 'joint_states', 10)
        self.pub_odom = self.create_publisher(Odometry, 'odom', 10)
        self.pub_imu = self.create_publisher(Imu, 'imu/data', 50)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_status = self.create_publisher(String, 'k1/status', latched)
        self.tf = TransformBroadcaster(self)
        self.create_subscription(Twist, 'cmd_vel', self.on_cmd, 10)
        self.create_service(Trigger, 'k1/start', self.srv_start)
        self.create_service(Trigger, 'k1/stop', self.srv_stop)
        self.create_service(Trigger, 'k1/reset', self.srv_reset)
        self.create_service(Trigger, 'k1/brake', self.srv_brake)

        self.viewer = None
        if self.use_viewer:
            try:
                import mujoco
                import mujoco.viewer
                self._mj = mujoco
                self.vdata = mujoco.MjData(self.sim.m)
                self.viewer = mujoco.viewer.launch_passive(self.sim.m, self.vdata)
            except Exception as e:                       # no display etc. -> keep running headless
                self.get_logger().warn(f'MuJoCo viewer disabled: {e}')

        self.running = True
        self.thread = threading.Thread(target=self.loop, daemon=True)
        self.thread.start()
        self.get_logger().info(f'K1 ecowalk sim running ({self.policy_set}, 50 Hz policy, 5 ms physics)')

    # ---------------------------------------------------------------- inputs
    def on_cmd(self, msg: Twist):
        with self.lock:
            self.sim.set_command(msg.linear.x, msg.angular.z)
            if abs(msg.linear.y) > 0.05 and 'lateral_not_trained' not in self.sim.envelope:
                self.sim.envelope.append('lateral_not_trained')
            self.last_cmd_wall = time.monotonic()
            if self.sim.applied != (0.0, 0.0):
                self.zero_since = None
                if self.auto_start and not self.sim.walking and not self.sim.fallen:
                    self.sim.start()
            elif self.zero_since is None:
                self.zero_since = time.monotonic()

    def srv_start(self, req, res):
        with self.lock:
            if self.sim.fallen:
                res.success, res.message = False, 'robot is down: call /k1/reset'
            else:
                self.sim.start(); self.zero_since = time.monotonic()
                res.success, res.message = True, 'walking (stepping in place until a speed is commanded)'
        return res

    def srv_stop(self, req, res):
        with self.lock:
            self.sim.set_command(0.0, 0.0); self.sim.stop()
        res.success, res.message = True, 'safe stop: decelerate -> step in place -> feet together'
        return res

    def srv_brake(self, req, res):
        with self.lock:
            if getattr(self.sim, 'has_brake', False):
                self.sim.set_command(0.0, 0.0); self.sim.brake()
                res.success, res.message = True, 'hard braking: brake to 1.8 m/s -> walk -> safe stop'
            else:
                self.sim.set_command(0.0, 0.0); self.sim.stop()
                res.success, res.message = True, 'this policy set has no hard braking: safe stop instead'
        return res

    def srv_reset(self, req, res):
        with self.lock:
            self.sim.reset()
        res.success, res.message = True, 'reset to standing pose'
        return res

    # ---------------------------------------------------------------- loop
    def loop(self):
        # torch's intra-op thread count is per thread: without this the policy inference in this worker thread
        # spawns a full OpenMP pool every step and the sim ran ~8x slower than in the main thread (measured)
        try:
            import torch
            torch.set_num_threads(1)
        except ImportError:
            pass
        period = self.sim.dt / max(self.rtf, 1e-3)
        nxt = time.perf_counter()
        k = 0
        overruns = 0
        while self.running and rclpy.ok():
            with self.lock:
                now = time.monotonic()
                if now - self.last_cmd_wall > self.cmd_timeout and self.sim.applied != (0.0, 0.0):
                    self.sim.set_command(0.0, 0.0)                     # deadman: keep stepping in place
                    self.zero_since = now
                if (self.idle_stop > 0 and self.sim.walking and self.zero_since is not None
                        and now - self.zero_since > self.idle_stop):
                    self.sim.stop()
                was_up = not self.sim.fallen
                self.sim.step()
                self.clock_t += self.sim.dt
                if was_up and self.sim.fallen:
                    self.get_logger().error(f'robot fell at t={self.sim.sim_time:.2f}s -> /k1/reset')
                self.publish(k)
            k += 1
            if self.viewer is not None:
                self.sync_viewer()
            nxt += period
            sl = nxt - time.perf_counter()
            if sl > 0:
                time.sleep(sl)
            else:
                overruns += 1
                if sl < -0.2:                                      # far behind: resync, do not burst
                    nxt = time.perf_counter()
                    self.get_logger().warn(f'sim slower than real time ({overruns} overruns)', throttle_duration_sec=5.0)

    def sync_viewer(self):
        if not self.viewer.is_running():
            self.viewer = None
            return
        mj = self._mj
        with self.lock:
            mj.mj_setState(self.sim.m, self.vdata, self.sim.env.state[0], self.sim.env.spec_state)
        mj.mj_forward(self.sim.m, self.vdata)
        with self.viewer.lock():
            self.viewer.cam.lookat[:] = self.vdata.qpos[0:3]
        self.viewer.sync()

    # ---------------------------------------------------------------- outputs
    def publish(self, k):
        sim = self.sim
        ts = stamp(self.clock_t)
        self.pub_clock.publish(Clock(clock=ts))

        q, qd = sim.joints()
        js = JointState()
        js.header.stamp = ts
        js.name = sim.joint_names
        js.position = q.tolist(); js.velocity = qd.tolist()
        self.pub_js.publish(js)

        pos, quat = sim.base_pose()                 # (w, x, y, z)
        v_loc, w_loc = sim.base_twist_local()
        tf = TransformStamped()
        tf.header.stamp = ts; tf.header.frame_id = self.odom_frame; tf.child_frame_id = self.base_frame
        tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = map(float, pos)
        r = tf.transform.rotation
        r.w, r.x, r.y, r.z = map(float, quat)
        self.tf.sendTransform(tf)

        od = Odometry()
        od.header = tf.header; od.child_frame_id = self.base_frame
        od.pose.pose.position.x, od.pose.pose.position.y, od.pose.pose.position.z = map(float, pos)
        od.pose.pose.orientation = r
        od.twist.twist.linear.x, od.twist.twist.linear.y, od.twist.twist.linear.z = map(float, v_loc)
        od.twist.twist.angular.x, od.twist.twist.angular.y, od.twist.twist.angular.z = map(float, w_loc)
        self.pub_odom.publish(od)               # ground truth (for evaluating VIO/LIO later, not as an input)

        iq, gyro, acc = sim.imu()
        imu = Imu()
        imu.header.stamp = ts; imu.header.frame_id = self.base_frame     # 'imu' site sits at the pelvis origin
        imu.orientation.w, imu.orientation.x, imu.orientation.y, imu.orientation.z = map(float, iq)
        imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z = map(float, gyro)
        imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z = map(float, acc)
        self.pub_imu.publish(imu)

        if k % 5 == 0:                                                     # 10 Hz
            self.pub_status.publish(String(data=json.dumps(sim.status())))

    def destroy_node(self):
        self.running = False
        self.thread.join(timeout=1.0)
        if self.viewer is not None:
            self.viewer.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = K1SimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
