# k1-mp-ecowalk-teleop

ROS 2 (Jazzy) teleoperation of the **K1 MP Eco-Walk** policies in MuJoCo, with a browser UI over rosbridge.
The policies, environments and gait manager come from
[k1-mp-ecowalk-public](https://github.com/Takeyuki-K/k1-mp-ecowalk-public) (default: its `main`, v5.6.3).

> **Simulation only.** MuJoCo, not tested on a real robot. Independent personal research, not affiliated with or
> endorsed by ROBOTIS.

| package | content |
|---|---|
| [`k1_ecowalk_ros/`](k1_ecowalk_ros/README.md) | sim node (MuJoCo + policies, 50 Hz), `/cmd_vel` → walking / running / turning / stop / hard brake, `/k1/status`, `/joint_states`, `/odom`, `/imu/data`, TF, browser UI (rosbridge), RViz, Docker image (Jazzy on an x86 host) |

Quick start (details and Docker: [k1_ecowalk_ros/README.md](k1_ecowalk_ros/README.md)):
```bash
git clone https://github.com/Takeyuki-K/k1-mp-ecowalk-public.git ~/k1-mp-ecowalk-public
mkdir -p ~/k1_ecowalk_ws/src && git clone https://github.com/Takeyuki-K/k1-mp-ecowalk-teleop.git ~/k1_ecowalk_ws/src/k1-mp-ecowalk-teleop
cd ~/k1_ecowalk_ws && source /opt/ros/jazzy/setup.bash && colcon build --symlink-install && source install/setup.bash
ros2 launch k1_ecowalk_ros teleop_sim.launch.py          # browser: http://localhost:8080
```

Idea & direction: Takeyuki-K · Implementation generated with Claude (Anthropic) under Takeyuki-K's direction.
License: Apache-2.0 ([LICENSE](LICENSE)); `k1_ecowalk_ros/web/roslib.min.js` is roslibjs (BSD, see
`k1_ecowalk_ros/web/roslib.LICENSE`).
