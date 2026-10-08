# SPDX-License-Identifier: Apache-2.0
"""MuJoCo K1 (ecowalk turn policy) + rosbridge + browser teleop UI (+ optional RViz).

ros2 launch k1_ecowalk_ros teleop_sim.launch.py ecowalk_root:=$HOME/k1-mp-ecowalk-public
then open http://localhost:8080  (from another PC: http://<sim-pc-ip>:8080)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def robot_description(root):
    desc = os.path.join(root, 'ai_sapiens', 'ai_sapiens_description')
    with open(os.path.join(desc, 'urdf', 'k1_rev1', 'k1_mp.urdf')) as f:
        urdf = f.read()
    # the ecowalk repo ships the description without a ROS package; resolve meshes by absolute path
    # (avoids clashing with ROBOTIS' own ai_sapiens_description if it is in the same workspace)
    return urdf.replace('package://ai_sapiens_description/', f'file://{desc}/')


def setup(context):
    root = os.path.abspath(os.path.expanduser(LaunchConfiguration('ecowalk_root').perform(context)))
    share = get_package_share_directory('k1_ecowalk_ros')
    port = LaunchConfiguration('http_port').perform(context)
    return [
        Node(package='k1_ecowalk_ros', executable='k1_sim', output='screen',
             parameters=[{'ecowalk_root': root,
                          'policy_set': LaunchConfiguration('policy_set'),
                          'policy': LaunchConfiguration('policy'),
                          'walk_policy': LaunchConfiguration('walk_policy'),
                          'run_policy': LaunchConfiguration('run_policy'),
                          'viewer': LaunchConfiguration('viewer'),
                          'real_time_factor': LaunchConfiguration('real_time_factor')}]),
        Node(package='robot_state_publisher', executable='robot_state_publisher', output='log',
             parameters=[{'robot_description': robot_description(root), 'use_sim_time': True}]),
        IncludeLaunchDescription(AnyLaunchDescriptionSource(os.path.join(
            get_package_share_directory('rosbridge_server'), 'launch', 'rosbridge_websocket_launch.xml')),
            launch_arguments={'port': LaunchConfiguration('ws_port')}.items()),
        ExecuteProcess(cmd=['python3', '-m', 'http.server', port, '--directory', os.path.join(share, 'web')],
                       output='log'),
        Node(package='rviz2', executable='rviz2', output='log', condition=IfCondition(LaunchConfiguration('rviz')),
             arguments=['-d', os.path.join(share, 'config', 'k1.rviz')], parameters=[{'use_sim_time': True}]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('ecowalk_root', default_value=os.environ.get(
            'ECOWALK_ROOT', os.path.expanduser('~/k1-mp-ecowalk-public'))),
        DeclareLaunchArgument('policy_set', default_value='v56', description="'v56' (level-ish walking) / 'v55' walk<->run + turning + braking, 'v3' walking turn"),
        DeclareLaunchArgument('policy', default_value='k1_mp_turn/runs/final/model.pt', description='v3 policy'),
        DeclareLaunchArgument('walk_policy', default_value='', description='walking policy (default: <folder of the policy set>/runs/final/walk.pt)'),
        DeclareLaunchArgument('run_policy', default_value='', description='running policy (default: <folder of the policy set>/runs/final/run.pt)'),
        DeclareLaunchArgument('viewer', default_value='true', description='MuJoCo passive viewer window'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('real_time_factor', default_value='1.0'),
        DeclareLaunchArgument('ws_port', default_value='9090'),
        DeclareLaunchArgument('http_port', default_value='8080'),
        OpaqueFunction(function=setup),
    ])
