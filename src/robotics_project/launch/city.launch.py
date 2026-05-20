import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, SetEnvironmentVariable
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    world_path = os.path.expanduser(
        '~/robot_navigation_project/src/Autonomous-car/src/robotics_project/worlds/city.world')

    tb3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    burger_sdf  = os.path.join(tb3_gazebo, 'models', 'turtlebot3_burger', 'model.sdf')
    bridge_yaml = os.path.join(tb3_gazebo, 'params', 'turtlebot3_burger_bridge.yaml')

    return LaunchDescription([

        SetEnvironmentVariable('TURTLEBOT3_MODEL', 'burger'),

        # Step 1 — Start Gazebo with OUR city world
        ExecuteProcess(
            cmd=['gz', 'sim', '-r', world_path],
            output='screen'
        ),

        # Step 2 — Spawn burger after 4 seconds
        TimerAction(
            period=4.0,
            actions=[
                Node(
                    package='ros_gz_sim',
                    executable='create',
                    arguments=[
                        '-name', 'burger',
                        '-file', burger_sdf,
                        '-x', '0.0',
                        '-y', '0.0',
                        '-z', '0.1'
                    ],
                    output='screen',
                )
            ]
        ),

        # Step 3 — Bridge ROS2 <-> Gazebo topics
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='ros_gz_bridge',
                    executable='parameter_bridge',
                    arguments=[
                        '--ros-args', '-p',
                        f'config_file:={bridge_yaml}'
                    ],
                    output='screen',
                    respawn=True,
                )
            ]
        ),

    ])
