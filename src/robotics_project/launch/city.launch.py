import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction,  SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    world_path = os.path.expanduser(
        '~/robot_navigation_project/src/Autonomous-car/src/robotics_project/worlds/city.world')

    ros_gz_sim = get_package_share_directory('ros_gz_sim')
    tb3_gazebo  = get_package_share_directory('turtlebot3_gazebo')

    urdf_path = os.path.expanduser(
        '~/robot_navigation_project/src/Autonomous-car/src/robotics_project/urdf/car.urdf')

    with open(urdf_path, 'r') as f:
        robot_desc = f.read()

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': f'-r -s -v2 {world_path}',
            'on_exit_shutdown': 'true'
        }.items()
    )

    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': '-g -v2',
            'on_exit_shutdown': 'true'
        }.items()
    )

    spawn_tb3 = TimerAction(
        period=3.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(tb3_gazebo, 'launch', 'spawn_turtlebot3.launch.py')
                ),
                launch_arguments={
                    'x_pose': '0.0',
                    'y_pose': '0.0',
                }.items()
            )
        ]
    )

    return LaunchDescription([
        SetEnvironmentVariable('TURTLEBOT3_MODEL', 'burger'),
        gzserver,
        gzclient,
        spawn_tb3,
    ])
