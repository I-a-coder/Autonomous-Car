import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg_dir = get_package_share_directory('robotics_project')

    launch_file_dir = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'), 'launch')
    ros_gz_sim = get_package_share_directory('ros_gz_sim')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # World and URDF paths from the package
    world = os.path.join(pkg_dir, 'worlds', 'city.world')
    prius_sdf = os.path.join(pkg_dir, 'urdf', 'prius_car.sdf')

    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': ['-r -s -v2 ', world],
            'on_exit_shutdown': 'true'
        }.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': '-g -v2 ',
            'on_exit_shutdown': 'true'
        }.items()
    )

    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    spawn_prius = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-world', 'city_world',
            '-name', 'prius',
            '-file', prius_sdf,
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.5',
            '-R', '0',
            '-P', '0',
            '-Y', '1.57'
        ],
        output='screen'
    )

    set_env_vars_resources = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(
            get_package_share_directory('turtlebot3_gazebo'),
            'models'))

    # --- ROS-Gazebo Bridges ---

    # Odometry bridge (Python): Gazebo /odom -> ROS /odom with frame correction
    bridge = Node(
        package='robotics_project',
        executable='gz_bridge',
        name='gz_bridge',
        output='screen'
    )

    # Command velocity bridge (ros_gz_bridge): ROS /cmd_vel -> Gazebo cmd_vel
    cmd_vel_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='cmd_vel_bridge',
        arguments=['/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist'],
        output='screen'
    )

    # --- ROS nodes ---
    rrt_planner_node = Node(
        package='robotics_project',
        executable='rrt_planner',
        name='rrt_planner',
        output='screen'
    )

    pure_pursuit_node = Node(
        package='robotics_project',
        executable='pure_pursuit_controller',
        name='pure_pursuit_controller',
        output='screen'
    )

    path_visualizer_node = Node(
        package='robotics_project',
        executable='path_visualizer',
        name='path_visualizer',
        output='screen'
    )

    safety_monitor_node = Node(
        package='robotics_project',
        executable='safety_monitor',
        name='safety_monitor',
        output='screen'
    )

    ld = LaunchDescription()
    ld.add_action(set_env_vars_resources)
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(spawn_prius)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(bridge)
    ld.add_action(cmd_vel_bridge)
    ld.add_action(rrt_planner_node)
    ld.add_action(pure_pursuit_node)
    ld.add_action(path_visualizer_node)
    ld.add_action(safety_monitor_node)
    return ld
