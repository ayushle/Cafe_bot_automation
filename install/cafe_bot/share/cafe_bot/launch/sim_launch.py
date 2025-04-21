import os
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    pkg_path = os.path.join(get_package_share_directory('cafe_bot'))

    xacro_file = os.path.join(pkg_path, 'description', 'bot_structure.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    
    params = {'robot_description': robot_description_config.toxml(), 'use_sim_time': use_sim_time}

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    world_path = os.path.join(pkg_path, 'world', 'cafe.world')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={'world': world_path}.items()
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'my_bot',
            '-x', '10.0',
            '-y', '2.0',
            '-z', '1.0',
            '-Y', '3.14'
        ],
        output='screen'
    )

    slam_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')]),
        launch_arguments={
            'slam_params_file': os.path.join(pkg_path, 'config', 'mapper_params_online_async.yaml'),
            'use_sim_time': use_sim_time
        }.items()
    )

    nav_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('nav2_bringup'), 'launch', 'rviz_launch.py')])
    )

    # 3-second delay then launch nav2
    navigation_node = TimerAction(
        period=3.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'gnome-terminal', '--',
                    'ros2', 'launch', 'nav2_bringup', 'navigation_launch.py',
                    'params_file:=./src/cafe_bot/config/nav2_params.yaml',
                    'use_sim_time:=true'
                ],
                output='screen'
            )
        ]
    )

    # Run `ros2 run cafe_bot order`
    run_order_node = ExecuteProcess(
        cmd=['ros2', 'run', 'cafe_bot', 'order'],
        output='screen'
    )

    # Run `ros2 run cafe_bot task`
    run_task_node = ExecuteProcess(
        cmd=['ros2', 'run', 'cafe_bot', 'task'],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),

        node_robot_state_publisher,
        gazebo,
        spawn_entity,
        slam_node,
        nav_bringup,
        navigation_node,
        run_order_node,
        run_task_node,
    ])
