import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    # 1. Paths and Robot Description
    pkg_description = get_package_share_directory('jax_description')
    pkg_controller = get_package_share_directory('jax_controller')
    
    xacro_file = os.path.join(pkg_description, 'urdf', 'jax.xacro')
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    # 1. The Clock Bridge Node
    # This replaces the manual "ros2 run ros_gz_bridge..." command
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    # 2. Robot State Publisher (The "Ghost" in the machine)
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw, 'use_sim_time': True}]
    )

    # 3. Gazebo Launch (The World)
    # Using 'gz_sim' for Gazebo Harmonic (Jazzy default)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(), # -r starts it immediately
    )

    # 4. Spawn the Robot (The "Physical" Drop)
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description', '-name', 'jax', '-z', '0.19'],
    )

    # 5. Load Controllers (The "Brain")
    # We load the broadcaster first, then the trajectory controller
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'],
        output='screen'
    )

    load_joint_trajectory_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_trajectory_controller'],
        output='screen'
    )

    return LaunchDescription([
        # Start Gazebo and Robot State Publisher
        gazebo,
        node_robot_state_publisher,
        spawn_entity,
        clock_bridge,
        # Wait for the robot to spawn before loading controllers
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_joint_state_broadcaster],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[load_joint_trajectory_controller],
            )
        ),
    ])