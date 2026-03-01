import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro

def generate_launch_description():

    # 1. Specify the name of the package and the path to the xacro file
    pkg_name = 'jax_description'
    file_subpath = 'urdf/jax.xacro'
    xacro_file = os.path.join(get_package_share_directory(pkg_name), file_subpath)

    # 2. Process the xacro file to URDF
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    # 3. Configure the Robot State Publisher node
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw}]
    )

    # 4. Configure Joint State Publisher GUI (to move the legs manually)
    node_joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen'
    )

    # 5. Configure RViz
    node_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )

    # Launch them all!
    return LaunchDescription([
        node_robot_state_publisher,
        node_joint_state_publisher_gui,
        node_rviz
    ])