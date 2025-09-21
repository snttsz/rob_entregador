from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os

def generate_launch_description():
    pkg_rob_entregador = get_package_share_directory('rob_entregador')
    pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')

    world_path = os.path.join(pkg_rob_entregador, 'worlds', 'com_obstaculo_na_frente.world')
    robot_model = 'waffle'
    sdf_file = os.path.join(pkg_turtlebot3_gazebo, 'models', 'turtlebot3_' + robot_model, 'model.sdf')

    gazebo_launch_dir = os.path.join(get_package_share_directory('gazebo_ros'), 'launch')

    # Inclui o launch oficial do gazebo_ros
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_launch_dir, 'gazebo.launch.py')),
        launch_arguments={'world': world_path}.items()
    )

    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', robot_model, '-file', sdf_file],
        output='screen'
    )

    vision_node = Node(
        package='rob_entregador',
        executable='vision_node',
        name='vision_node',
        output='screen'
    )

    controller_node = Node(
        package='rob_entregador',
        executable='controller_node',
        name='controller_node',
        output='screen'
    )

    ld = LaunchDescription()
    ld.add_action(gazebo)
    ld.add_action(spawn_robot)
    ld.add_action(vision_node)
    ld.add_action(controller_node)

    return ld
