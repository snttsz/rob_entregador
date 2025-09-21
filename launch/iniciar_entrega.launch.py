import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    pkg_rob_entregador = get_package_share_directory('rob_entregador')
    pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')

    world_path = os.path.join(pkg_rob_entregador, 'worlds', 'sem_obstaculos_encomenda_atras.world')
    robot_model = 'waffle'

    urdf_file = os.path.join(pkg_turtlebot3_gazebo, 'urdf', 'turtlebot3_' + robot_model + '.urdf')
    with open(urdf_file, 'r') as f:
        robot_description = {'robot_description': f.read()}
    
    sdf_file = os.path.join(pkg_turtlebot3_gazebo, 'models', 'turtlebot3_' + robot_model, 'model.sdf')

    start_gzserver_cmd = ExecuteProcess(
        cmd=['gzserver', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', world_path],
        output='screen'
    )

    start_gzclient_cmd = ExecuteProcess(
        cmd=['gzclient', '--render-engine', 'ogre'],
        output='screen'
    )

    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description]
    )

    start_spawn_entity_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', robot_model, '-file', sdf_file],
        output='screen'
    )
    
    start_vision_node_cmd = Node(
        package='rob_entregador',
        executable='vision_node',
        name='vision_node',
        output='screen'
    )

    start_controller_node_cmd = Node(
        package='rob_entregador',
        executable='controller_node',
        name='controller_node',
        output='screen'
    )

    ld = LaunchDescription()
    
    ld.add_action(start_gzserver_cmd)
    ld.add_action(start_gzclient_cmd)
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(start_spawn_entity_cmd)
    ld.add_action(start_vision_node_cmd)
    ld.add_action(start_controller_node_cmd)

    return ld