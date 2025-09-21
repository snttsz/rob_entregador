import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # --- CONFIGURAÇÃO DOS CAMINHOS ---
    pkg_rob_entregador = get_package_share_directory('rob_entregador')
    pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    pkg_turtlebot3_nav2 = get_package_share_directory('turtlebot3_navigation2')

    world_path = os.path.join(pkg_rob_entregador, 'worlds', 'meu_armazem.world')
    map_path = os.path.join(pkg_rob_entregador, 'config', 'mapa_vazio.yaml')
    params_path = os.path.join(pkg_rob_entregador, 'config', 'waffle_corrigido.yaml')
    robot_model = 'waffle'

    # --- ARQUIVOS DO ROBÔ ---
    # .urdf para o robot_state_publisher (TF tree)
    urdf_file = os.path.join(pkg_turtlebot3_gazebo, 'urdf', 'turtlebot3_' + robot_model + '.urdf')
    with open(urdf_file, 'r') as f:
        robot_description = {'robot_description': f.read()}
    
    # .sdf para o Gazebo (simulação)
    sdf_file = os.path.join(pkg_turtlebot3_gazebo, 'models', 'turtlebot3_' + robot_model, 'model.sdf')

    # --- DECLARAÇÃO DOS NÓS E PROCESSOS ---

    # 1. Iniciar Gazebo
    start_gzserver_cmd = ExecuteProcess(
        cmd=['gzserver', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', world_path],
        output='screen'
    )
    start_gzclient_cmd = ExecuteProcess(
        cmd=['gzclient'],
        output='screen'
    )

    # 2. Publicador do estado do robô (para o ROS)
    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description]
    )

    # 3. Adicionar o robô no Gazebo usando o arquivo SDF
    start_spawn_entity_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', robot_model,
            '-file', sdf_file, # <--- MUDANÇA CRÍTICA AQUI
        ],
        output='screen'
    )
    
    # 4. Iniciar Nav2
    start_nav2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'True',
            'map': map_path,
            'params_file': params_path,
            'autostart': 'True'
        }.items()
    )

    # 5. Iniciar nosso nó de entrega
    start_entregador_node_cmd = Node(
        package='rob_entregador',
        executable='entregador_node',
        name='entregador_node',
        output='screen'
    )
    
    # --- Montagem da Launch Description ---
    ld = LaunchDescription()
    
    ld.add_action(start_gzserver_cmd)
    ld.add_action(start_gzclient_cmd)
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(start_spawn_entity_cmd)
    ld.add_action(start_nav2_cmd)
    ld.add_action(start_entregador_node_cmd)

    return ld