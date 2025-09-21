import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    # --- CONFIGURAÇÃO DOS CAMINHOS ---
    pkg_rob_entregador = get_package_share_directory('rob_entregador')
    pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')

    world_path = os.path.join(pkg_rob_entregador, 'worlds', 'meu_armazem.world')
    robot_model = 'waffle'

    # --- ARQUIVOS DO ROBÔ ---
    urdf_file = os.path.join(pkg_turtlebot3_gazebo, 'urdf', 'turtlebot3_' + robot_model + '.urdf')
    with open(urdf_file, 'r') as f:
        robot_description = {'robot_description': f.read()}
    
    sdf_file = os.path.join(pkg_turtlebot3_gazebo, 'models', 'turtlebot3_' + robot_model, 'model.sdf')

    # --- DECLARAÇÃO DOS NÓS E PROCESSOS ---

    # 1. Iniciar Gazebo
    start_gzserver_cmd = ExecuteProcess(
        cmd=['gzserver', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', world_path],
        output='screen'
    )
    start_gzclient_cmd = ExecuteProcess(
        cmd=['gzclient', '--render-engine', 'ogre'],
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
            '-file', sdf_file,
        ],
        output='screen'
    )
    
    # 4. Iniciar nosso nó de entrega
    start_entregador_node_cmd = Node(
        package='rob_entregador',
        executable='entregador_node',
        name='entregador_node',
        output='screen'
    )

    # --- NOVO: Iniciar o RQT Image View ---
    start_rqt_image_view_cmd = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        name='image_viewer',
        output='screen'
    )
    # --- FIM DA SEÇÃO NOVA ---
    
    # --- Montagem da Launch Description ---
    ld = LaunchDescription()
    
    ld.add_action(start_gzserver_cmd)
    ld.add_action(start_gzclient_cmd)
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(start_spawn_entity_cmd)
    ld.add_action(start_entregador_node_cmd)
    ld.add_action(start_rqt_image_view_cmd) # <-- NOVA AÇÃO ADICIONADA AQUI

    return ld