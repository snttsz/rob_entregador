import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

def generate_launch_description():
    # --- 1. Localização dos Pacotes ---
    pkg_rob_entregador = get_package_share_directory('rob_entregador')
    pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # --- 2. Definição dos Argumentos ---
    # Argumento para o mundo. Padrão: 'com_obstaculo_na_frente.world'
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(pkg_rob_entregador, 'worlds', 'com_obstaculo_na_frente.world'),
        description='Caminho completo para o arquivo de mundo a ser usado'
    )

    # Argumento para ativar o modo de teste. Padrão: 'false'
    # Se for 'true', o nó de estatísticas será iniciado.
    test_mode_arg = DeclareLaunchArgument(
        'test_mode',
        default_value='false',
        description='Ativa o modo de teste, iniciando o nó de estatísticas.'
    )

    # Argumento para o timeout, usado apenas em modo de teste.
    timeout_arg = DeclareLaunchArgument(
        'timeout',
        default_value='300.0',
        description='Tempo máximo em segundos para o teste (usado apenas se test_mode=true)'
    )
    
    # --- 3. Inclusão do Gazebo ---
    # Sempre usa o tempo de simulação
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': LaunchConfiguration('world')}.items()
    )

    # --- 4. Definição dos Nós ---
    robot_model = 'waffle'
    sdf_file = os.path.join(pkg_turtlebot3_gazebo, 'models', 'turtlebot3_' + robot_model, 'model.sdf')
    
    # "Spawna" o robô no mundo
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', robot_model, '-file', sdf_file],
        output='screen'
    )

    # Nós básicos que sempre rodam
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

    # --- 5. Ações Condicionais para o Modo de Teste ---
    # Este grupo de ações só será executado se `test_mode` for 'true'
    test_actions = GroupAction(
        actions=[
            Node(
                package='rob_entregador',
                executable='statistics_collector',
                name='statistics_collector_node',
                output='screen',
                parameters=[{'timeout_seconds': LaunchConfiguration('timeout')}]
            )
        ],
        condition=IfCondition(LaunchConfiguration('test_mode'))
    )

    # --- 6. Montagem da Descrição de Lançamento ---
    ld = LaunchDescription()
    
    # Adiciona as declarações de argumentos
    ld.add_action(world_arg)
    ld.add_action(test_mode_arg)
    ld.add_action(timeout_arg)
    
    # Adiciona as ações e nós
    ld.add_action(gazebo)
    ld.add_action(spawn_robot)
    ld.add_action(vision_node)
    ld.add_action(controller_node)
    ld.add_action(test_actions) # Adiciona o grupo de ações de teste

    return ld