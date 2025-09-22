# SEU ARQUIVO: launch/automated_test.launch.py

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_rob_entregador = get_package_share_directory('rob_entregador')

    # --- Declaração dos mesmos argumentos que temos no test_runner.py ---
    world_arg = DeclareLaunchArgument(
        'world',
        description='Caminho completo para o arquivo de mundo do Gazebo'
    )
    timeout_arg = DeclareLaunchArgument(
        'timeout',
        default_value='300.0',
        description='Tempo máximo em segundos para o teste'
    )

    # --- Lógica Principal ---
    # 1. Inclui o seu launch file original, passando o argumento 'world' para ele
    iniciar_entrega_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_rob_entregador, 'launch', 'iniciar_entrega.launch.py')
        ),
        # Passa o valor do argumento 'world' para o launch file incluído
        launch_arguments={'world': LaunchConfiguration('world')}.items()
    )

    # 2. Adiciona APENAS o nó de estatísticas, que não faz parte da operação normal
    statistics_collector_node = Node(
        package='rob_entregador',
        executable='statistics_collector',
        name='statistics_collector_node',
        output='screen',
        parameters=[{'timeout_seconds': LaunchConfiguration('timeout')}]
    )

    return LaunchDescription([
        world_arg,
        timeout_arg,
        iniciar_entrega_launch, # Lança o seu setup completo
        statistics_collector_node, # Adiciona o coletor de estatísticas
    ])