# Robô Entregador com ROS 2

Este projeto implementa um robô autônomo de entrega utilizando o ROS 2. O robô é capaz de navegar em um ambiente simulado no Gazebo, identificar uma encomenda, coletá-la e retornar à sua base.

## Arquitetura

O sistema é dividido em dois nós principais:

* **`vision_node`**: Responsável pelo processamento das imagens da câmera. Ele detecta a encomenda (vermelha) e obstáculos (azuis) e publica essas informações.
* **`controller_node`**: Contém a máquina de estados e a lógica de controle do robô. Ele utiliza as informações do `vision_node` e dos sensores do robô para tomar decisões de navegação.

## Como Executar

1.  **Construir o Pacote:**
    ```bash
    colcon build --packages-select rob_entregador
    ```

2.  **Configurar o Ambiente:**
    ```bash
    source install/setup.bash
    ```

3.  **Iniciar a Simulação:**
    ```bash
    ros2 launch rob_entregador iniciar_entrega.launch.py
    ```

## Detalhes dos Nós

### `vision_node`

* **Subscribers**: `/camera/image_raw`
* **Publishers**: `/encomenda_info`, `/obstaculo_detectado`, `/image_processed`

### `controller_node`

* **Subscribers**: `/encomenda_info`, `/obstaculo_detectado`, `/odom`, `/scan`
* **Publishers**: `/cmd_vel`