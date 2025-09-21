"""
Nó de Controle para o Robô Entregador.

Este nó implementa a máquina de estados que governa o comportamento do robô.
Ele se baseia nas informações de visão (detecção de encomendas e obstáculos),
nos dados de odometria e no escaneamento a laser para navegar no ambiente,
coletar a encomenda e retornar à base.

Subscribers:
- /encomenda_info (geometry_msgs/Point): Informações sobre a encomenda detectada.
- /obstaculo_detectado (std_msgs/Bool): Flag que indica a detecção de um obstáculo.
- /odom (nav_msgs/Odometry): Dados de odometria do robô.
- /scan (sensor_msgs/LaserScan): Leituras do sensor a laser.

Publishers:
- /cmd_vel (geometry_msgs/Twist): Comandos de velocidade para o robô.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from enum import Enum
import time
import math

class Estados(Enum):
    """
    Enumeração dos estados possíveis da máquina de estados do robô.
    """
    BUSCANDO_ENCOMENDA = 1
    APROXIMANDO_ENCOMENDA = 2
    COLETANDO_ENCOMENDA = 3
    RETORNANDO_PARA_BASE = 4
    ENTREGANDO_ENCOMENDA = 5
    OBSTACULO_DETECTADO = 6

class ControllerNode(Node):
    """
    Classe que define o nó de controle do robô.
    """
    def __init__(self):
        """
        Construtor do ControllerNode.
        """
        super().__init__('controller_node')
        self.estado = Estados.BUSCANDO_ENCOMENDA
        self.get_logger().info(f'Iniciando em estado: {self.estado.name}')

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Subscribers
        self.encomenda_sub = self.create_subscription(Point, '/encomenda_info', self.encomenda_callback, 10)
        self.obstaculo_sub = self.create_subscription(Bool, '/obstaculo_detectado', self.obstaculo_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        # Timer para a máquina de estados
        self.timer = self.create_timer(0.1, self.rodar_maquina_estados)

        # Variáveis de estado
        self.encomenda_detectada = False
        self.centro_x_encomenda = 0
        self.obstaculo_detectado = False
        self.pose_atual = None
        self.scan_atual = None

    # ... (callbacks e máquina de estados como no `entregador_node` original, mas adaptados) ...
    def mudar_estado(self, novo_estado):
        if self.estado != novo_estado:
            self.get_logger().info(f'Mudando de estado: {self.estado.name} -> {novo_estado.name}')
            self.estado = novo_estado

    def encomenda_callback(self, msg: Point):
        """Callback para informações da encomenda."""
        if msg.z > 0: # Área da encomenda
            self.encomenda_detectada = True
            self.centro_x_encomenda = msg.x
        else:
            self.encomenda_detectada = False

    def obstaculo_callback(self, msg: Bool):
        """Callback para detecção de obstáculo."""
        self.obstaculo_detectado = msg.data
        if self.obstaculo_detectado:
            self.mudar_estado(Estados.OBSTACULO_DETECTADO)

    def odom_callback(self, msg: Odometry):
        """Callback para odometria."""
        self.pose_atual = msg.pose.pose

    def scan_callback(self, msg: LaserScan):
        """Callback para o scan a laser."""
        self.scan_atual = msg

    def rodar_maquina_estados(self):
        """
        Executa a lógica da máquina de estados em cada ciclo do timer.
        """
        if self.scan_atual is None:
            return
        
        distancia_frontal = min(min(self.scan_atual.ranges[0:15]), min(self.scan_atual.ranges[345:360]))

        if self.estado == Estados.OBSTACULO_DETECTADO:
            if not self.obstaculo_detectado:
                self.mudar_estado(Estados.BUSCANDO_ENCOMENDA)
            else:
                self.cmd_vel_pub.publish(Twist())

        elif self.estado == Estados.BUSCANDO_ENCOMENDA:
            if self.obstaculo_detectado and distancia_frontal < 0.4:
                self.mudar_estado(Estados.OBSTACULO_DETECTADO)
            elif self.encomenda_detectada:
                self.mudar_estado(Estados.APROXIMANDO_ENCOMENDA)
            else:
                twist = Twist(); twist.angular.z = 0.3; self.cmd_vel_pub.publish(twist)

        # ... (Restante da máquina de estados, similar à versão original)
        elif self.estado == Estados.APROXIMANDO_ENCOMENDA:
            if self.obstaculo_detectado and distancia_frontal < 0.4:
                self.mudar_estado(Estados.OBSTACULO_DETECTADO)
            elif not self.encomenda_detectada:
                self.mudar_estado(Estados.BUSCANDO_ENCOMENDA)
            elif distancia_frontal < 0.5:
                self.mudar_estado(Estados.COLETANDO_ENCOMENDA)
            else:
                erro = self.centro_x_encomenda - 320
                twist = Twist()
                twist.angular.z = -0.002 * erro
                twist.linear.x = 0.1
                self.cmd_vel_pub.publish(twist)

        elif self.estado == Estados.COLETANDO_ENCOMENDA:
            self.cmd_vel_pub.publish(Twist())
            self.get_logger().info('Encomenda COLETADA! Retornando...')
            time.sleep(1)
            self.mudar_estado(Estados.RETORNANDO_PARA_BASE)

        elif self.estado == Estados.RETORNANDO_PARA_BASE:
            # ... (Lógica de retorno para a base)
            if self.pose_atual is None: return

            if distancia_frontal < 0.4:
                twist = Twist()
                twist.linear.x = 0.0
                twist.angular.z = 0.4
                self.cmd_vel_pub.publish(twist)
                return

            pos_x, pos_y = self.pose_atual.position.x, self.pose_atual.position.y
            distancia_da_base = math.sqrt(pos_x**2 + pos_y**2)

            if distancia_da_base < 0.2:
                self.mudar_estado(Estados.ENTREGANDO_ENCOMENDA)
            else:
                # Navegação simples em direção à base (0,0)
                angulo_para_base = math.atan2(-pos_y, -pos_x)
                qz, qw = self.pose_atual.orientation.z, self.pose_atual.orientation.w
                angulo_atual = 2 * math.atan2(qz, qw)
                erro_angulo = angulo_para_base - angulo_atual

                twist = Twist()
                if abs(erro_angulo) > 0.1:
                    twist.angular.z = 0.3 if erro_angulo > 0 else -0.3
                else:
                    twist.linear.x = 0.15
                self.cmd_vel_pub.publish(twist)

        elif self.estado == Estados.ENTREGANDO_ENCOMENDA:
            self.cmd_vel_pub.publish(Twist())
            self.get_logger().info('Encomenda entregue! Missão cumprida.')
            self.timer.cancel()


def main(args=None):
    rclpy.init(args=args)
    controller_node = ControllerNode()
    rclpy.spin(controller_node)
    controller_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()