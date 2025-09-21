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
    BUSCANDO_ENCOMENDA = 1
    APROXIMANDO_ENCOMENDA = 2
    COLETANDO_ENCOMENDA = 3
    RETORNANDO_PARA_BASE = 4
    ENTREGANDO_ENCOMENDA = 5
    OBSTACULO_DETECTADO = 6

class ControllerNode(Node):
    def __init__(self):
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

        # Timer
        self.timer = self.create_timer(0.1, self.rodar_maquina_estados)

        # Variáveis de estado
        self.encomenda_detectada = False
        self.encomenda_coletada = False
        self.centro_x_encomenda = 0
        self.obstaculo_detectado = False
        self.pose_atual = None
        self.scan_atual = None

        # Para contorno de obstáculo
        self.lado_desvio = None
        self.tentativas_lado = 0

    # -------------------- FUNÇÕES AUXILIARES --------------------
    def mudar_estado(self, novo_estado):
        if self.estado != novo_estado:
            self.get_logger().info(f'Mudando de estado: {self.estado.name} -> {novo_estado.name}')
            self.estado = novo_estado

    def contornar_obstaculo(self):
        """Retorna Twist para contornar obstáculo"""
        n = len(self.scan_atual.ranges)
        
        # Calcula distância frontal
        distancia_frontal = min(
            min(self.scan_atual.ranges[0:int(n*0.04)]),
            min(self.scan_atual.ranges[int(n*0.96):n])
        )

        # Inicializa lado se ainda não definido
        if self.lado_desvio is None:
            ranges_left = self.scan_atual.ranges[int(n*0.16):int(n*0.33)]
            ranges_right = self.scan_atual.ranges[int(n*0.66):int(n*0.83)]
            media_left = sum([r for r in ranges_left if r > 0]) / len(ranges_left)
            media_right = sum([r for r in ranges_right if r > 0]) / len(ranges_right)
            self.lado_desvio = 'esquerda' if media_left > media_right else 'direita'
            self.tentativas_lado = 0
            self.get_logger().info(f'Lado do desvio escolhido: {self.lado_desvio}')

        twist = Twist()
        twist.linear.x = 0.1
        twist.angular.z = 0.5 if self.lado_desvio == 'esquerda' else -0.5

        self.tentativas_lado += 1

        # Se obstáculo muito próximo ou tentativas excedidas, recua e recalcula lado
        if self.tentativas_lado > 50 or distancia_frontal <= 0.2:
            self.get_logger().info('Obstáculo muito próximo! Recuando 1 metro e recalculando lado...')
            
            # Recua 1 metro
            recuo = Twist()
            recuo.linear.x = -0.15  # velocidade negativa
            tempo_recuo = 0.5 / 0.15  # distância/velocidade ~ 1 metro
            self.cmd_vel_pub.publish(recuo)
            time.sleep(tempo_recuo)
            self.cmd_vel_pub.publish(Twist())  # para o robô

            # Recalcula lado
            ranges_left = self.scan_atual.ranges[int(n*0.16):int(n*0.33)]
            ranges_right = self.scan_atual.ranges[int(n*0.66):int(n*0.83)]
            media_left = sum([r for r in ranges_left if r > 0]) / len(ranges_left)
            media_right = sum([r for r in ranges_right if r > 0]) / len(ranges_right)
            self.lado_desvio = 'esquerda' if media_left > media_right else 'direita'
            self.tentativas_lado = 0
            self.get_logger().info(f'Lado do desvio recalculado: {self.lado_desvio}')

        return twist

    # -------------------- CALLBACKS --------------------
    def encomenda_callback(self, msg: Point):
        if msg.z > 0:
            self.encomenda_detectada = True
            self.centro_x_encomenda = msg.x
        else:
            self.encomenda_detectada = False

    def obstaculo_callback(self, msg: Bool):
        self.obstaculo_detectado = msg.data
        if self.obstaculo_detectado and not self.encomenda_coletada:
            self.mudar_estado(Estados.OBSTACULO_DETECTADO)

    def odom_callback(self, msg: Odometry):
        self.pose_atual = msg.pose.pose

    def scan_callback(self, msg: LaserScan):
        self.scan_atual = msg

    # -------------------- MÁQUINA DE ESTADOS --------------------
    def rodar_maquina_estados(self):
        if self.scan_atual is None:
            return

        n = len(self.scan_atual.ranges)
        distancia_frontal = min(
            min(self.scan_atual.ranges[0:int(n*0.04)]),
            min(self.scan_atual.ranges[int(n*0.96):n])
        )
        self.get_logger().info(f'Distancia frontal: {distancia_frontal:.2f}', throttle_duration_sec=0.5)

        twist = Twist()

        # ---------- OBSTACULO_DETECTADO ----------
        if self.estado == Estados.OBSTACULO_DETECTADO:
            if not self.obstaculo_detectado:
                self.mudar_estado(Estados.RETORNANDO_PARA_BASE if self.encomenda_coletada else Estados.BUSCANDO_ENCOMENDA)
                self.lado_desvio = None
                self.tentativas_lado = 0
            else:
                twist = self.contornar_obstaculo()
                self.cmd_vel_pub.publish(twist)
                return

        # ---------- BUSCANDO_ENCOMENDA ----------
        elif self.estado == Estados.BUSCANDO_ENCOMENDA:
            if self.obstaculo_detectado and distancia_frontal < 0.4:
                self.mudar_estado(Estados.OBSTACULO_DETECTADO)
            elif self.encomenda_detectada and not self.encomenda_coletada:
                self.mudar_estado(Estados.APROXIMANDO_ENCOMENDA)
            else:
                twist.angular.z = 0.3
                self.cmd_vel_pub.publish(twist)

        # ---------- APROXIMANDO_ENCOMENDA ----------
        elif self.estado == Estados.APROXIMANDO_ENCOMENDA:
            if self.obstaculo_detectado and distancia_frontal < 0.4:
                self.mudar_estado(Estados.OBSTACULO_DETECTADO)
            elif self.encomenda_coletada:
                self.mudar_estado(Estados.RETORNANDO_PARA_BASE)
            elif not self.encomenda_detectada:
                self.mudar_estado(Estados.BUSCANDO_ENCOMENDA)
            elif distancia_frontal < 0.4:
                self.mudar_estado(Estados.COLETANDO_ENCOMENDA)
            else:
                erro = self.centro_x_encomenda - 320
                twist.angular.z = -0.002 * erro
                twist.linear.x = 0.3 if distancia_frontal >= 0.8 else 0.1
                self.cmd_vel_pub.publish(twist)

        # ---------- COLETANDO_ENCOMENDA ----------
        elif self.estado == Estados.COLETANDO_ENCOMENDA:
            self.cmd_vel_pub.publish(Twist())
            self.get_logger().info('Encomenda COLETADA! Retornando para base...')
            self.encomenda_coletada = True
            time.sleep(3)
            self.mudar_estado(Estados.RETORNANDO_PARA_BASE)

        # ---------- RETORNANDO_PARA_BASE ----------
        elif self.estado == Estados.RETORNANDO_PARA_BASE:
            if self.pose_atual is None:
                return

            if self.obstaculo_detectado and distancia_frontal < 0.4:
                twist = self.contornar_obstaculo()
                self.cmd_vel_pub.publish(twist)
                return

            pos_x, pos_y = self.pose_atual.position.x, self.pose_atual.position.y
            distancia_da_base = math.sqrt(pos_x**2 + pos_y**2)

            if distancia_da_base < 0.2:
                self.mudar_estado(Estados.ENTREGANDO_ENCOMENDA)
            else:
                angulo_para_base = math.atan2(-pos_y, -pos_x)
                qz, qw = self.pose_atual.orientation.z, self.pose_atual.orientation.w
                angulo_atual = 2 * math.atan2(qz, qw)
                erro_angulo = angulo_para_base - angulo_atual

                twist.angular.z = 0.3 if abs(erro_angulo) > 0.1 and erro_angulo > 0 else -0.3 if abs(erro_angulo) > 0.1 else 0.0
                twist.linear.x = 0.15 if abs(erro_angulo) <= 0.1 else 0.0
                self.cmd_vel_pub.publish(twist)

        # ---------- ENTREGANDO_ENCOMENDA ----------
        elif self.estado == Estados.ENTREGANDO_ENCOMENDA:
            self.cmd_vel_pub.publish(Twist())
            self.get_logger().info('Encomenda entregue! Missão cumprida.')
            self.timer.cancel()

# -------------------- MAIN --------------------
def main(args=None):
    rclpy.init(args=args)
    controller_node = ControllerNode()
    rclpy.spin(controller_node)
    controller_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
