import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from enum import Enum
import cv2
import numpy as np
from cv_bridge import CvBridge
import time
import message_filters
import math

class Estados(Enum):
    BUSCANDO_ENCOMENDA = 1
    APROXIMANDO_ENCOMENDA = 2
    COLETANDO_ENCOMENDA = 3
    RETORNANDO_PARA_BASE = 4
    ENTREGANDO_ENCOMENDA = 5

class EntregadorNode(Node):
    def __init__(self):
        super().__init__('entregador_node')

        self.estado = Estados.BUSCANDO_ENCOMENDA
        self.get_logger().info('Iniciando em estado: BUSCANDO_ENCOMENDA')

        self.bridge = CvBridge()
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Publisher para imagem de debug
        self.debug_image_pub = self.create_publisher(Image, '/image_processed', 10)

        # Subscriber simples para a imagem da câmera
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)

        # Subscribers para a navegação manual
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        
        self.timer = self.create_timer(0.1, self.rodar_maquina_estados)

        self.encomenda_detectada = False
        self.centro_x_encomenda = 0
        self.area_encomenda = 0 # Usaremos a área para decidir a coleta
        
        self.pose_atual = None
        self.scan_atual = None

    def odom_callback(self, msg):
        self.pose_atual = msg.pose.pose

    def scan_callback(self, msg):
        self.scan_atual = msg

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

            lower_red = np.array([0, 100, 50])
            upper_red = np.array([10, 255, 255])
            mask1 = cv2.inRange(hsv_image, lower_red, upper_red)
            lower_red = np.array([170, 100, 50])
            upper_red = np.array([180, 255, 255])
            mask2 = cv2.inRange(hsv_image, lower_red, upper_red)
            mask = mask1 + mask2
            
            processed_image = cv2.bitwise_and(cv_image, cv_image, mask=mask)
            debug_msg = self.bridge.cv2_to_imgmsg(processed_image, "bgr8")
            self.debug_image_pub.publish(debug_msg)

            contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            if contours:
                maior_contorno = max(contours, key=cv2.contourArea)
                self.area_encomenda = cv2.contourArea(maior_contorno)

                if self.area_encomenda > 500:
                    self.encomenda_detectada = True
                    M = cv2.moments(maior_contorno)
                    if M["m00"] != 0:
                        self.centro_x_encomenda = int(M["m10"] / M["m00"])
                else:
                    self.encomenda_detectada = False
            else:
                self.encomenda_detectada = False
        except Exception as e:
            self.get_logger().error(f"Erro no callback da imagem: {e}")

    def mudar_estado(self, novo_estado):
        if self.estado != novo_estado:
            self.get_logger().info(f'Mudando de estado: {self.estado.name} -> {novo_estado.name}')
            self.estado = novo_estado

    def rodar_maquina_estados(self):
        if self.estado == Estados.BUSCANDO_ENCOMENDA:
            if self.encomenda_detectada:
                self.mudar_estado(Estados.APROXIMANDO_ENCOMENDA)
            else:
                twist = Twist(); twist.angular.z = 0.3; self.cmd_vel_pub.publish(twist)
        
        elif self.estado == Estados.APROXIMANDO_ENCOMENDA:
            if not self.encomenda_detectada or self.scan_atual is None:
                self.mudar_estado(Estados.BUSCANDO_ENCOMENDA); return

            # Filtrar valores 'inf' e calcular a distância mínima
            ranges_frontais = self.scan_atual.ranges[0:15] + self.scan_atual.ranges[345:360]
            ranges_validos = [r for r in ranges_frontais if r > 0 and not math.isinf(r)]

            if not ranges_validos:
                distancia_frontal = float('inf')
            else:
                distancia_frontal = min(ranges_validos)

            self.get_logger().info(f"Distancia frontal: {distancia_frontal:.2f}m")

            if distancia_frontal < 0.5: # Aumentei um pouco a distancia para garantir a coleta
                self.get_logger().info(f'Distância: {distancia_frontal:.2f}m. Coletando!'); self.mudar_estado(Estados.COLETANDO_ENCOMENDA)
            else:
                erro = self.centro_x_encomenda - 320
                twist = Twist()
                twist.angular.z = -0.002 * erro
                # O robô deve avançar mesmo se não estiver perfeitamente alinhado
                twist.linear.x = 0.1
                self.cmd_vel_pub.publish(twist)

        elif self.estado == Estados.COLETANDO_ENCOMENDA:
            self.cmd_vel_pub.publish(Twist()); self.get_logger().info('Encomenda COLETADA! Retornando...'); time.sleep(1); self.mudar_estado(Estados.RETORNANDO_PARA_BASE)

        elif self.estado == Estados.RETORNANDO_PARA_BASE:
            if self.pose_atual is None or self.scan_atual is None: self.get_logger().info('Aguardando dados de odometria e scan...'); return
            pos_x, pos_y = self.pose_atual.position.x, self.pose_atual.position.y; qz, qw = self.pose_atual.orientation.z, self.pose_atual.orientation.w
            angulo_atual = 2 * math.atan2(qz, qw)
            distancia_da_base = math.sqrt(pos_x**2 + pos_y**2)
            if distancia_da_base < 0.2: self.mudar_estado(Estados.ENTREGANDO_ENCOMENDA); return
            angulo_para_base = math.atan2(-pos_y, -pos_x)
            erro_angulo = angulo_para_base - angulo_atual
            if erro_angulo > math.pi: erro_angulo -= 2 * math.pi
            if erro_angulo < -math.pi: erro_angulo += 2 * math.pi
            twist = Twist()
            distancia_frontal = min(min(self.scan_atual.ranges[0:15]), min(self.scan_atual.ranges[345:360]))
            if distancia_frontal < 0.4:
                twist.linear.x = 0.0; twist.angular.z = 0.4; self.get_logger().info(f'Obstáculo detectado a {distancia_frontal:.2f}m! Desviando...')
            elif abs(erro_angulo) > 0.1:
                twist.linear.x = 0.0; twist.angular.z = 0.3 if erro_angulo > 0 else -0.3; self.get_logger().info(f'Alinhando com a base...')
            else:
                twist.linear.x = 0.15; twist.angular.z = 0.0; self.get_logger().info(f'Caminho livre! Indo para a base a {distancia_da_base:.2f}m...')
            self.cmd_vel_pub.publish(twist)
        
        elif self.estado == Estados.ENTREGANDO_ENCOMENDA:
            self.cmd_vel_pub.publish(Twist()); self.get_logger().info('Encomenda entregue! Missão cumprida.'); self.timer.cancel()

def main(args=None):
    rclpy.init(args=args); node = EntregadorNode(); rclpy.spin(node); node.destroy_node(); rclpy.shutdown()
if __name__ == '__main__':
    main()