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
    OBSTACULO_DETECTADO = 6
    
class EntregadorNode(Node):
    def __init__(self):
        super().__init__('entregador_node')

        self.estado = Estados.BUSCANDO_ENCOMENDA
        self.get_logger().info('Iniciando em estado: BUSCANDO_ENCOMENDA')

        self.bridge = CvBridge()
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        self.debug_image_pub = self.create_publisher(Image, '/image_processed', 10)

        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)

        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        
        self.timer = self.create_timer(0.1, self.rodar_maquina_estados)

        self.encomenda_detectada = False
        self.centro_x_encomenda = 0
        self.area_encomenda = 0
        
        self.obstaculo_azul_detectado = False

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

            # --- Detecção da encomenda (vermelho) ---
            lower_red = np.array([0, 100, 50])
            upper_red = np.array([10, 255, 255])
            mask1 = cv2.inRange(hsv_image, lower_red, upper_red)
            lower_red = np.array([170, 100, 50])
            upper_red = np.array([180, 255, 255])
            mask2 = cv2.inRange(hsv_image, lower_red, upper_red)
            mask_red = mask1 + mask2

            # --- Detecção do obstáculo (azul) ---
            lower_blue = np.array([100, 150, 0])
            upper_blue = np.array([140, 255, 255])
            mask_blue = cv2.inRange(hsv_image, lower_blue, upper_blue)

            processed_image = cv2.bitwise_and(cv_image, cv_image, mask=mask_red)
            debug_msg = self.bridge.cv2_to_imgmsg(processed_image, "bgr8")
            self.debug_image_pub.publish(debug_msg)

            contours_red, _ = cv2.findContours(mask_red, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            contours_blue, _ = cv2.findContours(mask_blue, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            if contours_red:
                maior_contorno = max(contours_red, key=cv2.contourArea)
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
            
            self.obstaculo_azul_detectado = any(cv2.contourArea(c) > 500 for c in contours_blue)

            if (self.obstaculo_azul_detectado):
                self.mudar_estado(Estados.OBSTACULO_DETECTADO)
                self.get_logger().info(f'OBSTACULO DETECTADO -> {self.obstaculo_azul_detectado}')

        except Exception as e:
            self.get_logger().error(f"Erro no callback da imagem: {e}")

    def mudar_estado(self, novo_estado):
        if self.estado != novo_estado:
            self.get_logger().info(f'Mudando de estado: {self.estado.name} -> {novo_estado.name}')
            self.estado = novo_estado

    def rodar_maquina_estados(self):
        if self.scan_atual is None:
            return

        distancia_frontal = min(min(self.scan_atual.ranges[0:15]), min(self.scan_atual.ranges[345:360]))

        # --- Máquina de Estados com Lógica de Transição Integrada ---

        if self.estado == Estados.OBSTACULO_DETECTADO:
            self.get_logger().info('ENTRNADO EM OBSTACULO DETECTADO!!!!!')
            if not self.obstaculo_azul_detectado:
                self.get_logger().info('Obstáculo removido. Voltando a buscar a encomenda.')
                self.mudar_estado(Estados.BUSCANDO_ENCOMENDA)
            else:
                self.get_logger().info('Obstáculo azul à frente. Aguardando a passagem ser liberada...')
                twist = Twist()
                self.cmd_vel_pub.publish(twist)

        elif self.estado == Estados.BUSCANDO_ENCOMENDA:
            if self.obstaculo_azul_detectado and distancia_frontal < 0.4:
                self.mudar_estado(Estados.OBSTACULO_DETECTADO)
            elif self.encomenda_detectada:
                self.mudar_estado(Estados.APROXIMANDO_ENCOMENDA)
            else:
                twist = Twist(); twist.angular.z = 0.3; self.cmd_vel_pub.publish(twist)
        
        elif self.estado == Estados.APROXIMANDO_ENCOMENDA:
            if self.obstaculo_azul_detectado and distancia_frontal < 0.4:
                self.mudar_estado(Estados.OBSTACULO_DETECTADO)
            elif not self.encomenda_detectada:
                self.mudar_estado(Estados.BUSCANDO_ENCOMENDA)
            elif distancia_frontal < 0.5:
                self.get_logger().info(f'Distância: {distancia_frontal:.2f}m. Coletando!')
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
            if self.pose_atual is None: 
                self.get_logger().info('Aguardando dados de odometria...'); 
                return
            
            if distancia_frontal < 0.4:
                twist = Twist()
                twist.linear.x = 0.0
                twist.angular.z = 0.4
                self.get_logger().info(f'Obstáculo detectado a {distancia_frontal:.2f}m! Desviando...')
                self.cmd_vel_pub.publish(twist)
                return

            pos_x, pos_y = self.pose_atual.position.x, self.pose_atual.position.y
            qz, qw = self.pose_atual.orientation.z, self.pose_atual.orientation.w
            angulo_atual = 2 * math.atan2(qz, qw)
            
            distancia_da_base = math.sqrt(pos_x**2 + pos_y**2)
            if distancia_da_base < 0.2:
                self.mudar_estado(Estados.ENTREGANDO_ENCOMENDA)
                return
                
            angulo_para_base = math.atan2(-pos_y, -pos_x)
            erro_angulo = angulo_para_base - angulo_atual
            if erro_angulo > math.pi: erro_angulo -= 2 * math.pi
            if erro_angulo < -math.pi: erro_angulo += 2 * math.pi
            
            twist = Twist()
            if abs(erro_angulo) > 0.1:
                twist.linear.x = 0.0
                twist.angular.z = 0.3 if erro_angulo > 0 else -0.3
                self.get_logger().info(f'Alinhando com a base...')
            else:
                twist.linear.x = 0.15
                twist.angular.z = 0.0
                self.get_logger().info(f'Caminho livre! Indo para a base a {distancia_da_base:.2f}m...')
            self.cmd_vel_pub.publish(twist)
        
        elif self.estado == Estados.ENTREGANDO_ENCOMENDA:
            self.cmd_vel_pub.publish(Twist())
            self.get_logger().info('Encomenda entregue! Missão cumprida.')
            self.timer.cancel()

def main(args=None):
    rclpy.init(args=args); node = EntregadorNode(); rclpy.spin(node); node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()