import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, PoseStamped
from nav2_msgs.action import NavigateToPose
from enum import Enum
import cv2
import numpy as np
from cv_bridge import CvBridge
import time

# Definindo os estados da nossa máquina de estados
class Estados(Enum):
    BUSCANDO_ENCOMENDA = 1
    APROXIMANDO_ENCOMENDA = 2
    COLETANDO_ENCOMENDA = 3
    RETORNANDO_PARA_BASE = 4
    ENTREGANDO_ENCOMENDA = 5

class EntregadorNode(Node):
    def __init__(self):
        super().__init__('entregador_node')

        # Inicializa o estado
        self.estado = Estados.BUSCANDO_ENCOMENDA
        self.get_logger().info('Iniciando em estado: BUSCANDO_ENCOMENDA')

        # Posição inicial (base) - O robô começa em (0,0,0)
        self.pose_base = PoseStamped()
        self.pose_base.header.frame_id = 'map'
        self.pose_base.pose.position.x = 0.0
        self.pose_base.pose.position.y = 0.0
        self.pose_base.pose.orientation.w = 1.0

        # CV Bridge para converter imagem ROS para OpenCV
        self.bridge = CvBridge()

        # Publisher para controlar a velocidade do robô
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Subscriber para a imagem da câmera
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)

        # Action Client para o Nav2
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Timer para executar a máquina de estados
        self.timer = self.create_timer(0.1, self.rodar_maquina_estados)

        # Variáveis para a detecção
        self.encomenda_detectada = False
        self.centro_x_encomenda = 0

    def image_callback(self, msg):
        try:
            # Converte a imagem ROS para uma imagem OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            height, width, _ = cv_image.shape

            # Converte a imagem para o espaço de cores HSV
            hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

            # Define o range da cor vermelha em HSV
            # ATENÇÃO: Estes valores podem precisar de ajuste!
            lower_red = np.array([0, 120, 70])
            upper_red = np.array([10, 255, 255])
            mask1 = cv2.inRange(hsv_image, lower_red, upper_red)

            lower_red = np.array([170, 120, 70])
            upper_red = np.array([180, 255, 255])
            mask2 = cv2.inRange(hsv_image, lower_red, upper_red)

            mask = mask1 + mask2

            # Encontra os contornos na máscara
            contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            if contours:
                # Pega o maior contorno (assumindo que é a encomenda)
                maior_contorno = max(contours, key=cv2.contourArea)
                area = cv2.contourArea(maior_contorno)

                # Se o contorno for grande o suficiente, consideramos detectado
                if area > 500: # Limiar de área, ajuste se necessário
                    self.encomenda_detectada = True

                    # Calcula o centro do contorno
                    M = cv2.moments(maior_contorno)
                    if M["m00"] != 0:
                        self.centro_x_encomenda = int(M["m10"] / M["m00"])
                    else:
                        self.centro_x_encomenda = 0

                    # Lógica para centralizar o robô na encomenda
                    if self.estado == Estados.APROXIMANDO_ENCOMENDA:
                        erro = self.centro_x_encomenda - width / 2
                        twist = Twist()
                        # A velocidade angular é proporcional ao erro
                        twist.angular.z = -0.002 * erro 
                        # Se estiver bem centralizado, avança
                        if abs(erro) < 20: # Limiar de centralização
                            twist.linear.x = 0.15 # Velocidade de avanço
                            # Se a área for muito grande, significa que estamos perto
                            if area > 60000: # Limiar de área para "coleta"
                                self.mudar_estado(Estados.COLETANDO_ENCOMENDA)
                        self.cmd_vel_pub.publish(twist)
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
        # Lógica do estado BUSCANDO_ENCOMENDA
        if self.estado == Estados.BUSCANDO_ENCOMENDA:
            if self.encomenda_detectada:
                self.mudar_estado(Estados.APROXIMANDO_ENCOMENDA)
            else:
                # Gira lentamente para procurar a encomenda
                twist = Twist()
                twist.angular.z = 0.3
                self.cmd_vel_pub.publish(twist)

        # Lógica do estado APROXIMANDO_ENCOMENDA
        # (A lógica principal está no image_callback para ser mais reativa)
        elif self.estado == Estados.APROXIMANDO_ENCOMENDA:
            if not self.encomenda_detectada:
                # Se perdeu a encomenda de vista, volta a buscar
                self.mudar_estado(Estados.BUSCANDO_ENCOMENDA)

        # Lógica do estado COLETANDO_ENCOMENDA
        elif self.estado == Estados.COLETANDO_ENCOMENDA:
            # Para o robô
            self.cmd_vel_pub.publish(Twist()) 
            self.get_logger().info('Encomenda COLETADA! Retornando para a base em 3 segundos...')
            time.sleep(3) # Simula o tempo de coleta
            self.mudar_estado(Estados.RETORNANDO_PARA_BASE)

        # Lógica do estado RETORNANDO_PARA_BASE
        elif self.estado == Estados.RETORNANDO_PARA_BASE:
            self.get_logger().info('Enviando objetivo para o Nav2 retornar à base.')
            self.enviar_objetivo_nav2(self.pose_base)
            # Uma vez enviado o objetivo, mudamos de estado para não enviar de novo
            self.mudar_estado(Estados.ENTREGANDO_ENCOMENDA)

        # Lógica do estado ENTREGANDO_ENCOMENDA
        elif self.estado == Estados.ENTREGANDO_ENCOMENDA:
            # Aguarda o resultado do Nav2
            if self.nav_to_pose_client.get_result_future().done():
                self.get_logger().info('Chegou na base! Missão cumprida.')
                # Para o robô e cancela o timer
                self.cmd_vel_pub.publish(Twist())
                self.timer.cancel()

    def enviar_objetivo_nav2(self, pose):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self.get_logger().info('Aguardando o servidor de ação do Nav2...')
        self.nav_to_pose_client.wait_for_server()

        self.get_logger().info('Servidor de ação disponível. Enviando objetivo.')
        self.nav_to_pose_client.send_goal_async(goal_msg)


def main(args=None):
    rclpy.init(args=args)
    node = EntregadorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()