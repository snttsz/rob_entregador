"""
Nó de Visão Computacional para o Robô Entregador.

Este nó processa imagens da câmera do robô para detectar a encomenda (pela cor vermelha)
e obstáculos (pela cor azul). As informações detectadas, como a posição e a área da
encomenda, e a presença de obstáculos, são publicadas em tópicos específicos para
serem consumidas por outros nós, como o controlador do robô.

Subscribers:
- /camera/image_raw (sensor_msgs/Image): Imagem colorida da câmera.

Publishers:
- /encomenda_info (geometry_msgs/Point): Publica a posição (x, y) e a área (z) do
contorno da encomenda detectada.
- /obstaculo_detectado (std_msgs/Bool): Publica `True` se um obstáculo azul for
detectado.
- /image_processed (sensor_msgs/Image): Publica a imagem com os contornos
detectados para fins de depuração.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from geometry_msgs.msg import Point
import cv2
import numpy as np
from cv_bridge import CvBridge

class VisionNode(Node):
    """
    Classe que define o nó de visão computacional.
    """
    def __init__(self):
        """
        Construtor do VisionNode. Inicializa os subscribers, publishers e
        outras variáveis necessárias.
        """
        super().__init__('vision_node')
        self.bridge = CvBridge()

        # Publishers
        self.encomenda_pub = self.create_publisher(Point, '/encomenda_info', 10)
        self.obstaculo_pub = self.create_publisher(Bool, '/obstaculo_detectado', 10)
        self.debug_image_pub = self.create_publisher(Image, '/image_processed', 10)

        # Subscriber
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)

        self.get_logger().info('Vision Node iniciado com sucesso.')

    def image_callback(self, msg: Image):
        """
        Callback para o tópico de imagem. Processa a imagem recebida para
        detectar a encomenda e os obstáculos.

        Args:
            msg (Image): A mensagem de imagem recebida.
        """
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

            # Detecção da encomenda (vermelho)
            lower_red1 = np.array([0, 100, 50])
            upper_red1 = np.array([10, 255, 255])
            lower_red2 = np.array([170, 100, 50])
            upper_red2 = np.array([180, 255, 255])
            mask_red = cv2.inRange(hsv_image, lower_red1, upper_red1) + \
                       cv2.inRange(hsv_image, lower_red2, upper_red2)

            # Detecção do obstáculo (azul)
            lower_blue = np.array([100, 150, 0])
            upper_blue = np.array([140, 255, 255])
            mask_blue = cv2.inRange(hsv_image, lower_blue, upper_blue)

            self.processa_contornos(cv_image, mask_red, mask_blue)

        except Exception as e:
            self.get_logger().error(f"Erro no callback da imagem: {e}")

    def processa_contornos(self, image, mask_red, mask_blue):
        """
        Encontra e processa os contornos na imagem para identificar a encomenda
        e os obstáculos.

        Args:
            image: A imagem original em formato OpenCV.
            mask_red: A máscara de cor para a encomenda.
            mask_blue: A máscara de cor para os obstáculos.
        """
        # Encomenda
        contours_red, _ = cv2.findContours(mask_red, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        encomenda_msg = Point()
        if contours_red:
            maior_contorno = max(contours_red, key=cv2.contourArea)
            area = cv2.contourArea(maior_contorno)
            if area > 500:
                M = cv2.moments(maior_contorno)
                if M["m00"] != 0:
                    encomenda_msg.x = float(int(M["m10"] / M["m00"]))
                    encomenda_msg.y = float(int(M["m01"] / M["m00"]))
                    encomenda_msg.z = area  # Usando o campo z para a área
                    cv2.drawContours(image, [maior_contorno], -1, (0, 255, 0), 2)
        self.encomenda_pub.publish(encomenda_msg)

        # Obstáculo
        contours_blue, _ = cv2.findContours(mask_blue, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        obstaculo_detectado = any(cv2.contourArea(c) > 500 for c in contours_blue)
        #self.get_logger().error(f"Vision obstaculo -> {obstaculo_detectado}")
        self.obstaculo_pub.publish(Bool(data=obstaculo_detectado))

        if obstaculo_detectado:
            cv2.drawContours(image, contours_blue, -1, (255, 0, 0), 2)


        # Publica imagem de debug
        debug_msg = self.bridge.cv2_to_imgmsg(image, "bgr8")
        self.debug_image_pub.publish(debug_msg)


def main(args=None):
    rclpy.init(args=args)
    vision_node = VisionNode()
    rclpy.spin(vision_node)
    vision_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()