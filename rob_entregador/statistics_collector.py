# rob_entregador/statistics_collector.py

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import Log
import time

class StatisticsCollectorNode(Node):
    """
    Nó para coletar estatísticas de teste automatizado.
    Ouve o tópico /rosout para mensagens de log, cronometra a duração da missão
    e reporta o resultado (sucesso ou falha por timeout).
    """
    def __init__(self):
        super().__init__('statistics_collector_node')
        
        self.declare_parameter('timeout_seconds', 300.0)
        self.timeout = self.get_parameter('timeout_seconds').get_parameter_value().double_value
        
        self.mission_finished = False
        self.start_time = None

        self.log_sub = self.create_subscription(Log, '/rosout', self.log_callback, 10)
        
        self.get_logger().info('Coletor de estatísticas iniciado.')
        self.start_time = time.time()
        
        self.timeout_timer = self.create_timer(1.0, self.check_timeout)

    def log_callback(self, msg: Log):
        if self.mission_finished or 'statistics_collector_node' in msg.name:
            return

        log_message = msg.msg.lower()

        if 'encomenda entregue! missão cumprida.' in log_message:
            self.mission_finished = True
            end_time = time.time()
            duration = end_time - self.start_time
            
            # Imprime o resultado de forma fácil de ser capturada
            print(f"--- RESULT: SUCCESS ---", flush=True)
            print(f"--- DURATION: {duration:.2f} ---", flush=True)
            
            self.get_logger().info(f'Missão concluída com sucesso em {duration:.2f} segundos.')
            self.shutdown()

    def check_timeout(self):
        if self.mission_finished:
            return
            
        current_duration = time.time() - self.start_time
        if current_duration > self.timeout:
            self.mission_finished = True
            
            print(f"--- RESULT: FAILURE (TIMEOUT) ---", flush=True)
            print(f"--- DURATION: {self.timeout:.2f} ---", flush=True)

            self.get_logger().error(f'Missão falhou por timeout após {self.timeout:.2f} segundos.')
            self.shutdown()

    def shutdown(self):
        """
        Encerra o nó e força o shutdown do rclpy para garantir que o processo termine.
        """
        # Garante que não tentemos desligar duas vezes
        if not rclpy.ok():
            return
            
        self.get_logger().info('Desligando o coletor de estatísticas...')
        self.destroy_node()
        # Força o encerramento do processo do nó
        raise rclpy.executors.ExternalShutdownException()


def main(args=None):
    rclpy.init(args=args)
    node = StatisticsCollectorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        # Captura a exceção de shutdown para uma finalização limpa
        pass
    finally:
        if rclpy.ok():
           node.destroy_node()
           rclpy.shutdown()

if __name__ == '__main__':
    main()