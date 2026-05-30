import rclpy
from rclpy.node import Node

from turtlesim.msg import Pose
from geometry_msgs.msg import Twist

from math import atan2, pi # Utilizado para calcular o ângulo entre dois pontos


class ControleTartaruga(Node):

    def __init__(self):

        super().__init__('controle_tartaruga')

        # Cria um publisher para a tartaruga
        self.pub_cmd_vel = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        # Cria um subscriber para a tartaruga
        # O subscriber escuta a posição da tartaruga
        # e atualiza as variáveis de controle
        self.sub_pose = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.escuta_pose,
            10)
        
        self.sub_pose  # Evita warning de variável não utilizada

        timer_period = 0.5  # meio segundo ou 2 hz
        self.timer = self.create_timer(timer_period, self.loop_controle)

        # Inicializa as variáveis de controle
        self.angulo = 0
        self.x = 0
        self.y = 0
        self.alvo_x = 9
        self.alvo_y = 9


    def escuta_pose(self, msg):
        # Atualiza a posição e o ângulo da tartaruga
        self.x = msg.x
        self.y = msg.y
        self.angulo = msg.theta

    def loop_controle(self):
        # Verifica a posição da tartaruga e movimenta em direção ao alvo (9,9)
        # Se a tartaruga estiver na posição (9,9), não faz nada
        # Caso contrário, movimenta a tartaruga em direção ao alvo

        if self.x == self.alvo_x and self.y == self.alvo_y:
            self.get_logger().info('Tartaruga já está na posição alvo')
            return

        # Calcula o angulo entre a tartaruga e o alvo
        angulo_alvo = self.calcula_angulo(self.x, self.y, self.alvo_x, self.alvo_y)

        # Calcula o erro do ângulo
        erro_angulo = angulo_alvo - self.angulo

        # Normaliza o erro do ângulo para o intervalo [-pi, pi]

        if erro_angulo > pi:
            erro_angulo -= 2 * pi
        elif erro_angulo < -pi:
            erro_angulo += 2 * pi
        # Desta forma guiaremos a tartaruga para a direção do alvo
        # seguindo o menor angulo

        # Cria a mensagem de velocidade
        msg = Twist()

        # Preenchendo a mensagem de velocidade conforme as condições
        # Se a diferença do ângulo for maior que 0.1 radianos
        # A tartaruga gira
        # Caso contrário, a tartaruga se move em direção ao alvo
        # Se a tartaruga estiver próxima do alvo, para o movimento
        
        #Verifica se a diferença do ângulo é maior que 0.1 radianos
        if abs(erro_angulo) > 0.1:
            msg.angular.z = erro_angulo * 2.0
            msg.linear.x = 0.0
            self.pub_cmd_vel.publish(msg)
            self.get_logger().info('Girando para o alvo')
        else:

            # Verifica se a tartaruga está próxima do alvo
            if abs(self.x - self.alvo_x) < 0.1 and \
               abs(self.y - self.alvo_y) < 0.1:
                msg.angular.z = 0.0
                msg.linear.x = 0.0
                self.pub_cmd_vel.publish(msg)
                self.get_logger().info('Tartaruga chegou ao alvo')
            else:
                # Move a tartaruga em direção ao alvo
                msg = Twist()
                msg.angular.z = 0.0
                msg.linear.x = 1.0
                self.pub_cmd_vel.publish(msg)
                self.get_logger().info('Movendo em direção ao alvo')
        
    def calcula_angulo(self, x1, y1, x2, y2):
        # Calcula o ângulo entre dois pontos
        angulo = 0

        if x2 - x1 != 0:
            angulo = atan2(y2 - y1, x2 - x1)
        else:
            if y2 > y1:
                angulo = pi / 2
            else:
                angulo = -pi / 2
        return angulo

def main(args=None):
    rclpy.init(args=args)

    controle_tartaruga = ControleTartaruga()

    rclpy.spin(controle_tartaruga)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    controle_tartaruga.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
