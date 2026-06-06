#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan, Imu, Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

from scipy.spatial.transform import Rotation as R
from cv_bridge import CvBridge
import cv2
import numpy as np

# IMPORTS LUISA 05JUNHO
from nav_msgs.msg import OccupancyGrid
from bb8_control.maquina_estados import GerenciadorMissao


def clip(value, lower, upper):
    return max(lower, min(value, upper))


class ControleRobo(Node):
    def __init__(self):
        super().__init__("controle_robo")

        # Publisher para comando de velocidade
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # Subscribers
        self.create_subscription(LaserScan, "/scan", self.scan_callback, 10)
        self.create_subscription(Imu, "/imu", self.imu_callback, 10)

        self.create_subscription(
            Image, "/robot_cam/colored_map", self.camera_callback, 10
        )
        # TROQUEI ISSO:
        # self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        # POR ISSO (MUDANÇA LUISA 05/06)
        self.create_subscription(Odometry, "/odom_gt", self.odom_callback, 10)
        self.create_subscription(OccupancyGrid, "/map", self.map_callback, 10)
        self.gerenciador_missao = GerenciadorMissao()
        self.mapa_2d = None
        self.resolucao_mapa = 0.05
        self.origem_mapa_x = 0.0
        self.origem_mapa_y = 0.0
        self.posicao_robo_grid = None
        self.bandeira_vista = False
        self.distancia_frente = 1.0  # Usado para calcular a bandeira

        self.bridge = CvBridge()

        # Timer aumentado para 20Hz (0.05s) para controle mais suave e rápido
        self.timer = self.create_timer(0.05, self.move_robot)

        # --- VARIÁVEIS DE ESTADO DO SENSOR ---
        self.obstaculo_a_frente = False

        # --- FILTROS DE VELOCIDADE (SUA PARTE) ---
        self.linear_atual = 0.0
        self.angular_atual = 0.0

        # Limites Físicos de Velocidade (Deixando o robô rápido)
        self.MAX_LINEAR_VEL = 0.8  # m/s (Antes era 0.1)
        self.MAX_ANGULAR_VEL = 1.5  # rad/s (Antes era 0.3)

        # Taxa de aceleração permitida por ciclo (Slew Rate)
        self.MAX_LINEAR_ACCEL = 0.04  # m/s² por ciclo
        self.MAX_ANGULAR_ACCEL = 0.1  # rad/s² por ciclo

        # --- INTERFACE PARA A HEURÍSTICA (O GRUPO MUDA AQUI) ---
        # --- VARIÁVEIS AGORA CONTROLADAS PELA MÁQUINA DE ESTADOS E D* LITE ---
        self.velocidade_linear_desejada = 0.0
        self.velocidade_angular_desejada = 0.0

    # LUISA 05/06: FUNÇÃO NOVA QUE LÊ O MAPA
    def map_callback(self, msg: OccupancyGrid):
        largura = msg.info.width
        altura = msg.info.height
        self.resolucao_mapa = msg.info.resolution
        self.origem_mapa_x = msg.info.origin.position.x
        self.origem_mapa_y = msg.info.origin.position.y
        self.mapa_2d = np.array(msg.data).reshape((altura, largura))

    def scan_callback(self, msg: LaserScan):
        num_ranges = len(msg.ranges)
        if num_ranges == 0:
            return

        # Verifica obstáculo apenas na janela frontal (± 0.5 rad)
        self.obstaculo_a_frente = False
        # LUISA 05/06: ADICIONOU DISTANCIAS_FRENTE
        distancias_frente = []

        for i in range(num_ranges):
            angle = msg.angle_min + i * msg.angle_increment
            if -0.5 <= angle <= 0.5:
                if (
                    0.05 < msg.ranges[i] < 0.6
                ):  # Ignora o próprio chassi (<0.05) e detecta até 0.6m
                    self.obstaculo_a_frente = True
                    distancias_frente.append(msg.ranges[i])
                    # break

        # Salva a distância exata para podermos calcular onde a bandeira está!
        if distancias_frente:
            self.distancia_frente = min(distancias_frente)

    def imu_callback(self, msg: Imu):
        pass

    def odom_callback(self, msg: Odometry):
        # Antes odom_callback estava vazio (só o pass)
        # pass
        # LUISA 05/06:
        if self.mapa_2d is not None:
            # Salva posição e ângulo para a Trigonometria da Bandeira
            self.x_real = msg.pose.pose.position.x
            self.y_real = msg.pose.pose.position.y

            quat = [
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w,
            ]
            self.yaw_robo = R.from_quat(quat).as_euler("xyz")[2]

            # Converte Metros -> Grid
            coluna_x = int((self.x_real - self.origem_mapa_x) / self.resolucao_mapa)
            linha_y = int((self.y_real - self.origem_mapa_y) / self.resolucao_mapa)

            altura, largura = self.mapa_2d.shape
            self.posicao_robo_grid = (
                max(0, min(coluna_x, largura - 1)),
                max(0, min(linha_y, altura - 1)),
            )

    # MODIFICAÇÃO LUISA 05/06
    def camera_callback(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        target_color = np.array([171, 242, 0])
        mask = cv2.inRange(frame, target_color, target_color)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        self.bandeira_vista = len(contours) > 0
        cx_bandeira = None

        # Acha o pixel central (cx) da bandeira para o estado POSICIONANDO_PARA_COLETA
        if self.bandeira_vista:
            maior_contorno = max(contours, key=cv2.contourArea)
            M = cv2.moments(maior_contorno)
            if M["m00"] != 0:
                cx_bandeira = int(M["m10"] / M["m00"])

        """# CÓDIGO ANTIGO 
        # Lógica Básica de Comportamento para teste de movimentação
        if self.obstaculo_a_frente:
            # Desvio de emergência: Para de ir pra frente e gira rápido
            self.velocidade_linear_desejada = 0.0
            self.velocidade_angular_desejada = 0.8
        elif len(contours) > 0:
            # Achou a bandeira: Vai em direção a ela
            self.velocidade_linear_desejada = 0.4
            self.velocidade_angular_desejada = 0.0 # Aqui o grupo pode colocar um controle Proporcional para centralizar
        else:
            # Explorando (andando e girando levemente)
            self.velocidade_linear_desejada = 0.3
            self.velocidade_angular_desejada = -0.3
        """

        # INTEGRAÇÃO D* LITE E MÁQUINA DE ESTADOS

        coordenada_bandeira_grid = None

        # Se viu a bandeira, calcula onde ela está na matriz!
        if (
            self.bandeira_vista
            and self.mapa_2d is not None
            and hasattr(self, "yaw_robo")
        ):
            dist = self.distancia_frente
            x_band_real = self.x_real + (dist * np.cos(self.yaw_robo))
            y_band_real = self.y_real + (dist * np.sin(self.yaw_robo))

            c_band = int((x_band_real - self.origem_mapa_x) / self.resolucao_mapa)
            l_band = int((y_band_real - self.origem_mapa_y) / self.resolucao_mapa)

            altura, largura = self.mapa_2d.shape
            coordenada_bandeira_grid = (
                max(0, min(c_band, largura - 1)),
                max(0, min(l_band, altura - 1)),
            )

        # Pede para a Máquina de Estados qual é o próximo passo
        proximo_passo_grid = self.gerenciador_missao.atualizar_estado_e_caminho(
            self.mapa_2d,
            self.posicao_robo_grid,
            self.bandeira_vista,
            coordenada_bandeira_grid,
            self.distancia_frente,
        )

        # LÓGICA DE MOTOR BASEADA DIRETAMENTE NO ESTADO
        estado_atual = self.gerenciador_missao.ESTADO_ATUAL

        if estado_atual == "PROCURANDO_BANDEIRA":
            # Gira no próprio eixo devagar para achar a bandeira
            self.velocidade_linear_desejada = 0.0
            self.velocidade_angular_desejada = 0.5

        elif estado_atual == "POSICIONANDO_PARA_COLETA":
            # Usa o Controle Proporcional para o robô ficar com orientação adequada
            largura_imagem = frame.shape[1]
            if cx_bandeira is not None:
                erro_x = (largura_imagem / 2) - cx_bandeira

                if abs(erro_x) > 20:  # Margem de tolerância (20 pixels)
                    self.velocidade_linear_desejada = 0.0
                    self.velocidade_angular_desejada = 0.002 * erro_x
                else:
                    self.velocidade_linear_desejada = 0.0
                    self.velocidade_angular_desejada = 0.0
                    self.get_logger().info("Bandeira centralizada! PRONTO PARA COLETA.")

        else:
            # Estados EXPLORANDO e NAVIGANDO (Usa a navegação em Grid do D*)
            if proximo_passo_grid and self.posicao_robo_grid:
                dx = proximo_passo_grid[0] - self.posicao_robo_grid[0]
                dy = proximo_passo_grid[1] - self.posicao_robo_grid[1]

                if dx > 0:
                    self.velocidade_linear_desejada = 0.3
                    self.velocidade_angular_desejada = -0.5
                elif dx < 0:
                    self.velocidade_linear_desejada = 0.3
                    self.velocidade_angular_desejada = 0.5
                elif dy != 0:
                    self.velocidade_linear_desejada = 0.4
                    self.velocidade_angular_desejada = 0.0
            else:
                self.velocidade_linear_desejada = 0.0
                self.velocidade_angular_desejada = 0.0

        """# Transforma o Passo da Matriz em Velocidade Desejada
        # (O "Módulo Cinético" lá no move_robot vai puxar essas variáveis e suavizar!)
        if proximo_passo_grid and self.posicao_robo_grid:
            dx = proximo_passo_grid[0] - self.posicao_robo_grid[0]
            dy = proximo_passo_grid[1] - self.posicao_robo_grid[1]
            
            if dx > 0: # D* mandou ir para a direita
                self.velocidade_linear_desejada = 0.3
                self.velocidade_angular_desejada = -0.5
            elif dx < 0: # D* mandou ir para a esquerda
                self.velocidade_linear_desejada = 0.3
                self.velocidade_angular_desejada = 0.5
            elif dy != 0: # D* mandou ir reto (Frente ou Trás)
                self.velocidade_linear_desejada = 0.4
                self.velocidade_angular_desejada = 0.0
        else:
            # Se não tem rota, manda parar.
            self.velocidade_linear_desejada = 0.0
            self.velocidade_angular_desejada = 0.0
            """

    def move_robot(self):
        """
        Módulo Cinético: Filtra as velocidades desejadas e envia comandos suaves aos motores.
        """
        twist = Twist()

        # 1. Filtro Rampa (Aceleração Suave) Linear
        erro_linear = self.velocidade_linear_desejada - self.linear_atual
        if abs(erro_linear) > self.MAX_LINEAR_ACCEL:
            self.linear_atual += np.sign(erro_linear) * self.MAX_LINEAR_ACCEL
        else:
            self.linear_atual = self.velocidade_linear_desejada

        # 2. Filtro Rampa (Aceleração Suave) Angular
        erro_angular = self.velocidade_angular_desejada - self.angular_atual
        if abs(erro_angular) > self.MAX_ANGULAR_ACCEL:
            self.angular_atual += np.sign(erro_angular) * self.MAX_ANGULAR_ACCEL
        else:
            self.angular_atual = self.velocidade_angular_desejada

        # 3. Saturação Segura (Corte nos Limites)
        self.linear_atual = clip(
            self.linear_atual, -self.MAX_LINEAR_VEL, self.MAX_LINEAR_VEL
        )
        self.angular_atual = clip(
            self.angular_atual, -self.MAX_ANGULAR_VEL, self.MAX_ANGULAR_VEL
        )

        # 4. Publicação
        twist.linear.x = self.linear_atual
        twist.angular.z = self.angular_atual
        self.cmd_vel_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = ControleRobo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
