#!/usr/bin/env python3
import math
import time
from collections import deque
from enum import Enum, auto

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import Pose2D, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, Float64MultiArray


class Estado(Enum):
    SEGUINDO_PAREDE = auto()
    DETECTOU_BANDEIRA = auto()
    NAVEGANDO_PARA_BANDEIRA = auto()
    POSICIONANDO_FINAL = auto()
    EVITANDO_LOOP = auto()


# Joint order for /gripper_controller/commands:
#   [gripper_extension, arm_elbow, right_gripper_joint, left_gripper_joint]
ARM_RETRAIDO = [-1.5, -1.5, 0.0, 0.0]  # folded against body during navigation
ARM_ESTENDIDO = [
    0.0,
    0.0,
    0.0,
    0.0,
]  # fully forward; tip reaches ~0.67 m from LIDAR centre

# LIDAR safety and wall-follower
DIST_SEGURA_FRENTE = 0.40  # [m] emergency stop — below this the robot halts immediately
DIST_ALVO_PAREDE = 0.50  # [m] desired right-wall clearance for P-controller
DIST_BUSCA_PAREDE = (
    1.8  # [m] if every ray is farther than this, no wall is near → go straight
)
WALL_KP = 2.2  # [rad/s / m] proportional gain: ω = -KP × (right_dist − target)
VEL_SEGUINDO = 0.6  # [m/s] forward speed during wall-following
OMEGA_MAX = 1.0  # [rad/s] angular velocity clamp for all states

# Flag navigation
FLAG_KP = 1.8  # [rad/s / rad] bearing P-gain: ω = KP × bearing
VEL_BANDEIRA = 0.6  # [m/s] forward speed when heading toward the flag
FLAG_ALINHA_TOL = 0.20  # [rad] max bearing error to be considered "aligned" (~11°)
DIST_PARAR_FINAL = (
    0.75  # [m] stop distance in POSICIONANDO — chosen so arm tip clears flag pole
)
FLAG_PERDA_MAX = (
    10  # [ticks] consecutive no-detection ticks before giving up and re-exploring
)

# Obstacle avoidance within NAVEGANDO_PARA_BANDEIRA
DIST_NAV_DESVIO = (
    0.65  # [m] obstacle closer than this switches to contorno wall-follow mode
)
NAV_DESVIO_KP = (
    2.2  # [rad/s] fixed turn rate applied when emergency-stopping near obstacle
)

# POSICIONANDO entry gate (camera-only — LIDAR cannot distinguish flag from cylinder)
FLAG_MIN_AREA_POSICIONANDO = 1500  # [px] flag pixel area ≥ this ≈ flag within ~0.85 m

# Loop detection via occupancy grid
GRID_RES = 0.30  # [m] grid cell size for position discretisation
HIST_LEN = 60  # [cells] sliding window length; limits memory regardless of path length
REVISITAS_LOOP = 6  # revisits to same cell before triggering EVITANDO_LOOP

# Open-loop escape manoeuvre durations
TEMPO_GIRO_FUGA = 2.5  # [s] phase 0: spin in place to break the loop
TEMPO_FWD_FUGA = 3.5  # [s] phase 1: drive straight to reach a new area

FREQ_CONTROLE = 20  # [Hz] main control-loop rate


class ControleRobo(Node):
    def __init__(self):
        super().__init__("controle_robo")

        qos_be = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.create_subscription(LaserScan, "/scan", self._cb_scan, qos_be)
        self.create_subscription(Odometry, "/odom_gt", self._cb_odom, qos_be)
        self.create_subscription(Pose2D, "/vision/flag_detection", self._cb_visao, 10)
        self.create_subscription(Float32, "/vision/flag_bearing", self._cb_bearing, 10)

        self._pub_cmd = self.create_publisher(Twist, "/cmd_vel", 10)
        self._pub_braco = self.create_publisher(
            Float64MultiArray, "/gripper_controller/commands", 10
        )

        self._estado = Estado.SEGUINDO_PAREDE
        self._scan = None
        self._pos_x = 0.0
        self._pos_y = 0.0

        self._bandeira_detectada = False
        self._flag_bearing = 0.0  # [rad] positive = flag to the left
        self._flag_area = 0.0  # [px]  pixel count from vision_processor
        self._flag_perda_ticks = 0  # consecutive ticks without flag detection

        # Loop detection: discretise odometry into grid cells, count revisits
        self._historico_celulas = deque(maxlen=HIST_LEN)
        self._contagem_celulas = {}

        # EVITANDO_LOOP open-loop escape
        self._fase_fuga = 0  # 0 = spinning, 1 = driving forward
        self._t_fuga_inicio = 0.0

        self._missao_completa = False
        self._braco_estado = (
            None  # last sent arm command string; avoids redundant publishes
        )

        # NAVEGANDO sub-mode: "direto" steers to flag, "contornando" wall-follows an obstacle
        self._nav_modo = "direto"
        self._contorno_lado = (
            1.0  # +1 = obstacle on right (go left), -1 = obstacle on left (go right)
        )

        self._timer_init = self.create_timer(1.5, self._retrair_braco_inicial)
        self.create_timer(1.0 / FREQ_CONTROLE, self._loop)

        self.get_logger().info("[FSM] Estado inicial: SEGUINDO_PAREDE")

    def _cb_scan(self, msg):
        self._scan = msg

    def _cb_odom(self, msg):
        self._pos_x = msg.pose.pose.position.x
        self._pos_y = msg.pose.pose.position.y
        self._atualiza_historico()

    def _cb_visao(self, msg):
        # msg.theta > 0 means flag detected; msg.y carries pixel area (not centroid row)
        if msg.theta > 0.5:
            self._bandeira_detectada = True
            self._flag_area = msg.y
        else:
            self._bandeira_detectada = False
            self._flag_area = 0.0

    def _cb_bearing(self, msg):
        self._flag_bearing = msg.data

    def _retrair_braco_inicial(self):
        self._cmd_braco(ARM_RETRAIDO)
        self._timer_init.cancel()

    def _loop(self):
        if self._scan is None or self._missao_completa:
            return
        self._transicionar()
        self._executar_estado()

    def _set_estado(self, novo):
        anterior = self._estado
        self._estado = novo
        self.get_logger().info(
            f"[FSM] Alteração de Estado: {anterior.name} -> {novo.name}"
        )
        self._ao_entrar(novo)

    def _ao_entrar(self, estado):
        if estado == Estado.SEGUINDO_PAREDE:
            self._cmd_braco(ARM_RETRAIDO)
            self._contagem_celulas.clear()
            self._historico_celulas.clear()
            self._flag_perda_ticks = 0
        elif estado == Estado.NAVEGANDO_PARA_BANDEIRA:
            self._flag_perda_ticks = 0
            self._nav_modo = "direto"
            self._contorno_lado = 1.0
        elif estado == Estado.POSICIONANDO_FINAL:
            self._flag_perda_ticks = 0
        elif estado == Estado.EVITANDO_LOOP:
            self._fase_fuga = 0
            self._t_fuga_inicio = time.monotonic()

    def _transicionar(self):
        e = self._estado
        if e == Estado.SEGUINDO_PAREDE:
            if self._bandeira_detectada:
                self._set_estado(Estado.DETECTOU_BANDEIRA)
        elif e == Estado.DETECTOU_BANDEIRA:
            self._set_estado(Estado.NAVEGANDO_PARA_BANDEIRA)
        elif e == Estado.NAVEGANDO_PARA_BANDEIRA:
            if not self._bandeira_detectada:
                self._flag_perda_ticks += 1
                if self._flag_perda_ticks > FLAG_PERDA_MAX:
                    self._set_estado(Estado.SEGUINDO_PAREDE)
            else:
                self._flag_perda_ticks = 0

    def _executar_estado(self):
        e = self._estado
        if e == Estado.SEGUINDO_PAREDE:
            self._exec_seguindo_parede()
        elif e == Estado.DETECTOU_BANDEIRA:
            self._parar()
        elif e == Estado.NAVEGANDO_PARA_BANDEIRA:
            self._exec_navegando()
        elif e == Estado.POSICIONANDO_FINAL:
            self._exec_posicionando()
        elif e == Estado.EVITANDO_LOOP:
            self._exec_evitando_loop()

    def _exec_seguindo_parede(self):
        frente = self._setor_min(0.0, 25.0)
        direita = self._setor_min(270.0, 30.0)

        if frente < DIST_SEGURA_FRENTE:
            self._send_vel(0.0, 0.8)
            return

        validos = [
            r
            for r in self._scan.ranges
            if not math.isinf(r)
            and not math.isnan(r)
            and self._scan.range_min <= r <= self._scan.range_max
        ]
        min_geral = min(validos) if validos else float("inf")

        if min_geral > DIST_BUSCA_PAREDE:
            self._send_vel(VEL_SEGUINDO, 0.0)
        else:
            erro = direita - DIST_ALVO_PAREDE
            omega = max(-OMEGA_MAX, min(OMEGA_MAX, -WALL_KP * erro))
            self._send_vel(VEL_SEGUINDO, omega)

        if self._checar_loop():
            self._set_estado(Estado.EVITANDO_LOOP)

    def _exec_navegando(self):
        frente = self._setor_min(0.0, 25.0)
        esquerda = self._setor_min(60.0, 30.0)
        direita = self._setor_min(300.0, 30.0)

        bearing = self._flag_bearing
        alinhado = self._bandeira_detectada and abs(bearing) <= FLAG_ALINHA_TOL

        # LIDAR cannot distinguish flag from cylinder, so the transition is camera-only:
        # large pixel area means the flag is genuinely close and centred.
        if alinhado and self._flag_area >= FLAG_MIN_AREA_POSICIONANDO:
            self._nav_modo = "direto"
            self._set_estado(Estado.POSICIONANDO_FINAL)
            return

        if frente < DIST_SEGURA_FRENTE:
            omega_safe = NAV_DESVIO_KP if direita < esquerda else -NAV_DESVIO_KP
            self._send_vel(0.0, float(max(-2.0, min(2.0, omega_safe))))
            return

        obstacle_in_path = frente < DIST_NAV_DESVIO

        if obstacle_in_path and self._nav_modo == "direto":
            self._nav_modo = "contornando"
            self._contorno_lado = 1.0 if esquerda > direita else -1.0

        if not obstacle_in_path and self._nav_modo == "contornando":
            self._nav_modo = "direto"

        if self._nav_modo == "contornando":
            if (
                self._contorno_lado > 0
            ):  # obstacle on right — track right wall, turn left
                erro = direita - DIST_ALVO_PAREDE
                omega = float(max(0.3, min(OMEGA_MAX, -WALL_KP * erro)))
            else:  # obstacle on left — track left wall, turn right
                erro = esquerda - DIST_ALVO_PAREDE
                omega = float(min(-0.3, max(-OMEGA_MAX, WALL_KP * erro)))
            speed = float(VEL_BANDEIRA * max(0.3, frente / DIST_NAV_DESVIO))
            self._send_vel(speed, omega)
        else:
            omega_flag = FLAG_KP * bearing if self._bandeira_detectada else 0.0
            if obstacle_in_path:
                # Blend flag heading with avoidance; weight proportional to obstacle proximity
                w = (DIST_NAV_DESVIO - frente) / DIST_NAV_DESVIO
                omega_obs = NAV_DESVIO_KP if direita < esquerda else -NAV_DESVIO_KP
                omega = (1.0 - w) * omega_flag + w * omega_obs
                speed = VEL_BANDEIRA * max(0.3, frente / DIST_NAV_DESVIO)
            else:
                omega = omega_flag
                speed = VEL_BANDEIRA if alinhado else 0.0
            self._send_vel(
                float(max(0.0, speed)),
                float(max(-2.0, min(2.0, omega))),
            )

    def _exec_posicionando(self):
        frente = self._setor_min(0.0, 20.0)

        if frente > DIST_PARAR_FINAL:
            bearing = self._flag_bearing
            omega = max(-1.0, min(1.0, FLAG_KP * 0.5 * bearing))
            self._send_vel(0.15, omega)
        elif self._bandeira_detectada and self._flag_area >= FLAG_MIN_AREA_POSICIONANDO:
            self._parar()
            if not self._missao_completa:
                self._missao_completa = True
                self._cmd_braco(ARM_ESTENDIDO)
                self.get_logger().info(
                    "\n"
                    "╔══════════════════════════════════════════╗\n"
                    "║            Congratulations!              ║\n"
                    "║    Bandeira alcançada com sucesso!       ║\n"
                    "╚══════════════════════════════════════════╝"
                )
        else:
            # Something stopped us but the flag area is too small — likely a cylinder; retry.
            self._set_estado(Estado.NAVEGANDO_PARA_BANDEIRA)

    def _exec_evitando_loop(self):
        decorrido = time.monotonic() - self._t_fuga_inicio

        if self._fase_fuga == 0:
            if decorrido < TEMPO_GIRO_FUGA:
                self._send_vel(0.0, 1.0)
            else:
                self._fase_fuga = 1
                self._t_fuga_inicio = time.monotonic()
        else:
            frente = self._setor_min(0.0, 25.0)
            if decorrido < TEMPO_FWD_FUGA and frente > DIST_SEGURA_FRENTE:
                self._send_vel(VEL_SEGUINDO, 0.0)
            else:
                self._set_estado(Estado.SEGUINDO_PAREDE)

    def _atualiza_historico(self):
        cx = int(self._pos_x / GRID_RES)
        cy = int(self._pos_y / GRID_RES)
        celula = (cx, cy)
        if not self._historico_celulas or self._historico_celulas[-1] != celula:
            self._historico_celulas.append(celula)
            self._contagem_celulas[celula] = self._contagem_celulas.get(celula, 0) + 1

    def _checar_loop(self):
        if not self._contagem_celulas:
            return False
        return max(self._contagem_celulas.values()) >= REVISITAS_LOOP

    def _setor_min(self, centro_graus, meia_largura_graus):
        # Returns the minimum valid range in the angular sector [centre ± half_width].
        # Uses (angle − c + π) mod 2π − π to wrap the difference to [−π, π],
        # which correctly handles the 0°/360° seam in the LIDAR data.
        scan = self._scan
        c = math.radians(centro_graus)
        hw = math.radians(meia_largura_graus)
        melhor = float("inf")
        for i, r in enumerate(scan.ranges):
            if math.isinf(r) or math.isnan(r):
                continue
            if r < scan.range_min or r > scan.range_max:
                continue
            angulo = scan.angle_min + i * scan.angle_increment
            diff = (angulo - c + math.pi) % (2 * math.pi) - math.pi
            if abs(diff) <= hw:
                melhor = min(melhor, r)
        return melhor

    def _send_vel(self, linear, angular):
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self._pub_cmd.publish(msg)

    def _parar(self):
        self._send_vel(0.0, 0.0)

    def _cmd_braco(self, posicoes):
        chave = str(posicoes)
        if self._braco_estado == chave:
            return
        msg = Float64MultiArray()
        msg.data = posicoes
        self._pub_braco.publish(msg)
        self._braco_estado = chave


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
