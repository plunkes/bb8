#!/usr/bin/env python3
"""FSM de alto nível: orquestra m-explore (exploração de fronteiras) e Nav2.

Fluxo:
  INICIALIZANDO -> espera o Nav2 (action server navigate_to_pose) ficar pronto.
  EXPLORANDO    -> libera o explore_lite (explore/resume=True); ele manda goals
                   de fronteira ao Nav2 e o mapa cresce via slam_toolbox.
  NAVEGANDO_PARA_BANDEIRA -> ao detectar a bandeira, pausa o explore
                   (explore/resume=False, que cancela o goal atual no Nav2),
                   calcula a pose à frente da flag (bearing da câmera + range do
                   LIDAR), transforma para o frame 'map' e envia um NavigateToPose.
  POSICIONANDO_FINAL -> chegou: chama o Service /gripper/grab (estende+fecha).
  RETORNANDO_ORIGEM -> navega de volta à pose inicial (gravada na inicialização).

O Nav2 é dono do /cmd_vel (repassado ao diff_drive pelo relay_cmd_vel). Esta FSM
não publica velocidade nem comanda o braço por tópico — o gripper é acionado via
Service (/gripper/grab, gripper_server). Só emite o sinal de controle do explore.
"""

import math
from enum import Enum, auto

import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time

import tf2_geometry_msgs  # noqa: F401 — registra PoseStamped no tf2 (efeito colateral)
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose2D, PoseStamped
from nav2_msgs.action import NavigateToPose
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32
from std_srvs.srv import SetBool
from tf2_ros import Buffer, TransformListener


class Estado(Enum):
    INICIALIZANDO = auto()
    EXPLORANDO = auto()
    NAVEGANDO_PARA_BANDEIRA = auto()
    POSICIONANDO_FINAL = auto()
    RETORNANDO_ORIGEM = auto()


# Braço/gripper agora é acionado via Service (/gripper/grab, SetBool) no nó
# gripper_server — esta FSM não publica mais em /gripper_controller/commands.

# Detecção / aproximação da bandeira
FLAG_DETEC_MIN_TICKS = 3  # ticks consecutivos de detecção antes de comutar p/ navegação
FLAG_PERDA_MAX = 25  # ticks sem detecção, durante a navegação, antes de re-explorar
STOP_DIST = 0.75  # [m] distância de parada à frente da flag (braço alcança o mastro)
SETOR_BANDEIRA = math.radians(
    8.0
)  # meia-largura do setor LIDAR amostrado em torno do bearing
RANGE_FALLBACK = 2.5  # [m] estimativa de distância se o LIDAR não retornar no setor
NAV_RETRY_MAX = 3  # tentativas de reenvio de goal antes de desistir e re-explorar

FREQ_CONTROLE = 10  # [Hz] taxa do laço principal da FSM


class ControleRobo(Node):
    def __init__(self):
        super().__init__("controle_robo")

        qos_be = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # --- Entradas ---
        self.create_subscription(LaserScan, "/scan", self._cb_scan, qos_be)
        self.create_subscription(Pose2D, "/vision/flag_detection", self._cb_visao, 10)
        self.create_subscription(Float32, "/vision/flag_bearing", self._cb_bearing, 10)

        # --- Saídas ---
        # Controle do explore_lite: True retoma, False pausa (e cancela goal no Nav2).
        self._pub_explore = self.create_publisher(Bool, "explore/resume", 10)

        # Service do gripper (Task 4): True = estende+fecha, False = retrai+abre.
        self._gripper_cli = self.create_client(SetBool, "/gripper/grab")

        # --- TF: usado para transformar a pose da bandeira base_link -> map ---
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # --- Action client do Nav2 ---
        self._nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        # --- Estado interno ---
        self._estado = Estado.INICIALIZANDO
        self._scan = None
        self._bandeira_detectada = False
        self._flag_bearing = 0.0  # [rad] + = bandeira à esquerda
        self._flag_detec_ticks = 0  # ticks consecutivos COM detecção
        self._flag_perda_ticks = 0  # ticks consecutivos SEM detecção
        self._missao_completa = False
        self._pose_origem = None  # PoseStamped em 'map', gravada ao iniciar

        # Acompanhamento do goal do Nav2
        self._goal_handle = None
        self._nav_status = None  # None | 'pendente' | 'sucesso' | 'falha'
        self._nav_retries = 0

        self.create_timer(1.0 / FREQ_CONTROLE, self._loop)

        self.get_logger().info("[FSM] Estado inicial: INICIALIZANDO (aguardando Nav2)")

    # ------------------------------------------------------------------ #
    # Callbacks de sensores
    # ------------------------------------------------------------------ #
    def _cb_scan(self, msg):
        self._scan = msg

    def _cb_visao(self, msg):
        # theta > 0.5 indica bandeira detectada.
        self._bandeira_detectada = msg.theta > 0.5

    def _cb_bearing(self, msg):
        self._flag_bearing = msg.data

    # ------------------------------------------------------------------ #
    # Laço principal
    # ------------------------------------------------------------------ #
    def _loop(self):
        if self._missao_completa:
            return

        # Contadores de detecção (válidos em qualquer estado)
        if self._bandeira_detectada:
            self._flag_detec_ticks += 1
            self._flag_perda_ticks = 0
        else:
            self._flag_detec_ticks = 0
            self._flag_perda_ticks += 1

        e = self._estado
        if e == Estado.INICIALIZANDO:
            self._exec_inicializando()
        elif e == Estado.EXPLORANDO:
            self._exec_explorando()
        elif e == Estado.NAVEGANDO_PARA_BANDEIRA:
            self._exec_navegando()
        elif e == Estado.POSICIONANDO_FINAL:
            self._exec_posicionando()
        elif e == Estado.RETORNANDO_ORIGEM:
            self._exec_retornando()

    def _set_estado(self, novo):
        anterior = self._estado
        self._estado = novo
        self.get_logger().info(f"[FSM] {anterior.name} -> {novo.name}")
        self._ao_entrar(novo)

    def _ao_entrar(self, estado):
        if estado == Estado.EXPLORANDO:
            self._set_explore(True)  # libera o m-explore
        elif estado == Estado.NAVEGANDO_PARA_BANDEIRA:
            self._set_explore(False)  # pausa o m-explore (cancela goal no Nav2)
            self._nav_retries = 0
            self._nav_status = None
            self._goal_handle = None
            self._enviar_goal_bandeira()
        elif estado == Estado.POSICIONANDO_FINAL:
            self._set_explore(False)
            self._pegar_flag()  # (d) estende+fecha o gripper via Service
        elif estado == Estado.RETORNANDO_ORIGEM:
            self._set_explore(False)
            self._nav_retries = 0
            self._nav_status = None
            self._goal_handle = None
            self._enviar_goal_origem()  # (e) volta à pose inicial

    # ------------------------------------------------------------------ #
    # Estados
    # ------------------------------------------------------------------ #
    def _exec_inicializando(self):
        # Espera o Nav2 expor o action server e grava a origem antes de explorar.
        if not self._nav_client.server_is_ready():
            self.get_logger().info(
                "[FSM] Aguardando action server navigate_to_pose...",
                throttle_duration_sec=3.0,
            )
            return
        if self._pose_origem is None:
            self._pose_origem = self._pose_atual_em_map()
            if self._pose_origem is None:
                self.get_logger().info(
                    "[FSM] Aguardando TF map->base_link p/ gravar origem...",
                    throttle_duration_sec=2.0,
                )
                return
        self.get_logger().info("[FSM] Nav2 pronto e origem gravada.")
        self._set_estado(Estado.EXPLORANDO)

    def _exec_explorando(self):
        # O explore_lite cuida da navegação; só vigiamos a bandeira.
        if self._flag_detec_ticks >= FLAG_DETEC_MIN_TICKS:
            self._set_estado(Estado.NAVEGANDO_PARA_BANDEIRA)

    def _exec_navegando(self):
        # 1) Resultado do goal atual
        if self._nav_status == "sucesso":
            self._set_estado(Estado.POSICIONANDO_FINAL)
            return
        if self._nav_status == "falha":
            self._nav_status = None
            if self._nav_retries < NAV_RETRY_MAX and self._bandeira_detectada:
                self._nav_retries += 1
                self.get_logger().warn(
                    f"[FSM] Goal falhou — retry {self._nav_retries}/{NAV_RETRY_MAX}"
                )
                self._enviar_goal_bandeira()
            else:
                self.get_logger().warn(
                    "[FSM] Desistindo do goal — voltando a explorar."
                )
                self._cancelar_goal()
                self._set_estado(Estado.EXPLORANDO)
            return

        # 2) Perdeu a bandeira de vista por tempo demais -> volta a explorar
        if self._flag_perda_ticks > FLAG_PERDA_MAX:
            self.get_logger().warn("[FSM] Bandeira perdida — voltando a explorar.")
            self._cancelar_goal()
            self._set_estado(Estado.EXPLORANDO)

    def _exec_posicionando(self):
        # _pegar_flag (em _ao_entrar) já comutou para RETORNANDO_ORIGEM.
        pass

    def _exec_retornando(self):
        if self._nav_status == "sucesso":
            self._concluir_missao()
        elif self._nav_status == "falha":
            self._nav_status = None
            if self._nav_retries < NAV_RETRY_MAX:
                self._nav_retries += 1
                self.get_logger().warn(
                    f"[FSM] Retorno falhou — retry {self._nav_retries}/{NAV_RETRY_MAX}"
                )
                self._enviar_goal_origem()
            else:
                self.get_logger().error("[FSM] Não foi possível retornar à origem.")
                self._missao_completa = True

    # ------------------------------------------------------------------ #
    # Cálculo e envio do goal da bandeira
    # ------------------------------------------------------------------ #
    def _enviar_goal_bandeira(self):
        pose = self._calcular_pose_bandeira()
        if pose is None:
            # Sem TF/scan ainda: tenta de novo no próximo tick (não comuta estado).
            self.get_logger().warn(
                "[FSM] Não foi possível calcular a pose da bandeira ainda.",
                throttle_duration_sec=1.0,
            )
            self._nav_status = "falha"
            return

        if not self._nav_client.server_is_ready():
            self._nav_status = "falha"
            return

        goal = NavigateToPose.Goal()
        goal.pose = pose
        self._nav_status = "pendente"
        self.get_logger().info(
            f"[FSM] Enviando goal da bandeira: "
            f"({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f}) [map]"
        )
        send_future = self._nav_client.send_goal_async(goal)
        send_future.add_done_callback(self._on_goal_response)

    def _enviar_goal_origem(self):
        """(e) Manda o robô de volta à pose inicial gravada (frame 'map')."""
        if self._pose_origem is None or not self._nav_client.server_is_ready():
            self.get_logger().error("[FSM] Sem origem/Nav2 para retornar.")
            self._nav_status = "falha"
            return
        self._pose_origem.header.stamp = self.get_clock().now().to_msg()
        goal = NavigateToPose.Goal()
        goal.pose = self._pose_origem
        self._nav_status = "pendente"
        self.get_logger().info(
            f"[FSM] Retornando à origem: "
            f"({self._pose_origem.pose.position.x:.2f}, "
            f"{self._pose_origem.pose.position.y:.2f}) [map]"
        )
        send_future = self._nav_client.send_goal_async(goal)
        send_future.add_done_callback(self._on_goal_response)

    def _pose_atual_em_map(self):
        """Pose atual do base_link no frame 'map' (None se TF indisponível)."""
        try:
            t = self._tf_buffer.lookup_transform(
                "map", "base_link", Time(), timeout=Duration(seconds=0.3)
            )
        except Exception:
            return None
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = t.transform.translation.x
        pose.pose.position.y = t.transform.translation.y
        pose.pose.position.z = 0.0
        pose.pose.orientation = t.transform.rotation
        return pose

    def _calcular_pose_bandeira(self):
        """Funde bearing (câmera) + range (LIDAR) -> PoseStamped à frente da flag em 'map'."""
        if self._scan is None:
            return None

        bearing = self._flag_bearing
        rng = self._range_no_setor(bearing, SETOR_BANDEIRA)
        if rng is None:
            rng = RANGE_FALLBACK

        # Distância do goal: para STOP_DIST antes da flag. Se já estamos perto,
        # mira a própria posição da flag (clamp >= 0) — o controlador do Nav2 para.
        goal_rng = max(0.0, rng - STOP_DIST)

        # Posição na direção do bearing, no frame base_link (+x à frente, +y à esquerda).
        gx = goal_rng * math.cos(bearing)
        gy = goal_rng * math.sin(bearing)

        pose_base = PoseStamped()
        pose_base.header.frame_id = "base_link"
        pose_base.header.stamp = Time().to_msg()  # tempo 0 -> usa o TF mais recente
        pose_base.pose.position.x = gx
        pose_base.pose.position.y = gy
        pose_base.pose.position.z = 0.0
        qz, qw = math.sin(bearing / 2.0), math.cos(bearing / 2.0)  # yaw = bearing
        pose_base.pose.orientation.z = qz
        pose_base.pose.orientation.w = qw

        try:
            pose_map = self._tf_buffer.transform(
                pose_base, "map", timeout=Duration(seconds=0.3)
            )
        except Exception as exc:  # TransformException e afins
            self.get_logger().warn(
                f"[FSM] Falha ao transformar base_link->map: {exc}",
                throttle_duration_sec=1.0,
            )
            return None

        pose_map.header.stamp = self.get_clock().now().to_msg()
        return pose_map

    def _range_no_setor(self, centro_rad, meia_largura_rad):
        """Menor range válido do LIDAR num setor [centro ± meia_largura]. None se vazio."""
        scan = self._scan
        melhor = None
        for i, r in enumerate(scan.ranges):
            if math.isinf(r) or math.isnan(r):
                continue
            if r < scan.range_min or r > scan.range_max:
                continue
            ang = scan.angle_min + i * scan.angle_increment
            diff = (ang - centro_rad + math.pi) % (2 * math.pi) - math.pi
            if abs(diff) <= meia_largura_rad:
                melhor = r if melhor is None else min(melhor, r)
        return melhor

    # ------------------------------------------------------------------ #
    # Callbacks do action client (rodam na thread do executor)
    # ------------------------------------------------------------------ #
    def _on_goal_response(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn("[FSM] Goal REJEITADO pelo Nav2.")
            self._goal_handle = None
            self._nav_status = "falha"
            return
        self._goal_handle = handle
        result_future = handle.get_result_async()
        result_future.add_done_callback(self._on_goal_result)

    def _on_goal_result(self, future):
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("[FSM] Nav2 reportou goal ALCANÇADO.")
            self._nav_status = "sucesso"
        elif status == GoalStatus.STATUS_CANCELED:
            # Cancelamento deliberado nosso — não marca como falha.
            self.get_logger().info("[FSM] Goal cancelado.")
        else:
            self.get_logger().warn(
                f"[FSM] Goal terminou sem sucesso (status={status})."
            )
            self._nav_status = "falha"
        self._goal_handle = None

    def _cancelar_goal(self):
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
            self._goal_handle = None
        self._nav_status = None

    # ------------------------------------------------------------------ #
    # Conclusão da missão
    # ------------------------------------------------------------------ #
    def _pegar_flag(self):
        """(d) Estende o braço e fecha a garra via Service SetBool(True)."""
        if not self._gripper_cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("[FSM] Service /gripper/grab indisponível.")
        else:
            req = SetBool.Request()
            req.data = True
            self._gripper_cli.call_async(req)
            self.get_logger().info("[FSM] Flag agarrada — gripper estendido e fechado.")
        # Pegou a flag: volta à origem.
        self._set_estado(Estado.RETORNANDO_ORIGEM)

    def _concluir_missao(self):
        self._cancelar_goal()
        self._missao_completa = True
        self.get_logger().info(
            "\n"
            "╔══════════════════════════════════════════╗\n"
            "║            Congratulations!              ║\n"
            "║  Flag capturada e robô de volta à base!  ║\n"
            "╚══════════════════════════════════════════╝"
        )

    # ------------------------------------------------------------------ #
    # Utilitários
    # ------------------------------------------------------------------ #
    def _set_explore(self, ativar):
        self._pub_explore.publish(Bool(data=bool(ativar)))


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
