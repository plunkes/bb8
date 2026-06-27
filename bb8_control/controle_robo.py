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
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.time import Time

import tf2_geometry_msgs  # noqa: F401 — registra PoseStamped no tf2 (efeito colateral)
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose2D, PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
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
    BUSCANDO_PAREDE = auto()  # robô encravado em área aberta: avança p/ achar parede


# Braço/gripper agora é acionado via Service (/gripper/grab, SetBool) no nó
# gripper_server — esta FSM não publica mais em /gripper_controller/commands.

# Detecção / aproximação da bandeira
FLAG_DETEC_MIN_TICKS = (
    10  # ticks consecutivos de detecção antes de comutar p/ navegação
)
FLAG_PERDA_MAX = 25  # ticks sem detecção, durante a navegação, antes de re-explorar
STOP_DIST = 0.45  # [m] distância de parada à frente da flag (braço alcança o mastro)
SETOR_BANDEIRA = math.radians(
    8.0
)  # meia-largura do setor LIDAR amostrado em torno do bearing
RANGE_FALLBACK = 2.5  # [m] estimativa de distância se o LIDAR não retornar no setor
NAV_RETRY_MAX = 3  # tentativas de reenvio de goal antes de desistir e re-explorar
# Bandeira costuma estar fora da área já mapeada pelo LIDAR -> goal cai fora do
# grid do costmap e o Nav2 trava. Tenta o goal cheio e, se cair fora dos limites
# do mapa, vai pegando metades do trajeto até voltar pra dentro (incl. desconhecido).
GOAL_FRACOES = (1.0, 0.5, 0.25, 0.125, 0.0625)
# Se o LIDAR no setor da flag vier mais curto que a estimativa da câmera por mais
# que isto, há um OBSTÁCULO entre o robô e a flag -> confia na câmera (não no LIDAR).
OBSTACULO_TOL = 0.5  # [m]

# Critérios para ENTRAR em POSICIONANDO_FINAL (pegar a flag):
FLAG_AREA_MIN_PX = 900  # [px] flag ocupa >= isto na imagem => muito perto
FLAG_ALIGN_MAX = math.radians(6.0)  # [rad] |bearing| máximo => alinhado com a flag
FLAG_RANGE_MAX = 1.0  # [m] range do LIDAR até a flag p/ validar proximidade
GOAL_REFRESH_TICKS = 10  # re-mira o goal de aproximação a cada ~1 s (rastreia a flag)

# Servo visual (aproximação final): assume o controle quando a flag está perto,
# mantendo-a no centro da câmera. Evita o Nav2 girar/colapsar o goal de perto.
VS_DIST = 1.0  # [m] abaixo disto, controle visual simples em vez de goals do Nav2
VS_KP = 1.5  # ganho de giro [rad/s por rad de bearing]
VS_MAX_W = 1.2  # [rad/s] giro máximo no servo
VS_VEL = 0.25  # [m/s] avanço quando a flag está centrada
VS_ALIGN = math.radians(12.0)  # |bearing| p/ considerar centrada e poder avançar

# Sequência de captura em POSICIONANDO_FINAL (ticks @ FREQ_CONTROLE=10Hz):
GRIPPER_EXTEND_TICKS = 15  # ~1.5 s p/ o braço estender antes de avançar
GRIPPER_CLOSE_TICKS = 15  # ~1.5 s p/ a garra fechar na flag antes de erguer
GRIPPER_LIFT_TICKS = 15  # ~1.5 s p/ o ombro erguer a flag antes de retornar
GRAB_DIST = 0.3  # [m] para de avançar quando a flag está ao alcance do braço
CREEP_VEL = 0.12  # [m/s] avanço lento e final até encostar na flag
CREEP_MAX_TICKS = 20  # ticks ~3 s máx de avanço final (segurança)

# Footprint para a checagem de colisão do Nav2:
#   normal (braço retraído) vs com o braço estendido segurando a flag (+~0.4 m à frente).
FOOTPRINT_NORMAL = "[[0.23, 0.17], [0.23, -0.17], [-0.23, -0.17], [-0.23, 0.17]]"
FOOTPRINT_COM_BRACO = "[[0.62, 0.17], [0.62, -0.17], [-0.23, -0.17], [-0.23, 0.17]]"

# "Encravado" em área aberta: explore manda goals pro lugar onde o robô já está
# (LIDAR sem retorno -> sem fronteira). Detecta pouca movimentação e avança a esmo.
STUCK_MIN_MOVE = 0.15  # [m] deslocamento mínimo p/ considerar que está se movendo
STUCK_MAX_TICKS = 50  # ticks ~5 s quase parado em EXPLORANDO antes de avançar
AVANCO_MAX_TICKS = 40  # ticks ~4 s indo p/ frente antes de voltar a explorar
INICIO_PAREDE_TICKS = 30  # ticks ~3 s seguindo parede logo ao inicializar
AVANCO_VEL = 0.4  # [m/s] velocidade ao avançar à procura de parede
AVANCO_FRONT_SECTOR = math.radians(20.0)  # meia-largura do setor frontal vigiado
AVANCO_SAFE_DIST = 0.7  # [m] obstáculo mais perto que isto à frente -> não avança
AVANCO_GIRO = 0.6  # [rad/s] giro à esquerda quando há obstáculo à frente

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
        self.create_subscription(
            Float32, "/vision/flag_distance", self._cb_distancia, 10
        )
        # Global costmap (latched/transient_local) p/ checar se o goal da bandeira
        # cai em célula livre+conhecida antes de mandar p/ o Nav2.
        qos_costmap = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(
            OccupancyGrid, "/global_costmap/costmap", self._cb_costmap, qos_costmap
        )

        # --- Saídas ---
        # Controle do explore_lite: True retoma, False pausa (e cancela goal no Nav2).
        self._pub_explore = self.create_publisher(Bool, "explore/resume", 10)
        # Velocidade direta (só usada no estado BUSCANDO_PAREDE; Nav2 não tem goal lá).
        self._pub_cmd = self.create_publisher(Twist, "/cmd_vel", 10)
        # Modo "pole": pede ao vision_processor p/ olhar só a metade de baixo da
        # câmera (mastro), ignorando o painel da bandeira no topo. Ativo em POSICIONANDO.
        self._pub_pole_mode = self.create_publisher(Bool, "/vision/pole_mode", 10)

        # Services do gripper: estende (braço) e grab (fecha a garra).
        self._gripper_extend_cli = self.create_client(SetBool, "/gripper/extend")
        self._gripper_cli = self.create_client(SetBool, "/gripper/grab")
        self._gripper_lift_cli = self.create_client(SetBool, "/gripper/lift")
        # Clients p/ trocar o footprint dos costmaps do Nav2 em runtime (braço estendido).
        self._fp_local_cli = self.create_client(
            SetParameters, "/local_costmap/local_costmap/set_parameters"
        )
        self._fp_global_cli = self.create_client(
            SetParameters, "/global_costmap/global_costmap/set_parameters"
        )

        # --- TF: usado para transformar a pose da bandeira base_link -> map ---
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # --- Action client do Nav2 ---
        self._nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        # --- Estado interno ---
        self._estado = Estado.INICIALIZANDO
        self._scan = None
        self._costmap = None  # último OccupancyGrid do global_costmap
        self._bandeira_detectada = False
        self._flag_bearing = 0.0  # [rad] + = bandeira à esquerda
        self._flag_area = 0.0  # [px] área da flag na imagem (do vision_processor)
        self._flag_distance = 0.0  # [m] distância estimada pela câmera (pinhole)
        self._flag_detec_ticks = 0  # ticks consecutivos COM detecção
        self._flag_perda_ticks = 0  # ticks consecutivos SEM detecção
        self._goal_refresh_ticks = 0  # contador p/ re-mirar o goal de aproximação
        self._servo_ativo = False  # latch: controle visual assumiu (Nav2 desligado)

        # Detecção de "encravado" + avanço à procura de parede
        self._stuck_ref = None  # PoseStamped de referência p/ medir deslocamento
        self._stuck_ticks = 0  # ticks quase parado em EXPLORANDO
        self._avanco_ticks = 0  # ticks indo p/ frente em BUSCANDO_PAREDE
        self._avanco_budget = (
            AVANCO_MAX_TICKS  # duração da entrada atual em BUSCANDO_PAREDE
        )
        self._inicio_parede_pendente = True  # 1º BUSCANDO_PAREDE = só 3 s de início
        self._posic_ticks = 0  # ticks na sequência de captura (POSICIONANDO_FINAL)
        self._posic_fase = 0  # 0 = estendendo braço, 1 = garra fechando
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

    def _cb_costmap(self, msg):
        self._costmap = msg

    def _cb_visao(self, msg):
        # Pose2D do vision_processor: x=centroide_x, y=área[px], theta=1.0 se detectada.
        self._bandeira_detectada = msg.theta > 0.5
        self._flag_area = msg.y

    def _cb_bearing(self, msg):
        self._flag_bearing = msg.data

    def _cb_distancia(self, msg):
        self._flag_distance = msg.data

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
        elif e == Estado.BUSCANDO_PAREDE:
            self._exec_buscando_parede()

    def _set_estado(self, novo):
        anterior = self._estado
        self._estado = novo
        self.get_logger().info(f"[FSM] {anterior.name} -> {novo.name}")
        self._ao_entrar(novo)

    def _ao_entrar(self, estado):
        # Só olha o mastro (metade de baixo) durante a captura final.
        self._set_pole_mode(estado == Estado.POSICIONANDO_FINAL)
        if estado == Estado.EXPLORANDO:
            self._set_explore(True)  # libera o m-explore
            self._stuck_ref = None  # reinicia detecção de encravamento
            self._stuck_ticks = 0
        elif estado == Estado.BUSCANDO_PAREDE:
            self._set_explore(False)  # pausa o m-explore (libera o /cmd_vel)
            self._avanco_ticks = 0
            # 1ª vez (logo após inicializar) dura só ~3 s; depois é recovery de ~4 s.
            if self._inicio_parede_pendente:
                self._avanco_budget = INICIO_PAREDE_TICKS
                self._inicio_parede_pendente = False
            else:
                self._avanco_budget = AVANCO_MAX_TICKS
        elif estado == Estado.NAVEGANDO_PARA_BANDEIRA:
            self._set_explore(False)  # pausa o m-explore (cancela goal no Nav2)
            self._nav_retries = 0
            self._nav_status = None
            self._goal_handle = None
            self._goal_refresh_ticks = 0
            self._servo_ativo = False  # começa sob controle do Nav2
            self._enviar_goal_bandeira()
        elif estado == Estado.POSICIONANDO_FINAL:
            self._set_explore(False)
            self._posic_ticks = 0
            self._posic_fase = 0
            self._gripper_extend(True)  # fase 0: estende o braço (garra aberta)
            self.get_logger().info("[FSM] Na bandeira — estendendo o braço.")
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
        # Começa seguindo parede por ~3 s antes de liberar a exploração.
        self._set_estado(Estado.BUSCANDO_PAREDE)

    def _exec_explorando(self):
        # O explore_lite cuida da navegação; vigiamos a bandeira...
        if self._flag_detec_ticks >= FLAG_DETEC_MIN_TICKS:
            self._set_estado(Estado.NAVEGANDO_PARA_BANDEIRA)
            return
        # ...e se o robô encravou (parado recebendo goals pro próprio lugar).
        if self._detectar_encravado():
            self.get_logger().warn(
                "[FSM] Encravado em área aberta — avançando p/ achar parede."
            )
            self._set_estado(Estado.BUSCANDO_PAREDE)

    def _detectar_encravado(self):
        """True se o robô mal se moveu por STUCK_MAX_TICKS ticks em EXPLORANDO."""
        pos = self._pose_atual_em_map()
        if pos is None:
            return False
        if self._stuck_ref is None:
            self._stuck_ref = pos
            self._stuck_ticks = 0
            return False
        dx = pos.pose.position.x - self._stuck_ref.pose.position.x
        dy = pos.pose.position.y - self._stuck_ref.pose.position.y
        if math.hypot(dx, dy) > STUCK_MIN_MOVE:
            self._stuck_ref = pos  # andou: reseta referência
            self._stuck_ticks = 0
            return False
        self._stuck_ticks += 1
        return self._stuck_ticks > STUCK_MAX_TICKS

    def _exec_buscando_parede(self):
        # Achou a bandeira durante o avanço? vai atrás dela.
        if self._flag_detec_ticks >= FLAG_DETEC_MIN_TICKS:
            self._parar()
            self._set_estado(Estado.NAVEGANDO_PARA_BANDEIRA)
            return
        # Tempo de avanço esgotado -> volta a explorar (achou parede nova p/ mapear).
        self._avanco_ticks += 1
        if self._avanco_ticks > self._avanco_budget:
            self._parar()
            self._set_estado(Estado.EXPLORANDO)
            return
        # Desvio: olha o setor frontal. Obstáculo perto -> não avança, gira à esquerda.
        front = self._range_no_setor(0.0, AVANCO_FRONT_SECTOR)
        cmd = Twist()
        if front is not None and front < AVANCO_SAFE_DIST:
            cmd.linear.x = 0.0
            cmd.angular.z = AVANCO_GIRO  # vira à esquerda procurando passagem/parede
        else:
            cmd.linear.x = AVANCO_VEL
            cmd.angular.z = 0.0
        self._pub_cmd.publish(cmd)

    def _parar(self):
        """Publica velocidade zero (encerra o avanço manual)."""
        self._pub_cmd.publish(Twist())

    def _exec_navegando(self):
        # 1) Chegou? alinhado + flag grande na imagem + perto pelo LIDAR -> pega.
        if self._pronto_para_pegar():
            self.get_logger().info(
                "[FSM] Flag alinhada, grande e próxima -> POSICIONANDO_FINAL."
            )
            self._cancelar_goal()
            self._set_estado(Estado.POSICIONANDO_FINAL)
            return

        # 2) Perdeu a bandeira de vista por tempo demais -> volta a explorar.
        if self._flag_perda_ticks > FLAG_PERDA_MAX:
            self.get_logger().warn("[FSM] Bandeira perdida — voltando a explorar.")
            self._cancelar_goal()
            self._set_estado(Estado.EXPLORANDO)
            return

        # 2b) Perto (<VS_DIST): assume o CONTROLE VISUAL SIMPLES e o mantém travado
        #     (latch). O Nav2 manda paths confusos de perto; uma vez que o servo
        #     assume, não devolvemos o controle ao Nav2 (só sai daqui se pegar a
        #     flag ou perdê-la de vista por tempo demais, tratado acima).
        rng = self._estimar_range_flag()
        if self._servo_ativo or (rng is not None and rng <= VS_DIST):
            if not self._servo_ativo:
                self._servo_ativo = True
                self.get_logger().info(
                    "[FSM] Flag perto (<1m) — controle visual simples, Nav2 desligado."
                )
            self._cancelar_goal()
            self._servo_visual()
            return

        # 3) Goal de aproximação falhou -> retry; se esgotar, re-explora.
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

        # 4) Persegue: re-mira o goal periodicamente p/ rastrear a flag enquanto
        #    se aproxima (o bearing/range vão refinando à medida que chega perto).
        self._goal_refresh_ticks += 1
        if self._goal_refresh_ticks >= GOAL_REFRESH_TICKS:
            self._goal_refresh_ticks = 0
            if self._bandeira_detectada:
                self._enviar_goal_bandeira()

    def _pronto_para_pegar(self):
        """True se a flag está centrada na câmera e perto o suficiente p/ pegar."""
        if not self._bandeira_detectada:
            return False
        if abs(self._flag_bearing) > FLAG_ALIGN_MAX:  # precisa estar centrada
            return False
        rng = self._estimar_range_flag()
        if rng is None:
            return False
        # Perto e grande na imagem, OU muito perto (área pode falhar de tão perto).
        perto = rng <= FLAG_RANGE_MAX and self._flag_area >= FLAG_AREA_MIN_PX
        muito_perto = rng <= 0.6
        return perto or muito_perto

    def _servo_visual(self):
        """Aproximação final: gira p/ centrar a flag (bearing->0) e avança quando
        centrada. Publica /cmd_vel direto (Nav2 sem goal nesta fase)."""
        if not self._bandeira_detectada:
            # Sumiu neste frame: para (não gira perdido). Perda longa -> re-explora
            # é tratada no início de _exec_navegando.
            self._parar()
            return
        bearing = self._flag_bearing  # >0 = flag à esquerda -> girar à esquerda (+z)
        cmd = Twist()
        w = VS_KP * bearing
        cmd.angular.z = max(-VS_MAX_W, min(VS_MAX_W, w))
        # Só avança quando a flag está centrada; senão gira no lugar p/ centrar.
        cmd.linear.x = VS_VEL if abs(bearing) < VS_ALIGN else 0.0
        self._pub_cmd.publish(cmd)

    def _exec_posicionando(self):
        # Captura em 4 fases: (0) estende o braço no nível da flag, (1) avança até
        # encostar na flag, (2) fecha a garra, (3) ERGUE a flag (ombro 45°) — só
        # depois disso troca o footprint e retorna à origem. Erguer tira a flag da
        # frente do robô e o scan_masker mascara o setor frontal (não vira obstáculo).
        self._posic_ticks += 1

        if self._posic_fase == 0:  # estendendo o braço
            if self._posic_ticks >= GRIPPER_EXTEND_TICKS:
                self._posic_fase = 1
                self._posic_ticks = 0
            return

        if self._posic_fase == 1:  # centra no pole e avança até a flag ao alcance
            bearing = self._flag_bearing
            alinhado = self._bandeira_detectada and abs(bearing) <= FLAG_ALIGN_MAX
            front = self._range_no_setor(0.0, AVANCO_FRONT_SECTOR)
            perto = front is not None and front <= GRAB_DIST
            # Só FECHA a garra se estiver alinhado com o pole da flag.
            if perto and alinhado:
                self._parar()
                self._gripper_grab(True)  # fecha a garra na bandeira
                self.get_logger().info("[FSM] Alinhado e encostado — fechando a garra.")
                self._posic_fase = 2
                self._posic_ticks = 0
            elif self._posic_ticks > CREEP_MAX_TICKS:
                # Não conseguiu alinhar/encostar a tempo: NÃO pega torto — re-aproxima.
                self._parar()
                self.get_logger().warn(
                    "[FSM] Não alinhou com o pole a tempo — re-aproximando."
                )
                self._set_estado(Estado.NAVEGANDO_PARA_BANDEIRA)
            else:
                cmd = Twist()
                # Mantém o pole centrado; só avança quando alinhado.
                if self._bandeira_detectada:
                    w = VS_KP * bearing
                    cmd.angular.z = max(-VS_MAX_W, min(VS_MAX_W, w))
                cmd.linear.x = CREEP_VEL if alinhado else 0.0
                self._pub_cmd.publish(cmd)
            return

        if self._posic_fase == 2:  # garra fechando
            if self._posic_ticks >= GRIPPER_CLOSE_TICKS:
                # Garra fechou em volta da flag: agora SIM ergue a flag (ombro 45°).
                self._gripper_lift(True)
                self.get_logger().info("[FSM] Garra fechada — erguendo a flag.")
                self._posic_fase = 3
                self._posic_ticks = 0
            return

        if self._posic_fase == 3:  # erguendo a flag (ombro 45°)
            if self._posic_ticks >= GRIPPER_LIFT_TICKS:
                # Flag erguida e fora do FOV frontal (scan_masker mascara): aumenta
                # o footprint p/ manobrar e retorna à origem.
                self._set_footprint(FOOTPRINT_COM_BRACO)
                self.get_logger().info(
                    "[FSM] Flag erguida; footprint com braço — retornando à origem."
                )
                self._set_estado(Estado.RETORNANDO_ORIGEM)

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
        # Tenta o goal cheio e, se cair fora do mapa/em célula desconhecida (a flag
        # costuma estar além do que o LIDAR já mapeou), encurta o trajeto pela metade
        # sucessivamente até achar um goal alcançável dentro do global costmap.
        pose = None
        fracao_ok = 1.0
        for fracao in GOAL_FRACOES:
            cand = self._calcular_pose_bandeira(fracao)
            if cand is None:
                # Sem TF/scan ainda: tenta de novo no próximo tick (não comuta estado).
                self.get_logger().warn(
                    "[FSM] Não foi possível calcular a pose da bandeira ainda.",
                    throttle_duration_sec=1.0,
                )
                self._nav_status = "falha"
                return
            if self._goal_alcancavel(cand):
                pose = cand
                fracao_ok = fracao
                break

        if pose is None:
            self.get_logger().warn(
                "[FSM] Nenhuma fração do trajeto até a bandeira caiu em célula "
                "livre/conhecida; aguardando mais mapa.",
                throttle_duration_sec=1.0,
            )
            self._nav_status = "falha"
            return

        if fracao_ok < 1.0:
            self.get_logger().info(
                f"[FSM] Goal cheio inalcançável (flag fora do mapa); "
                f"mirando {fracao_ok * 100:.0f}% do trajeto."
            )

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

    def _estimar_range_flag(self):
        """Distância robusta até a flag, fundindo câmera (pinhole) + LIDAR.

        - LIDAR é preciso, MAS se houver obstáculo entre robô e flag o raio bate
          no obstáculo e vem curto. A câmera estima pelo tamanho aparente e ignora
          o obstáculo.
        - Regra: se o LIDAR vier mais curto que a estimativa da câmera por mais que
          OBSTACULO_TOL, há obstáculo no caminho -> usa a câmera. Senão, usa o LIDAR
          (mais preciso quando a linha de visão está livre).
        Retorna None se não há nenhuma fonte.
        """
        cam = self._flag_distance if self._flag_distance > 0.0 else None
        lidar = self._range_no_setor(self._flag_bearing, SETOR_BANDEIRA)
        if cam is not None and lidar is not None:
            return cam if lidar < cam - OBSTACULO_TOL else lidar
        if cam is not None:
            return cam
        return lidar  # pode ser None

    def _calcular_pose_bandeira(self, fracao=1.0):
        """Funde bearing (câmera) + range (câmera/LIDAR) -> PoseStamped em 'map'.

        `fracao` encurta o trajeto: 1.0 = goal cheio (junto à flag), 0.5 = metade
        do caminho, etc. Usado para puxar o goal p/ dentro da área mapeada quando
        a flag está fora do alcance já visto pelo LIDAR.
        """
        if self._scan is None:
            return None

        bearing = self._flag_bearing
        rng = self._estimar_range_flag()
        if rng is None:
            rng = RANGE_FALLBACK

        # Distância do goal: para STOP_DIST antes da flag. Se já estamos perto,
        # mira a própria posição da flag (clamp >= 0) — o controlador do Nav2 para.
        goal_rng = max(0.0, rng - STOP_DIST) * fracao

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

    def _goal_alcancavel(self, pose_map):
        """True se a pose (frame 'map') cai DENTRO dos limites do global costmap.

        Validade = estar dentro do mapa, e SÓ isso. Células desconhecidas (-1)
        contam como válidas (a flag costuma estar além do que o LIDAR já mapeou);
        o único caso inválido é o ponto cair fora do tamanho do grid -> aí o
        chamador encurta o trajeto até voltar pra dentro. Sem costmap -> True."""
        grid = self._costmap
        if grid is None:
            return True
        info = grid.info
        if info.resolution <= 0.0:
            return True
        mx = int((pose_map.pose.position.x - info.origin.position.x) / info.resolution)
        my = int((pose_map.pose.position.y - info.origin.position.y) / info.resolution)
        # Único critério de invalidez: fora dos limites do mapa.
        return 0 <= mx < info.width and 0 <= my < info.height

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
    def _gripper_extend(self, estender):
        """Chama /gripper/extend (True = braço à frente / garra aberta)."""
        self._chamar_gripper(self._gripper_extend_cli, "/gripper/extend", estender)

    def _gripper_grab(self, fechar):
        """Chama /gripper/grab (True = fecha a garra na flag)."""
        self._chamar_gripper(self._gripper_cli, "/gripper/grab", fechar)

    def _gripper_lift(self, erguer):
        """Chama /gripper/lift (True = ergue a flag, ombro 45°). Só após o grab."""
        self._chamar_gripper(self._gripper_lift_cli, "/gripper/lift", erguer)

    def _chamar_gripper(self, cli, nome, data):
        if not cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().error(f"[FSM] Service {nome} indisponível.")
            return
        req = SetBool.Request()
        req.data = bool(data)
        cli.call_async(req)

    def _set_footprint(self, poligono):
        """Troca o param 'footprint' dos costmaps local e global em runtime."""
        val = ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=poligono)
        param = Parameter(name="footprint", value=val)
        for cli, nome in (
            (self._fp_local_cli, "local_costmap"),
            (self._fp_global_cli, "global_costmap"),
        ):
            if not cli.wait_for_service(timeout_sec=2.0):
                self.get_logger().error(f"[FSM] set_parameters de {nome} indisponível.")
                continue
            req = SetParameters.Request()
            req.parameters = [param]
            cli.call_async(req)

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

    def _set_pole_mode(self, ativar):
        """Liga/desliga no vision_processor o foco só na metade de baixo (mastro)."""
        self._pub_pole_mode.publish(Bool(data=bool(ativar)))


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
