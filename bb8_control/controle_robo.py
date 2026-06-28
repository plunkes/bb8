#!/usr/bin/env python3
"""FSM de alto nível: orquestra exploração (explore_lite), navegação (Nav2),
captura e depósito da bandeira.

Estados / fluxo:
  INICIALIZANDO -> espera o Nav2 (navigate_to_pose) e grava a pose de origem.
  EXPLORANDO -> libera o explore_lite (explore/resume); vigia a bandeira e o
                "encravamento".
  BUSCANDO_PAREDE -> recovery: encravado em área aberta, avança e desvia p/ o lado
                mais aberto; volta a EXPLORANDO (ou à bandeira se a avistar).
  NAVEGANDO_PARA_BANDEIRA -> envia goals NavigateToPose à frente da flag (bearing
                câmera + range câmera/LIDAR); de perto assume o servo visual.
  POSICIONANDO_FINAL -> recua p/ a folga, estende o braço, centra/avança na haste,
                fecha a garra, ergue a flag e aumenta o footprint.
  RETORNANDO_ORIGEM -> Nav2 (mais rápido) p/ perto da base (standoff da origem).
  DEPOSITANDO -> visão da plataforma (label 28) p/ centrar + TF até o centro.
  SUCESSO -> abaixa o braço, solta a flag e imprime a vitória.

O Nav2 é dono do /cmd_vel na navegação; nas fases de perto (servo, captura,
recovery, depósito) a FSM publica /cmd_vel direto. O gripper é acionado por
Services (/gripper/extend, /gripper/grab, /gripper/lift) no gripper_server.
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
    DEPOSITANDO = auto()  # perto da base: visão centra na plataforma (label 28)
    SUCESSO = auto()  # chegou à base: abaixa o braço, solta a flag e celebra
    BUSCANDO_PAREDE = auto()  # robô encravado em área aberta: avança p/ achar parede


# Defaults de módulo (fallback). Os valores em uso vêm de config/fsm_params.yaml
# via _carregar_params(). Ângulos guardados em rad (yaml expõe em graus).

# Detecção / aproximação da bandeira
FLAG_DETEC_MIN_TICKS = (
    5  # ticks consecutivos de detecção antes de comutar p/ navegação (reage em ~0.5s)
)
FLAG_PERDA_MAX = 25  # ticks sem detecção, durante a navegação, antes de re-explorar
STOP_DIST = (
    0.40  # [m] parada à frente da flag (~alcance do braço 0.39: não atropela a haste)
)
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
MUITO_PERTO_DIST = 0.6  # [m] abaixo disto entra na captura só pelo range (a área da
#                         imagem falha de tão perto p/ validar proximidade)
GOAL_REFRESH_TICKS = 10  # re-mira o goal de aproximação a cada ~1 s (rastreia a flag)

# Servo visual (aproximação final): assume o controle quando a flag está perto,
# mantendo-a no centro da câmera. Evita o Nav2 girar/colapsar o goal de perto.
VS_DIST = 1.0  # [m] abaixo disto, controle visual simples em vez de goals do Nav2
VS_KP = 1.5  # ganho de giro [rad/s por rad de bearing]
VS_MAX_W = 1.2  # [rad/s] giro máximo no servo
VS_VEL = 0.30  # [m/s] avanço quando a flag está centrada
VS_ALIGN = math.radians(12.0)  # |bearing| p/ considerar centrada e poder avançar

# Sequência de captura em POSICIONANDO_FINAL (ticks @ FREQ_CONTROLE=10Hz):
GRIPPER_EXTEND_TICKS = (
    10  # ~1.0 s p/ o braço estender (juntas a vel 3.0 chegam a tempo)
)
GRIPPER_CLOSE_TICKS = 10  # ~1.0 s p/ a garra fechar na flag antes de erguer
GRIPPER_LIFT_TICKS = 10  # ~1.0 s p/ o ombro erguer a flag antes de retornar
GRAB_DIST = 0.40  # [m] fecha a garra (na_dist): > platô da haste (~0.36), para antes de empurrar
CREEP_VEL = (
    0.12  # [m/s] avanço final até a haste (na_dist para antes de empurrar)
)
CREEP_MAX_TICKS = 20  # ticks ~3 s máx de avanço final (segurança)
GRAB_DIST_TOL = (
    0.05  # [m] banda em torno de GRAB_DIST p/ "na distância" (dá ré se passar)
)
GRAB_STALL_EPS = (
    0.01  # [m] progresso mín. de rng p/ não contar como "encostou" na haste
)
GRAB_STALL_TICKS = (
    6  # ticks alinhado sem aproximar (rng parou) = haste na palma -> fecha
)
VEL_RETORNO = 1.5  # [m/s] velocidade na volta à base (caminho já conhecido)
RETORNO_OBSTACLE_SCALE = 1.0  # peso de obstáculo (DWB) na volta: mais cuidado c/ a flag
RE_DIST = 0.55  # [m] folga p/ estender/abrir o braço sem bater no pole (> alcance ~0.4)

# Depósito da flag na plataforma de início (label 28). Volta pela origem gravada,
# mas para a DEPOSIT_STANDOFF dela e usa a VISÃO p/ centrar e subir na plataforma.
DEPOSIT_STANDOFF = 1.0  # [m] para a esta distância da origem (plataforma à frente)
DEPOSIT_VEL = 0.15  # [m/s] avanço lento centrando na plataforma
DEPOSIT_MAX_TICKS = 80  # ticks ~8 s máx procurando/subindo na plataforma (fallback)
DEPOSIT_PERDA_TICKS = 8  # ticks sem ver a plataforma (após vê-la) = está por cima dela
DEPOSIT_DROP_DIST = (
    0.4  # [m] distância ao centro (origem) p/ baixar o braço e soltar a flag
)

# Footprint do Nav2 ao segurar a flag (trocado em runtime ao pegar): MAIOR p/ cobrir
# braço + bandeira à frente e dar margem lateral -> Nav2 desvia mais na volta.
# Inscrito = min(0.20, 0.25) = 0.20; +footprint_padding 0.04 = 0.24 < inflation 0.25.
FOOTPRINT_COM_BRACO = "[[0.65, 0.20], [0.65, -0.20], [-0.25, -0.20], [-0.25, 0.20]]"

# "Encravado" em área aberta: explore manda goals pro lugar onde o robô já está
# (LIDAR sem retorno -> sem fronteira). Detecta pouca movimentação e avança a esmo.
STUCK_MIN_MOVE = 0.15  # [m] deslocamento mínimo p/ considerar que está se movendo
STUCK_MAX_TICKS = (
    30  # ticks ~3 s quase parado em EXPLORANDO antes de avançar (entra cedo)
)
AVANCO_MAX_TICKS = (
    60  # ticks ~6 s indo p/ frente antes de voltar a explorar (busca mais longa)
)
AVANCO_VEL = 0.4  # [m/s] velocidade ao avançar à procura de parede
AVANCO_FRONT_SECTOR = math.radians(20.0)  # meia-largura do setor frontal vigiado
AVANCO_SAFE_DIST = 0.5  # [m] obstáculo mais perto que isto à frente -> não avança
AVANCO_GIRO = 0.6  # [rad/s] giro à esquerda quando há obstáculo à frente

FREQ_CONTROLE = 10  # [Hz] taxa do laço principal da FSM


class ControleRobo(Node):
    def __init__(self):
        super().__init__("controle_robo")

        # Carrega as constantes de tuning de parâmetros ROS (config/fsm_params.yaml),
        # sobrescrevendo os defaults de módulo. Antes de qualquer uso (ex.: timer).
        self._carregar_params()

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
        # Velocidade direta (/cmd_vel) nas fases de perto: servo, captura, recovery, depósito.
        self._pub_cmd = self.create_publisher(Twist, "/cmd_vel", 10)
        # Modo "pole": pede ao vision_processor p/ olhar só a metade de baixo da
        # câmera (mastro), ignorando o painel da bandeira no topo. Ativo em POSICIONANDO.
        self._pub_pole_mode = self.create_publisher(Bool, "/vision/pole_mode", 10)
        # Liga a detecção da plataforma (label 28) no vision_processor no depósito.
        self._pub_deposit = self.create_publisher(Bool, "/vision/deposit_mode", 10)

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
        # Client p/ acelerar o Nav2 (DWB) na volta à base.
        self._ctrl_param_cli = self.create_client(
            SetParameters, "/controller_server/set_parameters"
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
        self._posic_ticks = 0  # ticks na sequência de captura (POSICIONANDO_FINAL)
        self._posic_fase = 0  # 0 = folga+estende braço, 1 = posiciona+fecha garra
        self._braco_estendido = False  # braço já estendido nesta captura
        self._grab_rng_min = None  # menor rng à flag visto na aproximação final
        self._grab_stall = 0  # ticks alinhado sem aproximar mais (encostou na haste)
        self._plat_visto = False  # plataforma já avistada no depósito
        self._plat_perda = 0  # ticks sem ver a plataforma após avistá-la
        self._missao_completa = False
        self._explore_on = None  # último valor publicado em explore/resume (None = nunca)
        self._pose_origem = None  # PoseStamped em 'map', gravada ao iniciar

        # Acompanhamento do goal do Nav2
        self._goal_handle = None
        self._nav_status = None  # None | 'pendente' | 'sucesso' | 'falha'
        self._nav_retries = 0

        self.create_timer(1.0 / FREQ_CONTROLE, self._loop)

        self.get_logger().info("[FSM] Estado inicial: INICIALIZANDO (aguardando Nav2)")

    # ------------------------------------------------------------------ #
    # Parâmetros (tuning via config/fsm_params.yaml)
    # ------------------------------------------------------------------ #
    def _carregar_params(self):
        """Lê as constantes de tuning de parâmetros ROS e sobrescreve os defaults
        de módulo. Ângulos expostos em GRAUS (sufixo _deg) e convertidos p/ rad."""
        global FLAG_DETEC_MIN_TICKS, FLAG_PERDA_MAX, STOP_DIST, SETOR_BANDEIRA
        global RANGE_FALLBACK, NAV_RETRY_MAX, OBSTACULO_TOL
        global FLAG_AREA_MIN_PX, FLAG_ALIGN_MAX, FLAG_RANGE_MAX, MUITO_PERTO_DIST
        global GOAL_REFRESH_TICKS, VS_DIST, VS_KP, VS_MAX_W, VS_VEL, VS_ALIGN
        global GRIPPER_EXTEND_TICKS, GRIPPER_CLOSE_TICKS, GRIPPER_LIFT_TICKS
        global GRAB_DIST, CREEP_VEL, CREEP_MAX_TICKS
        global STUCK_MIN_MOVE, STUCK_MAX_TICKS, AVANCO_MAX_TICKS, AVANCO_VEL
        global AVANCO_FRONT_SECTOR, AVANCO_SAFE_DIST, AVANCO_GIRO, FREQ_CONTROLE
        global GRAB_DIST_TOL, VEL_RETORNO, RE_DIST, GRAB_STALL_EPS, GRAB_STALL_TICKS
        global RETORNO_OBSTACLE_SCALE
        global DEPOSIT_STANDOFF, DEPOSIT_VEL, DEPOSIT_MAX_TICKS, DEPOSIT_PERDA_TICKS
        global DEPOSIT_DROP_DIST

        def num(name, default):
            self.declare_parameter(name, default)
            return self.get_parameter(name).value

        def ang(name, default_rad):
            self.declare_parameter(name, math.degrees(default_rad))
            return math.radians(self.get_parameter(name).value)

        FLAG_DETEC_MIN_TICKS = num("flag_detec_min_ticks", FLAG_DETEC_MIN_TICKS)
        FLAG_PERDA_MAX = num("flag_perda_max", FLAG_PERDA_MAX)
        STOP_DIST = num("stop_dist", STOP_DIST)
        SETOR_BANDEIRA = ang("setor_bandeira_deg", SETOR_BANDEIRA)
        RANGE_FALLBACK = num("range_fallback", RANGE_FALLBACK)
        NAV_RETRY_MAX = num("nav_retry_max", NAV_RETRY_MAX)
        OBSTACULO_TOL = num("obstaculo_tol", OBSTACULO_TOL)
        FLAG_AREA_MIN_PX = num("flag_area_min_px", FLAG_AREA_MIN_PX)
        FLAG_ALIGN_MAX = ang("flag_align_max_deg", FLAG_ALIGN_MAX)
        FLAG_RANGE_MAX = num("flag_range_max", FLAG_RANGE_MAX)
        MUITO_PERTO_DIST = num("muito_perto_dist", MUITO_PERTO_DIST)
        GOAL_REFRESH_TICKS = num("goal_refresh_ticks", GOAL_REFRESH_TICKS)
        VS_DIST = num("vs_dist", VS_DIST)
        VS_KP = num("vs_kp", VS_KP)
        VS_MAX_W = num("vs_max_w", VS_MAX_W)
        VS_VEL = num("vs_vel", VS_VEL)
        VS_ALIGN = ang("vs_align_deg", VS_ALIGN)
        GRIPPER_EXTEND_TICKS = num("gripper_extend_ticks", GRIPPER_EXTEND_TICKS)
        GRIPPER_CLOSE_TICKS = num("gripper_close_ticks", GRIPPER_CLOSE_TICKS)
        GRIPPER_LIFT_TICKS = num("gripper_lift_ticks", GRIPPER_LIFT_TICKS)
        GRAB_DIST = num("grab_dist", GRAB_DIST)
        CREEP_VEL = num("creep_vel", CREEP_VEL)
        CREEP_MAX_TICKS = num("creep_max_ticks", CREEP_MAX_TICKS)
        GRAB_DIST_TOL = num("grab_dist_tol", GRAB_DIST_TOL)
        VEL_RETORNO = num("vel_retorno", VEL_RETORNO)
        RETORNO_OBSTACLE_SCALE = num("retorno_obstacle_scale", RETORNO_OBSTACLE_SCALE)
        RE_DIST = num("re_dist", RE_DIST)
        GRAB_STALL_EPS = num("grab_stall_eps", GRAB_STALL_EPS)
        GRAB_STALL_TICKS = num("grab_stall_ticks", GRAB_STALL_TICKS)
        DEPOSIT_STANDOFF = num("deposit_standoff", DEPOSIT_STANDOFF)
        DEPOSIT_VEL = num("deposit_vel", DEPOSIT_VEL)
        DEPOSIT_MAX_TICKS = num("deposit_max_ticks", DEPOSIT_MAX_TICKS)
        DEPOSIT_PERDA_TICKS = num("deposit_perda_ticks", DEPOSIT_PERDA_TICKS)
        DEPOSIT_DROP_DIST = num("deposit_drop_dist", DEPOSIT_DROP_DIST)
        STUCK_MIN_MOVE = num("stuck_min_move", STUCK_MIN_MOVE)
        STUCK_MAX_TICKS = num("stuck_max_ticks", STUCK_MAX_TICKS)
        AVANCO_MAX_TICKS = num("avanco_max_ticks", AVANCO_MAX_TICKS)
        AVANCO_VEL = num("avanco_vel", AVANCO_VEL)
        AVANCO_FRONT_SECTOR = ang("avanco_front_sector_deg", AVANCO_FRONT_SECTOR)
        AVANCO_SAFE_DIST = num("avanco_safe_dist", AVANCO_SAFE_DIST)
        AVANCO_GIRO = num("avanco_giro", AVANCO_GIRO)
        FREQ_CONTROLE = num("freq_controle", FREQ_CONTROLE)

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
        elif e == Estado.DEPOSITANDO:
            self._exec_depositando()
        elif e == Estado.SUCESSO:
            self._exec_sucesso()
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
            self._avanco_budget = AVANCO_MAX_TICKS  # recovery de ~4 s
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
            self._braco_estendido = False
            self._grab_rng_min = None
            self._grab_stall = 0
        elif estado == Estado.RETORNANDO_ORIGEM:
            self._set_explore(False)
            self._nav_retries = 0
            self._nav_status = None
            self._goal_handle = None
            self._set_vel_nav(VEL_RETORNO)  # caminho já conhecido: volta mais rápido
            self._enviar_goal_origem()  # volta p/ perto da base (standoff)
        elif estado == Estado.DEPOSITANDO:
            self._set_explore(False)
            self._set_deposit_mode(True)  # visão detecta a plataforma (label 28)
            self._posic_ticks = 0
            self._plat_visto = False
            self._plat_perda = 0
        elif estado == Estado.SUCESSO:
            self._set_explore(False)
            self._set_deposit_mode(False)  # desliga a detecção da plataforma
            self._posic_ticks = 0
            self._posic_fase = 0
            self._gripper_lift(False)  # abaixa o ombro (45° -> 0); garra ainda fechada

    # ------------------------------------------------------------------ #
    # Estados
    # ------------------------------------------------------------------ #
    def _exec_inicializando(self):
        # Espera o Nav2 expor o action server e grava a origem antes de explorar.
        if not self._nav_client.server_is_ready():
            return
        if self._pose_origem is None:
            self._pose_origem = self._pose_atual_em_map()
            if self._pose_origem is None:
                return
        # LIDAR é 360°: o mapa já nasce povoado ao redor -> explora/planeja direto.
        self._set_estado(Estado.EXPLORANDO)

    def _exec_explorando(self):
        # O explore_lite cuida da navegação; vigiamos a bandeira...
        if self._flag_detec_ticks >= FLAG_DETEC_MIN_TICKS:
            self._set_estado(Estado.NAVEGANDO_PARA_BANDEIRA)
            return
        # ...e se o robô encravou (parado recebendo goals pro próprio lugar).
        if self._detectar_encravado():
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
        # Desvio: olha o setor frontal. Obstáculo perto -> não avança; gira para o
        # LADO MAIS ABERTO (esq vs dir) p/ escapar de caixa, em vez de girar sempre
        # à esquerda (ficava preso entre dois obstáculos).
        front = self._range_no_setor(0.0, AVANCO_FRONT_SECTOR)
        cmd = Twist()
        if front is not None and front < AVANCO_SAFE_DIST:
            esq = self._range_no_setor(math.pi / 2, AVANCO_FRONT_SECTOR)
            dire = self._range_no_setor(-math.pi / 2, AVANCO_FRONT_SECTOR)
            esq = esq if esq is not None else float("inf")
            dire = dire if dire is not None else float("inf")
            cmd.linear.x = 0.0
            cmd.angular.z = AVANCO_GIRO if esq >= dire else -AVANCO_GIRO
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
            self._cancelar_goal()
            self._set_estado(Estado.POSICIONANDO_FINAL)
            return

        # 2) Perdeu a bandeira de vista por tempo demais -> volta a explorar.
        if self._flag_perda_ticks > FLAG_PERDA_MAX:
            self._cancelar_goal()
            self._set_estado(Estado.EXPLORANDO)
            return

        # 2b) Perto (<VS_DIST): assume o CONTROLE VISUAL SIMPLES e o mantém travado
        #     (latch). O Nav2 manda paths confusos de perto; uma vez que o servo
        #     assume, não devolvemos o controle ao Nav2 (só sai daqui se pegar a
        #     flag ou perdê-la de vista por tempo demais, tratado acima).
        rng = self._estimar_range_flag()
        if self._servo_ativo or (rng is not None and rng <= VS_DIST):
            self._servo_ativo = True  # latch: servo visual assume, Nav2 desligado
            self._cancelar_goal()
            self._servo_visual()
            return

        # 3) Goal de aproximação falhou -> retry; se esgotar, re-explora.
        if self._nav_status == "falha":
            self._nav_status = None
            if self._nav_retries < NAV_RETRY_MAX and self._bandeira_detectada:
                self._nav_retries += 1
                self._enviar_goal_bandeira()
            else:
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
        muito_perto = rng <= MUITO_PERTO_DIST
        return perto or muito_perto

    def _servo_visual(self):
        """Aproximação final (Nav2 sem goal): centra a flag e mantém a DISTÂNCIA de
        garra (GRAB_DIST), dando RÉ se passou perto demais (corrige overshoot)."""
        if not self._bandeira_detectada:
            # Sumiu neste frame: para (não gira perdido). Perda longa -> re-explora
            # é tratada no início de _exec_navegando.
            self._parar()
            return
        rng = self._estimar_range_flag()  # distância à flag (não cone frontal largo)
        self._pub_cmd.publish(self._ajuste_fino(RE_DIST, rng, VS_VEL))

    def _ajuste_fino(self, alvo, front, vel):
        """Twist p/ manter a flag à distância 'alvo' (m) e CENTRADA (bearing~0).
        Avança a 'vel' se longe, dá RÉ (-vel) se perto demais, para na banda
        [alvo ± GRAB_DIST_TOL]. Corrige overshoot de distância e orientação."""
        cmd = Twist()
        bearing = self._flag_bearing
        cmd.angular.z = max(-VS_MAX_W, min(VS_MAX_W, VS_KP * bearing))
        if front is None:
            cmd.linear.x = 0.0
        elif front > alvo + GRAB_DIST_TOL:
            # Longe: só avança quando já razoavelmente centrado (senão gira no lugar).
            cmd.linear.x = vel if abs(bearing) < VS_ALIGN else 0.0
        elif front < alvo - GRAB_DIST_TOL:
            cmd.linear.x = -vel  # perto demais: RÉ (continua centrando)
        else:
            cmd.linear.x = 0.0  # na banda correta
        return cmd

    def _exec_posicionando(self):
        # Captura em 4 fases:
        #   0 = recua até a folga (RE_DIST) e ESTENDE o braço (garra aberta);
        #   1 = avança/centra na haste e FECHA a garra (na_dist OU encostou);
        #   2 = espera a garra fechar e ERGUE a flag (ombro 45°);
        #   3 = espera erguer, aumenta o footprint e vai p/ RETORNANDO_ORIGEM.
        self._posic_ticks += 1

        if self._posic_fase == 0:  # recua até a FOLGA (RE_DIST) e estende o braço
            rng = (
                self._estimar_range_flag()
            )  # distância à flag (não cone frontal largo)
            # Perto demais p/ o braço estender/abrir sem bater no pole -> dá RÉ
            # até a folga (RE_DIST > alcance do braço). Só então estende.
            if (
                not self._braco_estendido
                and rng is not None
                and rng < RE_DIST - GRAB_DIST_TOL
            ):
                self._pub_cmd.publish(self._ajuste_fino(RE_DIST, rng, CREEP_VEL))
                self._posic_ticks = 0
                return
            if not self._braco_estendido:
                self._parar()
                self._gripper_extend(True)  # estende o braço (garra aberta) COM folga
                self._braco_estendido = True
                self._posic_ticks = 0
                return
            self._posic_ticks += 1
            if self._posic_ticks >= GRIPPER_EXTEND_TICKS:
                self._posic_fase = 1
                self._posic_ticks = 0
            return

        if self._posic_fase == 1:  # ajusta p/ a BANDA de garra (avança/ré) e fecha
            bearing = self._flag_bearing
            alinhado = self._bandeira_detectada and abs(bearing) <= FLAG_ALIGN_MAX
            # Distância à FLAG pelo estimador (LIDAR no bearing da flag, setor estreito,
            # + câmera fundidos). NÃO usar o cone frontal largo: ele pega o objeto mais
            # próximo (braço, borda da plataforma, parede), não a haste.
            rng = self._estimar_range_flag()
            # Avança RETO (sem ré) e fecha quando rng <= grab_dist OU "encostou":
            # alinhado e avançando, mas rng PAROU de diminuir (haste chegou à palma /
            # braço barra) -> está posicionado, fecha mesmo que rng > grab_dist (o
            # LIDAR mede centro->haste ~0.36 e nem sempre chega a grab_dist).
            na_dist = rng is not None and rng <= GRAB_DIST
            if alinhado and rng is not None:
                if (
                    self._grab_rng_min is None
                    or rng < self._grab_rng_min - GRAB_STALL_EPS
                ):
                    self._grab_rng_min = rng  # aproximou: reseta o stall
                    self._grab_stall = 0
                    self._posic_ticks = 0  # progredindo: não conta p/ o timeout
                else:
                    self._grab_stall += 1  # não aproximou mais
            else:
                self._grab_stall = 0
            encostou = alinhado and self._grab_stall >= GRAB_STALL_TICKS
            if (na_dist or encostou) and alinhado:
                self._parar()
                self._gripper_grab(True)  # fecha a garra na bandeira
                self._posic_fase = 2
                self._posic_ticks = 0
            elif self._posic_ticks > CREEP_MAX_TICKS:
                # Não conseguiu posicionar a tempo: NÃO pega torto — re-aproxima.
                self._parar()
                self._set_estado(Estado.NAVEGANDO_PARA_BANDEIRA)
            else:
                # Avança RETO (sem ré) centrando o pole, até rng <= grab_dist.
                cmd = Twist()
                if self._bandeira_detectada:
                    cmd.angular.z = max(-VS_MAX_W, min(VS_MAX_W, VS_KP * bearing))
                cmd.linear.x = CREEP_VEL if alinhado else 0.0
                self._pub_cmd.publish(cmd)
            return

        if self._posic_fase == 2:  # garra fechando
            if self._posic_ticks >= GRIPPER_CLOSE_TICKS:
                # Garra fechou em volta da flag: agora SIM ergue a flag (ombro 45°).
                self._gripper_lift(True)
                self._posic_fase = 3
                self._posic_ticks = 0
            return

        if self._posic_fase == 3:  # erguendo a flag (ombro 45°)
            if self._posic_ticks >= GRIPPER_LIFT_TICKS:
                # Flag erguida e fora do FOV frontal (scan_masker mascara): aumenta
                # o footprint p/ manobrar e retorna à origem.
                self._set_footprint(FOOTPRINT_COM_BRACO)
                self._set_estado(Estado.RETORNANDO_ORIGEM)

    def _exec_retornando(self):
        if self._nav_status == "sucesso":
            self._set_estado(Estado.DEPOSITANDO)
        elif self._nav_status == "falha":
            self._nav_status = None
            if self._nav_retries < NAV_RETRY_MAX:
                self._nav_retries += 1
                self._enviar_goal_origem()
            else:
                self._missao_completa = True

    def _exec_sucesso(self):
        # Chegou à base com a flag erguida: abaixa o braço, solta a flag e celebra.
        self._posic_ticks += 1
        if self._posic_fase == 0:  # abaixando o ombro (45° -> 0)
            if self._posic_ticks >= GRIPPER_LIFT_TICKS:
                self._gripper_grab(False)  # abre a garra -> solta a bandeira
                self._posic_fase = 1
                self._posic_ticks = 0
            return
        if self._posic_fase == 1:  # garra abrindo (soltando a flag)
            if self._posic_ticks >= GRIPPER_CLOSE_TICKS:
                self._concluir_missao()  # mensagem de vitória + missão completa
            return

    def _exec_depositando(self):
        # Perto da base: CENTRA na plataforma (visão, label 28, metade de baixo) e
        # avança até ficar a ~DEPOSIT_DROP_DIST do CENTRO (origem gravada, via TF).
        # Aí vai p/ SUCESSO, que baixa o braço e solta a flag no centro. A distância
        # vem da TF (a plataforma é plana -> LIDAR não a mede); a visão só alinha.
        self._posic_ticks += 1

        pos = self._pose_atual_em_map()
        if pos is not None and self._pose_origem is not None:
            dx = pos.pose.position.x - self._pose_origem.pose.position.x
            dy = pos.pose.position.y - self._pose_origem.pose.position.y
            if math.hypot(dx, dy) <= DEPOSIT_DROP_DIST:
                self._parar()
                self._set_estado(Estado.SUCESSO)  # no centro: baixa o braço e solta
                return

        if self._posic_ticks > DEPOSIT_MAX_TICKS:
            self._parar()
            self._set_estado(Estado.SUCESSO)  # fallback por tempo
            return

        # Centra na plataforma (visão) e avança devagar; sem detecção, segue reto
        # (já está virado p/ a origem) usando a TF p/ saber quando parar (acima).
        cmd = Twist()
        if self._bandeira_detectada:
            bearing = self._flag_bearing
            cmd.angular.z = max(-VS_MAX_W, min(VS_MAX_W, VS_KP * bearing))
            cmd.linear.x = DEPOSIT_VEL if abs(bearing) < VS_ALIGN else 0.0
        else:
            cmd.linear.x = DEPOSIT_VEL
        self._pub_cmd.publish(cmd)

    # ------------------------------------------------------------------ #
    # Cálculo e envio do goal da bandeira
    # ------------------------------------------------------------------ #
    def _enviar_goal_bandeira(self):
        # Tenta o goal cheio e, se cair fora do mapa/em célula desconhecida (a flag
        # costuma estar além do que o LIDAR já mapeou), encurta o trajeto pela metade
        # sucessivamente até achar um goal alcançável dentro do global costmap.
        pose = None
        for fracao in GOAL_FRACOES:
            cand = self._calcular_pose_bandeira(fracao)
            if cand is None:
                # Sem TF/scan ainda: tenta de novo no próximo tick (não comuta estado).
                self._nav_status = "falha"
                return
            if self._goal_alcancavel(cand):
                pose = cand
                break

        if pose is None:
            # Nenhuma fração caiu dentro do mapa: aguarda mais mapa.
            self._nav_status = "falha"
            return

        if not self._nav_client.server_is_ready():
            self._nav_status = "falha"
            return

        goal = NavigateToPose.Goal()
        goal.pose = pose
        self._nav_status = "pendente"
        send_future = self._nav_client.send_goal_async(goal)
        send_future.add_done_callback(self._on_goal_response)

    def _enviar_goal_origem(self):
        """Volta p/ PERTO da base: para a DEPOSIT_STANDOFF da origem, do lado de onde o
        robô vem e VIRADO p/ a origem — a plataforma fica à frente p/ a visão centrar e
        o robô subir nela (depósito da flag). O depósito fino fica no DEPOSITANDO."""
        if self._pose_origem is None or not self._nav_client.server_is_ready():
            self._nav_status = "falha"
            return
        ox = self._pose_origem.pose.position.x
        oy = self._pose_origem.pose.position.y
        gx, gy, yaw = ox, oy, 0.0
        atual = self._pose_atual_em_map()
        if atual is not None:
            dx = atual.pose.position.x - ox
            dy = atual.pose.position.y - oy
            d = math.hypot(dx, dy)
            if d > 1e-3:
                gx = ox + DEPOSIT_STANDOFF * dx / d
                gy = oy + DEPOSIT_STANDOFF * dy / d
                yaw = math.atan2(oy - gy, ox - gx)  # virado p/ a origem (plataforma)
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = gx
        goal.pose.pose.position.y = gy
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)
        self._nav_status = "pendente"
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
        except Exception:  # TransformException e afins
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
            self._goal_handle = None
            self._nav_status = "falha"
            return
        self._goal_handle = handle
        result_future = handle.get_result_async()
        result_future.add_done_callback(self._on_goal_result)

    def _on_goal_result(self, future):
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self._nav_status = "sucesso"
        elif status == GoalStatus.STATUS_CANCELED:
            pass  # cancelamento deliberado nosso — não marca como falha
        else:
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
                continue
            req = SetParameters.Request()
            req.parameters = [param]
            cli.call_async(req)

    def _set_vel_nav(self, vx):
        """Reconfigura o controlador Nav2 (DWB) em runtime p/ a volta à base:
        velocidade linear máx = vx (caminho conhecido) e peso de obstáculo maior
        (RETORNO_OBSTACLE_SCALE) p/ desviar mais carregando a flag."""
        cli = self._ctrl_param_cli
        if not cli.wait_for_service(timeout_sec=2.0):
            return

        def dv(v):
            return ParameterValue(
                type=ParameterType.PARAMETER_DOUBLE, double_value=float(v)
            )

        req = SetParameters.Request()
        req.parameters = [
            Parameter(name="FollowPath.max_vel_x", value=dv(vx)),
            Parameter(name="FollowPath.max_speed_xy", value=dv(vx)),
            Parameter(name="FollowPath.BaseObstacle.scale", value=dv(RETORNO_OBSTACLE_SCALE)),
        ]
        cli.call_async(req)

    def _concluir_missao(self):
        self._cancelar_goal()
        self._missao_completa = True
        self.get_logger().info(
            "\n"
            "╔══════════════════════════════════════════╗\n"
            "║          🏁  VITÓRIA!  🏁                 ║\n"
            "║  Flag capturada, entregue na base e      ║\n"
            "║  braço abaixado. Missão concluída!       ║\n"
            "╚══════════════════════════════════════════╝"
        )

    # ------------------------------------------------------------------ #
    # Utilitários
    # ------------------------------------------------------------------ #
    def _set_explore(self, ativar):
        # Publica só na MUDANÇA de valor: re-enviar resume=False faz o explore_lite
        # cancelar os goals do navigate_to_pose — cancelava o goal de retorno à base
        # recém-enviado (robô não voltava). Idempotente agora.
        ativar = bool(ativar)
        if ativar == self._explore_on:
            return
        self._explore_on = ativar
        self._pub_explore.publish(Bool(data=ativar))

    def _set_pole_mode(self, ativar):
        """Liga/desliga no vision_processor o foco só na metade de baixo (mastro)."""
        self._pub_pole_mode.publish(Bool(data=bool(ativar)))

    def _set_deposit_mode(self, ativar):
        """Liga/desliga no vision_processor a detecção da plataforma (label 28)."""
        self._pub_deposit.publish(Bool(data=bool(ativar)))


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
