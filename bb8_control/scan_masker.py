#!/usr/bin/env python3
"""Filtro dinâmico do LaserScan condicionado à pose do braço.

Problema: quando a Junta 1 (shoulder_pitch) levanta o braço para 45 graus, o
braço entra no campo de visão do LIDAR e gera falsas leituras de obstáculo bem
à frente do robô.

Solução: este nó monitora /joint_states. Sempre que shoulder_pitch estiver na
posição de gatilho (~45 graus, com tolerância configurável), as leituras do
LaserScan num setor frontal (por padrão -15° a +15°, ou seja 30°) são mascaradas
(range = inf). Quando o braço volta a 0°, o scan volta a passar intacto.

Tópicos:
  sub  /joint_states  (sensor_msgs/JointState)  -> estado da junta do ombro
  sub  /scan          (sensor_msgs/LaserScan)   -> scan bruto do LIDAR
  pub  /scan_filtered (sensor_msgs/LaserScan)   -> scan mascarado (SLAM/Nav2 assinam este)

Parâmetros:
  shoulder_joint_name (str)   nome da junta monitorada      (default: shoulder_pitch)
  trigger_position    (float) posição-gatilho em rad        (default: 0.785398 = 45°)
  position_tolerance  (float) tolerância em rad p/ o gatilho (default: 0.10)
  mask_min_deg        (float) limite inferior do setor (deg) (default: -15.0)
  mask_max_deg        (float) limite superior do setor (deg) (default:  15.0)
  scan_topic          (str)   tópico do scan bruto           (default: /scan)
  filtered_topic      (str)   tópico do scan filtrado        (default: /scan_filtered)
"""
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, LaserScan


def _normalize(angle):
    """Normaliza um ângulo para o intervalo [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


class ScanMasker(Node):
    def __init__(self):
        super().__init__("scan_masker")

        self.declare_parameter("shoulder_joint_name", "shoulder_pitch")
        self.declare_parameter("trigger_position", 0.785398)  # 45 graus
        self.declare_parameter("position_tolerance", 0.10)
        self.declare_parameter("mask_min_deg", -15.0)
        self.declare_parameter("mask_max_deg", 15.0)
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("filtered_topic", "/scan_filtered")

        self._joint_name = self.get_parameter("shoulder_joint_name").value
        self._trigger = float(self.get_parameter("trigger_position").value)
        self._tol = float(self.get_parameter("position_tolerance").value)
        self._mask_min = math.radians(float(self.get_parameter("mask_min_deg").value))
        self._mask_max = math.radians(float(self.get_parameter("mask_max_deg").value))
        scan_topic = self.get_parameter("scan_topic").value
        filtered_topic = self.get_parameter("filtered_topic").value

        # Índice da junta em /joint_states (cacheado após primeira leitura).
        self._joint_idx = None
        # Flag de mascaramento ativada/desativada pelo callback de joint_states.
        self._mascarar = False

        self._pub = self.create_publisher(LaserScan, filtered_topic, 10)
        self.create_subscription(JointState, "/joint_states", self._cb_joints, 10)
        self.create_subscription(LaserScan, scan_topic, self._cb_scan, 10)

        self.get_logger().info(
            f"[scan_masker] {scan_topic} -> {filtered_topic} | "
            f"junta='{self._joint_name}' gatilho={self._trigger:.3f}rad "
            f"tol={self._tol:.3f} setor=[{math.degrees(self._mask_min):.0f}°,"
            f"{math.degrees(self._mask_max):.0f}°]"
        )

    def _cb_joints(self, msg: JointState):
        # Resolve (e cacheia) o índice da junta do ombro pelo nome.
        if self._joint_idx is None or self._joint_idx >= len(msg.name) \
                or msg.name[self._joint_idx] != self._joint_name:
            try:
                self._joint_idx = msg.name.index(self._joint_name)
            except ValueError:
                return  # junta ainda não publicada neste ciclo

        pos = msg.position[self._joint_idx]
        ativo = abs(pos - self._trigger) <= self._tol
        if ativo != self._mascarar:
            self._mascarar = ativo
            estado = "MASCARANDO frontal" if ativo else "scan LIVRE"
            self.get_logger().info(
                f"[scan_masker] ombro={pos:.3f}rad -> {estado}"
            )

    def _cb_scan(self, msg: LaserScan):
        if not self._mascarar:
            self._pub.publish(msg)
            return

        # Copia mutável dos ranges e zera (inf) o setor frontal.
        ranges = list(msg.ranges)
        n = len(ranges)
        for i in range(n):
            ang = _normalize(msg.angle_min + i * msg.angle_increment)
            if self._mask_min <= ang <= self._mask_max:
                ranges[i] = float("inf")

        out = LaserScan()
        out.header = msg.header
        out.angle_min = msg.angle_min
        out.angle_max = msg.angle_max
        out.angle_increment = msg.angle_increment
        out.time_increment = msg.time_increment
        out.scan_time = msg.scan_time
        out.range_min = msg.range_min
        out.range_max = msg.range_max
        out.ranges = ranges
        out.intensities = msg.intensities
        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ScanMasker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
