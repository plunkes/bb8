#!/usr/bin/env python3
"""Odometria ground-truth a partir da pose do Gazebo.

Absorvido para dentro do bb8_control (antes vivia em prm_2026/ground_truth_odometry).
Assina a pose ground-truth publicada pela bridge (/model/prm_robot/pose) e
republica como nav_msgs/Odometry em /odom_gt, além de publicar o TF
odom -> base_link.

Na stack SLAM/Nav2 a pose ground-truth faz o papel da camada de odometria:
o frame pai do TF é 'odom' (não 'odom_gt'), de modo que o slam_toolbox publica
'map -> odom' por cima, completando a árvore map -> odom -> base_link.
Os frames são parametrizáveis (base_frame, odom_frame).
"""

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster


class GroundTruthOdomPublisher(Node):
    def __init__(self):
        super().__init__("ground_truth_odom_publisher")

        # Assinatura no tópico de pose vinda do simulador
        self.create_subscription(Pose, "/model/prm_robot/pose", self.pose_callback, 10)

        # Publicador de odometria ground truth
        self.odom_pub = self.create_publisher(Odometry, "/odom_gt", 10)

        # Broadcaster de TF: odom -> base_link
        self.tf_broadcaster = TransformBroadcaster(self)

        # Frames (parametrizáveis).
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("odom_frame", "odom")
        self.base_frame = self.get_parameter("base_frame").value
        self.odom_frame = self.get_parameter("odom_frame").value

        self._n = 0  # contador de poses recebidas (p/ diagnóstico)
        # Aviso se nenhuma pose chegar (bridge /model/prm_robot/pose ausente ->
        # SEM TF odom->base_link -> Nav2 não move o robô).
        self._warn_timer = self.create_timer(3.0, self._warn_sem_pose)
        self.get_logger().info(
            f"[odom_gt] assinando /model/prm_robot/pose; publicando /odom_gt e "
            f"TF {self.odom_frame}->{self.base_frame}"
        )

    def _warn_sem_pose(self):
        if self._n == 0:
            self.get_logger().warn(
                "[odom_gt] NENHUMA pose recebida em /model/prm_robot/pose ainda — "
                "robô não terá TF odom->base_link (verifique a bridge/spawn 'prm_robot').",
            )

    def pose_callback(self, msg: Pose):
        now = self.get_clock().now().to_msg()
        self._n += 1
        if self._n == 1:
            self.get_logger().info("[odom_gt] 1ª pose recebida — publicando TF/odom.")
        elif self._n % 100 == 0:
            self.get_logger().info(
                f"[odom_gt] {self._n} poses | pos=({msg.position.x:.2f}, "
                f"{msg.position.y:.2f})",
            )

        # Publica a odometria no tópico /odom_gt
        odom_msg = Odometry()
        odom_msg.header.stamp = now
        odom_msg.header.frame_id = self.odom_frame
        odom_msg.child_frame_id = self.base_frame
        odom_msg.pose.pose = msg
        self.odom_pub.publish(odom_msg)

        # Publica também o TF correspondente
        tf_msg = TransformStamped()
        tf_msg.header.stamp = now
        tf_msg.header.frame_id = self.odom_frame
        tf_msg.child_frame_id = self.base_frame
        tf_msg.transform.translation.x = msg.position.x
        tf_msg.transform.translation.y = msg.position.y
        tf_msg.transform.translation.z = msg.position.z
        tf_msg.transform.rotation = msg.orientation
        self.tf_broadcaster.sendTransform(tf_msg)


def main(args=None):
    rclpy.init(args=args)
    node = GroundTruthOdomPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
