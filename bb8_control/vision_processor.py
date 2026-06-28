#!/usr/bin/env python3
import math

import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose2D
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32, String


class VisionProcessorNode(Node):
    def __init__(self):
        super().__init__("vision_processor")

        self.declare_parameter("flag_label_ids", [25])
        self.declare_parameter("camera_hfov_deg", 90.0)
        self.declare_parameter("min_flag_pixels", 6)
        # Altura real do blob rotulado (mastro+painel) p/ estimar distância (pinhole).
        self.declare_parameter("flag_real_height_m", 0.5)

        self._flag_labels = set(self.get_parameter("flag_label_ids").value)
        hfov_deg = self.get_parameter("camera_hfov_deg").value
        self._hfov = math.radians(hfov_deg)
        self._min_pixels = self.get_parameter("min_flag_pixels").value
        self._flag_h = self.get_parameter("flag_real_height_m").value
        # Distância focal em px (assume pixels quadrados: fy = fx). Preenchida no
        # 1º frame a partir da largura da imagem e do HFOV.
        self._focal_px = None

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.create_subscription(Image, "/robot_cam/labels_map", self._cb_labels, qos)
        # Modo "pole": a FSM liga durante POSICIONANDO p/ usar só a metade de baixo
        # da imagem (mastro), ignorando o painel da bandeira no topo, que puxa o
        # centroide pra cima e impede o robô de mirar o centro do mastro.
        self._pole_mode = False
        self.create_subscription(Bool, "/vision/pole_mode", self._cb_pole_mode, 10)

        self._pub_detection = self.create_publisher(
            Pose2D, "/vision/flag_detection", 10
        )
        self._pub_bearing = self.create_publisher(Float32, "/vision/flag_bearing", 10)
        self._pub_distance = self.create_publisher(Float32, "/vision/flag_distance", 10)
        self._pub_scene_class = self.create_publisher(String, "/vision/scene_class", 10)

        self._bridge = CvBridge()
        self.get_logger().info(
            f"VisionProcessor started | flag_labels={self._flag_labels} HFOV={hfov_deg:.0f}°"
        )

    def _cb_pole_mode(self, msg: Bool):
        if msg.data != self._pole_mode:
            self._pole_mode = msg.data
            self.get_logger().info(
                f"[vision] pole_mode={'ON (só metade de baixo)' if msg.data else 'OFF'}"
            )

    def _cb_labels(self, msg: Image):
        try:
            img = self._decode_label_image(msg)
        except Exception as exc:
            self.get_logger().error(
                f"Image decode error: {exc}", throttle_duration_sec=2.0
            )
            return

        if img is None:
            return

        # Máscara COMPLETA da flag (mastro+painel): usada p/ a ALTURA na distância.
        flag_mask_full = np.isin(img, list(self._flag_labels))
        # No modo pole, zera a metade de cima -> só o mastro (parte de baixo) conta
        # p/ centroide/bearing, levando o robô ao centro do mastro, não da bandeira.
        flag_mask = flag_mask_full
        if self._pole_mode:
            flag_mask = flag_mask_full.copy()
            meio = img.shape[0] // 2
            flag_mask[:meio, :] = False
        area = int(np.sum(flag_mask))
        detected = area >= self._min_pixels

        pose_msg = Pose2D()
        bearing_msg = Float32()
        distance_msg = Float32()

        img_h, img_w = float(img.shape[0]), float(img.shape[1])
        if self._focal_px is None:
            # fx = (largura/2) / tan(HFOV/2); pixels quadrados => fy = fx.
            self._focal_px = (img_w / 2.0) / math.tan(self._hfov / 2.0)

        if detected:
            ys, xs = np.where(flag_mask)
            cx = float(np.mean(xs))
            cy = float(np.mean(ys))
            total_px = img_w * img_h

            bearing = (img_w / 2.0 - cx) / (img_w / 2.0) * (self._hfov / 2.0)
            fraction_pct = area / total_px * 100.0

            # Distância por pinhole pela ALTURA aparente do blob (imune a obstáculo
            # que esteja entre o robô e a flag — usa o tamanho, não o range do LIDAR).
            # IMPORTANTE: usa a flag INTEIRA (flag_mask_full), NÃO a máscara do
            # pole_mode. Senão a meia-imagem corta bbox_h e SUPERESTIMA a distância
            # (robô chega perto demais antes de fechar a garra). flag_h é a altura
            # real da flag inteira -> bbox_h também tem que ser da flag inteira.
            ys_full = np.where(flag_mask_full)[0]
            bbox_h = float(ys_full.max() - ys_full.min() + 1) if ys_full.size else 0.0
            distance = (
                self._focal_px * self._flag_h / bbox_h if bbox_h > 0 else 0.0
            )

            pose_msg.x = cx
            pose_msg.y = float(area)
            pose_msg.theta = 1.0
            bearing_msg.data = float(bearing)
            distance_msg.data = float(distance)

            self.get_logger().info(
                f"Flag @ ({cx:.0f}, {cy:.0f})px  bearing={math.degrees(bearing):.1f}°"
                f"  area={area}px ({fraction_pct:.2f}%)  dist~{distance:.2f}m",
                throttle_duration_sec=1.0,
            )
            scene_class = "objective"
        else:
            pose_msg.theta = 0.0
            bearing_msg.data = 0.0
            distance_msg.data = 0.0
            obstacle_detected = bool(np.any((img > 0) & ~flag_mask))
            scene_class = "obstacle" if obstacle_detected else "clear"

        self._pub_detection.publish(pose_msg)
        self._pub_bearing.publish(bearing_msg)
        self._pub_distance.publish(distance_msg)
        self._pub_scene_class.publish(String(data=scene_class))

    def _decode_label_image(self, msg: Image):
        enc = msg.encoding.lower()

        if enc in ("mono8", "8uc1"):
            return self._bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")

        if enc in ("mono16", "16uc1"):
            return self._bridge.imgmsg_to_cv2(msg, desired_encoding="mono16")

        if enc in ("rgb8", "bgr8"):
            colour_img = self._bridge.imgmsg_to_cv2(
                msg, desired_encoding="rgb8" if enc == "rgb8" else "bgr8"
            )
            ch = 0 if enc == "rgb8" else 2
            return colour_img[:, :, ch].astype(np.uint16)

        img = self._bridge.imgmsg_to_cv2(msg)
        if img.ndim == 3:
            img = img[:, :, 0]
        return img.astype(np.uint16)


def main(args=None):
    rclpy.init(args=args)
    node = VisionProcessorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
