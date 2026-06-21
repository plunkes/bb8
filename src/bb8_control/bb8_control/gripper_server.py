#!/usr/bin/env python3
"""Servidor de gripper: expõe um Service SetBool no lugar de controle por tópico.

  /gripper/grab  (std_srvs/SetBool)
    data=True  -> estende o braço e fecha a garra (pega a flag)
    data=False -> abre a garra e retrai o braço (postura de navegação)

É o ÚNICO nó que publica em /gripper_controller/commands. No construtor já
comanda a postura retraída, garantindo início fechado/recolhido. O estado físico
inicial também é forçado via initial_value no URDF (ros2_control).
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import SetBool

# Ordem das juntas em /gripper_controller/commands (igual ao controller_config.yaml):
#   [gripper_extension, arm_elbow, right_gripper_joint, left_gripper_joint]
ARM_RETRAIDO = [-1.5, -1.5, 0.0, 0.0]            # recolhido + garra aberta (navegação)
ARM_ESTENDIDO_FECHADO = [0.0, 0.0, -0.06, 0.06]  # estendido + garra fechada (pega flag)


class GripperServer(Node):
    def __init__(self):
        super().__init__("gripper_server")
        self._pub = self.create_publisher(
            Float64MultiArray, "/gripper_controller/commands", 10
        )
        self._srv = self.create_service(SetBool, "/gripper/grab", self._cb_grab)
        # Início retraído: publica assim que o controlador estiver no ar.
        self._timer = self.create_timer(2.0, self._init_retraido)
        self.get_logger().info("[gripper] service /gripper/grab pronto")

    def _init_retraido(self):
        self._enviar(ARM_RETRAIDO)
        self._timer.cancel()
        self.get_logger().info("[gripper] postura inicial: RETRAÍDO")

    def _cb_grab(self, req, resp):
        if req.data:
            self._enviar(ARM_ESTENDIDO_FECHADO)
            resp.message = "gripper estendido e fechado"
        else:
            self._enviar(ARM_RETRAIDO)
            resp.message = "gripper retraído e aberto"
        resp.success = True
        self.get_logger().info(f"[gripper] {resp.message}")
        return resp

    def _enviar(self, posicoes):
        msg = Float64MultiArray()
        msg.data = posicoes
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GripperServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
