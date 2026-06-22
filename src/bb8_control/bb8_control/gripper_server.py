#!/usr/bin/env python3
"""Servidor de gripper: expõe Services SetBool no lugar de controle por tópico.

Sequência de captura (a FSM chama em ordem):
  1) /gripper/extend True  -> estende o braço com a garra ABERTA (chega na flag)
  2) /gripper/grab   True  -> FECHA a garra (pega a flag)

  /gripper/extend (std_srvs/SetBool)
    data=True  -> braço estendido à frente, garra aberta
    data=False -> braço retraído (postura de navegação)
  /gripper/grab (std_srvs/SetBool)
    data=True  -> fecha a garra (mantém o braço estendido)
    data=False -> abre a garra (mantém o braço estendido)

É o ÚNICO nó que publica em /gripper_controller/commands. No construtor já
comanda a postura retraída, garantindo início fechado/recolhido. O estado físico
inicial também é forçado via initial_value no URDF (ros2_control).
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import SetBool

# Ordem das juntas em /gripper_controller/commands (igual ao controller_config.yaml):
#   [shoulder_pitch, gripper_extension, arm_elbow, right_gripper_joint, left_gripper_joint]
# shoulder_pitch (Junta 1): 0.0 = reto p/ frente; 0.785398 = levantado 45 graus
#   (nessa posição o braço entra no FOV do LIDAR -> scan_masker mascara o frontal).
# Dedos: [right_gripper_joint, left_gripper_joint]. 0,0 = FECHADO (gap 0.02);
#        -0.06,0.06 = ABERTO (gap 0.14). (ver limites das juntas no URDF)
ARM_RETRAIDO = [0.0, -1.5, -1.5, 0.0, 0.0]            # ombro baixo, recolhido + garra FECHADA (início)
ARM_ESTENDIDO_ABERTO = [0.785398, 0.0, 0.0, -0.06, 0.06]   # ombro 45°, estendido, garra ABERTA
ARM_ESTENDIDO_FECHADO = [0.785398, 0.0, 0.0, 0.0, 0.0]     # ombro 45°, estendido, garra FECHADA (pega flag)


class GripperServer(Node):
    def __init__(self):
        super().__init__("gripper_server")
        self._pub = self.create_publisher(
            Float64MultiArray, "/gripper_controller/commands", 10
        )
        self._estendido = False  # garra aberta/fechada só faz sentido se estendido
        self._srv_extend = self.create_service(
            SetBool, "/gripper/extend", self._cb_extend
        )
        self._srv_grab = self.create_service(SetBool, "/gripper/grab", self._cb_grab)
        # Início retraído: publica assim que o controlador estiver no ar.
        self._timer = self.create_timer(2.0, self._init_retraido)
        self.get_logger().info("[gripper] services /gripper/extend e /gripper/grab prontos")

    def _init_retraido(self):
        self._enviar(ARM_RETRAIDO)
        self._timer.cancel()
        self.get_logger().info("[gripper] postura inicial: RETRAÍDO")

    def _cb_extend(self, req, resp):
        if req.data:
            self._enviar(ARM_ESTENDIDO_ABERTO)
            self._estendido = True
            resp.message = "braço estendido, garra aberta"
        else:
            self._enviar(ARM_RETRAIDO)
            self._estendido = False
            resp.message = "braço retraído"
        resp.success = True
        self.get_logger().info(f"[gripper] {resp.message}")
        return resp

    def _cb_grab(self, req, resp):
        # Mantém o braço estendido; só muda os dedos.
        self._enviar(ARM_ESTENDIDO_FECHADO if req.data else ARM_ESTENDIDO_ABERTO)
        resp.success = True
        resp.message = "garra fechada" if req.data else "garra aberta"
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
