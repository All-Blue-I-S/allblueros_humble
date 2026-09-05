#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Accel
from std_msgs.msg import String, Bool


class CommandMux(Node):

    def __init__(self):
        super().__init__('cmd_accel_mux')

        # ============================================================
        # Estado
        # ============================================================

        # Pode ser:
        #   GO2  -> controlador Go2
        #   VS   -> Visual Servoing
        #   IDLE -> parado
        self.active_controller = "IDLE"

        # Modo de emergência
        self.emergency_mode = False

        # ============================================================
        # Publisher
        # ============================================================

        # Tópico REAL de comando do AUV
        self.cmd_pub = self.create_publisher(
            Accel,
            '/cmd_accel',
            10
        )

        # ============================================================
        # Subscribers
        # ============================================================

        # Behavior Tree seleciona quem possui autoridade
        self.mode_sub = self.create_subscription(
            String,
            '/mux/cmd_vel',
            self.select_callback,
            10
        )

        # Controlador GO2
        self.go2_sub = self.create_subscription(
            Accel,
            '/cmd_accel/go2',
            self.go2_callback,
            10
        )

        # Visual Servoing
        self.vs_sub = self.create_subscription(
            Accel,
            '/cmd_accel/vs',
            self.vs_callback,
            10
        )

        # Emergency
        self.sub_emergency = self.create_subscription(
            Bool,
            '/emergency',
            self.emergency_callback,
            10
        )

        self.get_logger().info(
            'Mux de Aceleração iniciado. '
            'Modo inicial: IDLE'
        )

    # ================================================================
    # Emergency
    # ================================================================

    def emergency_callback(self, msg):
        """
        Callback do modo de emergência.

        Quando emergência é ativada, o mux força o controlador
        GO2 e ignora novas solicitações de mudança de modo.
        """

        previous_emergency = self.emergency_mode

        self.emergency_mode = msg.data

        if self.emergency_mode and not previous_emergency:

            self.get_logger().warn(
                'Mux: Emergência ativada! '
                'Forçando modo para GO2.'
            )

            self.active_controller = "GO2"

        elif not self.emergency_mode and previous_emergency:

            self.get_logger().info(
                'Mux: Emergência desativada.'
            )

    # ================================================================
    # Mode selection
    # ================================================================

    def select_callback(self, msg):
        """
        Recebe a solicitação da Behavior Tree para selecionar
        qual controlador possui autoridade.
        """

        if self.emergency_mode:

            self.get_logger().warn(
                f"Mux: Recebida solicitação de mudança para "
                f"'{msg.data}', mas estamos em modo de emergência. "
                f"Ignorando."
            )

            return

        requested_mode = msg.data.upper()

        if requested_mode in ["GO2", "VS", "IDLE"]:

            if self.active_controller != requested_mode:

                self.get_logger().info(
                    f'Trocando autoridade de controle para: '
                    f'{requested_mode}'
                )

                self.active_controller = requested_mode

                # ----------------------------------------------------
                # Se mudou para IDLE, envia parada imediatamente
                # ----------------------------------------------------

                if self.active_controller == "IDLE":

                    stop_msg = Accel()

                    self.cmd_pub.publish(
                        stop_msg
                    )

        else:

            self.get_logger().warn(
                f'Modo de Mux desconhecido: '
                f'{requested_mode}'
            )

    # ================================================================
    # GO2
    # ================================================================

    def go2_callback(self, msg):

        if self.active_controller == "GO2":

            self.cmd_pub.publish(msg)

    # ================================================================
    # Visual Servoing
    # ================================================================

    def vs_callback(self, msg):

        if self.active_controller == "VS":

            self.cmd_pub.publish(msg)


def main(args=None):

    rclpy.init(args=args)

    node = CommandMux()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
