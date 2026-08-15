#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Bool
from auv_navigation.msg import CurveReference


class DVZMux(Node):

    def __init__(self):
        super().__init__('dvz_mux')

        # ============================================================
        # Estado
        # ============================================================

        # PATH ou ROTATE
        self.input = "PATH"

        self.emergency_mode = False

        # ============================================================
        # Publisher
        # ============================================================

        self.pub = self.create_publisher(
            CurveReference,
            '/dvz_reference',
            10
        )

        # ============================================================
        # Subscribers
        # ============================================================

        self.sub_select = self.create_subscription(
            String,
            '/mux/dvz_input',
            self.select_callback,
            10
        )

        self.sub_rotate = self.create_subscription(
            CurveReference,
            '/rotate_reference',
            self.rotate_callback,
            10
        )

        self.sub_curve = self.create_subscription(
            CurveReference,
            '/curve_reference',
            self.curve_ref_callback,
            10
        )

        self.sub_emergency = self.create_subscription(
            Bool,
            '/emergency',
            self.emergency_callback,
            10
        )

        self.get_logger().info(
            'DVZ Mux iniciado. Input inicial: PATH'
        )

    # ================================================================
    # Emergency
    # ================================================================

    def emergency_callback(self, msg):
        self.emergency_mode = msg.data

        if self.emergency_mode:
            self.input = "PATH"

            self.get_logger().warn(
                'DVZ Mux: Emergência ativada! '
                'Forçando input para PATH.'
            )

    # ================================================================
    # Seleção do input
    # ================================================================

    def select_callback(self, msg):

        if self.emergency_mode:
            self.get_logger().warn(
                f"DVZ Mux: Recebida solicitação de mudança para "
                f"'{msg.data}', mas estamos em modo de emergência. "
                f"Ignorando."
            )
            return

        requested_input = msg.data.upper()

        if requested_input in ["PATH", "ROTATE"]:

            self.input = requested_input

            self.get_logger().info(
                f'DVZ Mux: Selecionado {self.input}'
            )

        else:

            self.get_logger().warn(
                f"DVZ Mux: Entrada desconhecida "
                f"'{requested_input}'"
            )

    # ================================================================
    # Curve reference
    # ================================================================

    def curve_ref_callback(self, msg):

        if self.input == "PATH":
            self.pub.publish(msg)

    # ================================================================
    # Rotate reference
    # ================================================================

    def rotate_callback(self, msg):

        if self.input == "ROTATE":
            self.pub.publish(msg)


def main(args=None):

    rclpy.init(args=args)

    node = DVZMux()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
