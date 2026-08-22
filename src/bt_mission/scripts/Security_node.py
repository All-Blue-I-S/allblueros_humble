#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32, Bool
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped


class SecurityNode(Node):
    """
    Nó de Segurança para monitorar o estado do AUV e tomar
    ações preventivas.

    Monitora:
        - nível da bateria;
        - odometria.

    Caso a bateria atinja um nível crítico, o nó:
        1. ativa o modo de emergência;
        2. gera uma trajetória para a superfície.
    """

    def __init__(self):
        super().__init__('security_node')

        # ============================================================
        # Subscribers
        # ============================================================

        self.sub_battery = self.create_subscription(
            Float32,
            '/battery_state',
            self.battery_callback,
            10
        )

        self.sub_odom = self.create_subscription(
            Odometry,
            '/ground_truth/odom',
            self.odom_callback,
            10
        )

        # ============================================================
        # Publishers
        # ============================================================

        self.pub_path = self.create_publisher(
            Path,
            '/emergency_path',
            10
        )

        self.pub_emergence = self.create_publisher(
            Bool,
            '/emergency',
            10
        )

        # ============================================================
        # Estado
        # ============================================================

        self.emergency_state = False

        self.battery_level = None

        # Limite crítico de bateria
        self.battery_threshold = 20.0

        self.odom_received = False

        self.odom = None

        self.started_emergency_procedure = False

        # ============================================================
        # Timer
        # ============================================================

        self.timer = self.create_timer(
            0.1,  # 10 Hz
            self.control_loop
        )

        self.get_logger().info(
            'Security Node iniciado.'
        )

    # ================================================================
    # Battery
    # ================================================================

    def battery_callback(self, msg):
        self.battery_level = msg.data

        if self.battery_level < self.battery_threshold:

            if not self.emergency_state:

                self.get_logger().warn(
                    f'Nível de bateria crítico: '
                    f'{self.battery_level:.2f}%. '
                    f'Emergência ativada!'
                )

            self.emergency_state = True

    # ================================================================
    # Odometry
    # ================================================================

    def odom_callback(self, msg):
        self.odom_received = True
        self.odom = msg

    # ================================================================
    # Emergency procedure
    # ================================================================

    def emergency_procedure(self):
        """
        Executa o procedimento de emergência.

        Atualmente:
            - ativa o estado de emergência;
            - publica uma trajetória para a superfície.
        """

        if self.odom is None:
            self.get_logger().warn(
                'Não é possível executar emergência: '
                'odometria ainda não recebida.'
            )
            return

        self.get_logger().warn(
            'Executando procedimento de emergência: Emergindo.'
        )

        # ============================================================
        # Ativa emergência
        # ============================================================

        emergency_msg = Bool()
        emergency_msg.data = True

        self.pub_emergence.publish(
            emergency_msg
        )

        # ============================================================
        # Pose atual
        # ============================================================

        current_pose = self.odom.pose.pose

        # ------------------------------------------------------------
        # IMPORTANTE:
        # Não devemos fazer:
        #
        # desired_pose = current_pose
        #
        # pois isso referencia o mesmo objeto.
        #
        # Criamos uma cópia explícita.
        # ------------------------------------------------------------

        desired_pose = type(current_pose)()

        desired_pose.position.x = current_pose.position.x
        desired_pose.position.y = current_pose.position.y
        desired_pose.position.z = 0.2

        desired_pose.orientation.x = current_pose.orientation.x
        desired_pose.orientation.y = current_pose.orientation.y
        desired_pose.orientation.z = current_pose.orientation.z
        desired_pose.orientation.w = current_pose.orientation.w

        # ============================================================
        # Path
        # ============================================================

        path = Path()

        path.header.frame_id = 'map'
        path.header.stamp = self.get_clock().now().to_msg()

        # ------------------------------------------------------------
        # Pose atual
        # ------------------------------------------------------------

        current_pose_stamped = PoseStamped()

        current_pose_stamped.header = path.header
        current_pose_stamped.pose = current_pose

        # ------------------------------------------------------------
        # Pose desejada
        # ------------------------------------------------------------

        desired_pose_stamped = PoseStamped()

        desired_pose_stamped.header = path.header
        desired_pose_stamped.pose = desired_pose

        # ------------------------------------------------------------
        # Adiciona ao Path
        # ------------------------------------------------------------

        path.poses.append(
            current_pose_stamped
        )

        path.poses.append(
            desired_pose_stamped
        )

        self.pub_path.publish(
            path
        )

    # ================================================================
    # Control loop
    # ================================================================

    def control_loop(self):

        if (
            self.emergency_state
            and not self.started_emergency_procedure
        ):

            self.started_emergency_procedure = True

            self.emergency_procedure()


def main(args=None):

    rclpy.init(args=args)

    security_node = SecurityNode()

    try:
        rclpy.spin(security_node)

    except KeyboardInterrupt:
        pass

    finally:
        security_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
