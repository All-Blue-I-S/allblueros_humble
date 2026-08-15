#!/usr/bin/env python3

import numpy as np

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import (
    Pose,
    Twist,
    Accel,
    Vector3,
    Quaternion,
    PoseStamped
)
from std_msgs.msg import Header, Bool

from auv_navigation.msg import CurveReference

from curve3d import Curve3D


class CurveReferenceNode(Node):
    """
    Nó ROS responsável por receber uma trajetória (Path), convertê-la em uma
    curva paramétrica contínua (Curve3D) e publicar referências cinemáticas
    completas (posição, orientação, velocidade e aceleração) para o
    controlador do AUV.
    """

    def __init__(self):

        super().__init__('curve_reference_node')

        # ============================================================
        # Parâmetros
        # ============================================================

        self.declare_parameter(
            'interp_method',
            'cubic'
        )

        self.declare_parameter(
            'use_lookahead',
            False
        )

        self.declare_parameter(
            'lookahead_gain',
            1.0
        )

        self.declare_parameter(
            'lookahead_min',
            0.5
        )

        self.declare_parameter(
            'lookahead_max',
            3.0
        )

        self.declare_parameter(
            'search_window',
            5.0
        )

        self.declare_parameter(
            'nominal_speed',
            0.7
        )

        self.declare_parameter(
            'max_lateral_acc',
            2.0
        )

        self.declare_parameter(
            'goal_tolerance',
            0.3
        )

        self.declare_parameter(
            'interpolation_points',
            200
        )

        self.declare_parameter(
            'publish_interpolated_curve',
            True
        )

        self.declare_parameter(
            'window_horizon',
            15.0
        )

        self.declare_parameter(
            'window_points',
            20
        )

        self.declare_parameter(
            'window_rate',
            5.0
        )

        # ============================================================
        # Recuperação dos parâmetros
        # ============================================================

        self.interp_method = self.get_parameter(
            'interp_method'
        ).value

        self.use_lookahead = self.get_parameter(
            'use_lookahead'
        ).value

        self.lookahead_gain = self.get_parameter(
            'lookahead_gain'
        ).value

        self.lookahead_min = self.get_parameter(
            'lookahead_min'
        ).value

        self.lookahead_max = self.get_parameter(
            'lookahead_max'
        ).value

        self.search_window = self.get_parameter(
            'search_window'
        ).value

        self.nominal_speed = self.get_parameter(
            'nominal_speed'
        ).value

        self.max_lateral_acc = self.get_parameter(
            'max_lateral_acc'
        ).value

        self.tolerance = self.get_parameter(
            'goal_tolerance'
        ).value

        self.num_interp_points = self.get_parameter(
            'interpolation_points'
        ).value

        self.publish_curve_flag = self.get_parameter(
            'publish_interpolated_curve'
        ).value

        self.window_horizon = self.get_parameter(
            'window_horizon'
        ).value

        self.window_points = self.get_parameter(
            'window_points'
        ).value

        self.window_rate = self.get_parameter(
            'window_rate'
        ).value

        # ============================================================
        # Estado interno
        # ============================================================

        self.curve = None

        self.lam_current = 0.0

        self.initialized = False

        self.current_curve_stamp = self.get_clock().now().to_msg()

        self.emergency_mode = False

        # ============================================================
        # Subscribers
        # ============================================================

        self.path_sub = self.create_subscription(
            Path,
            '/path',
            self.path_callback,
            10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            '/ground_truth/odom',
            self.odom_callback,
            10
        )

        # ------------------------------------------------------------
        # Emergência
        # ------------------------------------------------------------

        self.sub_emergency = self.create_subscription(
            Bool,
            '/emergency',
            self.emergency_callback,
            10
        )

        self.sub_emergency_path = self.create_subscription(
            Path,
            '/emergency_path',
            self.emergency_path_callback,
            10
        )

        # ============================================================
        # Publishers
        # ============================================================

        self.ref_pub = self.create_publisher(
            CurveReference,
            '/curve_reference',
            10
        )

        if self.publish_curve_flag:

            self.curve_pub = self.create_publisher(
                Path,
                '/path_visualization',
                1
            )

        self.window_pub = self.create_publisher(
            Path,
            '/planned_trajectory',
            1
        )

        # ============================================================
        # Timer
        # ============================================================

        self.window_timer = self.create_timer(
            1.0 / self.window_rate,
            self.publish_sliding_window
        )

        self.get_logger().info(
            '[curve_reference_node] Inicializado.'
        )

    # ================================================================
    # CALLBACKS
    # ================================================================

    def emergency_callback(self, msg):
        """
        Ativa o modo de emergência quando uma mensagem é recebida.
        """

        if not self.emergency_mode:

            self.get_logger().warn(
                '[curve_reference_node] '
                'Modo de emergência ativado.'
            )

        self.emergency_mode = True

    # ----------------------------------------------------------------

    def emergency_path_callback(self, msg):
        """
        Recebe um Path de emergência e gera uma nova Curve3D.
        """

        if not self.emergency_mode:
            return

        self.get_logger().warn(
            '[curve_reference_node] '
            'Recebido Path de emergência. '
            'Gerando curva de emergência.'
        )

        if len(msg.poses) < 2:

            self.get_logger().warn(
                'Path inválido: necessita de pelo menos 2 waypoints.'
            )

            return

        waypoints = np.array(
            [
                [
                    p.pose.position.x,
                    p.pose.position.y,
                    p.pose.position.z
                ]
                for p in msg.poses
            ]
        )

        self.curve = Curve3D(
            waypoints,
            method=self.interp_method
        )

        self.lam_current = 0.0

        self.initialized = True

        self.current_curve_stamp = msg.header.stamp

        self.publish_interpolated_curve()

        self.get_logger().info(
            '[curve_reference_node] '
            'Curva de emergência gerada com sucesso.'
        )

    # ----------------------------------------------------------------

    def path_callback(self, msg):
        """
        Recebe uma nova mensagem nav_msgs/Path, extrai os waypoints
        e gera uma curva contínua.
        """

        if self.emergency_mode:

            self.get_logger().warn(
                '[curve_reference_node] '
                'Recebido novo Path, mas estamos em modo de emergência. '
                'Ignorando.'
            )

            return

        if len(msg.poses) < 2:

            self.get_logger().warn(
                'Path inválido: necessita de pelo menos 2 waypoints.'
            )

            return

        waypoints = np.array(
            [
                [
                    p.pose.position.x,
                    p.pose.position.y,
                    p.pose.position.z
                ]
                for p in msg.poses
            ]
        )

        self.curve = Curve3D(
            waypoints,
            method=self.interp_method
        )

        self.lam_current = 0.0

        self.initialized = True

        self.current_curve_stamp = msg.header.stamp

        self.publish_interpolated_curve()

        self.get_logger().info(
            '[curve_reference_node] '
            'Nova curva contínua gerada com sucesso.'
        )

    # ----------------------------------------------------------------

    def odom_callback(self, msg):
        """
        Atualiza o parâmetro da curva atual usando a posição atual
        do AUV e publica a referência correspondente.
        """

        if not self.initialized:
            return

        pos = np.array(
            [
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z
            ]
        )

        self.update_lam(pos)

        self.publish_reference()

    # ================================================================
    # PROJEÇÃO E LOOKAHEAD
    # ================================================================

    def update_lam(self, position):
        """
        Encontra o ponto mais próximo da curva em relação à posição
        atual do robô.
        """

        lam_min = max(
            0.0,
            self.lam_current - self.search_window
        )

        lam_max = min(
            self.curve.length,
            self.lam_current + self.search_window
        )

        # Busca local densa
        lam_samples = np.linspace(
            lam_min,
            lam_max,
            50
        )

        pts = self.curve.position(
            lam_samples
        )

        distances = np.linalg.norm(
            pts - position,
            axis=1
        )

        idx = np.argmin(
            distances
        )

        lam_closest = lam_samples[idx]

        # Impede regressão ao longo da curva
        self.lam_current = max(
            self.lam_current,
            lam_closest
        )

    # ----------------------------------------------------------------

    def compute_lookahead(self, curvature):
        """
        Calcula a distância de lookahead baseada na curvatura.
        """

        if not self.use_lookahead:
            return 0.0

        L = (
            self.lookahead_gain
            / (1.0 + curvature)
        )

        return np.clip(
            L,
            self.lookahead_min,
            self.lookahead_max
        )

    # ================================================================
    # PUBLICAÇÃO DA JANELA DESLIZANTE
    # ================================================================

    def publish_sliding_window(self):

        if (
            not self.initialized
            or self.curve is None
        ):
            return

        if (
            self.lam_current
            >= self.curve.length - self.tolerance
        ):

            empty_path = Path()

            empty_path.header.stamp = (
                self.current_curve_stamp
            )

            empty_path.header.frame_id = 'map'

            self.window_pub.publish(
                empty_path
            )

            return

        lam_end = min(
            self.curve.length,
            self.lam_current + self.window_horizon
        )

        lam_samples = np.linspace(
            self.lam_current,
            lam_end,
            self.window_points
        )

        positions = self.curve.position(
            lam_samples
        )

        next_wp_indices = (
            self.curve.get_next_waypoint_index(
                lam_samples
            )
        )

        path_msg = Path()

        path_msg.header.stamp = (
            self.current_curve_stamp
        )

        path_msg.header.frame_id = 'map'

        for i in range(
            len(lam_samples)
        ):

            pose = PoseStamped()

            pose.header.stamp = (
                path_msg.header.stamp
            )

            pose.header.frame_id = (
                path_msg.header.frame_id
            )

            pose.pose.position.x = (
                positions[i, 0]
            )

            pose.pose.position.y = (
                positions[i, 1]
            )

            pose.pose.position.z = (
                positions[i, 2]
            )

            pose.pose.orientation.w = 1.0

            # ========================================================
            # ATENÇÃO:
            # ROS 1 tinha Header.seq.
            #
            # ROS 2 não possui mais header.seq.
            #
            # Portanto, essa linha do ROS 1:
            #
            # pose.header.seq = int(next_wp_indices[i])
            #
            # foi removida.
            # ========================================================

            path_msg.poses.append(
                pose
            )

        self.window_pub.publish(
            path_msg
        )

    # ================================================================
    # REFERÊNCIA CINEMÁTICA
    # ================================================================

    def publish_reference(self):
        """
        Calcula as referências cinemáticas no ponto atual ou no
        ponto de lookahead.
        """

        # ------------------------------------------------------------
        # Curvatura
        # ------------------------------------------------------------

        kappa = self.curve.curvature(
            np.array([self.lam_current])
        )[0]

        # ------------------------------------------------------------
        # Parada / movimento
        # ------------------------------------------------------------

        if (
            self.lam_current
            >= self.curve.length - self.tolerance
        ):

            self.lam_current = (
                self.curve.length
            )

            v_ref = 0.0

        else:

            if kappa > 1e-6:

                v_ref = min(
                    self.nominal_speed,
                    np.sqrt(
                        self.max_lateral_acc / kappa
                    )
                )

            else:

                v_ref = self.nominal_speed

        # ------------------------------------------------------------
        # Lookahead
        # ------------------------------------------------------------

        L = self.compute_lookahead(
            kappa
        )

        lam_ref = np.clip(
            self.lam_current + L,
            0.0,
            self.curve.length
        )

        # ------------------------------------------------------------
        # Referências cinemáticas
        # ------------------------------------------------------------

        (
            pos,
            quat_wxyz,
            vel_lin,
            vel_ang,
            acc_lin,
            acc_ang
        ) = self.curve.kinematic_references(
            np.array([lam_ref]),
            v_ref
        )

        pos = pos[0]

        quat = quat_wxyz[0]

        vel_lin = vel_lin[0]
        vel_ang = vel_ang[0]

        acc_lin = acc_lin[0]
        acc_ang = acc_ang[0]

        # ------------------------------------------------------------
        # Frenet frame
        # ------------------------------------------------------------

        t, n, b = self.curve.frenet_frame(
            np.array([lam_ref])
        )

        t = t[0]
        n = n[0]
        b = b[0]

        # ============================================================
        # Mensagens ROS
        # ============================================================

        twist = Twist()

        twist.linear.x = vel_lin[0]
        twist.linear.y = vel_lin[1]
        twist.linear.z = vel_lin[2]

        twist.angular.x = vel_ang[0]
        twist.angular.y = vel_ang[1]
        twist.angular.z = vel_ang[2]

        accel = Accel()

        accel.linear.x = acc_lin[0]
        accel.linear.y = acc_lin[1]
        accel.linear.z = acc_lin[2]

        accel.angular.x = acc_ang[0]
        accel.angular.y = acc_ang[1]
        accel.angular.z = acc_ang[2]

        pose = Pose()

        pose.position.x = pos[0]
        pose.position.y = pos[1]
        pose.position.z = pos[2]

        # Curve3D retorna [w, x, y, z]
        # ROS Quaternion espera x, y, z, w

        pose.orientation = Quaternion(
            x=quat[1],
            y=quat[2],
            z=quat[3],
            w=quat[0]
        )

        # ============================================================
        # CurveReference
        # ============================================================

        msg = CurveReference()

        msg.header = Header()

        msg.header.stamp = (
            self.get_clock().now().to_msg()
        )

        msg.header.frame_id = 'map'

        msg.pose = pose

        msg.twist = twist

        msg.accel = accel

        msg.s = float(lam_ref)

        msg.curvature = float(kappa)

        msg.tangent = Vector3(
            x=float(t[0]),
            y=float(t[1]),
            z=float(t[2])
        )

        msg.normal = Vector3(
            x=float(n[0]),
            y=float(n[1]),
            z=float(n[2])
        )

        msg.binormal = Vector3(
            x=float(b[0]),
            y=float(b[1]),
            z=float(b[2])
        )

        self.ref_pub.publish(
            msg
        )

    # ================================================================
    # CURVA INTERPOLADA
    # ================================================================

    def publish_interpolated_curve(self):
        """
        Publica uma representação densa da curva para visualização.
        """

        if (
            not self.publish_curve_flag
            or self.curve is None
        ):
            return

        lam_samples = np.linspace(
            0.0,
            self.curve.length,
            self.num_interp_points
        )

        positions = self.curve.position(
            lam_samples
        )

        quats_wxyz = (
            self.curve.level_flight_quaternion(
                lam_samples
            )
        )

        path_msg = Path()

        path_msg.header.stamp = (
            self.get_clock().now().to_msg()
        )

        path_msg.header.frame_id = 'map'

        for i in range(
            len(lam_samples)
        ):

            pose = PoseStamped()

            pose.header.stamp = (
                path_msg.header.stamp
            )

            pose.header.frame_id = (
                path_msg.header.frame_id
            )

            pose.pose.position.x = (
                positions[i, 0]
            )

            pose.pose.position.y = (
                positions[i, 1]
            )

            pose.pose.position.z = (
                positions[i, 2]
            )

            # Curve3D:
            # [w, x, y, z]
            #
            # ROS:
            # [x, y, z, w]

            pose.pose.orientation = Quaternion(
                x=quats_wxyz[i, 1],
                y=quats_wxyz[i, 2],
                z=quats_wxyz[i, 3],
                w=quats_wxyz[i, 0]
            )

            path_msg.poses.append(
                pose
            )

        self.curve_pub.publish(
            path_msg
        )


def main(args=None):

    np.set_printoptions(
        precision=3,
        suppress=True
    )

    rclpy.init(
        args=args
    )

    node = CurveReferenceNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
