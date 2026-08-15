#!/usr/bin/env python3

import numpy as np

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Accel
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry

from auv_control.srv import SetTrackingMode


class PDTrajectoryController(Node):
    """
    Controlador de Trajetória Dinâmico usando
    Image-Based Visual Servoing (IBVS).

    Calcula as velocidades desejadas e converte
    em aceleração usando a Odometria.
    """

    def __init__(self):
        super().__init__('trajectory_controller')

        # ============================================================
        # Publishers
        # ============================================================

        self.desired_accel_pub = self.create_publisher(
            Accel,
            '/cmd_accel/vs',
            10
        )

        # ============================================================
        # Subscribers
        # ============================================================

        self.features_sub_detected = self.create_subscription(
            Float32MultiArray,
            '/auv/image/features/estimed',
            self.features_detected_callback,
            10
        )

        self.features_sub_desired = self.create_subscription(
            Float32MultiArray,
            '/align/desired',
            self.features_desired_callback,
            10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            '/ground_truth/odom',
            self.odom_callback,
            10
        )

        # ============================================================
        # Service
        # ============================================================

        self.mode_service = self.create_service(
            SetTrackingMode,
            '~/set_mode',
            self.handle_set_mode
        )

        # ============================================================
        # Estado
        # ============================================================

        self.control_mode = "TRACK"
        self.orbit_velocity = 0.5

        self.features_detected = None
        self.features_desired = None
        self.vel_curr_body = None

        # ============================================================
        # Parâmetros
        # ============================================================

        self.declare_parameter(
            'focal_length',
            546.1979
        )

        self.declare_parameter(
            'gain_kp',
            1.0
        )

        self.declare_parameter(
            'image_width',
            640.0
        )

        self.declare_parameter(
            'image_height',
            480.0
        )

        self.f = self.get_parameter(
            'focal_length'
        ).value

        self.kp = self.get_parameter(
            'gain_kp'
        ).value

        self.image_width = self.get_parameter(
            'image_width'
        ).value

        self.image_height = self.get_parameter(
            'image_height'
        ).value

        self.cu = self.image_width / 2.0
        self.cv = self.image_height / 2.0

        # ============================================================
        # Timer
        # ============================================================

        self.control_rate = 30.0

        self.timer = self.create_timer(
            1.0 / self.control_rate,
            self.publish_sensor_data
        )

        self.get_logger().info(
            'Controlador PD Dinâmico iniciado.'
        )

    # ================================================================
    # Service
    # ================================================================

    def handle_set_mode(self, request, response):
        """
        Callback do serviço para alterar o modo de controle.
        """

        requested_mode = request.mode.upper()

        if requested_mode in ["TRACK", "ORBIT"]:

            self.control_mode = requested_mode
            self.orbit_velocity = request.orbit_velocity

            msg = (
                f"Modo alterado para {self.control_mode} | "
                f"V_orb: {self.orbit_velocity:.2f}"
            )

            self.get_logger().info(msg)

            response.success = True
            response.message = msg

        else:

            msg = (
                f"Modo invalido: {requested_mode}. "
                "Use 'TRACK' ou 'ORBIT'."
            )

            self.get_logger().warn(msg)

            response.success = False
            response.message = msg

        return response

    # ================================================================
    # Subscribers callbacks
    # ================================================================

    def odom_callback(self, msg):
        """
        Atualiza a velocidade atual do robô
        vinda do simulador (Body Frame).
        """

        self.vel_curr_body = np.array(
            [
                msg.twist.twist.linear.x,
                msg.twist.twist.linear.y,
                msg.twist.twist.linear.z,
                msg.twist.twist.angular.x,
                msg.twist.twist.angular.y,
                msg.twist.twist.angular.z
            ]
        )

    def features_detected_callback(self, msg):
        self.features_detected = np.array(
            msg.data,
            dtype=float
        )

    def features_desired_callback(self, msg):
        self.features_desired = np.array(
            msg.data,
            dtype=float
        )

    # ================================================================
    # Control loop
    # ================================================================

    def publish_sensor_data(self):
        """
        Loop periódico do controlador.
        """

        accel = np.zeros(6)

        if (
            self.features_detected is not None
            and self.features_desired is not None
            and self.vel_curr_body is not None
        ):

            try:
                accel = self.compute_pd_control()

            except Exception as e:
                self.get_logger().warn(
                    f'Erro no controle: {e}'
                )

        # Limitação da aceleração
        if np.linalg.norm(accel) > 2.0:
            accel = (
                accel /
                np.linalg.norm(accel)
            )

        accel_msg = Accel()

        accel_msg.linear.x = accel[0]
        accel_msg.linear.y = accel[1]
        accel_msg.linear.z = accel[2]

        accel_msg.angular.x = accel[3]
        accel_msg.angular.y = accel[4]
        accel_msg.angular.z = accel[5]

        self.desired_accel_pub.publish(
            accel_msg
        )

    # ================================================================
    # Interaction Matrix
    # ================================================================

    def compute_interaction_matrix(
        self,
        features_detected
    ):
        """
        Calcula a matriz de interação da câmera.
        """

        L = []

        for i in range(0, len(features_detected), 3):

            u, v, z = features_detected[i:i + 3]

            u_c = u - self.cu
            v_c = v - self.cv

            if abs(z) < 1e-6:
                z = (
                    np.sign(z) * 1e-6
                    if z != 0
                    else 1e-6
                )

            L_i = np.array(
                [
                    [
                        -self.f / z,
                        0,
                        u_c / z,
                        -(self.f**2 + u_c**2) / self.f
                    ],
                    [
                        0,
                        -self.f / z,
                        v_c / z,
                        -u_c * v_c / self.f
                    ]
                ]
            )

            L.append(L_i)

        return np.vstack(L)

    # ================================================================
    # PD / IBVS Controller
    # ================================================================

    def compute_pd_control(self):

        n_detected_pts = (
            len(self.features_detected) // 3
        )

        n_desired_pts = (
            len(self.features_desired) // 2
        )

        if (
            n_detected_pts == 0
            or n_detected_pts != n_desired_pts
        ):
            raise ValueError(
                "Features incompatíveis."
            )

        # ------------------------------------------------------------
        # Erro de features
        # ------------------------------------------------------------

        uv_detected = (
            self.features_detected
            .reshape(-1, 3)[:, :2]
            .flatten()
        )

        feature_error = (
            self.features_desired
            - uv_detected
        )

        sdot = self.kp * feature_error

        # ------------------------------------------------------------
        # Atualiza profundidade estimada
        # ------------------------------------------------------------

        self.features_detected = np.array(
            [
                self.features_desired[0],
                self.features_desired[1],
                8.4,

                self.features_desired[2],
                self.features_desired[3],
                8.4,

                self.features_desired[4],
                self.features_desired[5],
                8.4,

                self.features_desired[6],
                self.features_desired[7],
                8.4
            ]
        )

        # ------------------------------------------------------------
        # Interaction matrix
        # ------------------------------------------------------------

        L_base = self.compute_interaction_matrix(
            self.features_detected
        )

        control_cam = np.zeros(6)

        # ============================================================
        # ORBIT
        # ============================================================

        if self.control_mode == "ORBIT":

            # Vy, Vz, Wy
            active_cols = [1, 2, 3]

            L_reduced = (
                L_base[:, active_cols]
            )

            L_pinv = np.linalg.pinv(
                L_reduced
            )

            control_reduced = (
                L_pinv @ sdot
            )

            control_cam[0] = (
                self.orbit_velocity
            )

            control_cam[1] = (
                control_reduced[0]
            )

            control_cam[2] = (
                control_reduced[1]
            )

            control_cam[4] = (
                control_reduced[2]
            )

        # ============================================================
        # TRACK
        # ============================================================

        else:

            # Vx, Vy, Vz, Wy
            active_cols = [0, 1, 2, 3]

            L_reduced = (
                L_base[:, active_cols]
            )

            L_pinv = np.linalg.pinv(
                L_reduced
            )

            control_reduced = (
                L_pinv @ sdot
            )

            control_cam[0] = (
                control_reduced[0]
            )

            control_cam[1] = (
                control_reduced[1]
            )

            control_cam[2] = (
                control_reduced[2]
            )

            control_cam[4] = (
                control_reduced[3]
            )

        # ============================================================
        # Camera -> Body
        # ============================================================

        control_body = np.zeros(6)

        control_body[0] = (
            control_cam[2]
        )

        control_body[1] = (
            -control_cam[0]
        )

        control_body[2] = (
            -control_cam[1]
        )

        control_body[3] = (
            control_cam[5]
        )

        control_body[4] = (
            -control_cam[3]
        )

        control_body[5] = (
            -control_cam[4]
        )

        # ============================================================
        # PD
        # ============================================================

        accel = 2.0 * (
            control_body
            - self.vel_curr_body
        )

        return accel


def main(args=None):

    np.set_printoptions(
        precision=3,
        suppress=True
    )

    rclpy.init(args=args)

    node = PDTrajectoryController()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
