#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import numpy as np
import quaternion

from nav_msgs.msg import Odometry
from auv_navigation.msg import CurveReference
from geometry_msgs.msg import Accel, Twist


class DoubleIntegratorController(Node):
    """
    Controlador para modelo de Duplo Integrador em 6 DOF.

    Lê:
        - odometria atual;
        - referência de trajetória;
        - velocidade repulsiva do DVZ.

    E publica:
        - aceleração desejada em /cmd_accel/go2.
    """

    def __init__(self):
        super().__init__('double_integrator_controller')

        # ============================================================
        # Parâmetros ROS 2
        # ============================================================

        self.declare_parameter(
            'kp',
            [9.0, 9.0, 9.0, 8.0, 8.0, 8.0]
        )

        self.declare_parameter(
            'kd',
            [6.0, 6.0, 6.0, 4.0, 4.0, 4.0]
        )

        self.declare_parameter(
            'use_body_frame',
            True
        )

        self.declare_parameter(
            'control_rate',
            30.0
        )

        # Recuperação dos parâmetros

        self.kp = np.array(
            self.get_parameter('kp').value,
            dtype=float
        )

        self.kd = np.array(
            self.get_parameter('kd').value,
            dtype=float
        )

        self.use_body_frame = self.get_parameter(
            'use_body_frame'
        ).value

        self.control_rate = self.get_parameter(
            'control_rate'
        ).value

        # Verificação básica

        if self.kp.shape != (6,):
            raise ValueError(
                'O parâmetro kp deve possuir 6 elementos.'
            )

        if self.kd.shape != (6,):
            raise ValueError(
                'O parâmetro kd deve possuir 6 elementos.'
            )

        # ============================================================
        # Publishers
        # ============================================================

        self.accel_pub = self.create_publisher(
            Accel,
            '/cmd_accel/go2',
            10
        )

        # ============================================================
        # Subscribers
        # ============================================================

        self.current_odom_sub = self.create_subscription(
            Odometry,
            '/ground_truth/odom',
            self.current_odom_callback,
            10
        )

        self.desired_odom_sub = self.create_subscription(
            CurveReference,
            '/dvz_reference',
            self.desired_odom_callback,
            10
        )

        self.dvz_sub = self.create_subscription(
            Twist,
            '/dvz_cmd_vel',
            self.dvz_callback,
            10
        )

        # ============================================================
        # Estados
        # ============================================================

        self.current_odom = None
        self.desired_odom = None

        # Velocidade repulsiva do DVZ
        self.dvz_vel_body = np.zeros(6)

        # ============================================================
        # Timer
        # ============================================================

        timer_period = 1.0 / self.control_rate

        self.timer = self.create_timer(
            timer_period,
            self.control_loop
        )

        self.get_logger().info(
            'Double Integrator Controller iniciado.'
        )

    # ================================================================
    # Callbacks
    # ================================================================

    def current_odom_callback(self, msg):
        """Atualiza a odometria atual."""

        self.current_odom = msg

    def desired_odom_callback(self, msg):
        """Atualiza a referência desejada."""

        self.desired_odom = msg

    def dvz_callback(self, msg):
        """
        Atualiza a velocidade repulsiva gerada pelo nó do DVZ.
        """

        self.dvz_vel_body = np.array(
            [
                msg.linear.x,
                msg.linear.y,
                msg.linear.z,
                msg.angular.x,
                msg.angular.y,
                msg.angular.z
            ],
            dtype=float
        )

    # ================================================================
    # Control loop
    # ================================================================

    def control_loop(self):
        """
        Loop principal do controlador.

        Em ROS 1 era implementado com rospy.Rate.
        Em ROS 2 usamos um timer.
        """

        if (
            self.current_odom is None
            or self.desired_odom is None
        ):
            return

        try:
            accel_cmd = self.compute_control()

            accel_msg = Accel()

            accel_msg.linear.x = float(accel_cmd[0])
            accel_msg.linear.y = float(accel_cmd[1])
            accel_msg.linear.z = float(accel_cmd[2])

            accel_msg.angular.x = float(accel_cmd[3])
            accel_msg.angular.y = float(accel_cmd[4])
            accel_msg.angular.z = float(accel_cmd[5])

            self.accel_pub.publish(accel_msg)

        except Exception as e:
            self.get_logger().warn(
                f'Erro no cálculo do controle: {e}'
            )

    # ================================================================
    # Matemática
    # ================================================================

    def S(self, v):
        """
        Matriz antissimétrica para produto vetorial.
        """

        return np.array(
            [
                [0.0, -v[2], v[1]],
                [v[2], 0.0, -v[0]],
                [-v[1], v[0], 0.0]
            ]
        )

    def compute_control(self):
        # ============================================================
        # Estado atual
        # ============================================================

        pos = np.array(
            [
                self.current_odom.pose.pose.position.x,
                self.current_odom.pose.pose.position.y,
                self.current_odom.pose.pose.position.z
            ],
            dtype=float
        )

        q = np.quaternion(
            self.current_odom.pose.pose.orientation.w,
            self.current_odom.pose.pose.orientation.x,
            self.current_odom.pose.pose.orientation.y,
            self.current_odom.pose.pose.orientation.z
        )

        v_curr = np.array(
            [
                self.current_odom.twist.twist.linear.x,
                self.current_odom.twist.twist.linear.y,
                self.current_odom.twist.twist.linear.z,
                self.current_odom.twist.twist.angular.x,
                self.current_odom.twist.twist.angular.y,
                self.current_odom.twist.twist.angular.z
            ],
            dtype=float
        )

        # ============================================================
        # Referência desejada
        # ============================================================

        pos_ref = np.array(
            [
                self.desired_odom.pose.position.x,
                self.desired_odom.pose.position.y,
                self.desired_odom.pose.position.z
            ],
            dtype=float
        )

        q_ref = np.quaternion(
            self.desired_odom.pose.orientation.w,
            self.desired_odom.pose.orientation.x,
            self.desired_odom.pose.orientation.y,
            self.desired_odom.pose.orientation.z
        )

        v_ref = np.array(
            [
                self.desired_odom.twist.linear.x,
                self.desired_odom.twist.linear.y,
                self.desired_odom.twist.linear.z,
                self.desired_odom.twist.angular.x,
                self.desired_odom.twist.angular.y,
                self.desired_odom.twist.angular.z
            ],
            dtype=float
        )

        a_ref = np.array(
            [
                self.desired_odom.accel.linear.x,
                self.desired_odom.accel.linear.y,
                self.desired_odom.accel.linear.z,
                self.desired_odom.accel.angular.x,
                self.desired_odom.accel.angular.y,
                self.desired_odom.accel.angular.z
            ],
            dtype=float
        )

        # ============================================================
        # Velocidade no referencial inercial
        # ============================================================

        R = quaternion.as_rotation_matrix(q)

        v_curr_inertial = v_curr.copy()

        v_curr_inertial[:3] = (
            R @ v_curr[:3]
        )

        v_curr_inertial[3:] = (
            R @ v_curr[3:]
        )

        # ============================================================
        # Referência de velocidade do DVZ
        # ============================================================

        v_dvz_inertial = np.zeros(6)

        v_dvz_inertial[:3] = (
            R @ self.dvz_vel_body[:3]
        )

        v_ref_modificada = (
            v_ref + v_dvz_inertial
        )

        # ============================================================
        # Erro de posição
        # ============================================================

        pos_error = pos_ref - pos

        # ============================================================
        # Erro de orientação
        # ============================================================

        q_e_v = self.compute_quaternion_error(
            q,
            q_ref
        )

        pose_error = np.hstack(
            (
                pos_error,
                q_e_v
            )
        )

        # ============================================================
        # Erro de velocidade
        # ============================================================

        vel_error = (
            v_ref_modificada - v_curr_inertial
        )

        # ============================================================
        # Lei de controle
        # ============================================================

        an = (
            self.kp * pose_error
            + self.kd * vel_error
            + a_ref
        )

        # ============================================================
        # Transformação para frame local
        # ============================================================

        if self.use_body_frame:

            a_local = np.zeros(6)

            a_local[:3] = (
                R.T @ an[:3]
            )

            a_local[3:] = (
                R.T @ an[3:]
            )

            return a_local

        return an

    # ================================================================
    # Quaternion
    # ================================================================

    def compute_quaternion_error(self, q, q_ref):
        """
        Retorna o vetor de erro entre a orientação atual
        e a orientação desejada.
        """

        if np.abs(q_ref) != 0:
            q_ref = q_ref / np.abs(q_ref)

        q_e = q_ref * q.conjugate()

        if q_e.w < 0:
            q_e = -q_e

        return np.array(
            [
                q_e.x,
                q_e.y,
                q_e.z
            ],
            dtype=float
        )


def main(args=None):

    rclpy.init(args=args)

    controller = DoubleIntegratorController()

    try:
        rclpy.spin(controller)

    except KeyboardInterrupt:
        pass

    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
