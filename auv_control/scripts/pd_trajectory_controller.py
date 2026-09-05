#!/usr/bin/env python3

import numpy as np
import quaternion

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Accel


class PosePDTrajectoryController(Node):

    def __init__(self):
        super().__init__('pose_pd_controller')

        # ============================================================
        # Publishers
        # ============================================================

        self.control_pub = self.create_publisher(
            Accel,
            '/auv/desired/accel',
            10
        )

        # ============================================================
        # Subscribers
        # ============================================================

        self.sensor_odom_sub = self.create_subscription(
            Odometry,
            '/auv/true/odometry',
            self.odom_callback,
            10
        )

        self.desired_odom_sub = self.create_subscription(
            Odometry,
            '/auv/desired/odometry',
            self.desired_odom_callback,
            10
        )

        # ============================================================
        # Parameters
        # ============================================================

        self.declare_parameter(
            'control_rate',
            30.0
        )

        self.control_rate = self.get_parameter(
            'control_rate'
        ).value

        # ============================================================
        # Estado
        # ============================================================

        self.pose = None
        self.vel = None

        self.desired_pose = None
        self.desired_vel = None

        # ============================================================
        # Timer
        # ============================================================

        timer_period = 1.0 / self.control_rate

        self.timer = self.create_timer(
            timer_period,
            self.publish_sensor_data
        )

        self.get_logger().info(
            'Pose PD trajectory controller iniciado.'
        )

    # ================================================================
    # Control loop
    # ================================================================

    def publish_sensor_data(self):

        control = np.zeros(6)

        if (
            self.pose is not None
            and self.vel is not None
            and self.desired_pose is not None
            and self.desired_vel is not None
        ):

            try:
                control = self.PDcontrol()

            except Exception as e:
                self.get_logger().warn(
                    f'Erro no controlador PD: {e}'
                )

        # ============================================================
        # Publicação
        # ============================================================

        control_msg = Accel()

        control_msg.linear.x = control[0]
        control_msg.linear.y = control[1]
        control_msg.linear.z = control[2]

        control_msg.angular.x = control[3]
        control_msg.angular.y = control[4]
        control_msg.angular.z = control[5]

        self.control_pub.publish(
            control_msg
        )

    # ================================================================
    # Odometry callback
    # ================================================================

    def odom_callback(self, msg):
        """
        Atualiza pose e velocidade atuais do AUV.
        """

        self.pose = np.array(
            [
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z,

                msg.pose.pose.orientation.w,
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z
            ]
        )

        self.vel = np.array(
            [
                msg.twist.twist.linear.x,
                msg.twist.twist.linear.y,
                msg.twist.twist.linear.z,

                msg.twist.twist.angular.x,
                msg.twist.twist.angular.y,
                msg.twist.twist.angular.z
            ]
        )

    # ================================================================
    # Desired odometry callback
    # ================================================================

    def desired_odom_callback(self, msg):
        """
        Armazena a odometria de referência desejada.
        """

        self.desired_pose = np.array(
            [
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z,

                msg.pose.pose.orientation.w,
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z
            ]
        )

        self.desired_vel = np.array(
            [
                msg.twist.twist.linear.x,
                msg.twist.twist.linear.y,
                msg.twist.twist.linear.z,

                msg.twist.twist.angular.x,
                msg.twist.twist.angular.y,
                msg.twist.twist.angular.z
            ]
        )

    # ================================================================
    # PD Controller
    # ================================================================

    def PDcontrol(self):

        # ------------------------------------------------------------
        # Ganhos
        # ------------------------------------------------------------

        a = 3
        b = 3

        w = np.array(
            [a, a, a, b, b, b]
        )

        Kp = np.diag(
            w ** 2
        )

        Kd = np.diag(
            2 * w
        )

        # ------------------------------------------------------------
        # Quaternion atual
        # ------------------------------------------------------------

        q = np.quaternion(
            *self.pose[3:]
        )

        q_e_v = self.computeQuaternionError()

        R = self.Rotation(q)

        # ------------------------------------------------------------
        # Erro de posição
        # ------------------------------------------------------------

        position_error = (
            self.desired_pose[:3]
            - self.pose[:3]
        )

        # ------------------------------------------------------------
        # Erro de velocidade
        # ------------------------------------------------------------

        vel_error = (
            self.desired_vel
            - self.vel
        )

        vel_error[:3] = (
            R @ vel_error[:3]
        )

        vel_error[3:] = (
            R @ vel_error[3:]
        )

        # ------------------------------------------------------------
        # Ação de controle
        # ------------------------------------------------------------

        pose_error = np.block(
            [
                position_error,
                q_e_v
            ]
        )

        control = (
            Kp @ pose_error
            + Kd @ vel_error
        )

        # ------------------------------------------------------------
        # Rotação para o referencial móvel
        # ------------------------------------------------------------

        R_dot = (
            R @ self.S(self.vel[3:])
        )

        control[:3] = (
            R.T
            @ (
                control[:3]
                - R_dot @ self.vel[:3]
            )
        )

        control[3:] = (
            R.T @ control[3:]
        )

        self.get_logger().debug(
            f'Controle PD: {control}'
        )

        return control

    # ================================================================
    # Skew-symmetric matrix
    # ================================================================

    def S(self, v):

        return np.array(
            [
                [0,    -v[2],  v[1]],
                [v[2],  0,    -v[0]],
                [-v[1], v[0],  0]
            ]
        )

    # ================================================================
    # Rotation matrix
    # ================================================================

    def Rotation(self, q):

        return quaternion.as_rotation_matrix(q)

    # ================================================================
    # Rotate vector
    # ================================================================

    def rotateVector(self, v, q):

        v_quat = np.quaternion(
            0,
            *v
        )

        v_rotated_quat = (
            q
            * v_quat
            * q.conjugate()
        )

        v_rotated = np.array(
            [
                v_rotated_quat.x,
                v_rotated_quat.y,
                v_rotated_quat.z
            ]
        )

        return v_rotated

    # ================================================================
    # Quaternion error
    # ================================================================

    def computeQuaternionError(self):

        q = np.quaternion(
            *self.pose[3:]
        )

        q_d = np.quaternion(
            *self.desired_pose[3:]
        )

        if np.abs(q_d) != 0:
            q_d = q_d / np.abs(q_d)

        q_e = (
            q_d
            * q.conjugate()
        )

        if q_e.w < 0:
            q_e = -q_e

        q_e_v = np.array(
            [
                q_e.x,
                q_e.y,
                q_e.z
            ]
        )

        return q_e_v


def main(args=None):

    np.set_printoptions(
        precision=3,
        suppress=True
    )

    rclpy.init(args=args)

    node = PosePDTrajectoryController()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
