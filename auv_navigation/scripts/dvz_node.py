#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import numpy as np
import quaternion
import tf2_ros

from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Twist

from sensor_msgs_py import point_cloud2

from dvz import DVZ


class DVZNode(Node):
    """
    Nó dedicado para evasão de obstáculos usando
    Zonas Virtuais Deformáveis (DVZ).
    """

    def __init__(self):
        super().__init__('dvz_node')

        # ============================================================
        # Parâmetros ROS 2
        # ============================================================

        self.declare_parameter(
            'control_rate',
            30.0
        )

        self.declare_parameter(
            'max_repulse',
            100.0
        )

        self.declare_parameter(
            'base_frame',
            'base_link'
        )

        self.declare_parameter(
            'c_min',
            [2.0, 1.0, 1.0]
        )

        self.declare_parameter(
            'lambda_c',
            [0.2, 0.2, 0.2]
        )

        self.declare_parameter(
            'r0',
            [0.0, 0.0, 0.0]
        )

        self.declare_parameter(
            'k_gain',
            6.0
        )

        # ============================================================
        # Recuperação dos parâmetros
        # ============================================================

        self.control_rate = self.get_parameter(
            'control_rate'
        ).value

        self.max_repulse = self.get_parameter(
            'max_repulse'
        ).value

        self.base_frame = self.get_parameter(
            'base_frame'
        ).value

        c_min = self.get_parameter(
            'c_min'
        ).value

        lambda_c = self.get_parameter(
            'lambda_c'
        ).value

        r0 = self.get_parameter(
            'r0'
        ).value

        k_gain = self.get_parameter(
            'k_gain'
        ).value

        # ============================================================
        # Inicialização do DVZ
        # ============================================================

        self.dvz = DVZ(
            c_min=c_min,
            lambda_c=lambda_c,
            r0=r0,
            k_gain=k_gain
        )

        # ============================================================
        # TF2
        # ============================================================

        self.tf_buffer = tf2_ros.Buffer()

        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer,
            self
        )

        # ============================================================
        # Publisher
        # ============================================================

        self.dvz_pub = self.create_publisher(
            Twist,
            '/dvz_cmd_vel',
            1
        )

        # ============================================================
        # Subscribers
        # ============================================================

        self.odom_sub = self.create_subscription(
            Odometry,
            '/ground_truth/odom',
            self.odom_callback,
            10
        )

        self.pointcloud_sub = self.create_subscription(
            PointCloud2,
            '/camera/depth/points',
            self.pointcloud_callback,
            1
        )

        # ============================================================
        # Estados
        # ============================================================

        self.current_twist_body = None

        self.latest_cloud = None

        self.new_pc = False

        # ============================================================
        # Timer
        # ============================================================

        timer_period = 1.0 / self.control_rate

        self.timer = self.create_timer(
            timer_period,
            self.control_loop
        )

        self.get_logger().info(
            'Nó DVZ iniciado.'
        )

    # ================================================================
    # Odometry
    # ================================================================

    def odom_callback(self, msg):
        """
        Atualiza a velocidade linear atual no frame do corpo.
        """

        self.current_twist_body = np.array(
            [
                msg.twist.twist.linear.x,
                msg.twist.twist.linear.y,
                msg.twist.twist.linear.z
            ],
            dtype=float
        )

    # ================================================================
    # PointCloud2
    # ================================================================

    def pointcloud_callback(self, msg):
        """
        Recebe a nuvem de pontos, transforma para base_frame
        e armazena a nuvem mais recente.
        """

        try:

            # --------------------------------------------------------
            # Obtém transformação:
            #
            # base_frame <- frame da câmera
            # --------------------------------------------------------

            trans = self.tf_buffer.lookup_transform(
                self.base_frame,
                msg.header.frame_id,
                msg.header.stamp
            )

            # --------------------------------------------------------
            # Extrai os pontos da PointCloud2
            # --------------------------------------------------------

            points = point_cloud2.read_points(
                msg,
                field_names=('x', 'y', 'z'),
                skip_nans=True
            )

            raw_points = np.array(
                list(points),
                dtype=np.float32
            )

            if len(raw_points) == 0:
                return

            # --------------------------------------------------------
            # Quaternion da transformação
            # --------------------------------------------------------

            q_tf = np.quaternion(
                trans.transform.rotation.w,
                trans.transform.rotation.x,
                trans.transform.rotation.y,
                trans.transform.rotation.z
            )

            R_tf = quaternion.as_rotation_matrix(
                q_tf
            )

            # --------------------------------------------------------
            # Translação
            # --------------------------------------------------------

            t_tf = np.array(
                [
                    trans.transform.translation.x,
                    trans.transform.translation.y,
                    trans.transform.translation.z
                ],
                dtype=float
            )

            # --------------------------------------------------------
            # Transformação dos pontos
            # --------------------------------------------------------

            cloud_transformed = (
                raw_points @ R_tf.T
                + t_tf
            )

            # --------------------------------------------------------
            # Salva no buffer
            # --------------------------------------------------------

            if len(cloud_transformed) > 0:

                self.latest_cloud = cloud_transformed

                self.new_pc = True

        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException
        ) as e:

            self.get_logger().debug(
                f'Não foi possível transformar PointCloud2: {e}'
            )

        except Exception as e:

            self.get_logger().warn(
                f'Erro processando PointCloud2: {e}'
            )

    # ================================================================
    # Control loop
    # ================================================================

    def control_loop(self):
        """
        Loop principal do DVZ.

        A sequência é:

        1. Atualizar nuvem;
        2. Atualizar velocidade;
        3. Calcular velocidade repulsiva;
        4. Aplicar saturação;
        5. Publicar comando.
        """

        if self.current_twist_body is None:
            return

        # ============================================================
        # 1. Atualiza a nuvem
        # ============================================================

        if self.new_pc and self.latest_cloud is not None:

            self.dvz.update_cloud(
                self.latest_cloud
            )

            self.new_pc = False

        # ============================================================
        # 2. Atualiza velocidade
        # ============================================================

        self.dvz.update_speed(
            self.current_twist_body
        )

        # ============================================================
        # 3. Obtém velocidade de controle
        # ============================================================

        v_zvd_b, omega_zvd_b = (
            self.dvz.get_control_velocities()
        )

        # ============================================================
        # 4. Saturação de segurança
        # ============================================================

        norm_v = np.linalg.norm(
            v_zvd_b
        )

        if norm_v > self.max_repulse:

            v_zvd_b = (
                v_zvd_b / norm_v
            ) * self.max_repulse

        # ============================================================
        # 5. Monta mensagem
        # ============================================================

        cmd_msg = Twist()

        cmd_msg.linear.x = float(
            v_zvd_b[0]
        )

        cmd_msg.linear.y = float(
            v_zvd_b[1]
        )

        cmd_msg.linear.z = float(
            v_zvd_b[2]
        )

        # Atualmente o DVZ só gera repulsão linear.
        cmd_msg.angular.x = 0.0
        cmd_msg.angular.y = 0.0
        cmd_msg.angular.z = 0.0

        self.dvz_pub.publish(
            cmd_msg
        )


def main(args=None):

    rclpy.init(args=args)

    node = DVZNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
