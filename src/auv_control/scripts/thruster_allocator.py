#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import numpy as np
from mavros_msgs.msg import OverrideRCIn
from geometry_msgs.msg import Accel

class ThrusterAllocator(Node):
    """
    Nó ROS responsável por realizar a **alocação de empuxo** de um AUV com 7 motores.

    Este nó recebe o vetor de aceleração desejado (linear e angular) no referencial do corpo
    e converte em sinais PWM correspondentes aos motores do veículo, levando em conta
    a nova geometria do AUV (posições e direções exatas) e a limitação de força de cada propulsor.

    A permutação abaixo corrige o mapeamento entre canais de saída (código)
    e motores físicos montados no veículo, determinada empiricamente:

        Canal (código): 1  2  3  4  5  6  7
        Motor físico:   1  5  6  3  4  7  2

    Em 0-based: perm = [0, 4, 5, 2, 3, 6, 1]
    """

    def __init__(self):
        super().__init__('thruster_allocator')

        # ============================================================
        # Parâmetros ROS
        # ============================================================

        self.declare_parameter('thruster_pub_topic', '/mavros/rc/override')

        self.declare_parameter('accel_sub_topic', '/cmd_accel')

        self.declare_parameter('control_rate', 30.0)

        self.declare_parameter('thrust_constant', 0.4256)

        self.declare_parameter('pwm_gain', 8.1 / 1000.0)

        self.declare_parameter('pwm_offset', -8.1 * 1.5)

        self.declare_parameter('max_force', 6.980904)

        self.declare_parameter('pwm_min', 1100)

        self.declare_parameter('pwm_max', 1900)

        self.declare_parameter('pwm_neutral', 1500)

        self.declare_parameter('motor_directions', [1, 1, 1, 1, 1, 1, 1])

        # ============================================================
        # Recuperação dos parâmetros
        # ============================================================

        self.thruster_pub_topic = self.get_parameter('thruster_pub_topic').value

        self.accel_sub_topic = self.get_parameter('accel_sub_topic').value

        self.control_rate = self.get_parameter('control_rate').value

        self.k = self.get_parameter('thrust_constant').value

        self.a = self.get_parameter('pwm_gain').value

        self.b = self.get_parameter('pwm_offset').value

        self.max_force = self.get_parameter('max_force').value

        self.pwm_min = self.get_parameter('pwm_min').value

        self.pwm_max = self.get_parameter('pwm_max').value

        self.pwm_neutral = self.get_parameter('pwm_neutral').value

        self.motor_directions = np.array(
            self.get_parameter('motor_directions').value
        )

        # ============================================================
        # Publishers
        # ============================================================

        self.thruster_pub = self.create_publisher(
            OverrideRCIn,
            self.thruster_pub_topic,
            10
        )

        # ============================================================
        # Subscribers
        # ============================================================

        self.accel_sub = self.create_subscription(
            Accel,
            self.accel_sub_topic,
            self.accel_callback,
            10
        )

        # ============================================================
        # Timer
        # ============================================================

        # timer_period = 1.0 / self.control_rate

        # self.timer = self.create_timer(
        #     timer_period,
        #     self.timer_callback
        # )

        # ============================================================
        # Matriz de alocação
        # ============================================================

        self._build_allocation_matrix()

        self.get_logger().info(
            'Nó inicializado com sucesso para 7 motores.'
        )

    # ================================================================
    # Callbacks
    # ================================================================

    def _build_allocation_matrix(self):
        """
        Constrói a matriz de alocação (6x7) que relaciona as forças e torques
        do corpo com os esforços individuais dos 7 motores baseados nas posições (T)
        e direções (D).
        """

        # Posição dos motores
        pos_T = np.array(
            [
                [-0.301500, 0.000000,-0.029229],    # 1
                [ 0.301500, 0.166500,-0.029229],    # 2
                [ 0.301500,-0.166500,-0.029229],    # 3
                [-0.265214, 0.166286,-0.113000],    # 4
                [-0.265214,-0.166286,-0.113000],    # 5
                [ 0.192214, 0.161286,-0.113000],    # 6
                [ 0.192214,-0.161286,-0.113000],    # 7
            ]
        )

        # Direção dos motores
        dir_D = np.array(
            [
                [ 0, 0,-1],    # 1
                [ 0, 0,-1],    # 2
                [ 0, 0,-1],    # 3
                [-1, 1, 0],    # 4
                [-1,-1, 0],    # 5
                [ 1, 1, 0],    # 6
                [ 1,-1, 0],    # 7
            ],
            dtype=float,
        )

        # Aplica a inversão de rotação dos motores via software
        dir_D = dir_D * self.motor_directions[:, np.newaxis]
        norms = np.linalg.norm(dir_D, axis=1, keepdims=True)
        dir_D_norm = dir_D / norms

        R = pos_T.T
        F = dir_D_norm.T

        T = np.cross(pos_T, dir_D_norm).T

        M = np.vstack((F, T))
        self.inv_M = np.linalg.pinv(M)

    def accel_callback(self, msg):
        """
        Callback chamado ao receber acelerações desejadas.
        Converte o vetor de aceleração em forças de motor e envia o comando PWM.
        """
        accel = np.array(
            [
                msg.linear.x,
                msg.linear.y,
                msg.linear.z,
                msg.angular.x,
                msg.angular.y,
                msg.angular.z,
            ]
        )

        accel_max = np.array([19.745, 19.745, 27.924, 5.585, 13.962, 5.924])
        accel_weights = np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0])

        accel_clip = np.clip(accel, -accel_max, accel_max)
        accel_clip = accel_clip * accel_weights

        m = self.inv_M @ accel_clip

        max_m = np.max(np.abs(m))
        if max_m > self.max_force:
            m = m * (self.max_force / max_m)
        pwmT = self.force_to_pwm(m)

        
        motors_order = [0, 1, 2, 3, 4, 5, 6]
        pwm = []
        for i in motors_order:
            pwm.append(pwmT[i])

        motor_msg = OverrideRCIn()
        pwm_channels = np.zeros(8, dtype=np.uint16)
        pwm_channels[:7] = np.round(pwm).astype(np.uint16)
        motor_msg.channels[:8] = pwm_channels

        self.thruster_pub.publish(motor_msg)

        self.get_logger().debug(f"[ThrusterAllocator] PWM enviado: {np.round(pwm, 2)}")

    def force_to_pwm(self, forces):
        """
        Converte forças (N) em valores PWM, conforme o modelo empírico:

            F = k * ω² ,   ω = a * PWM + b

        Args:
            forces (np.ndarray): vetor (7,) de forças individuais dos motores [N]

        Returns:
            np.ndarray: vetor (7,) com valores PWM correspondentes
        """
        if self.k <= 0 or self.a == 0:
            return np.full_like(forces, self.pwm_neutral)
        pwm = (np.sign(forces) * np.sqrt(np.abs(forces) / self.k) - self.b) / self.a
        pwm_clipped = np.clip(pwm, self.pwm_min, self.pwm_max)
        return pwm_clipped




if __name__ == "__main__":
    rclpy.init(args=None)

    node = ThrusterAllocator()

    try:
        np.set_printoptions(precision=3, suppress=True)
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()

