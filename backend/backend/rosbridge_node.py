#!/usr/bin/env python3
import rclpy
from numpy import interp as it
from rclpy.node import Node
import json
from std_msgs.msg import String, Float32
from sensor_msgs.msg import Imu, NavSatFix
from geometry_msgs.msg import Twist, Accel


class RosbridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("rosbridge_node")
        self.data = {
            "pressure": None,
            "temperature": None,
            "battery": None,
            "imu": None,
            "gps": None,
            "local_position": None,
        }
        self.create_subscription(
            Float32, "/mavros/imu/atm_pressure", self.pressure_cb, 10
        )
        self.create_subscription(
            Float32, "/mavros/imu/temperature_imu", self.temperature_cb, 10
        )
        self.create_subscription(Float32, "/mavros/battery", self.battery_cb, 10)
        self.create_subscription(Imu, "/mavros/imu/data", self.imu_cb, 10)
        self.create_subscription(
            NavSatFix, "/mavros/global_position/global", self.gps_cb, 10
        )
        self.gui_pub = self.create_publisher(String, "/rov/gui_data", 10)

        self.create_subscription(String, "/rov/joystick_cmd", self.joystick_cmd_cb, 10)

        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.cmd_accel_pub = self.create_publisher(Accel, "/auv/desired/accel", 10)

        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info("Rosbridge node started.")

    def pressure_cb(self, msg):
        self.data["pressure"] = msg.data

    def temperature_cb(self, msg):
        self.data["temperature"] = msg.data

    def battery_cb(self, msg):
        self.data["battery"] = msg.data

    def imu_cb(self, msg):
        self.data["imu"] = {
            "orientation": {
                "x": msg.orientation.x,
                "y": msg.orientation.y,
                "z": msg.orientation.z,
                "w": msg.orientation.w,
            },
            "angular_velocity": {
                "x": msg.angular_velocity.x,
                "y": msg.angular_velocity.y,
                "z": msg.angular_velocity.z,
            },
            "linear_acceleration": {
                "x": msg.linear_acceleration.x,
                "y": msg.linear_acceleration.y,
                "z": msg.linear_acceleration.z,
            },
        }

    def gps_cb(self, msg):
        self.data["gps"] = {
            "latitude": msg.latitude,
            "longitude": msg.longitude,
            "altitude": msg.altitude,
            "status": msg.status.status,
        }

    # --- Callback para processar comandos da GUI ---

    def joystick_cmd_cb(self, msg):
        """Analisa o comando JSON da GUI e publica uma mensagem Twist (ou Accel)."""
        try:
            cmd_data = json.loads(msg.data)
            print("cmd_data", cmd_data)

            # Função auxiliar para mapear de [-5, 5] para [-1.0, 1.0]
            def map_axis(key):
                raw_val = float(cmd_data.get(key, 0.0))
                return float(it(raw_val, [-5.0, 5.0], [-1.0, 1.0]))

            # Valores normalizados no intervalo [-1.0, 1.0]
            lx = map_axis("linear_x")
            ly = map_axis("linear_y")
            lz = map_axis("linear_z")
            ax = map_axis("angular_x")
            ay = map_axis("angular_y")
            az = map_axis("angular_z")

            # --- Accel Msg (se for utilizar) ---
            accel_msg = Accel()
            accel_msg.linear.x = lx * 19.745
            accel_msg.linear.y = ly * 19.745
            accel_msg.linear.z = lz * 27.924
            accel_msg.angular.x = (ax + 0.5) * 5.585
            accel_msg.angular.y = ay * 13.962
            accel_msg.angular.z = az * 5.924

            # self.cmd_accel_pub.publish(accel_msg)
            # self.get_logger().info(
            #     f"Comando de aceleração publicado: linear_x={accel_msg.linear.x}, angular_z={accel_msg.angular.z}"
            # )

            # --- Twist Msg ---
            vel_msg = Twist()
            vel_msg.linear.x = lx
            vel_msg.linear.y = ly
            vel_msg.linear.z = lz
            vel_msg.angular.x = ax
            vel_msg.angular.y = ay
            vel_msg.angular.z = az

            self.cmd_vel_pub.publish(vel_msg)
            self.get_logger().info(
                f"Comando de velocidade publicado: linear_x={vel_msg.linear.x}, angular_z={vel_msg.angular.z}"
            )
        except Exception as e:
            self.get_logger().error(f"Erro ao processar comando do joystick: {e}")

    def timer_callback(self):
        msg = String()
        json_data = json.dumps(self.data)
        msg.data = json_data
        self.gui_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RosbridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
