#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import json
from std_msgs.msg import String, Float32
from sensor_msgs.msg import Imu
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
        self.create_subscrition(
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
        self.create_subscription(
            PoseStamped, "/mavros/local_position/pose", self.local_position_cb, 10
        )

        self.gui_pub = self.create_publisher(String, "/rov/gui_data", 10)

        self.create_subscription(String, "/rov/joystick_cmd", self.joystick_cmd_cb, 10)

        self.cmd_vel_pub = self.create_publisher(Twist, "/auv/desired/vel", 10)
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

    def local_position_cb(self, msg):
        self.data["local_position"] = {
            "position": {
                "x": msg.pose.position.x,
                "y": msg.pose.position.y,
                "z": msg.pose.position.z,
            },
            "orientation": {
                "x": msg.pose.orientation.x,
                "y": msg.pose.orientation.y,
                "z": msg.pose.orientation.z,
                "w": msg.pose.orientation.w,
            },
        }

    # --- Callback para processar comandos da GUI ---

    def joystick_cmd_cb(self, msg):
        """Analisa o comando JSON da GUI e publica uma mensagem Accel."""
        try:
            cmd_data = json.loads(msg.data)
            print("cmd_data", cmd_data)

            accel_msg = Accel()
            accel_msg.linear.x = float(cmd_data.get("linear_x", 0.0)) * 19.745
            accel_msg.linear.y = float(cmd_data.get("linear_y", 0.0)) * 19.745
            accel_msg.linear.z = float(cmd_data.get("linear_z", 0.0)) * 27.924
            accel_msg.angular.x = (float(cmd_data.get("angular_x", 0.0)) + 0.5) * 5.585
            accel_msg.angular.y = float(cmd_data.get("angular_y", 0.0)) * 13.962
            accel_msg.angular.z = float(cmd_data.get("angular_z", 0.0)) * 5.924

            self.cmd_accel_pub.publish(accel_msg)
            self.get_logger().info(
                f"Comando de aceleração publicado: linear_x={accel_msg.linear.x}, angular_z={accel_msg.angular.z}"
            )

        except json.JSONDecodeError:
            self.get_logger().warn(
                f"JSON malformado recebido em /rov/joystick_cmd: {msg.data}"
            )
        except Exception as e:
            self.get_logger().error(f"Erro ao processar comando do joystick: {e}")

        def timer_callback(self):
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
