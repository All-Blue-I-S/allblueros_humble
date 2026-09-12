#!/usr/bin/env python3
import time
from collections import deque
import rclpy
from rclpy.node import Node

import board
from adafruit_bme280 import basic as adafruit_bme280

from sensor_msgs.msg import FluidPressure, Temperature, RelativeHumidity


class BarometerNode(Node):
    def __init__(self):
        super().__init__('barometer_node')

        # Publishers for sensor topics (topic names can be changed later)
        self.pub_pressure = self.create_publisher(FluidPressure, 'bme280/pressure', 10)
        self.pub_temp = self.create_publisher(Temperature, 'bme280/temperature', 10)
        self.pub_humidity = self.create_publisher(RelativeHumidity, 'bme280/humidity', 10)

        # Hardware Setup (I2C)
        self.i2c_address = 0x76
        try:
            i2c = board.I2C()
            self.bme280 = adafruit_bme280.Adafruit_BME280_I2C(i2c, address=self.i2c_address)
        except ValueError:
            self.get_logger().error(f"Critical Error: BME280 not found on I2C address {hex(self.i2c_address)}.")
            raise SystemExit(1)

        # Thermodynamic Filter Parameters (Rolling Window)
        self.update_rate_sec = 0.5  # 2Hz
        self.window_analysis_sec = 3.0

        self.buffer_size = int(self.window_analysis_sec / self.update_rate_sec)
        self.hist_T = deque(maxlen=self.buffer_size)
        self.hist_P = deque(maxlen=self.buffer_size)
        self.hist_H = deque(maxlen=self.buffer_size)

        # Timer loop matching the update rate
        self.timer = self.create_timer(self.update_rate_sec, self.timer_callback)
        self.get_logger().info("Barometer node running and publishing measurements...")

    def timer_callback(self):
        try:
            # Physical Reading
            T_atual = self.bme280.temperature
            P_hpa = self.bme280.pressure  # BME280 returns hPa
            H_atual = self.bme280.relative_humidity

            delta_T, delta_P, delta_H = 0.0, 0.0, 0.0

            # Thermodynamic Filter Calculation (Delta over window)
            if len(self.hist_T) == self.buffer_size:
                delta_T = T_atual - self.hist_T[0]
                delta_P = P_hpa - self.hist_P[0]
                delta_H = H_atual - self.hist_H[0]

            # Update moving window buffers
            self.hist_T.append(T_atual)
            self.hist_P.append(P_hpa)
            self.hist_H.append(H_atual)

            # Publish Messages
            now = self.get_clock().now().to_msg()

            # FluidPressure expects Pascals (Pa). Convert hPa to Pa (1 hPa = 100 Pa)
            msg_p = FluidPressure()
            msg_p.header.stamp = now
            msg_p.header.frame_id = 'bme280_link'
            msg_p.fluid_pressure = P_hpa * 100.0
            msg_p.variance = 0.0

            # Temperature expects Celsius
            msg_t = Temperature()
            msg_t.header.stamp = now
            msg_t.header.frame_id = 'bme280_link'
            msg_t.temperature = T_atual
            msg_t.variance = 0.0

            # RelativeHumidity expects ratio (0.0 to 1.0)
            msg_h = RelativeHumidity()
            msg_h.header.stamp = now
            msg_h.header.frame_id = 'bme280_link'
            msg_h.relative_humidity = H_atual / 100.0
            msg_h.variance = 0.0

            self.pub_pressure.publish(msg_p)
            self.pub_temp.publish(msg_t)
            self.pub_humidity.publish(msg_h)

            # Terminal log with filtering deltas
            self.get_logger().info(
                f"T: {T_atual:5.1f}C (dT:{delta_T:+5.1f}) | "
                f"P: {P_hpa:6.1f}hPa (dP:{delta_P:+5.1f}) | "
                f"H: {H_atual:4.1f}% (dH:{delta_H:+4.1f})"
            )

        except Exception as e:
            self.get_logger().warn(f"I2C read skipped due to error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = BarometerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()