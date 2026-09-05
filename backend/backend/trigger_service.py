#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from mavros_msgs.srv import CommandBool, StreamRate


class TriggerService(Node):
    def __init__(self):
        super().__init__("trigger_service")
        self.arm_service = "/mavros/cmd/arming"
        self.rate_service = "/mavros/set_stream_rate"

        # 1. Create the client first
        self.arm_service = self.create_client(CommandBool, self.arm_service)
        self.rate_service = self.create_client(StreamRate, self.rate_service)

    def arm_and_configure(self):
        # 2. Use self.get_logger() instead of rclpy.get_logger()
        self.get_logger().info(f"Waiting for service {self.arm_service}...")

        # 3. Call wait_for_service on the client object itself
        self.arm_service.wait_for_service()
        self.get_logger().info(f"Service {self.rate_service} is available.")

        # Prepare the request
        req = CommandBool.Request()
        req.value = True  # Set to True to arm the vehicle

        # 4. Use call_async() and spin to wait for the response
        future = self.arm_service.call_async(req)

        # This keeps the node spinning until the service returns a result
        rclpy.spin_until_future_complete(self, future)

    def configure_stream_rate(self):
        self.get_logger().info(f"Waiting for service {self.rate_service}...")

        self.rate_service.wait_for_service()
        self.get_logger().info(f"Service {self.rate_service} is available.")

        req = StreamRate.Request()
        req.stream_id = 0
        req.message_rate = 10
        req.on_off = True

        future = self.rate_service.call_async(req)

        rclpy.spin_until_future_complete(self, future)


def main(args=None):
    rclpy.init(args=args)
    trigger_service = TriggerService()

    try:
        trigger_service.arm_and_configure()
        trigger_service.configure_stream_rate()
    except Exception as e:
        trigger_service.get_logger().error(f"Error during execution: {e}")
    finally:
        trigger_service.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
