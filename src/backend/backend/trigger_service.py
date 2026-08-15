#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import subprocess
from mavros_msgs.srv import CommandBool


class TriggerService(Node):
    def __init__(self):
        super().__init__("trigger_service")
        self.service_name = "/mavros/cmd/arming"

        # 1. Create the client first
        self.arm_service = self.create_client(CommandBool, self.service_name)

    def arm_and_configure(self):
        # 2. Use self.get_logger() instead of rclpy.get_logger()
        self.get_logger().info(f"Waiting for service {self.service_name}...")

        # 3. Call wait_for_service on the client object itself
        self.arm_service.wait_for_service()
        self.get_logger().info(f"Service {self.service_name} is available.")

        # Prepare the request
        req = CommandBool.Request()
        req.value = True  # Set to True to arm the vehicle

        # 4. Use call_async() and spin to wait for the response
        future = self.arm_service.call_async(req)

        # This keeps the node spinning until the service returns a result
        rclpy.spin_until_future_complete(self, future)

        try:
            # Retrieve the response from the future
            resp = future.result()

            if resp.success:
                self.get_logger().info("Vehicle armed successfully.")

                # Subprocess call (works fine for CLI tool execution)
                try:
                    subprocess.run(
                        ["ros2", "run", "mavros", "mavsys", "rate", "--all", "10"],
                        check=True,
                    )
                    self.get_logger().info("MAVROS rate set to 10 Hz successfully.")
                except subprocess.CalledProcessError as e:
                    self.get_logger().error(f"Failed to set MAVROS rate: {e}")
            else:
                self.get_logger().error("Failed to arm the vehicle.")

        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")


def main(args=None):
    rclpy.init(args=args)
    trigger_service = TriggerService()

    try:
        trigger_service.arm_and_configure()
    except Exception as e:
        trigger_service.get_logger().error(f"Error during execution: {e}")
    finally:
        trigger_service.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
