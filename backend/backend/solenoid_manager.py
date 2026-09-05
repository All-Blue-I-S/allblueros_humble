#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from mavros_msgs.srv import CommandLong  # Changed from .msg to .srv


class SolenoidManager(Node):
    def __init__(self, service_timeout=5.0):
        super().__init__("solenoid_manager")
        self.service_name = "/mavros/cmd/command"

        self.solenoids = {
            "solenoid1": {"pin": 0, "active": False},
            "solenoid2": {"pin": 1, "active": False},
            "solenoid3": {"pin": 2, "active": False},
            "solenoid4": {"pin": 3, "active": False},
        }

        # Em ROS 2, parâmetros precisam ser declarados primeiro
        self.declare_parameter("fire_duration", 3.0)
        self.fire_duration = self.get_parameter("fire_duration").value

        # Criação do cliente de serviço
        self.command_service = self.create_client(CommandLong, self.service_name)

        # Aguarda o serviço ficar online
        self.get_logger().info(f"Aguardando serviço {self.service_name}...")
        ready = self.command_service.wait_for_service(timeout_sec=service_timeout)
        if not ready:
            self.get_logger().error(
                f"Service {self.service_name} not available after {service_timeout} seconds."
            )
            self.command_service = None
        else:
            self.get_logger().info("Serviço de comandos conectado com sucesso.")

    def set_relay(self, solenoid, state):
        if self.command_service is None or not self.command_service.wait_for_service(
            timeout_sec=1.0
        ):
            self.get_logger().error("Command service is not available.")
            return False

        if solenoid not in self.solenoids:
            self.get_logger().error(f"Solenóide não reconhecido: {solenoid}")
            return False

        # Prepara a requisição do serviço
        req = CommandLong.Request()
        req.command = 182  # MAV_CMD_DO_SET_RELAY

        # O ROS 2 é rigoroso com tipagem, garantindo que os valores sejam convertidos para float
        req.param1 = float(self.solenoids[solenoid]["pin"])
        req.param2 = float(self.fire_duration) if state else 0.0
        req.param3 = 0.5
        req.param4 = 0.0
        req.param5 = 0.0
        req.param6 = 0.0
        req.param7 = 0.0

        # Faz a chamada de forma assíncrona para não travar o rclpy.spin()
        future = self.command_service.call_async(req)

        # Adiciona um callback para tratar o resultado assim que o serviço responder
        future.add_done_callback(self.service_result_callback)
        return True

    def service_result_callback(self, future):
        """Callback acionado quando o serviço CommandLong retorna uma resposta"""
        try:
            response = future.result()
            self.get_logger().info(
                f"Service call successful. Result/Success: {response.success}"
            )
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")


def main(args=None):
    rclpy.init(args=args)
    manager = SolenoidManager()

    try:
        # Mantém o nó vivo e processando callbacks
        rclpy.spin(manager)
    except KeyboardInterrupt:
        manager.get_logger().info("ROS Interrupt Exception! Shutting down the node.")
    finally:
        manager.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
