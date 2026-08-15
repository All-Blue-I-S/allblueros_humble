#!/usr/bin/env python3

import os
import json
import time
import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor

from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, Int32MultiArray
from cv_bridge import CvBridge

# TODO: confirm that in ROS2 your actions are generated in package `bt_qualify`
# and that the action classes are named TrackColor, Align, Config.
# If they live in another package or have different names, adjust the import below.
try:
    from bt_qualify.action import TrackColor, Align, Config
except Exception:
    # If actions are in a different package, edit the import above.
    TrackColor = Align = Config = None


class IdentificaCor(Node):
    def __init__(self):
        super().__init__('identifica_cor')
        self.get_logger().info('Nó de detecção de cor (Action Server) iniciado no modo IDLE')

        self.bridge = CvBridge()

        # ROS2 parameter (declare then read)
        self.declare_parameter('show_windows', True)
        self.show_windows = self.get_parameter('show_windows').value

        # --- Configurações de Referências e Cores ---
        self.ref_red = [95.0, 90.0, 545.0, 90.0, 545.0, 390.0, 95.0, 390.0]
        self.ref_black = [95.0, 90.0, 545.0, 90.0, 545.0, 390.0, 95.0, 390.0]
        self.ref_purple = [270.0, 40.0, 370.0, 40.0, 370.0, 440.0, 270.0, 440.0]

        # Filtros HSV
        self.hsv_ranges_red = [
            (np.array([0, 100, 100]), np.array([10, 255, 255])),
            (np.array([160, 100, 100]), np.array([179, 255, 255])),
        ]
        self.hsv_ranges_black = [(np.array([0, 0, 0]), np.array([179, 255, 15]))]
        self.hsv_ranges_purple = [(np.array([125, 50, 50]), np.array([150, 255, 255]))]

        # Estado inicial
        self.current_color = "IDLE"
        self.active_refs = []
        self.active_hsv = []
        self.ref_points = []
        self.last_detected_pts = None

        self.cam = "FRONT"

        # Publishers / Subscribers
        self.pub_referencia = self.create_publisher(Int32MultiArray, '/auv/image/features/detected', 10)
        self.pub_pontos_ref = self.create_publisher(Float32MultiArray, '/auv/image/features/desired', 1)
        self.image_sub = self.create_subscription(Image, '/oak/rgb/image_raw', self._process_image, 10)

        # Load JSON references (used by align action)
        self.load_json_refs()

        # --- Action Servers (rclpy) ---
        if TrackColor is not None:
            self._track_action_server = ActionServer(
                self,
                TrackColor,
                'track_color',
                execute_callback=self.execute_track_color
            )
        else:
            self.get_logger().warn('TrackColor action not available; action server not started.')

        if Align is not None:
            self._align_action_server = ActionServer(
                self,
                Align,
                'align_action_server',
                execute_callback=self.execute_align_action
            )
        else:
            self.get_logger().warn('Align action not available; action server not started.')

        if Config is not None:
            self._config_action_server = ActionServer(
                self,
                Config,
                'config_server',
                execute_callback=self.execute_config_action
            )
        else:
            self.get_logger().warn('Config action not available; action server not started.')

    # ----------------- Action callbacks -----------------
    def execute_track_color(self, goal_handle):
        """
        ROS2 action execute callback for TrackColor.
        goal_handle.request has the goal fields (e.g. target_color).
        """
        goal = goal_handle.request
        self.get_logger().info(f"Ação Recebida! Alvo solicitado: {getattr(goal, 'target_color', 'UNKNOWN')}")

        # Handle IDLE explicit command
        if getattr(goal, 'target_color', '').upper() == "IDLE":
            self.current_color = "IDLE"
            self.active_refs = []
            self.active_hsv = []
            self.ref_points = []
            self.last_detected_pts = None
            self.get_logger().info("Visão em repouso. Retornando SUCCESS para o comando IDLE.")
            result = TrackColor.Result()
            # assuming action Result has field 'success' (adjust if different)
            setattr(result, 'success', True)
            goal_handle.succeed()
            return result

        # Map requested color to ranges/refs
        tc = getattr(goal, 'target_color', '').upper()
        if tc == "PURPLE":
            self.current_color = "PURPLE"
            self.active_refs = self.ref_purple
            self.active_hsv = self.hsv_ranges_purple
        elif tc == "RED":
            self.current_color = "RED"
            self.active_refs = self.ref_red
            self.active_hsv = self.hsv_ranges_red
        elif tc == "BLACK":
            self.current_color = "BLACK"
            self.active_refs = self.ref_black
            self.active_hsv = self.hsv_ranges_black
        else:
            self.get_logger().warn(f"Cor desconhecida solicitada: {getattr(goal, 'target_color', None)}. Abortando ação.")
            goal_handle.abort()
            return TrackColor.Result()

        self.ref_points = self._format_refs(self.active_refs)
        self.publicar_pontos_referencia()
        self.last_detected_pts = None

        # Loop de monitoramento do erro
        success = False
        error_threshold = 30.0  # Patamar aceitável
        last_detection_time = self.get_clock().now().nanoseconds / 1e9

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                self.get_logger().info("Ação Cancelada pela BT.")
                goal_handle.canceled()
                return TrackColor.Result()

            if self.last_detected_pts is not None:
                erro_total = np.sum(np.abs(self.last_detected_pts - np.array(self.active_refs)))
                # publish feedback
                feedback = TrackColor.Feedback()
                # assuming Feedback has field 'current_error'; adjust if different
                setattr(feedback, 'current_error', float(erro_total))
                goal_handle.publish_feedback(feedback)

                if erro_total < error_threshold:
                    self.get_logger().info(f"Alvo {self.current_color} alinhado! Erro estabilizado em: {erro_total:.2f}")
                    success = True
                    break

                last_detection_time = self.get_clock().now().nanoseconds / 1e9

            # timeout (not detected)
            if (self.get_clock().now().nanoseconds / 1e9 - last_detection_time) > 30.0:
                self.get_logger().info("Objeto não detectado (timeout).")
                goal_handle.abort()
                return TrackColor.Result()

            time.sleep(0.1)

        if success:
            result = TrackColor.Result()
            setattr(result, 'success', True)
            goal_handle.succeed()
            return result

        # fallback
        goal_handle.abort()
        return TrackColor.Result()

    def execute_align_action(self, goal_handle):
        """
        Align action: expects goal fields item_group, item, config, cam
        """
        goal = goal_handle.request
        group = getattr(goal, 'item_group', None)
        item = getattr(goal, 'item', None)
        config = getattr(goal, 'config', None)
        cam = getattr(goal, 'cam', None)

        if not hasattr(self, 'json_data'):
            self.get_logger().info(f"[{self.get_name()}] JSON de referencias não carregado.")
            goal_handle.abort()
            return Align.Result()

        if group not in self.json_data:
            self.get_logger().info(f"[{self.get_name()}]Grupo indicado não encontrado: {group}")
            goal_handle.abort()
            return Align.Result()

        if item not in self.json_data[group]:
            self.get_logger().info(f"[{self.get_name()}]Item indicado não encontrado: {group}:{item}")
            goal_handle.abort()
            return Align.Result()

        item_data = self.json_data[group][item]
        if config not in item_data:
            self.get_logger().info(f"[{self.get_name()}]Configuração indicada não encontrada: {group}:{item}:{config}")
            goal_handle.abort()
            return Align.Result()

        if item_data.get("camera") != cam:
            self.get_logger().info(f"[{self.get_name()}]Câmera passada pela action diverge: {group}:{item}")
            goal_handle.abort()
            return Align.Result()

        if item_data.get("tool") != "COLOR":
            self.get_logger().info(f"[{self.get_name()}]Erro na ferramenta utilizada para o item: {group}:{item}")
            goal_handle.abort()
            return Align.Result()

        self.cam = cam
        # assume HSV stored as lists of tuples/lists in json
        self.active_hsv = [(np.array(lower), np.array(upper)) for lower, upper in item_data["HSV"]]

        self.active_refs = item_data[config]["points"]
        self.ref_points = self._format_refs(self.active_refs)

        self.publicar_pontos_referencia()
        self.last_detected_pts = None

        # Loop de monitoramento do erro
        success = False
        error_threshold = 30.0
        last_detection_time = self.get_clock().now().nanoseconds / 1e9

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                self.get_logger().info(f"[{self.get_name()}]Ação Cancelada pela BT.")
                goal_handle.canceled()
                return Align.Result()

            if self.last_detected_pts is not None:
                erro_total = np.sum(np.abs(self.last_detected_pts - np.array(self.active_refs)))
                feedback = Align.Feedback()
                setattr(feedback, 'current_error', float(erro_total))
                goal_handle.publish_feedback(feedback)

                if erro_total < error_threshold:
                    self.get_logger().info(f"Alvo {self.current_color} alinhado! Erro estabilizado em: {erro_total:.2f}")
                    success = True
                    break

                last_detection_time = self.get_clock().now().nanoseconds / 1e9

            # timeout
            if (self.get_clock().now().nanoseconds / 1e9 - last_detection_time) > 30.0:
                self.get_logger().info(f"[{self.get_name()}]Objeto não detectado.")
                goal_handle.abort()
                return Align.Result()

            time.sleep(0.1)

        if success:
            result = Align.Result()
            setattr(result, 'success', True)
            goal_handle.succeed()
            return result

        goal_handle.abort()
        return Align.Result()

    def execute_config_action(self, goal_handle):
        # Placeholder: adapt to the actual Config action fields & behavior
        self.get_logger().info('Config action received (not implemented)')
        result = Config.Result()
        goal_handle.succeed()
        return result

    # ----------------- Image processing -----------------
    def _process_image(self, msg: Image):
        # unified dispatcher that replaces the earlier undefined process_image
        try:
            if self.cam == "FRONT":
                self.process_image_front(msg)
            elif self.cam == "BOTTOM":
                self.process_image_bottom(msg)
        except Exception as err:
            self.get_logger().error(f"Erro no processamento da imagem: {err}")

    def process_image_front(self, msg: Image):
        try:
            imagem = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.identificador(imagem)
        except Exception as err:
            self.get_logger().error(f"Erro no processamento da imagem (front): {err}")

    def process_image_bottom(self, msg: Image):
        try:
            imagem = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.identificador(imagem)
        except Exception as err:
            self.get_logger().error(f"Erro no processamento da imagem (bottom): {err}")

    # ----------------- Helpers -----------------
    def gerar_mascara(self, hsv):
        mascara_total = None
        for baixo, alto in self.active_hsv:
            mascara = cv2.inRange(hsv, baixo, alto)
            if mascara_total is None:
                mascara_total = mascara
            else:
                mascara_total = cv2.bitwise_or(mascara_total, mascara)
        kernel = np.ones((2, 2), np.uint8)
        return cv2.dilate(mascara_total, kernel, iterations=2)

    def desenhar_pontos_referencia(self, imagem):
        for x, y in self.ref_points:
            cv2.circle(imagem, (x, y), 10, (0, 0, 0), -1)

    def identificador(self, imagem):
        # NOTE: original always set self.current_color = "RED" here
        # but we might want to remove it so the current_color set by the action is respected.
        self.current_color = "RED"

        if self.current_color == "IDLE":
            if self.show_windows:
                cv2.putText(
                    imagem,
                    "STATUS: IDLE (Aguardando Comando BT)",
                    (20, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 0, 255),
                    2,
                )
                cv2.imshow("Camera Tracking", imagem)
                cv2.waitKey(1)
            return

        hsv = cv2.cvtColor(imagem, cv2.COLOR_BGR2HSV)
        mascara = self.gerar_mascara(hsv)

        contornos, _ = cv2.findContours(mascara, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for contorno in contornos:
            if cv2.contourArea(contorno) < 100:
                continue

            M = cv2.moments(contorno)
            if M.get("m00", 0) == 0:
                continue

            epsilon = 0.02 * cv2.arcLength(contorno, True)
            aprox = cv2.approxPolyDP(contorno, epsilon, True)

            if len(aprox) >= 4:
                pts = aprox.reshape(-1, 2)
                s = pts.sum(axis=1)
                diff = np.diff(pts, axis=1)

                tl = pts[np.argmin(s)]
                br = pts[np.argmax(s)]
                tr = pts[np.argmin(diff)]
                bl = pts[np.argmax(diff)]

                self.last_detected_pts = np.array(
                    [tl[0], tl[1], tr[0], tr[1], br[0], br[1], bl[0], bl[1]], dtype=float
                )

                ponto_msg = Int32MultiArray(data=[int(x) for x in self.last_detected_pts])
                self.pub_referencia.publish(ponto_msg)

                cv2.drawContours(imagem, [aprox], -1, (0, 255, 0), 2)
                for pt, color in zip([tl, tr, br, bl], [(0, 0, 255), (255, 0, 0), (0, 255, 255), (255, 0, 255)]):
                    cv2.circle(imagem, (int(pt[0]), int(pt[1])), 7, color, -1)

                cv2.putText(imagem, f"TRACKING: {self.current_color}", (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        self.desenhar_pontos_referencia(imagem)

        if self.show_windows:
            cv2.imshow("Camera Tracking", imagem)
            cv2.waitKey(1)

    def load_json_refs(self):
        script_dir = os.path.dirname(__file__)
        caminho_json = os.path.join(script_dir, 'objetos_config.json')
        try:
            with open(caminho_json, 'r') as f:
                self.json_data = json.load(f)
        except Exception as e:
            self.get_logger().warn(f"Não foi possível carregar {caminho_json}: {e}")
            self.json_data = {}

    def _format_refs(self, ref_list):
        return [(int(ref_list[i]), int(ref_list[i + 1])) for i in range(0, len(ref_list), 2)]

    def publicar_pontos_referencia(self):
        if not self.ref_points:
            return
        dados = [float(coord) for ponto in self.ref_points for coord in ponto]
        msg = Float32MultiArray(data=dados)
        self.pub_pontos_ref.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = IdentificaCor()
    try:
        # Use a multithreaded executor if you expect callbacks + action servers to run concurrently
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        try:
            executor.spin()
        finally:
            executor.shutdown()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()