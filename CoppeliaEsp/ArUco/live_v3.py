#!/usr/bin/env python3
"""Homography-based ArUco tracker.

This version converts camera detections to real-world planar coordinates using
four reference ArUco markers with known world positions. It then stores one CSV
and one plot per tracked car.
"""

from __future__ import annotations

import csv
import json
import math
import os
import time
from datetime import datetime
from typing import Dict, List, Optional, Tuple

import cv2
import matplotlib.pyplot as plt
import numpy as np

try:
    import zmq
except ImportError:
    raise ImportError("Instala pyzmq: pip install pyzmq")


class ArucoTracker:
    def __init__(
        self,
        url: str,
        marker_size_m: float = 0.1,
        zmq_address: str = "tcp://*:5555",
        car_target_map: Optional[Dict[int, int]] = None,
        marker_world_map: Optional[Dict[int, Tuple[float, float]]] = None,
        output_root: Optional[str] = None,
    ) -> None:
        self.url = url
        self.marker_size_m = marker_size_m
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

        self.car_target_map = car_target_map or {}
        self.target_ids = set(self.car_target_map.values()) if self.car_target_map else {2}

        self.marker_world_map = marker_world_map or {
            10: (0.0, 0.0),
            11: (1.0, 0.0),
            12: (1.0, 1.0),
            13: (0.0, 1.0),
        }
        self.reference_ids = set(self.marker_world_map.keys())
        self.homography: Optional[np.ndarray] = None

        self.filtered_cars: Dict[int, Dict[str, Optional[float]]] = {}
        self.filtered_targets: Dict[int, Dict[str, Optional[float]]] = {}
        self.position_history: Dict[int, Dict[str, List[float]]] = {}

        self.alpha_pos = 0.3
        self.alpha_angle = 0.3

        self.start_time: Optional[datetime] = None
        self.frame_count = 0

        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.PUB)
        self.socket.bind(zmq_address)
        self.topic = b"aruco"
        time.sleep(0.3)
        print(f"[ZMQ] Publisher en {zmq_address}")

        self.source_type = self._detect_source_type(url)
        self.cap = cv2.VideoCapture(url)
        if not self.cap.isOpened():
            raise RuntimeError(f"No se pudo abrir la fuente: {url}")

        if self.source_type == "stream":
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1080)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

        self.window_name = "ArUco Tracker - Homography"
        self.max_display_width = 1280
        self.max_display_height = 720
        self.output_root = output_root or os.path.abspath(
            os.path.join(os.path.dirname(__file__), "..", "Cars_Trajectory_V3")
        )

    def _detect_source_type(self, source: str) -> str:
        if source.startswith(("http://", "https://", "rtsp://", "rtmp://")):
            return "stream"
        if os.path.isfile(source):
            return "video_file"
        if source.startswith(("./", "../")) or source.startswith('.\\'):
            return "video_file"
        try:
            int(source)
            return "webcam"
        except ValueError:
            return "video_file"

    def normalize_angle(self, angle: float) -> float:
        while angle > 180:
            angle -= 360
        while angle < -180:
            angle += 360
        return angle

    def exponential_filter(self, current_value: float, previous_filtered: Optional[float], alpha: float) -> float:
        if previous_filtered is None:
            return current_value
        return alpha * current_value + (1 - alpha) * previous_filtered

    def calculate_marker_info(self, corner: np.ndarray) -> Dict[str, object]:
        c = corner.reshape(4, 2).astype(np.float32)
        center_x = float(np.mean(c[:, 0]))
        center_y = float(np.mean(c[:, 1]))

        top_center_x = float((c[0][0] + c[1][0]) / 2)
        top_center_y = float((c[0][1] + c[1][1]) / 2)
        angle_rad = math.atan2(top_center_y - center_y, top_center_x - center_x)
        angle_deg = math.degrees(angle_rad)

        return {"center": (center_x, center_y), "angle": self.normalize_angle(angle_deg)}

    def compute_homography(self, reference_centers_px: Dict[int, Tuple[float, float]]) -> Optional[np.ndarray]:
        ordered_ids = [marker_id for marker_id in sorted(self.marker_world_map) if marker_id in reference_centers_px]
        if len(ordered_ids) < 4:
            return None

        src = np.float32([reference_centers_px[marker_id] for marker_id in ordered_ids]).reshape(-1, 1, 2)
        dst = np.float32([self.marker_world_map[marker_id] for marker_id in ordered_ids]).reshape(-1, 1, 2)
        homography, _ = cv2.findHomography(src, dst)
        return homography

    def pixel_to_world(self, point_px: Tuple[float, float]) -> Optional[Tuple[float, float]]:
        if self.homography is None:
            return None
        point = np.array([[[float(point_px[0]), float(point_px[1])]]], dtype=np.float32)
        world_point = cv2.perspectiveTransform(point, self.homography)[0][0]
        return float(world_point[0]), float(world_point[1])

    def apply_filter_to_marker(
        self,
        marker_data: Dict[str, object],
        filtered_data: Dict[str, Optional[float]],
    ) -> Dict[str, Optional[float]]:
        x_value, y_value = marker_data["position"]
        filtered_data["x"] = self.exponential_filter(float(x_value), filtered_data["x"], self.alpha_pos)
        filtered_data["y"] = self.exponential_filter(float(y_value), filtered_data["y"], self.alpha_pos)

        angle = float(marker_data["angle"])
        if filtered_data["angle"] is None:
            filtered_data["angle"] = angle
        else:
            diff = angle - float(filtered_data["angle"])
            if diff > 180:
                diff -= 360
            elif diff < -180:
                diff += 360
            filtered_data["angle"] = self.normalize_angle(float(filtered_data["angle"]) + self.alpha_angle * diff)

        return filtered_data

    def update_position_history(self, car_id: int, x_m: float, y_m: float, elapsed_time: float) -> None:
        if car_id not in self.position_history:
            self.position_history[car_id] = {"x": [], "y": [], "time": []}
        self.position_history[car_id]["x"].append(x_m)
        self.position_history[car_id]["y"].append(y_m)
        self.position_history[car_id]["time"].append(elapsed_time)

    def publish_pose(
        self,
        car_id: int,
        x_m: float,
        y_m: float,
        angle: float,
        detected: bool,
        distance_to_target: Optional[float],
        target_id: Optional[int] = None,
    ) -> None:
        data = {
            "car_id": int(car_id),
            "x": float(x_m),
            "y": float(y_m),
            "angle": float(angle),
            "detected": bool(detected),
            "target_id": int(target_id) if target_id is not None else None,
            "distance_to_target": float(distance_to_target) if distance_to_target is not None else None,
            "timestamp": time.time(),
        }
        payload = json.dumps(data).encode("utf-8")
        self.socket.send_multipart([self.topic, payload])

    def draw_info_panel(
        self,
        frame: np.ndarray,
        cars_data: Dict[int, Dict[str, object]],
        targets_data: Dict[int, Dict[str, object]],
    ) -> np.ndarray:
        h, w = frame.shape[:2]
        panel_width = 350
        panel = np.zeros((h, panel_width, 3), dtype=np.uint8)
        panel[:] = (30, 30, 30)
        combined = np.zeros((h, w + panel_width, 3), dtype=np.uint8)
        combined[:, :w] = frame
        combined[:, w:] = panel

        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.55
        line_height = 22
        margin_x = w + 15
        start_y = 35

        def draw_text(text: str, y: int, color=(255, 255, 255), scale=font_scale, thickness=1) -> int:
            cv2.putText(combined, text, (margin_x, y), font, scale, color, thickness)
            return y + line_height

        y = start_y
        y = draw_text("=== ARUCO TRACKER ===", y, (0, 255, 255), 0.7, 2)
        y = draw_text(f"Homography: {'READY' if self.homography is not None else 'WAITING'}", y, (255, 200, 100), 0.55, 1)
        y += 10
        y = draw_text("REFERENCE MARKERS", y, (180, 180, 255), 0.65, 2)
        for ref_id in sorted(self.marker_world_map):
            world_x, world_y = self.marker_world_map[ref_id]
            y = draw_text(f"  ID {ref_id}: ({world_x:.2f}, {world_y:.2f})", y, (200, 200, 200), 0.5)

        y += 10
        y = draw_text("CARROS DETECTADOS", y, (0, 165, 255), 0.65, 2)
        for car_id, car_data in sorted(cars_data.items()):
            y = draw_text(f"  ID {car_id}:", y, (100, 200, 255), 0.55, 1)
            if car_data["detected"]:
                y = draw_text(f"    Pos: ({car_data['x']:.3f}, {car_data['y']:.3f}) m", y, (200, 200, 200))
                y = draw_text(f"    Angulo: {car_data['angle']:.1f}°", y, (200, 200, 200))
                target_id = car_data.get("target_id")
                if target_id is not None:
                    y = draw_text(f"    Objetivo: ID {target_id}", y, (180, 220, 120), 0.5)
                if car_data.get("distance_to_target") is not None:
                    y = draw_text(f"    Distancia: {car_data['distance_to_target']:.3f} m", y, (0, 255, 255), 0.55, 2)
            else:
                y = draw_text("    No detectado", y, (100, 100, 100))

        y += 10
        y = draw_text("TARGETS", y, (0, 255, 0), 0.65, 2)
        for target_id, target_data in sorted(targets_data.items()):
            y = draw_text(f"  ID {target_id}:", y, (120, 255, 120), 0.55, 1)
            if target_data["detected"]:
                y = draw_text(f"    Pos: ({target_data['x']:.3f}, {target_data['y']:.3f}) m", y, (200, 200, 200))
                y = draw_text(f"    Angulo: {target_data['angle']:.1f} deg", y, (200, 200, 200))
            else:
                y = draw_text("    No detectado", y, (100, 100, 100))

        return combined

    def draw_visualization(
        self,
        frame: np.ndarray,
        cars_data: Dict[int, Dict[str, object]],
        targets_data: Dict[int, Dict[str, object]],
        reference_centers: Dict[int, Tuple[float, float]],
    ) -> np.ndarray:
        for ref_id, (cx, cy) in reference_centers.items():
            cv2.circle(frame, (int(cx), int(cy)), 10, (0, 255, 255), -1)
            cv2.putText(frame, f"REF {ref_id}", (int(cx) + 10, int(cy) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

        for car_id, car_data in cars_data.items():
            if car_data["detected"]:
                cx, cy = int(car_data["x_px"]), int(car_data["y_px"])
                color = (int(100 + car_id * 40) % 256, int(165 + car_id * 20) % 256, 255)
                cv2.circle(frame, (cx, cy), 8, color, -1)
                cv2.circle(frame, (cx, cy), 10, (255, 255, 255), 2)
                arrow_len = 40
                angle_rad = math.radians(float(car_data["angle"]))
                end_x = int(cx + arrow_len * math.cos(angle_rad))
                end_y = int(cy + arrow_len * math.sin(angle_rad))
                cv2.arrowedLine(frame, (cx, cy), (end_x, end_y), color, 3, tipLength=0.3)
                cv2.putText(frame, f"CARRO {car_id}", (cx + 15, cy - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

        for target_id, target_data in targets_data.items():
            if target_data["detected"]:
                cx, cy = int(target_data["x_px"]), int(target_data["y_px"])
                cv2.circle(frame, (cx, cy), 8, (0, 255, 0), -1)
                cv2.circle(frame, (cx, cy), 10, (255, 255, 255), 2)
                arrow_len = 40
                angle_rad = math.radians(float(target_data["angle"]))
                end_x = int(cx + arrow_len * math.cos(angle_rad))
                end_y = int(cy + arrow_len * math.sin(angle_rad))
                cv2.arrowedLine(frame, (cx, cy), (end_x, end_y), (0, 255, 0), 3, tipLength=0.3)
                cv2.putText(frame, f"OBJETIVO {target_id}", (cx + 15, cy - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        for car_id, car_data in cars_data.items():
            target_id = car_data.get("target_id")
            if (
                car_data["detected"]
                and car_data.get("distance_to_target") is not None
                and target_id in targets_data
                and targets_data[target_id]["detected"]
            ):
                car_cx, car_cy = int(car_data["x_px"]), int(car_data["y_px"])
                target_cx, target_cy = int(targets_data[target_id]["x_px"]), int(targets_data[target_id]["y_px"])
                cv2.line(frame, (car_cx, car_cy), (target_cx, target_cy), (255, 255, 0), 2)
                mid_x = (car_cx + target_cx) // 2
                mid_y = (car_cy + target_cy) // 2
                dist_text = f"T{target_id}: {car_data['distance_to_target']:.2f}m"
                (text_w, text_h), _ = cv2.getTextSize(dist_text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2)
                cv2.rectangle(
                    frame,
                    (mid_x - text_w // 2 - 5, mid_y - text_h - 5),
                    (mid_x + text_w // 2 + 5, mid_y + 5),
                    (0, 0, 0),
                    -1,
                )
                cv2.putText(frame, dist_text, (mid_x - text_w // 2, mid_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)

        return frame

    def generate_trajectory_plots(self) -> None:
        if not self.position_history:
            print("No hay historial de posiciones para graficar.")
            return

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        output_dir = os.path.join(self.output_root, timestamp)
        os.makedirs(output_dir, exist_ok=True)

        for car_id, history in sorted(self.position_history.items()):
            xs = history["x"]
            ys = history["y"]
            times = history["time"]
            if not xs or not ys or not times:
                continue

            plt.figure(figsize=(6, 6))
            plt.plot(xs, ys, marker="o", linestyle="-", label=f"Carro {car_id}")
            plt.xlabel("x (m)")
            plt.ylabel("y (m)")
            plt.title(f"Trayectoria real - Carro {car_id}")
            plt.grid(True)
            plt.axis("equal")
            plt.legend()
            plt.tight_layout()

            plot_filename = os.path.join(output_dir, f"carro_{car_id}.png")
            plt.savefig(plot_filename, dpi=150, bbox_inches="tight")
            plt.close()
            print(f"Grafica guardada: {plot_filename}")

            csv_filename = os.path.join(output_dir, f"carro_{car_id}.csv")
            with open(csv_filename, mode="w", newline="", encoding="utf-8") as f:
                writer = csv.writer(f)
                writer.writerow(["time", "x", "y"])
                for elapsed_time, x_value, y_value in zip(times, xs, ys):
                    writer.writerow([elapsed_time, x_value, y_value])
            print(f"CSV guardado: {csv_filename}")

        print(f"Todas las graficas fueron guardadas en: {output_dir}")

    def run(self) -> None:
        print("=" * 60)
        print("ARUCO TRACKER - HOMOGRAPHY VERSION")
        print("=" * 60)
        print(f"Tipo de fuente: {self.source_type.upper()}")
        print(f"URL/Archivo: {self.url}")
        print(f"Objetivos configurados: {sorted(self.target_ids)}")
        print(f"Asignacion carro->objetivo: {self.car_target_map}")
        print(f"Marcadores de referencia: {self.marker_world_map}")
        print("Los marcadores de referencia deben estar visibles para calcular la homografia")
        print("=" * 60)

        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window_name, self.max_display_width + 350, self.max_display_height)

        self.start_time = datetime.now()
        frame_count = 0
        paused = False
        last_frame = None

        while True:
            if not paused:
                ret, frame = self.cap.read()
                if not ret:
                    print("\nFin del video/stream. Generando graficas...")
                    self.generate_trajectory_plots()
                    break
                last_frame = frame
            else:
                frame = last_frame

            frame_count += 1
            self.frame_count = frame_count
            elapsed_time = (datetime.now() - self.start_time).total_seconds()

            h, w = frame.shape[:2]
            scale = min(self.max_display_width / w, self.max_display_height / h)
            new_w = int(w * scale)
            new_h = int(h * scale)
            if new_w != w or new_h != h:
                frame = cv2.resize(frame, (new_w, new_h), interpolation=cv2.INTER_LINEAR)

            cars_data: Dict[int, Dict[str, object]] = {}
            targets_data: Dict[int, Dict[str, object]] = {
                target_id: {"detected": False, "x": 0.0, "y": 0.0, "x_px": 0.0, "y_px": 0.0, "angle": 0.0}
                for target_id in self.target_ids
            }

            corners, ids, _ = cv2.aruco.detectMarkers(frame, self.dictionary)

            reference_centers_px: Dict[int, Tuple[float, float]] = {}
            marker_info_by_id: Dict[int, Dict[str, object]] = {}
            if ids is not None:
                cv2.aruco.drawDetectedMarkers(frame, corners, ids)
                for corner, marker_id in zip(corners, ids.flatten()):
                    info = self.calculate_marker_info(corner)
                    marker_info_by_id[int(marker_id)] = info
                    if int(marker_id) in self.reference_ids:
                        reference_centers_px[int(marker_id)] = info["center"]

            if len(reference_centers_px) >= 4:
                homography = self.compute_homography(reference_centers_px)
                if homography is not None:
                    self.homography = homography

            if self.homography is None:
                cv2.putText(frame, "Esperando 4 marcadores de referencia...", (30, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
                display = self.draw_info_panel(frame, cars_data, targets_data)
                cv2.imshow(self.window_name, display)
                if cv2.waitKey(1) & 0xFF == 27:
                    break
                continue

            for marker_id, info in marker_info_by_id.items():
                if marker_id in self.reference_ids:
                    continue

                world_position = self.pixel_to_world(info["center"])
                if world_position is None:
                    continue

                filtered_key = {"x": None, "y": None, "angle": None}
                if marker_id in self.target_ids:
                    if marker_id not in self.filtered_targets:
                        self.filtered_targets[marker_id] = filtered_key.copy()

                    self.filtered_targets[marker_id] = self.apply_filter_to_marker(
                        {"position": world_position, "angle": info["angle"]},
                        self.filtered_targets[marker_id],
                    )
                    targets_data[marker_id]["detected"] = True
                    targets_data[marker_id]["x"] = self.filtered_targets[marker_id]["x"]
                    targets_data[marker_id]["y"] = self.filtered_targets[marker_id]["y"]
                    targets_data[marker_id]["x_px"] = float(info["center"][0])
                    targets_data[marker_id]["y_px"] = float(info["center"][1])
                    targets_data[marker_id]["angle"] = self.filtered_targets[marker_id]["angle"]
                else:
                    if marker_id not in self.filtered_cars:
                        self.filtered_cars[marker_id] = filtered_key.copy()

                    self.filtered_cars[marker_id] = self.apply_filter_to_marker(
                        {"position": world_position, "angle": info["angle"]},
                        self.filtered_cars[marker_id],
                    )
                    cars_data[marker_id] = {
                        "detected": True,
                        "x_px": float(info["center"][0]),
                        "y_px": float(info["center"][1]),
                        "x": self.filtered_cars[marker_id]["x"],
                        "y": self.filtered_cars[marker_id]["y"],
                        "angle": self.filtered_cars[marker_id]["angle"],
                        "target_id": self.car_target_map.get(marker_id),
                        "distance_to_target": None,
                    }

            for car_id in self.filtered_cars:
                if car_id not in cars_data:
                    cars_data[car_id] = {
                        "detected": False,
                        "x_px": 0.0,
                        "y_px": 0.0,
                        "x": 0.0,
                        "y": 0.0,
                        "angle": self.filtered_cars[car_id]["angle"] or 0.0,
                        "target_id": self.car_target_map.get(car_id),
                        "distance_to_target": None,
                    }

            for car_id, car_data in cars_data.items():
                assigned_target_id = car_data.get("target_id")
                if not car_data["detected"]:
                    continue
                if assigned_target_id is None or assigned_target_id not in targets_data:
                    continue

                target_data = targets_data[assigned_target_id]
                if target_data["detected"]:
                    dx = float(target_data["x"]) - float(car_data["x"])
                    dy = float(target_data["y"]) - float(car_data["y"])
                    car_data["distance_to_target"] = math.hypot(dx, dy)
                else:
                    car_data["distance_to_target"] = None

            for car_id, car_data in cars_data.items():
                if car_data["detected"]:
                    self.publish_pose(
                        car_id=car_id,
                        x_m=float(car_data["x"]),
                        y_m=float(car_data["y"]),
                        angle=float(car_data["angle"]),
                        detected=True,
                        distance_to_target=car_data.get("distance_to_target"),
                        target_id=car_data.get("target_id"),
                    )
                    self.update_position_history(car_id, float(car_data["x"]), float(car_data["y"]), elapsed_time)

            for car_id in cars_data:
                dist = cars_data[car_id].get("distance_to_target")
                dist_text = f"{dist:.3f}m" if dist is not None else "N/A"
                print(
                    f"[ZMQ] Car {car_id}: x={float(cars_data[car_id]['x']):.3f}m, y={float(cars_data[car_id]['y']):.3f}m, "
                    f"angle={float(cars_data[car_id]['angle']):.1f}°, target={cars_data[car_id].get('target_id')}, dist={dist_text}"
                )

            frame = self.draw_visualization(frame, cars_data, targets_data, reference_centers_px)
            display = self.draw_info_panel(frame, cars_data, targets_data)

            if paused:
                cv2.putText(display, "PAUSADO", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 0, 255), 3)

            cv2.imshow(self.window_name, display)

            wait_time = 33 if self.source_type == "video_file" else 1
            key = cv2.waitKey(wait_time) & 0xFF

            if key == 27:
                print("\nGenerando graficas...")
                self.generate_trajectory_plots()
                break
            if key == ord("s") or key == ord("S"):
                filename = f"aruco_capture_{frame_count:04d}.png"
                cv2.imwrite(filename, display)
                print(f"Captura guardada: {filename}")
            elif key == ord(" ") and self.source_type == "video_file":
                paused = not paused
                print("Pausado" if paused else "Reanudado")

        self.cap.release()
        cv2.destroyAllWindows()
        self.socket.close()
        self.context.term()
        print("Programa finalizado.")


if __name__ == "__main__":
    ##url = r"C:\Users\jaide\Desktop\Proyecto-UnB-UDLA-IEEE-rob-tica-mobil\CoppeliaEsp\ArUco\tester4.mp4"
    url = "http://172.27.6.11:4747/video"
    marker_world_map = {
        10: (0.0, 0.7),
        11: (1.0, 0.7),
        12: (1.0, -0.7),
        13: (0.0, -0.7),
    }

    car_target_map = {
        0: 3,
        1: 4,
        2: 5,
    }

    try:
        tracker = ArucoTracker(
            url,
            marker_size_m=0.1,
            car_target_map=car_target_map,
            marker_world_map=marker_world_map,
        )
        tracker.run()
    except RuntimeError as e:
        print(f"Error: {e}")
