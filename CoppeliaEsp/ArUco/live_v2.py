import csv
import cv2
import numpy as np
import math
import matplotlib.pyplot as plt
from datetime import datetime
import os
import time
import json

try:
    import zmq
except ImportError:
    raise ImportError("Instala pyzmq: pip install pyzmq")


class ArucoTracker:
    def __init__(self, url, marker_size_m=0.1, zmq_address="tcp://*:5555", car_target_map=None):
        """
        Tracker de marcadores ArUco con publicacion ZMQ

        Args:
            url: URL de la camara
            marker_size_m: Tamaño real del marcador en metros
            zmq_address: Direccion ZMQ para publicar poses
            car_target_map: Mapa de asignacion {car_id: target_id}
        """
        self.url = url
        self.marker_size_m = marker_size_m

        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

        # Configuracion de IDs
        self.car_target_map = car_target_map or {}
        self.target_ids = set(self.car_target_map.values()) if self.car_target_map else {2}

        # Variables de estado
        self.filtered_cars = {}  # {car_id: {'x': None, 'y': None, 'angle': None}}
        self.filtered_targets = {}  # {target_id: {'x': None, 'y': None, 'angle': None}}

        # Historial de posiciones para graficas
        self.position_history = {}  # {car_id: {'x': [...], 'y': [...], 'time': [...]}}
        self.start_time = None
        self.frame_count = 0

        # Filtros para suavizado
        self.alpha_pos = 0.3
        self.alpha_angle = 0.3
        self.alpha_ppm = 0.2  # Filtro para pixeles por metro

        # Variables para calibracion de distancia
        self.target_ppm = {}  # {target_id: ppm_filtrado}
        self.filtered_ppm = {}  # {car_id: filtered_ppm}
        self.calibration_factor = 1.010  # Factor de calibracion ajustable

        # ZMQ Publisher
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.PUB)
        self.socket.bind(zmq_address)
        self.topic = "aruco".encode()
        time.sleep(0.3)  # Dar tiempo a que subscribers se conecten
        print(f"[ZMQ] Publisher en {zmq_address}")

        # Conectar a la camara o video
        self.source_type = self._detect_source_type(url)
        self.cap = cv2.VideoCapture(url)

        if not self.cap.isOpened():
            raise RuntimeError(f"No se pudo abrir la fuente: {url}")

        if self.source_type == "stream":
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1080)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

        # Ventana de visualizacion
        self.window_name = "ArUco Tracker - Multiples Carros vs Objetivo"
        self.max_display_width = 1280
        self.max_display_height = 720

    def _detect_source_type(self, source):
        if source.startswith(('http://', 'https://', 'rtsp://', 'rtmp://')):
            return "stream"
        elif os.path.isfile(source):
            return "video_file"
        elif source.startswith(('./', '../')) or source.startswith('.\\'):
            return "video_file"
        else:
            try:
                int(source)
                return "webcam"
            except ValueError:
                return "video_file"

    def normalize_angle(self, angle):
        while angle > 180:
            angle -= 360
        while angle < -180:
            angle += 360
        return angle

    def exponential_filter(self, current_value, previous_filtered, alpha):
        if previous_filtered is None:
            return current_value
        return alpha * current_value + (1 - alpha) * previous_filtered

    def calculate_marker_info(self, corner):
        c = corner.reshape(4, 2).astype(np.float32)
        center_x = np.mean(c[:, 0])
        center_y = np.mean(c[:, 1])
        marker_width_px = np.linalg.norm(c[0] - c[1])
        pixels_per_meter = marker_width_px / self.marker_size_m if marker_width_px > 0 else 1000

        top_center_x = (c[0][0] + c[1][0]) / 2
        top_center_y = (c[0][1] + c[1][1]) / 2
        angle_rad = math.atan2(top_center_y - center_y, top_center_x - center_x)
        angle_deg = math.degrees(angle_rad)

        return {
            'center': (center_x, center_y),
            'pixels_per_meter': pixels_per_meter,
            'angle': self.normalize_angle(angle_deg)
        }

    def apply_filter_to_marker(self, marker_info, filtered_data):
        cx, cy = marker_info['center']
        filtered_data['x'] = self.exponential_filter(cx, filtered_data['x'], self.alpha_pos)
        filtered_data['y'] = self.exponential_filter(cy, filtered_data['y'], self.alpha_pos)

        angle = marker_info['angle']
        if filtered_data['angle'] is None:
            filtered_data['angle'] = angle
        else:
            diff = angle - filtered_data['angle']
            if diff > 180:
                diff -= 360
            elif diff < -180:
                diff += 360
            filtered_data['angle'] = self.normalize_angle(filtered_data['angle'] + self.alpha_angle * diff)

        return filtered_data

    def update_position_history(self, car_id, x_m, y_m, elapsed_time):
        if car_id not in self.position_history:
            self.position_history[car_id] = {'x': [], 'y': [], 'time': []}
        self.position_history[car_id]['x'].append(x_m)
        self.position_history[car_id]['y'].append(y_m)
        self.position_history[car_id]['time'].append(elapsed_time)

    def publish_pose(self, car_id, x_m, y_m, angle, detected, distance_to_target, target_id=None):
        """Publica pose por ZMQ"""
        data = {
            'car_id': int(car_id),
            'x': float(x_m),
            'y': float(y_m),
            'angle': float(angle),
            'detected': bool(detected),
            'target_id': int(target_id) if target_id is not None else None,
            'distance_to_target': float(distance_to_target) if distance_to_target is not None else None,
            'timestamp': time.time()
        }
        payload = json.dumps(data).encode('utf-8')
        self.socket.send_multipart([self.topic, payload])

    def calibrate_distance(self, detected_distance):
        """Calibra el factor de conversion basado en distancia real conocida"""
        print("\n" + "="*60)
        print("MODO CALIBRACION DE DISTANCIA")
        print("="*60)
        print(f"Distancia detectada: {detected_distance:.3f} m")

        try:
            real_distance = float(input("Ingresa la distancia REAL medida (en metros): "))
            if real_distance <= 0:
                print("Error: La distancia debe ser mayor a 0")
                return

            new_factor = real_distance / detected_distance
            self.calibration_factor = new_factor

            print(f"\n✓ Calibracion completada!")
            print(f"  Distancia real: {real_distance:.3f} m")
            print(f"  Distancia detectada: {detected_distance:.3f} m")
            print(f"  Factor de calibracion: {self.calibration_factor:.4f}")
            print("="*60 + "\n")

        except ValueError:
            print("Error: Ingresa un numero valido")
        except ZeroDivisionError:
            print("Error: La distancia detectada no puede ser 0")

    def draw_info_panel(self, frame, cars_data, targets_data):
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

        def draw_text(text, y, color=(255, 255, 255), scale=font_scale, thickness=1):
            cv2.putText(combined, text, (margin_x, y), font, scale, color, thickness)
            return y + line_height

        y = start_y
        y = draw_text("=== ARUCO TRACKER ===", y, (0, 255, 255), 0.7, 2)
        y += 10
        y = draw_text("OBJETIVOS", y, (0, 255, 0), 0.65, 2)

        if targets_data:
            for target_id, target_data in sorted(targets_data.items()):
                y = draw_text(f"  ID {target_id}:", y, (120, 255, 120), 0.55, 1)
                if target_data['detected']:
                    y = draw_text(f"    Pos: ({target_data['x']:.3f}, {target_data['y']:.3f}) m", y, (200, 200, 200))
                    y = draw_text(f"    Angulo: {target_data['angle']:.1f} deg", y, (200, 200, 200))
                else:
                    y = draw_text("    No detectado", y, (100, 100, 100))
        else:
            y = draw_text("  Ninguno configurado", y, (100, 100, 100))

        y += 10
        y = draw_text("CARROS DETECTADOS", y, (0, 165, 255), 0.65, 2)

        if cars_data:
            for car_id, car_data in sorted(cars_data.items()):
                y = draw_text(f"  ID {car_id}:", y, (100, 200, 255), 0.55, 1)
                if car_data['detected']:
                    y = draw_text(f"    Pos: ({car_data['x']:.3f}, {car_data['y']:.3f}) m", y, (200, 200, 200))
                    y = draw_text(f"    Angulo: {car_data['angle']:.1f}°", y, (200, 200, 200))
                    target_id = car_data.get('target_id')
                    if target_id is not None:
                        y = draw_text(f"    Objetivo: ID {target_id}", y, (180, 220, 120), 0.5)
                    if 'distance_to_target' in car_data and car_data['distance_to_target'] is not None:
                        y = draw_text(f"    Distancia: {car_data['distance_to_target']:.3f} m", y, (0, 255, 255), 0.55, 2)
                        if 'ppm' in car_data:
                            y = draw_text(f"    PPM: {car_data['ppm']:.1f} px/m", y, (150, 150, 150), 0.45)
                else:
                    y = draw_text(f"    No detectado", y, (100, 100, 100))
        else:
            y = draw_text("  Ninguno detectado", y, (100, 100, 100))

        y += 15
        y = draw_text("CONTROLES", y, (200, 200, 200), 0.6, 1)
        y = draw_text("  ESC: Salir y graficar", y, (150, 150, 150), 0.5)
        y = draw_text("  S: Guardar captura", y, (150, 150, 150), 0.5)
        y = draw_text("  +/-: Ajustar calibracion", y, (150, 150, 150), 0.5)
        y = draw_text("  C: Calibrar distancia", y, (150, 150, 150), 0.5)

        y += 10
        y = draw_text("CALIBRACION", y, (255, 200, 100), 0.6, 1)
        y = draw_text(f"  Factor: {self.calibration_factor:.3f}", y, (255, 200, 100), 0.55, 2)
        if self.target_ppm:
            for target_id, target_ppm in sorted(self.target_ppm.items()):
                y = draw_text(f"  PPM T{target_id}: {target_ppm:.1f}", y, (150, 150, 150), 0.45)

        return combined

    def draw_visualization(self, frame, cars_data, targets_data):
        h, w = frame.shape[:2]
        for car_id, car_data in cars_data.items():
            if car_data['detected']:
                cx, cy = int(car_data['x_px']), int(car_data['y_px'])
                color = (int(100 + car_id * 40) % 256, int(165 + car_id * 20) % 256, 255)
                cv2.circle(frame, (cx, cy), 8, color, -1)
                cv2.circle(frame, (cx, cy), 10, (255, 255, 255), 2)
                arrow_len = 40
                angle_rad = math.radians(car_data['angle'])
                end_x = int(cx + arrow_len * math.cos(angle_rad))
                end_y = int(cy + arrow_len * math.sin(angle_rad))
                cv2.arrowedLine(frame, (cx, cy), (end_x, end_y), color, 3, tipLength=0.3)
                cv2.putText(frame, f"CARRO {car_id}", (cx + 15, cy - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

        for target_id, target_data in targets_data.items():
            if target_data['detected']:
                cx, cy = int(target_data['x_px']), int(target_data['y_px'])
                cv2.circle(frame, (cx, cy), 8, (0, 255, 0), -1)
                cv2.circle(frame, (cx, cy), 10, (255, 255, 255), 2)
                arrow_len = 40
                angle_rad = math.radians(target_data['angle'])
                end_x = int(cx + arrow_len * math.cos(angle_rad))
                end_y = int(cy + arrow_len * math.sin(angle_rad))
                cv2.arrowedLine(frame, (cx, cy), (end_x, end_y), (0, 255, 0), 3, tipLength=0.3)
                cv2.putText(frame, f"OBJETIVO {target_id}", (cx + 15, cy - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        for car_id, car_data in cars_data.items():
            target_id = car_data.get('target_id')
            if (
                car_data['detected']
                and car_data.get('distance_to_target') is not None
                and target_id in targets_data
                and targets_data[target_id]['detected']
            ):
                car_cx, car_cy = int(car_data['x_px']), int(car_data['y_px'])
                target_cx, target_cy = int(targets_data[target_id]['x_px']), int(targets_data[target_id]['y_px'])
                cv2.line(frame, (car_cx, car_cy), (target_cx, target_cy), (255, 255, 0), 2)
                mid_x = (car_cx + target_cx) // 2
                mid_y = (car_cy + target_cy) // 2
                dist_text = f"T{target_id}: {car_data['distance_to_target']:.2f}m"
                (text_w, text_h), _ = cv2.getTextSize(dist_text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2)
                cv2.rectangle(frame, (mid_x - text_w // 2 - 5, mid_y - text_h - 5),
                              (mid_x + text_w // 2 + 5, mid_y + 5), (0, 0, 0), -1)
                cv2.putText(frame, dist_text, (mid_x - text_w // 2, mid_y),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)

        return frame

    def generate_trajectory_plots(self):
        if not self.position_history:
            print("No hay historial de posiciones para graficar.")
            return

        base_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "Cars_Trajectory"))
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        output_dir = os.path.join(base_dir, timestamp)
        os.makedirs(output_dir, exist_ok=True)

        for idx, (car_id, history) in enumerate(sorted(self.position_history.items())):
            xs = history['x']
            ys = history['y']
            times = history['time']

            if not xs or not ys or not times:
                continue

            plt.figure(figsize=(6, 5))
            plt.plot(xs, ys, marker="o", linestyle="-", label=f"Carro {car_id}")
            plt.xlabel("x_out")
            plt.ylabel("y_out")
            plt.title(f"Trayectoria - Carro {car_id}")
            plt.grid(True)
            plt.legend()

            filename = os.path.join(output_dir, f'carro_{car_id}.png')
            plt.savefig(filename, dpi=150, bbox_inches='tight')
            plt.close()
            print(f"Grafica guardada: {filename}")

            csv_filename = os.path.join(output_dir, f'carro_{car_id}.csv')
            with open(csv_filename, mode="w", newline="", encoding="utf-8") as f:
                writer = csv.writer(f)
                writer.writerow(["time", "x", "y"])
                for elapsed_time, x_value, y_value in zip(times, xs, ys):
                    writer.writerow([elapsed_time, x_value, y_value])
            print(f"CSV guardado: {csv_filename}")

        print(f"Todas las graficas fueron guardadas en: {output_dir}")

    def run(self):
        print("=" * 60)
        print("ARUCO TRACKER - Multiples Carros vs Objetivo")
        print("=" * 60)
        print(f"Tipo de fuente: {self.source_type.upper()}")
        print(f"  URL/Archivo: {self.url}")
        print(f"Objetivos configurados: {sorted(self.target_ids)}")
        print(f"Asignacion carro->objetivo: {self.car_target_map}")
        print("Los IDs en objetivos son tratados como objetivos, el resto como carros")
        print("=" * 60)
        print("Controles:")
        print("  ESC - Salir y generar graficas")
        print("  S - Guardar captura de pantalla")
        print("  C - CALIBRAR distancia (ingresa distancia real)")
        print("  +/- - Ajustar factor de calibracion (ajuste fino)")
        if self.source_type == "video_file":
            print("  ESPACIO - Pausar/Reanudar")
            print("  Flechas izq/der - Retroceder/Avanzar frame")
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

            cars_data = {}
            targets_data = {
                target_id: {'detected': False, 'x': 0, 'y': 0, 'x_px': 0, 'y_px': 0, 'angle': 0, 'ppm': 0}
                for target_id in self.target_ids
            }

            corners, ids, _ = cv2.aruco.detectMarkers(frame, self.dictionary)

            if ids is not None:
                cv2.aruco.drawDetectedMarkers(frame, corners, ids)

                for corner, marker_id in zip(corners, ids.flatten()):
                    info = self.calculate_marker_info(corner)
                    px, py = info['center']
                    ppm = info['pixels_per_meter']

                    if marker_id in self.target_ids:
                        if marker_id not in self.filtered_targets:
                            self.filtered_targets[marker_id] = {'x': None, 'y': None, 'angle': None}

                        self.filtered_targets[marker_id] = self.apply_filter_to_marker(info, self.filtered_targets[marker_id])
                        targets_data[marker_id]['detected'] = True
                        targets_data[marker_id]['x_px'] = self.filtered_targets[marker_id]['x']
                        targets_data[marker_id]['y_px'] = self.filtered_targets[marker_id]['y']
                        targets_data[marker_id]['x'] = (self.filtered_targets[marker_id]['x'] / ppm) * self.calibration_factor
                        targets_data[marker_id]['y'] = (self.filtered_targets[marker_id]['y'] / ppm) * self.calibration_factor
                        targets_data[marker_id]['angle'] = self.filtered_targets[marker_id]['angle']
                        targets_data[marker_id]['ppm'] = ppm
                    else:
                        if marker_id not in self.filtered_cars:
                            self.filtered_cars[marker_id] = {'x': None, 'y': None, 'angle': None}

                        self.filtered_cars[marker_id] = self.apply_filter_to_marker(info, self.filtered_cars[marker_id])

                        cars_data[marker_id] = {
                            'detected': True,
                            'x_px': self.filtered_cars[marker_id]['x'],
                            'y_px': self.filtered_cars[marker_id]['y'],
                            'x': (self.filtered_cars[marker_id]['x'] / ppm) * self.calibration_factor,
                            'y': (self.filtered_cars[marker_id]['y'] / ppm) * self.calibration_factor,
                            'angle': self.filtered_cars[marker_id]['angle'],
                            'ppm': ppm,
                            'target_id': self.car_target_map.get(marker_id),
                            'distance_to_target': 0.0
                        }

                        self.update_position_history(marker_id, cars_data[marker_id]['x'], 
                                                     cars_data[marker_id]['y'], elapsed_time)

            # Agregar carros no detectados
            for car_id in self.filtered_cars:
                if car_id not in cars_data:
                    cars_data[car_id] = {
                        'detected': False,
                        'x': 0, 'y': 0,
                        'x_px': self.filtered_cars[car_id]['x'] or 0,
                        'y_px': self.filtered_cars[car_id]['y'] or 0,
                        'angle': self.filtered_cars[car_id]['angle'] or 0,
                        'ppm': 1000,
                        'target_id': self.car_target_map.get(car_id),
                        'distance_to_target': None
                    }

            # Calcular distancia al objetivo asignado con calibracion
            for car_id in cars_data:
                car_data = cars_data[car_id]
                assigned_target_id = car_data.get('target_id')

                if not car_data['detected']:
                    car_data['distance_to_target'] = None
                    continue

                if assigned_target_id is None or assigned_target_id not in targets_data:
                    car_data['distance_to_target'] = None
                    continue

                target_data = targets_data[assigned_target_id]
                if target_data['detected']:
                    dx_pixels = target_data['x_px'] - car_data['x_px']
                    dy_pixels = target_data['y_px'] - car_data['y_px']
                    dist_pixels = math.sqrt(dx_pixels**2 + dy_pixels**2)

                    car_ppm = car_data['ppm']
                    if car_id not in self.filtered_ppm:
                        self.filtered_ppm[car_id] = car_ppm
                    self.filtered_ppm[car_id] = self.exponential_filter(
                        car_ppm, self.filtered_ppm[car_id], self.alpha_ppm
                    )

                    target_ppm_current = target_data.get('ppm', car_ppm)
                    if assigned_target_id not in self.target_ppm:
                        self.target_ppm[assigned_target_id] = target_ppm_current
                    else:
                        self.target_ppm[assigned_target_id] = self.exponential_filter(
                            target_ppm_current, self.target_ppm[assigned_target_id], self.alpha_ppm
                        )

                    avg_ppm = (self.filtered_ppm[car_id] + self.target_ppm[assigned_target_id]) / 2.0
                    car_data['distance_to_target'] = (dist_pixels / avg_ppm) * self.calibration_factor
                else:
                    car_data['distance_to_target'] = None

            # PUBLICAR POR ZMQ
            for car_id, car_data in cars_data.items():
                if car_data['detected']:
                    self.publish_pose(
                        car_id=car_id,
                        x_m=car_data['x'],
                        y_m=car_data['y'],
                        angle=car_data['angle'],
                        detected=True,
                        distance_to_target=car_data.get('distance_to_target', None),
                        target_id=car_data.get('target_id')
                    )
                    dist = car_data.get('distance_to_target')
                    dist_text = f"{dist:.3f}m" if dist is not None else "N/A"
                    print(
                        f"[ZMQ] Car {car_id}: x={car_data['x']:.3f}m, y={car_data['y']:.3f}m, "
                        f"angle={car_data['angle']:.1f}°, target={car_data.get('target_id')}, dist={dist_text}"
                    )

            # Dibujar
            frame = self.draw_visualization(frame, cars_data, targets_data)
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
            elif key == ord('s') or key == ord('S'):
                filename = f"aruco_capture_{frame_count:04d}.png"
                cv2.imwrite(filename, display)
                print(f"Captura guardada: {filename}")
            elif key == ord('c') or key == ord('C'):
                valid_distances = [
                    car_data['distance_to_target'] 
                    for car_data in cars_data.values() 
                    if car_data['detected'] and car_data['distance_to_target'] is not None
                ]
                if valid_distances:
                    detected_dist = valid_distances[0]
                    paused = True
                    self.calibrate_distance(detected_dist)
                else:
                    print("\nNo hay carros detectados para calibrar.")
            elif key == ord('+') or key == ord('='):
                self.calibration_factor += 0.01
                print(f"Factor de calibracion: {self.calibration_factor:.3f}")
            elif key == ord('-') or key == ord('_'):
                self.calibration_factor = max(0.01, self.calibration_factor - 0.01)
                print(f"Factor de calibracion: {self.calibration_factor:.3f}")
            elif key == ord(' ') and self.source_type == "video_file":
                paused = not paused
                print("Pausado" if paused else "Reanudado")
            elif key == 81 and self.source_type == "video_file":
                current_frame = int(self.cap.get(cv2.CAP_PROP_POS_FRAMES))
                self.cap.set(cv2.CAP_PROP_POS_FRAMES, max(0, current_frame - 5))
                paused = True
            elif key == 83 and self.source_type == "video_file":
                self.cap.set(cv2.CAP_PROP_POS_FRAMES, int(self.cap.get(cv2.CAP_PROP_POS_FRAMES)) + 5)
                paused = False

        self.cap.release()
        cv2.destroyAllWindows()
        self.socket.close()
        self.context.term()
        print("Programa finalizado.")


if __name__ == "__main__":
    ##url = "http://192.168.1.75:4747/video"
    url = r"C:\Users\jaide\Desktop\Proyecto-UnB-UDLA-IEEE-rob-tica-mobil\CoppeliaEsp\ArUco\tester4.mp4"

    marker_size_m = 0.1

    car_target_map = {
        0: 3,
        1: 4,        
        2: 5        
    }

    try:
        tracker = ArucoTracker(url, marker_size_m, car_target_map=car_target_map)
        tracker.run()
    except RuntimeError as e:
        print(f"Error: {e}")