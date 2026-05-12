import cv2
import numpy as np
import math
import matplotlib.pyplot as plt
from datetime import datetime
import os
import time
from aruco_communicator import ArucoDataPublisher

class ArucoTracker:
    def __init__(self, url, marker_size_m=0.1):
        """
        Inicializa el tracker de marcadores ArUco

        Args:
            url: URL de la camara
            marker_size_m: Tamaño real del marcador en metros
        """
        self.url = url
        self.marker_size_m = marker_size_m

        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

        # Configuracion de IDs
        self.target_id = 2  # ID del marcador del objetivo

        # Variables de estado 
        self.cars = {}  # {car_id: {...datos...}}
        self.filtered_cars = {}  # {car_id: {'x': None, 'y': None, 'angle': None}}
        self.filtered_target = {'x': None, 'y': None, 'angle': None}
        
        # Historial de posiciones para graficas
        self.position_history = {}  # {car_id: {'x': [...], 'y': [...], 'time': [...]}}
        self.start_time = None
        self.frame_count = 0

        # Filtros para suavizado
        self.alpha_pos = 0.3
        self.alpha_angle = 0.3
        self.alpha_ppm = 0.2  # Filtro para pixeles por metro

        # Umbral de movimiento
        self.movement_threshold = 2
        
        # Variables para calibracion de distancia
        self.target_ppm = None  # Pixeles por metro del objetivo
        self.filtered_ppm = {}  # {car_id: filtered_ppm}
        self.calibration_factor = 0.750  # Factor de calibracion ajustable (default 0.750)
        
        # ArUco Data Publisher para comunicación con follow_ball.py
        self.publisher = ArucoDataPublisher(data_dir=".")

        # Conectar a la camara o video
        self.source_type = self._detect_source_type(url)
        self.cap = cv2.VideoCapture(url)
        
        if not self.cap.isOpened():
            raise RuntimeError(f"No se pudo abrir la fuente: {url}")

        # Configurar resolucion de la camara (1080x1080 para pantalla cuadrada)
        # Solo para streams en vivo, no para archivos locales
        if self.source_type == "stream":
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1080)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

        # Ventana de visualizacion - RESIZABLE
        self.window_name = "ArUco Tracker - Multiples Carros vs Objetivo"
        self.max_display_width = 1280  # Ancho maximo de la ventana
        self.max_display_height = 720  # Alto maximo de la ventana

    def _detect_source_type(self, source):
        """Detecta si la fuente es un archivo local o una URL"""
        # Verificar si es una URL (http, https, rtsp, etc.)
        if source.startswith(('http://', 'https://', 'rtsp://', 'rtmp://')):
            return "stream"
        # Verificar si es un archivo local
        elif os.path.isfile(source):
            return "video_file"
        # Si comienza con ./ o ../ es un archivo local relativo
        elif source.startswith(('./', '../')) or source.startswith('.\\'):
            return "video_file"
        # Por defecto, asumir que es un archivo local (puede ser camara con indice numerico)
        else:
            try:
                # Si puede convertirse a entero, es un indice de camara
                int(source)
                return "webcam"
            except ValueError:
                return "video_file"

    def normalize_angle(self, angle):
        """Normaliza el angulo al rango [-180°, 180°]"""
        while angle > 180:
            angle -= 360
        while angle < -180:
            angle += 360
        return angle

    def exponential_filter(self, current_value, previous_filtered, alpha):
        """Aplica filtro exponencial"""
        if previous_filtered is None:
            return current_value
        return alpha * current_value + (1 - alpha) * previous_filtered

    def calculate_marker_info(self, corner):
        """Extrae informacion de un marcador desde sus esquinas"""
        c = corner.reshape(4, 2).astype(np.float32)

        # Centro del marcador
        center_x = np.mean(c[:, 0])
        center_y = np.mean(c[:, 1])

        # Calcular escala (pixeles por metro)
        marker_width_px = np.linalg.norm(c[0] - c[1])
        pixels_per_meter = marker_width_px / self.marker_size_m if marker_width_px > 0 else 1000

        # Calcular orientacion (angulo desde el centro al borde superior)
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
        """Aplica filtro exponencial a la informacion del marcador"""
        cx, cy = marker_info['center']

        filtered_data['x'] = self.exponential_filter(cx, filtered_data['x'], self.alpha_pos)
        filtered_data['y'] = self.exponential_filter(cy, filtered_data['y'], self.alpha_pos)

        # Filtrado especial para angulo (manejo de wrap-around)
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
        """Actualiza el historial de posiciones para un carro"""
        if car_id not in self.position_history:
            self.position_history[car_id] = {'x': [], 'y': [], 'time': []}
        
        self.position_history[car_id]['x'].append(x_m)
        self.position_history[car_id]['y'].append(y_m)
        self.position_history[car_id]['time'].append(elapsed_time)

    def draw_info_panel(self, frame, cars_data, target_data):
        """Dibuja el panel de informacion en la parte derecha"""
        h, w = frame.shape[:2]
        panel_width = 350

        # Crear panel negro semi-transparente
        panel = np.zeros((h, panel_width, 3), dtype=np.uint8)
        panel[:] = (30, 30, 30)

        # Combinar frame con panel
        combined = np.zeros((h, w + panel_width, 3), dtype=np.uint8)
        combined[:, :w] = frame
        combined[:, w:] = panel

        # Configuracion de texto
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.55
        line_height = 22
        margin_x = w + 15
        start_y = 35

        # Funcion auxiliar para dibujar texto
        def draw_text(text, y, color=(255, 255, 255), scale=font_scale, thickness=1):
            cv2.putText(combined, text, (margin_x, y), font, scale, color, thickness)
            return y + line_height

        y = start_y

        # Titulo
        y = draw_text("=== ARUCO TRACKER ===", y, (0, 255, 255), 0.7, 2)
        y += 10

        # === INFORMACIoN DEL OBJETIVO ===
        y = draw_text("OBJETIVO (ID {})".format(self.target_id), y, (0, 255, 0), 0.65, 2)

        if target_data['detected']:
            y = draw_text(f"  Pos. Pixel: ({target_data['x']:.0f}, {target_data['y']:.0f})", y, (200, 200, 200))
            y = draw_text(f"  Angulo: {target_data['angle']:.1f} deg", y, (200, 200, 200))
        else:
            y = draw_text("  No detectado", y, (100, 100, 100))

        y += 10

        # === INFORMACIoN DE LOS CARROS ===
        y = draw_text("CARROS DETECTADOS", y, (0, 165, 255), 0.65, 2)

        if cars_data:
            for car_id, car_data in sorted(cars_data.items()):
                y = draw_text(f"  ID {car_id}:", y, (100, 200, 255), 0.55, 1)
                if car_data['detected']:
                    y = draw_text(f"    Pixel: ({car_data['x']:.0f}, {car_data['y']:.0f})", y, (200, 200, 200))
                    y = draw_text(f"    Angulo: {car_data['angle']:.1f}°", y, (200, 200, 200))
                    
                    # Mostrar distancia al objetivo si esta disponible
                    if 'distance_to_target' in car_data and car_data['distance_to_target'] is not None:
                        y = draw_text(f"    Distancia: {car_data['distance_to_target']:.3f} m", y, (0, 255, 255), 0.55, 2)
                        # Mostrar ppm para debugging
                        if 'ppm' in car_data:
                            y = draw_text(f"    PPM: {car_data['ppm']:.1f} px/m", y, (150, 150, 150), 0.45)
                    else:
                        y = draw_text(f"    Distancia: N/A", y, (100, 100, 100))
                else:
                    y = draw_text(f"    No detectado", y, (100, 100, 100))
        else:
            y = draw_text("  Ninguno detectado", y, (100, 100, 100))

        y += 15

        # === CONTROLES ===
        y = draw_text("=== CONTROLES ===", y, (200, 200, 200), 0.6, 1)
        y = draw_text("  ESC: Salir y graficar", y, (150, 150, 150), 0.5)
        y = draw_text("  S: Guardar captura", y, (150, 150, 150), 0.5)
        y = draw_text("  +/-: Ajustar calibracion", y, (150, 150, 150), 0.5)
        
        y += 10
        
        # === INFORMACIoN DE CALIBRACIoN ===
        y = draw_text("=== CALIBRACIoN ===", y, (255, 200, 100), 0.6, 1)
        y = draw_text(f"  Factor: {self.calibration_factor:.3f}", y, (255, 200, 100), 0.55, 2)
        if self.target_ppm is not None:
            y = draw_text(f"  Target PPM: {self.target_ppm:.1f}", y, (150, 150, 150), 0.45)
        
        # Mostrar instrucciones de calibracion
        y = draw_text("  'C' calibrate", y, (150, 200, 100), 0.45)

        return combined

    def draw_visualization(self, frame, cars_data, target_data):
        """Dibuja elementos visuales en el frame"""
        h, w = frame.shape[:2]

        # Dibujar marcadores de los carros
        for car_id, car_data in cars_data.items():
            if car_data['detected']:
                cx, cy = int(car_data['x']), int(car_data['y'])

                # Circulo central con color unico por ID
                color = (int(100 + car_id * 40) % 256, int(165 + car_id * 20) % 256, 255)
                cv2.circle(frame, (cx, cy), 8, color, -1)
                cv2.circle(frame, (cx, cy), 10, (255, 255, 255), 2)

                # Flecha de orientacion
                arrow_len = 40
                angle_rad = math.radians(car_data['angle'])
                end_x = int(cx + arrow_len * math.cos(angle_rad))
                end_y = int(cy + arrow_len * math.sin(angle_rad))
                cv2.arrowedLine(frame, (cx, cy), (end_x, end_y), color, 3, tipLength=0.3)

                # Etiqueta
                cv2.putText(frame, f"CARRO {car_id}", (cx + 15, cy - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

        # Dibujar marcador del objetivo
        if target_data['detected']:
            cx, cy = int(target_data['x']), int(target_data['y'])

            # Circulo central
            cv2.circle(frame, (cx, cy), 8, (0, 255, 0), -1)
            cv2.circle(frame, (cx, cy), 10, (255, 255, 255), 2)

            # Flecha de orientacion
            arrow_len = 40
            angle_rad = math.radians(target_data['angle'])
            end_x = int(cx + arrow_len * math.cos(angle_rad))
            end_y = int(cy + arrow_len * math.sin(angle_rad))
            cv2.arrowedLine(frame, (cx, cy), (end_x, end_y), (0, 255, 0), 3, tipLength=0.3)

            # Etiqueta
            cv2.putText(frame, "OBJETIVO", (cx + 15, cy - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            # Dibujar lineas desde cada carro al objetivo con distancia
            for car_id, car_data in cars_data.items():
                if car_data['detected'] and 'distance_to_target' in car_data:
                    car_cx, car_cy = int(car_data['x']), int(car_data['y'])
                    target_cx, target_cy = int(target_data['x']), int(target_data['y'])
                    
                    # Linea entre carro y objetivo
                    cv2.line(frame, (car_cx, car_cy), (target_cx, target_cy), (255, 255, 0), 2)
                    
                    # Punto medio para mostrar distancia
                    mid_x = (car_cx + target_cx) // 2
                    mid_y = (car_cy + target_cy) // 2
                    
                    # Texto de distancia
                    dist_text = f"{car_data['distance_to_target']:.2f}m"
                    (text_w, text_h), _ = cv2.getTextSize(dist_text, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
                    
                    # Fondo para el texto
                    cv2.rectangle(frame, (mid_x - text_w//2 - 5, mid_y - text_h - 5),
                                 (mid_x + text_w//2 + 5, mid_y + 5), (0, 0, 0), -1)
                    cv2.putText(frame, dist_text, (mid_x - text_w//2, mid_y), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)

        return frame

    def calibrate_distance(self, detected_distance):
        """
        Calibra el factor de conversion basado en una distancia real conocida
        
        Args:
            detected_distance: Distancia detectada por el sistema (en metros)
        """
        print("\n" + "="*60)
        print("MODO CALIBRACIoN DE DISTANCIA")
        print("="*60)
        print(f"Distancia detectada: {detected_distance:.3f} m")
        
        try:
            real_distance = float(input("Ingresa la distancia REAL medida (en metros): "))
            if real_distance <= 0:
                print("Error: La distancia debe ser mayor a 0")
                return
            
            # Calcular el nuevo factor de calibracion
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

    def generate_trajectory_plots(self):
        """Genera graficas de las trayectorias de los carros"""
        if not self.position_history:
            print("No hay historial de posiciones para graficar.")
            return

        # Crear figura con subplots
        num_cars = len(self.position_history)
        fig, axes = plt.subplots(1, num_cars + 1, figsize=(15, 5))
        
        # Asegurar que axes sea siempre una lista iterable
        if num_cars == 1:
            axes = [axes, fig.add_subplot(1, num_cars + 1, num_cars + 1)]
        else:
            axes = list(axes)

        # Colores para cada carro
        colors = plt.cm.get_cmap('tab10')

        # Graficar trayectoria de cada carro en el plano X-Y
        for idx, (car_id, history) in enumerate(sorted(self.position_history.items())):
            ax = axes[idx]
            ax.plot(history['x'], history['y'], 'o-', label=f'Carro {car_id}', 
                   color=colors(idx), markersize=3, linewidth=2)
            ax.set_xlabel('X (metros)')
            ax.set_ylabel('Y (metros)')
            ax.set_title(f'Trayectoria - Carro {car_id}')
            ax.grid(True, alpha=0.3)
            ax.legend()
            ax.axis('equal')

        # Grafica final: todas las trayectorias juntas
        ax_all = axes[-1]
        for idx, (car_id, history) in enumerate(sorted(self.position_history.items())):
            ax_all.plot(history['x'], history['y'], 'o-', label=f'Carro {car_id}',
                       color=colors(idx), markersize=3, linewidth=2)
            # Marcar inicio y fin
            ax_all.plot(history['x'][0], history['y'][0], 'g^', markersize=10, label=f'Inicio {car_id}')
            ax_all.plot(history['x'][-1], history['y'][-1], 'rs', markersize=10, label=f'Fin {car_id}')

        ax_all.set_xlabel('X (metros)')
        ax_all.set_ylabel('Y (metros)')
        ax_all.set_title('Todas las Trayectorias')
        ax_all.grid(True, alpha=0.3)
        ax_all.legend(fontsize=8)
        ax_all.axis('equal')

        plt.tight_layout()
        
        # Guardar figura
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"trayectorias_{timestamp}.png"
        plt.savefig(filename, dpi=150, bbox_inches='tight')
        print(f"Grafica guardada: {filename}")
        
        # Mostrar grafica
        plt.show()

        # Crear grafica adicional: Posicion vs Tiempo
        fig2, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8))

        for idx, (car_id, history) in enumerate(sorted(self.position_history.items())):
            ax1.plot(history['time'], history['x'], 'o-', label=f'X - Carro {car_id}',
                    color=colors(idx), markersize=4, linewidth=2)
            ax2.plot(history['time'], history['y'], 'o-', label=f'Y - Carro {car_id}',
                    color=colors(idx), markersize=4, linewidth=2)

        ax1.set_xlabel('Tiempo (s)')
        ax1.set_ylabel('Posicion X (metros)')
        ax1.set_title('Posicion X vs Tiempo')
        ax1.grid(True, alpha=0.3)
        ax1.legend()

        ax2.set_xlabel('Tiempo (s)')
        ax2.set_ylabel('Posicion Y (metros)')
        ax2.set_title('Posicion Y vs Tiempo')
        ax2.grid(True, alpha=0.3)
        ax2.legend()

        plt.tight_layout()
        
        # Guardar figura
        filename2 = f"posiciones_tiempo_{timestamp}.png"
        plt.savefig(filename2, dpi=150, bbox_inches='tight')
        print(f"Grafica guardada: {filename2}")
        
        plt.close('all')  # Cerrar todas las figuras

    def run(self):
        """Ejecuta el tracking principal"""
        print("=" * 60)
        print("ARUCO TRACKER - Multiples Carros vs Objetivo")
        print("=" * 60)
        print(f"Tipo de fuente: {self.source_type.upper()}")
        print(f"  URL/Archivo: {self.url}")
        print(f"Objetivo: ID {self.target_id}")
        print("Otros IDs seran tratados como carros")
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

        # Crear ventana resizable
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

            # Calcular tiempo transcurrido
            elapsed_time = (datetime.now() - self.start_time).total_seconds()

            # Obtener dimensiones originales de la camara
            h, w = frame.shape[:2]

            # Redimensionar manteniendo la proporcion
            scale = min(self.max_display_width / w, self.max_display_height / h)
            new_w = int(w * scale)
            new_h = int(h * scale)

            if new_w != w or new_h != h:
                frame = cv2.resize(frame, (new_w, new_h), interpolation=cv2.INTER_LINEAR)

            # Datos de los marcadores
            cars_data = {}
            target_data = {'detected': False, 'x': 0, 'y': 0, 'angle': 0}

            # Detectar marcadores
            corners, ids, _ = cv2.aruco.detectMarkers(frame, self.dictionary)

            if ids is not None:
                cv2.aruco.drawDetectedMarkers(frame, corners, ids)
                print(f"[ArUco] Detected {len(ids)} markers: {ids.flatten().tolist()}")

                # Procesar cada marcador detectado
                for corner, marker_id in zip(corners, ids.flatten()):
                    info = self.calculate_marker_info(corner)

                    if marker_id == self.target_id:
                        # Procesar objetivo
                        self.filtered_target = self.apply_filter_to_marker(info, self.filtered_target)
                        target_data['detected'] = True
                        target_data['x'] = self.filtered_target['x']
                        target_data['y'] = self.filtered_target['y']
                        target_data['angle'] = self.filtered_target['angle']
                        target_data['ppm'] = info['pixels_per_meter']
                    else:
                        # Procesar carro
                        if marker_id not in self.filtered_cars:
                            self.filtered_cars[marker_id] = {'x': None, 'y': None, 'angle': None}
                        
                        self.filtered_cars[marker_id] = self.apply_filter_to_marker(info, self.filtered_cars[marker_id])
                        
                        cars_data[marker_id] = {
                            'detected': True,
                            'x': self.filtered_cars[marker_id]['x'],
                            'y': self.filtered_cars[marker_id]['y'],
                            'angle': self.filtered_cars[marker_id]['angle'],
                            'ppm': info['pixels_per_meter'],
                            'distance_to_target': 0.0
                        }
                        
                        # Actualizar historial de posiciones
                        # Convertir pixeles a metros (usar ppm del marcador)
                        x_m = cars_data[marker_id]['x'] / info['pixels_per_meter']
                        y_m = cars_data[marker_id]['y'] / info['pixels_per_meter']
                        self.update_position_history(marker_id, x_m, y_m, elapsed_time)

            # Agregar carros que no fueron detectados en este frame
            for car_id in self.filtered_cars:
                if car_id not in cars_data:
                    cars_data[car_id] = {
                        'detected': False,
                        'x': self.filtered_cars[car_id]['x'] if self.filtered_cars[car_id]['x'] else 0,
                        'y': self.filtered_cars[car_id]['y'] if self.filtered_cars[car_id]['y'] else 0,
                        'angle': self.filtered_cars[car_id]['angle'] if self.filtered_cars[car_id]['angle'] else 0,
                        'ppm': 1000,
                        'distance_to_target': 0.0
                    }
            
            # Calcular distancia de cada carro al objetivo
            if target_data['detected']:
                for car_id in cars_data:
                    if cars_data[car_id]['detected']:
                        # Calcular distancia en pixeles
                        dx_pixels = target_data['x'] - cars_data[car_id]['x']
                        dy_pixels = target_data['y'] - cars_data[car_id]['y']
                        dist_pixels = math.sqrt(dx_pixels**2 + dy_pixels**2)
                        
                        # Obtener y filtrar el ppm del carro
                        car_ppm = cars_data[car_id]['ppm']
                        if car_id not in self.filtered_ppm:
                            self.filtered_ppm[car_id] = car_ppm
                        self.filtered_ppm[car_id] = self.exponential_filter(
                            car_ppm, self.filtered_ppm[car_id], self.alpha_ppm
                        )
                        
                        # Filtrar el ppm del objetivo si lo detectamos
                        if self.target_ppm is None:
                            self.target_ppm = target_data.get('ppm', car_ppm)
                        else:
                            target_ppm_current = target_data.get('ppm', self.target_ppm)
                            self.target_ppm = self.exponential_filter(
                                target_ppm_current, self.target_ppm, self.alpha_ppm
                            )
                        
                        # Usar el promedio del ppm del carro y el objetivo para mayor precision
                        avg_ppm = (self.filtered_ppm[car_id] + self.target_ppm) / 2.0
                        
                        # Convertir a metros y aplicar factor de calibracion
                        cars_data[car_id]['distance_to_target'] = (dist_pixels / avg_ppm) * self.calibration_factor
                    else:
                        cars_data[car_id]['distance_to_target'] = None

            # PUBLICAR DATOS DE ARUCO A JSON PARA follow_ball.py
            for car_id, car_data in cars_data.items():
                if car_data['detected']:
                    # Convertir píxeles a metros usando PPM del marcador
                    ppm = float(car_data.get('ppm', 1000))
                    x_m = float(car_data['x']) / ppm if ppm > 0 else 0.0
                    y_m = float(car_data['y']) / ppm if ppm > 0 else 0.0
                    
                    success = self.publisher.publish(int(car_id), {
                        'x': float(x_m),                                    # Metros (convertido)
                        'y': float(y_m),                                    # Metros (convertido)
                        'angle': float(car_data['angle']),                  # Grados
                        'detected': bool(True),
                        'distance_to_target': float(car_data.get('distance_to_target', 0.0)),
                        'timestamp': time.time()
                    })
                    if success:
                        print(f"[ArUco Publish] Car {car_id}: x={x_m:.3f}m, y={y_m:.3f}m, angle={car_data['angle']:.1f}°, dist={car_data.get('distance_to_target', 0.0):.3f}m")

            # Dibujar visualizacion
            frame = self.draw_visualization(frame, cars_data, target_data)

            # Crear panel de informacion
            display = self.draw_info_panel(frame, cars_data, target_data)
            
            # Agregar indicador de pausa si esta pausado
            if paused:
                cv2.putText(display, "PAUSADO", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 0, 255), 3)

            # Mostrar frame
            cv2.imshow(self.window_name, display)

            # Ajustar velocidad segun el tipo de fuente
            if self.source_type == "video_file":
                wait_time = 33  # ~30 fps para archivos locales
            else:
                wait_time = 1   # Maxima velocidad para streams en vivo

            key = cv2.waitKey(wait_time) & 0xFF

            if key == 27:  # ESC
                print("\nGenerando graficas...")
                self.generate_trajectory_plots()
                break
            elif key == ord('s') or key == ord('S'):  # Guardar captura
                filename = f"aruco_capture_{frame_count:04d}.png"
                cv2.imwrite(filename, display)
                print(f"Captura guardada: {filename}")
            elif key == ord('c') or key == ord('C'):  # Calibracion de distancia
                # Encontrar la primera distancia valida del carro mas cercano
                valid_distances = [
                    car_data['distance_to_target'] 
                    for car_data in cars_data.values() 
                    if car_data['detected'] and car_data['distance_to_target'] is not None
                ]
                
                if valid_distances:
                    # Usar la distancia del primer carro detectado
                    detected_dist = valid_distances[0]
                    paused = True  # Pausar el video durante la calibracion
                    self.calibrate_distance(detected_dist)
                else:
                    print("\nNo hay carros detectados para calibrar. Asegurate de que el objetivo y al menos un carro esten visibles.")
            elif key == ord('+') or key == ord('='):  # Aumentar calibracion
                self.calibration_factor += 0.01
                print(f"Factor de calibracion: {self.calibration_factor:.3f}")
            elif key == ord('-') or key == ord('_'):  # Disminuir calibracion
                self.calibration_factor = max(0.01, self.calibration_factor - 0.01)
                print(f"Factor de calibracion: {self.calibration_factor:.3f}")
            elif key == ord(' ') and self.source_type == "video_file":  # Espacio - Pausar/Reanudar
                paused = not paused
                status = "Pausado" if paused else "Reanudado"
                print(f"{status}")
            elif key == 81 and self.source_type == "video_file":  # Flecha izquierda - Retroceder
                current_frame = int(self.cap.get(cv2.CAP_PROP_POS_FRAMES))
                self.cap.set(cv2.CAP_PROP_POS_FRAMES, max(0, current_frame - 5))
                paused = True
            elif key == 83 and self.source_type == "video_file":  # Flecha derecha - Avanzar
                self.cap.set(cv2.CAP_PROP_POS_FRAMES, int(self.cap.get(cv2.CAP_PROP_POS_FRAMES)) + 5)
                paused = False
        
        self.cap.release()
        cv2.destroyAllWindows()
        print("Programa finalizado.")



if __name__ == "__main__":
    
    url = "http://192.168.1.74:4747/video"    
   
    # url = "C:\\Users\\jaide\\OneDrive\\Escritorio\\Proyecto UnB-UDLA-IEEE robotica mobil\\CoppeliaEsp\\ArUco\\tester3.mp4"
    # url = "0"  # Webcam por defecto
        
    marker_size_m = 0.1  # 10 cm

    try:
        tracker = ArucoTracker(url, marker_size_m)
        tracker.run()
    except RuntimeError as e:
        print(f"Error: {e}")
        
