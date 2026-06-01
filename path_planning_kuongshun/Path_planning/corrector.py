#!/usr/bin/env python3
"""
corrector.py

Corrector de trayectoria basado en ArUco con CORRECCIÓN RELATIVA DE POSICIÓN
y CORRECCIÓN ABSOLUTA DE ÁNGULO con offset de calibración.

ARQUITECTURA:
    live.py (ZMQ PUB) ──► corrector.py ──► lee pose real
    path_tracking_kuongshun.py ──► CoppeliaSim ──► pose simulada
                                      ▲
                                      │ (remote API)
    corrector.py ──► compara real vs sim ──► corrige ──► ESP32

Este archivo:
  1. Se suscribe a ZMQ para recibir poses reales del robot (desde live.py).
  2. Se conecta a CoppeliaSim para leer la pose simulada del robot.
  3. AL INICIO: captura pose inicial de ambos sistemas.
  4. POSICIÓN: error relativo = (sim - sim_inicial) - (real - real_inicial)
  5. ÁNGULO: error absoluto con offset = (sim - real) - offset_inicial
     donde offset_inicial = (sim_inicial - real_inicial)
     Esto equivale a: error_ang = (sim - sim_inicial) - (real - real_inicial)
     PERO con normalización correcta y manejo de wrap-around.
  6. Si hay error > umbral, calcula corrección PID y envía al ESP32.
  7. Si no hay error o no hay datos ArUco, no envía nada.

Uso:
    python corrector.py --car-id 2 --target-id 1 --esp32-ip 172.27.25.233
"""

import math
import time
import json
import socket
import threading

try:
    import zmq
except ImportError:
    raise ImportError("Instala pyzmq: pip install pyzmq")

try:
    import sim
except ImportError:
    print("[WARNING] sim.py no encontrado. El modo de lectura de CoppeliaSim no funcionara.")
    sim = None


# ═══════════════════════════════════════════════════════════════════════════════
# CONFIGURACIÓN — EDITAR AQUÍ LOS VALORES POR DEFECTO
# ═══════════════════════════════════════════════════════════════════════════════
DEFAULT_CAR_ID = 0
DEFAULT_TARGET_ID = 1
DEFAULT_ESP32_IP = "192.168.1.89"
DEFAULT_ESP32_PORT = 80
DEFAULT_ZMQ_ADDRESS = "tcp://localhost:5555"
DEFAULT_COPPELIA_ROBOT_NAME = "Car_A"
DEFAULT_COPPELIA_PORT = 19999

# PID de corrección
DEFAULT_KP_POS = 30.0       # Corrección PWM por metro de error relativo
DEFAULT_KP_ANGLE = 1.5      # Corrección PWM por grado de error angular
DEFAULT_KI_POS = 5.0        # Integral posicional
DEFAULT_KD_POS = 10.0       # Derivativo posicional

# Umbrales
DEFAULT_THRESHOLD_POS = 0.05    # metros: empieza a corregir
DEFAULT_THRESHOLD_ANGLE = 10.0  # grados: empieza a corregir
DEFAULT_CRITICAL_POS = 0.20     # metros: modo recovery
DEFAULT_CRITICAL_ANGLE = 30.0   # grados: modo recovery

# Timing
DEFAULT_CONTROL_HZ = 20.0   # Frecuencia de control


# ═══════════════════════════════════════════════════════════════════════════════
# ZMQ SUBSCRIBER
# ═══════════════════════════════════════════════════════════════════════════════
class ArucoSubscriber:
    """Lee poses de ArUco desde live.py via ZMQ."""

    def __init__(self, address="tcp://localhost:5555", topic="aruco", timeout=2.0):
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.SUB)
        self.socket.connect(address)
        self.socket.setsockopt_string(zmq.SUBSCRIBE, topic)
        self.socket.setsockopt(zmq.RCVTIMEO, 100)
        self.timeout = timeout
        self._cache = {}
        self._running = False
        self._thread = None
        print(f"[ZMQ] Subscriber conectado a {address}")

    def _receive_loop(self):
        while self._running:
            try:
                topic, payload = self.socket.recv_multipart()
                data = json.loads(payload.decode('utf-8'))
                self._cache[data['car_id']] = data
            except zmq.error.Again:
                continue
            except Exception as e:
                time.sleep(0.01)

    def start(self):
        self._running = True
        self._thread = threading.Thread(target=self._receive_loop, daemon=True)
        self._thread.start()
        print("[ZMQ] Recepción iniciada")

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=1.0)
        self.socket.close()
        self.context.term()

    def get_pose(self, car_id):
        data = self._cache.get(car_id)
        if data is None:
            return None
        age = time.time() - data.get('timestamp', 0)
        if age > self.timeout:
            return None
        return data


# ═══════════════════════════════════════════════════════════════════════════════
# COPPELIASIM READER
# ═══════════════════════════════════════════════════════════════════════════════
class CoppeliaReader:
    """Lee pose del robot desde CoppeliaSim."""

    def __init__(self, robot_name, port=19999, host="127.0.0.1"):
        self.robot_name = robot_name
        self.port = port
        self.host = host
        self.client_id = -1
        self.handle_robot = -1
        self._connected = False

    def connect(self):
        if sim is None:
            return False
        try:
            sim.simxFinish(-1)
            self.client_id = sim.simxStart(self.host, self.port, True, True, 2000, 5)
            if self.client_id == -1:
                print(f"[Coppelia] Error conectando a {self.host}:{self.port}")
                return False
            _, self.handle_robot = sim.simxGetObjectHandle(self.client_id, self.robot_name, sim.simx_opmode_blocking)
            if self.handle_robot == -1:
                print(f"[Coppelia] Robot '{self.robot_name}' no encontrado")
                return False
            sim.simxGetObjectPosition(self.client_id, self.handle_robot, -1, sim.simx_opmode_streaming)
            sim.simxGetObjectOrientation(self.client_id, self.handle_robot, -1, sim.simx_opmode_streaming)
            self._connected = True
            print(f"[Coppelia] Conectado. Robot '{self.robot_name}'")
            return True
        except Exception as e:
            print(f"[Coppelia] Error: {e}")
            return False

    def get_pose(self):
        if not self._connected or sim is None:
            return None
        try:
            _, pos = sim.simxGetObjectPosition(self.client_id, self.handle_robot, -1, sim.simx_opmode_buffer)
            _, orient = sim.simxGetObjectOrientation(self.client_id, self.handle_robot, -1, sim.simx_opmode_buffer)
            if pos is None or orient is None:
                return None
            return {
                'x': float(pos[0]),
                'y': float(pos[1]),
                'angle': math.degrees(float(orient[2]))
            }
        except Exception as e:
            return None

    def disconnect(self):
        if self.client_id != -1 and sim is not None:
            sim.simxFinish(self.client_id)
            self.client_id = -1
            self._connected = False
            print("[Coppelia] Desconectado")


# ═══════════════════════════════════════════════════════════════════════════════
# ESP32 CONTROLLER
# ═══════════════════════════════════════════════════════════════════════════════
class ESP32Controller:
    """Envía comandos PWM al ESP32 por TCP."""

    def __init__(self, ip, port=80):
        self.ip = ip
        self.port = port
        self.socket = None
        self.connected = False
        self._last_pwm = (0, 0)
        self._lock = threading.Lock()

    def connect(self):
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.settimeout(3.0)
            self.socket.connect((self.ip, self.port))
            self.connected = True
            print(f"[ESP32] Conectado a {self.ip}:{self.port}")
            return True
        except Exception as e:
            print(f"[ESP32] Error conexión: {e}")
            self.connected = False
            return False

    def send_pwm(self, left_pwm, right_pwm):
        with self._lock:
            left_pwm = int(max(-255, min(255, left_pwm)))
            right_pwm = int(max(-255, min(255, right_pwm)))

            if (left_pwm, right_pwm) == self._last_pwm:
                return True

            if not self.connected:
                if not self.connect():
                    return False

            try:
                msg = f"M:{left_pwm},{right_pwm}\n"
                self.socket.send(msg.encode())
                self._last_pwm = (left_pwm, right_pwm)
                return True
            except Exception as e:
                print(f"[ESP32] Error enviando: {e}")
                self.connected = False
                return False

    def send_stop(self):
        self.send_pwm(0, 0)

    def disconnect(self):
        with self._lock:
            if self.socket:
                try:
                    self.send_stop()
                    self.socket.close()
                except:
                    pass
            self.connected = False
            print("[ESP32] Desconectado")


# ═══════════════════════════════════════════════════════════════════════════════
# CORRECTOR PID RELATIVO (POSICIÓN) + ABSOLUTO CON OFFSET (ÁNGULO)
# ═══════════════════════════════════════════════════════════════════════════════
class CorrectorPID:
    """
    Calcula corrección basada en:
    - POSICIÓN: desviación relativa respecto al punto de partida
    - ÁNGULO: diferencia absoluta con offset de calibración inicial
    
    Esto elimina el offset de coordenadas para posición, pero maneja correctamente
    la diferencia de referencia angular entre los dos sistemas.
    """

    def __init__(self, kp_pos=30.0, kp_angle=1.5, ki_pos=5.0, kd_pos=10.0,
                 dt=0.05, threshold_pos=0.05, threshold_angle=10.0,
                 critical_pos=0.20, critical_angle=30.0):
        self.kp_pos = kp_pos
        self.kp_angle = kp_angle
        self.ki_pos = ki_pos
        self.kd_pos = kd_pos
        self.dt = dt
        self.threshold_pos = threshold_pos
        self.threshold_angle = threshold_angle
        self.critical_pos = critical_pos
        self.critical_angle = critical_angle

        # Poses iniciales (se capturan al iniciar)
        self.pose_real_initial = None
        self.pose_sim_initial = None
        self.angle_offset = 0.0  # Diferencia de ángulos al inicio: sim - real

        # Estado del PID
        self.integral_x = 0.0
        self.integral_y = 0.0
        self.prev_error_x = 0.0
        self.prev_error_y = 0.0

        self.correction_count = 0
        self.passthrough_count = 0

    def set_initial_poses(self, pose_real, pose_sim):
        """Captura las poses iniciales de ambos sistemas."""
        self.pose_real_initial = {
            'x': pose_real['x'],
            'y': pose_real['y'],
            'angle': self.normalize_angle(pose_real['angle'])
        }
        self.pose_sim_initial = {
            'x': pose_sim['x'],
            'y': pose_sim['y'],
            'angle': self.normalize_angle(pose_sim['angle'])
        }
        # Calcular offset angular: cuánto diffiere el sistema de coordenadas de CoppeliaSim del de la cámara
        self.angle_offset = self.normalize_angle(self.pose_sim_initial['angle'] - self.pose_real_initial['angle'])
        
        print(f"[INIT] Pose real inicial: ({pose_real['x']:.3f}, {pose_real['y']:.3f}, {pose_real['angle']:.1f}°)")
        print(f"[INIT] Pose sim inicial:  ({pose_sim['x']:.3f}, {pose_sim['y']:.3f}, {pose_sim['angle']:.1f}°)")
        print(f"[INIT] Offset angular: {self.angle_offset:.1f}° (sim - real)")

    def is_initialized(self):
        return self.pose_real_initial is not None and self.pose_sim_initial is not None

    def reset(self):
        self.integral_x = 0.0
        self.integral_y = 0.0
        self.prev_error_x = 0.0
        self.prev_error_y = 0.0
        self.pose_real_initial = None
        self.pose_sim_initial = None
        self.angle_offset = 0.0

    def normalize_angle(self, angle):
        while angle > 180:
            angle -= 360
        while angle < -180:
            angle += 360
        return angle

    def angle_difference(self, a, b):
        """Diferencia angular a -> b, resultado en [-180, 180]."""
        diff = b - a
        while diff > 180:
            diff -= 360
        while diff < -180:
            diff += 360
        return diff

    def compute(self, pose_real, pose_sim, pwm_base_left=0, pwm_base_right=0):
        """
        Calcula PWM corregido.

        POSICIÓN: error relativo = (sim - sim_inicial) - (real - real_inicial)
        ÁNGULO: error = (sim - real) - angle_offset
                = (sim - sim_inicial) - (real - real_inicial)  [matemáticamente equivalente]
                PERO calculado con función de diferencia angular robusta
        
        Args:
            pose_real: dict con 'x', 'y', 'angle' desde ArUco (posición actual)
            pose_sim: dict con 'x', 'y', 'angle' desde CoppeliaSim (posición actual)
            pwm_base_left, pwm_base_right: PWM que enviaría CoppeliaSim

        Returns:
            (pwm_left, pwm_right, debug_info)
        """
        # Si no está inicializado, no corregir
        if not self.is_initialized():
            debug = {
                'error_pos': 0.0,
                'error_angle': 0.0,
                'mode': 'not_initialized'
            }
            return pwm_base_left, pwm_base_right, debug

        # ═══════════════════════════════════════════════════════════════════
        # ERROR DE POSICIÓN (RELATIVO)
        # ═══════════════════════════════════════════════════════════════════
        dev_real_x = pose_real['x'] - self.pose_real_initial['x']
        dev_real_y = pose_real['y'] - self.pose_real_initial['y']

        dev_sim_x = pose_sim['x'] - self.pose_sim_initial['x']
        dev_sim_y = pose_sim['y'] - self.pose_sim_initial['y']

        error_x = dev_sim_x - dev_real_x
        error_y = dev_sim_y - dev_real_y
        error_pos = math.hypot(error_x, error_y)

        # ═══════════════════════════════════════════════════════════════════
        # ERROR DE ÁNGULO (CON OFFSET DE CALIBRACIÓN)
        # ═══════════════════════════════════════════════════════════════════
        # El ángulo del simulado "corregido" al frame de la cámara:
        # sim_angle_in_camera_frame = pose_sim['angle'] - angle_offset
        # error = sim_angle_in_camera_frame - pose_real['angle']
        #       = (pose_sim['angle'] - angle_offset) - pose_real['angle']
        # Pero angle_offset = sim_initial - real_initial
        # Entonces: error = (pose_sim['angle'] - sim_initial) - (pose_real['angle'] - real_initial)
        # Que es exactamente la desviación relativa angular.
        
        # Sin embargo, usamos una forma más robusta:
        # Calculamos la diferencia actual y restamos el offset inicial
        current_diff = self.angle_difference(pose_real['angle'], pose_sim['angle'])
        error_angle = self.normalize_angle(current_diff - self.angle_offset)
        
        # Alternativa equivalente (desviación relativa):
        # dev_real_angle = self.angle_difference(self.pose_real_initial['angle'], pose_real['angle'])
        # dev_sim_angle = self.angle_difference(self.pose_sim_initial['angle'], pose_sim['angle'])
        # error_angle = self.normalize_angle(dev_sim_angle - dev_real_angle)
        
        # Ambas deberían dar el mismo resultado. Usamos la primera por claridad.

        debug = {
            'dev_real': (dev_real_x, dev_real_y),
            'dev_sim': (dev_sim_x, dev_sim_y),
            'current_diff': current_diff,
            'angle_offset': self.angle_offset,
            'error_pos': error_pos,
            'error_angle': error_angle,
            'mode': 'passthrough'
        }

        # Si error es pequeño, no corregir
        if error_pos < self.threshold_pos and abs(error_angle) < self.threshold_angle:
            self.passthrough_count += 1
            return pwm_base_left, pwm_base_right, debug

        self.correction_count += 1

        # Detectar modo crítico
        critical = (error_pos > self.critical_pos) or (abs(error_angle) > self.critical_angle)
        factor = 2.0 if critical else 1.0
        debug['mode'] = 'critical' if critical else 'correction'

        # Rotar error al frame del robot REAL (usamos el ángulo real actual)
        robot_yaw = math.radians(pose_real['angle'])
        err_long = error_x * math.cos(robot_yaw) + error_y * math.sin(robot_yaw)
        err_lat = -error_x * math.sin(robot_yaw) + error_y * math.cos(robot_yaw)

        # Integral con anti-windup
        self.integral_x += error_x * self.dt
        self.integral_y += error_y * self.dt
        self.integral_x = max(-1.0, min(1.0, self.integral_x))
        self.integral_y = max(-1.0, min(1.0, self.integral_y))

        # Derivativo
        d_err_x = (error_x - self.prev_error_x) / self.dt
        d_err_y = (error_y - self.prev_error_y) / self.dt
        self.prev_error_x = error_x
        self.prev_error_y = error_y

        # Correcciones
        corr_long = factor * self.kp_pos * err_long
        corr_long += factor * self.ki_pos * (self.integral_x * math.cos(robot_yaw) + self.integral_y * math.sin(robot_yaw))
        corr_long += factor * self.kd_pos * (d_err_x * math.cos(robot_yaw) + d_err_y * math.sin(robot_yaw))

        corr_lat = factor * self.kp_pos * err_lat * 2.0
        corr_angle = factor * self.kp_angle * error_angle

        # Aplicar a PWM base
        pwm_left = pwm_base_left + corr_long - corr_lat - corr_angle
        pwm_right = pwm_base_right + corr_long + corr_lat + corr_angle

        # Saturar
        pwm_left = int(max(-255, min(255, pwm_left)))
        pwm_right = int(max(-255, min(255, pwm_right)))

        debug.update({
            'corr_long': corr_long,
            'corr_lat': corr_lat,
            'corr_angle': corr_angle,
            'pwm_base': (pwm_base_left, pwm_base_right),
            'pwm_corrected': (pwm_left, pwm_right)
        })

        return pwm_left, pwm_right, debug


# ═══════════════════════════════════════════════════════════════════════════════
# MAIN CORRECTOR
# ═══════════════════════════════════════════════════════════════════════════════
class Corrector:
    """Orquesta la corrección completa con inicialización relativa."""

    def __init__(self, car_id, target_id, esp32_ip, esp32_port,
                 zmq_address, coppelia_robot_name, coppelia_port,
                 kp_pos, kp_angle, ki_pos, kd_pos,
                 threshold_pos, threshold_angle,
                 critical_pos, critical_angle,
                 control_hz):

        self.car_id = car_id
        self.target_id = target_id
        self.dt = 1.0 / control_hz

        self.aruco = ArucoSubscriber(zmq_address)
        self.coppelia = CoppeliaReader(coppelia_robot_name, coppelia_port)
        self.esp32 = ESP32Controller(esp32_ip, esp32_port)
        self.pid = CorrectorPID(kp_pos, kp_angle, ki_pos, kd_pos, self.dt,
                                 threshold_pos, threshold_angle,
                                 critical_pos, critical_angle)

        self._running = False
        self._initialized = False

    def run(self, duration_sec=None):
        print("=" * 60)
        print("CORRECTOR DE TRAYECTORIA - MODO RELATIVO")
        print("=" * 60)
        print(f"Car ID (ArUco): {self.car_id}")
        print(f"Coppelia robot: {self.coppelia.robot_name}")
        print(f"ESP32: {self.esp32.ip}:{self.esp32.port}")
        print(f"Frecuencia: {1/self.dt:.1f} Hz")
        print("=" * 60)
        print("MODO RELATIVO: Se asume que al inicio ambos robots")
        print("están alineados. Solo se corrigen desviaciones relativas.")
        print("=" * 60)

        self.aruco.start()

        if not self.coppelia.connect():
            print("[ERROR] No se pudo conectar a CoppeliaSim. Abortando.")
            self.aruco.stop()
            return

        if not self.esp32.connect():
            print("[ERROR] No se pudo conectar al ESP32. Abortando.")
            self.coppelia.disconnect()
            self.aruco.stop()
            return

        self._running = True
        t_start = time.time()
        last_status = t_start
        pose_loss_count = 0

        try:
            while self._running:
                loop_start = time.time()

                # Leer poses
                pose_real = self.aruco.get_pose(self.car_id)
                pose_sim = self.coppelia.get_pose()

                # Fallback sin ArUco
                if pose_real is None:
                    pose_loss_count += 1
                    if pose_loss_count > 20:
                        print(f"[SAFETY] Robot {self.car_id} no detectado. Deteniendo corrección.")
                        self.esp32.send_stop()
                    time.sleep(self.dt)
                    continue

                pose_loss_count = 0

                # Fallback sin CoppeliaSim
                if pose_sim is None:
                    print("[WARNING] No se pudo leer pose de CoppeliaSim.")
                    time.sleep(self.dt)
                    continue

                # INICIALIZACIÓN: Capturar poses iniciales en el primer ciclo válido
                if not self._initialized:
                    self.pid.set_initial_poses(pose_real, pose_sim)
                    self._initialized = True
                    print("[INIT] Corrector inicializado con poses de referencia.")
                    print("[INIT] Esperando 2 segundos para estabilizar...")
                    time.sleep(2.0)
                    continue

                # Calcular corrección relativa (modo paralelo: PWM base = 0)
                pwm_left, pwm_right, debug = self.pid.compute(pose_real, pose_sim, 0, 0)

                # Solo enviar si hay corrección activa
                if debug['mode'] != 'passthrough':
                    self.esp32.send_pwm(pwm_left, pwm_right)
                    print(f"[CORRECTION] RelErrPos={debug['error_pos']:.3f}m, "
                          f"RelErrAng={debug['error_angle']:.1f}° | "
                          f"Mode={debug['mode']} | "
                          f"PWM=({pwm_left}, {pwm_right})")

                # Status periódico
                if time.time() - last_status > 3.0:
                    dev_real = debug.get('dev_real', (0, 0))
                    dev_sim = debug.get('dev_sim', (0, 0))
                    print(f"[STATUS] Real=({pose_real['x']:.3f}, {pose_real['y']:.3f}, {pose_real['angle']:.1f}°) | "
                          f"Sim=({pose_sim['x']:.3f}, {pose_sim['y']:.3f}, {pose_sim['angle']:.1f}°) | "
                          f"DevReal=({dev_real[0]:.3f}, {dev_real[1]:.3f}) | "
                          f"DevSim=({dev_sim[0]:.3f}, {dev_sim[1]:.3f}) | "
                          f"RelErr=({debug['error_pos']:.3f}m, {debug['error_angle']:.1f}°) | "
                          f"Corr={self.pid.correction_count}, Pass={self.pid.passthrough_count}")
                    last_status = time.time()

                # Timing
                elapsed = time.time() - loop_start
                sleep_time = self.dt - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)

                if duration_sec and (time.time() - t_start) > duration_sec:
                    break

        except KeyboardInterrupt:
            print("[USER] Interrupción.")

        finally:
            self._running = False
            self.esp32.send_stop()
            self.esp32.disconnect()
            self.coppelia.disconnect()
            self.aruco.stop()
            print("[SHUTDOWN] Corrector detenido.")


# ═══════════════════════════════════════════════════════════════════════════════
# ENTRY POINT
# ═══════════════════════════════════════════════════════════════════════════════
if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Corrector de trayectoria ArUco + CoppeliaSim + ESP32 (MODO RELATIVO)")
    parser.add_argument("--car-id", type=int, default=DEFAULT_CAR_ID)
    parser.add_argument("--target-id", type=int, default=DEFAULT_TARGET_ID)
    parser.add_argument("--esp32-ip", default=DEFAULT_ESP32_IP)
    parser.add_argument("--esp32-port", type=int, default=DEFAULT_ESP32_PORT)
    parser.add_argument("--zmq-addr", default=DEFAULT_ZMQ_ADDRESS)
    parser.add_argument("--robot-name", default=DEFAULT_COPPELIA_ROBOT_NAME)
    parser.add_argument("--coppelia-port", type=int, default=DEFAULT_COPPELIA_PORT)
    parser.add_argument("--kp-pos", type=float, default=DEFAULT_KP_POS)
    parser.add_argument("--kp-angle", type=float, default=DEFAULT_KP_ANGLE)
    parser.add_argument("--ki-pos", type=float, default=DEFAULT_KI_POS)
    parser.add_argument("--kd-pos", type=float, default=DEFAULT_KD_POS)
    parser.add_argument("--threshold-pos", type=float, default=DEFAULT_THRESHOLD_POS)
    parser.add_argument("--threshold-angle", type=float, default=DEFAULT_THRESHOLD_ANGLE)
    parser.add_argument("--critical-pos", type=float, default=DEFAULT_CRITICAL_POS)
    parser.add_argument("--critical-angle", type=float, default=DEFAULT_CRITICAL_ANGLE)
    parser.add_argument("--hz", type=float, default=DEFAULT_CONTROL_HZ)
    parser.add_argument("--duration", type=float, default=None)

    args = parser.parse_args()

    corrector = Corrector(
        car_id=args.car_id,
        target_id=args.target_id,
        esp32_ip=args.esp32_ip,
        esp32_port=args.esp32_port,
        zmq_address=args.zmq_addr,
        coppelia_robot_name=args.robot_name,
        coppelia_port=args.coppelia_port,
        kp_pos=args.kp_pos,
        kp_angle=args.kp_angle,
        ki_pos=args.ki_pos,
        kd_pos=args.kd_pos,
        threshold_pos=args.threshold_pos,
        threshold_angle=args.threshold_angle,
        critical_pos=args.critical_pos,
        critical_angle=args.critical_angle,
        control_hz=args.hz
    )

    corrector.run(duration_sec=args.duration)