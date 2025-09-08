import threading
import queue
import time
import socket
import pygame
import cv2
from typing import Optional

try:
    from ultralytics import YOLO
except Exception:
    YOLO = None

# ========= Shared Fire Flag =========
fire_detected = False

# ================== ESP8266 Joystick Sender ==================
class ESP8266Joystick(threading.Thread):
    def __init__(self, discovery_port=5006, data_port=5005, update_rate=0.02):
        super().__init__(daemon=True, name="ESP8266Joystick")
        self.discovery_port = discovery_port
        self.data_port = data_port
        self.update_rate = update_rate
        self.stop_event = threading.Event()
        self.sock = None
        self.udp_ip = None

    def discover(self):
        sock_discovery = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock_discovery.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        sock_discovery.bind(("", self.discovery_port))
        print("🔍 Waiting for ESP8266 to broadcast its IP...")
        data, addr = sock_discovery.recvfrom(1024)
        udp_ip = data.decode().strip()
        if "ESP8266_IP:" in udp_ip:
            udp_ip = udp_ip.split(":")[1]
        print(f"✅ Discovered ESP8266 at {udp_ip}")
        sock_discovery.close()
        return udp_ip

    def apply_deadzone(self, value, threshold=0.15):
        return 0.0 if abs(value) < threshold else value

    def map_range(self, value, in_min=-1.0, in_max=1.0, out_min=0, out_max=1023):
        return int((value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

    def run(self):
        global fire_detected
        try:
            self.udp_ip = self.discover()
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

            pygame.init()
            pygame.joystick.init()
            if pygame.joystick.get_count() == 0:
                print("❌ No joystick found. Please connect a controller.")
                return

            joystick = pygame.joystick.Joystick(0)
            joystick.init()
            print(f"🎮 Using joystick: {joystick.get_name()}")

            while not self.stop_event.is_set():
                pygame.event.pump()

                lx = self.apply_deadzone(round(joystick.get_axis(0), 2))
                ly = self.apply_deadzone(round(joystick.get_axis(1), 2))
                rx = self.apply_deadzone(round(joystick.get_axis(2), 2))
                ry = self.apply_deadzone(round(joystick.get_axis(3), 2))

                btn_cross    = joystick.get_button(0)
                btn_circle   = joystick.get_button(1)
                btn_square   = joystick.get_button(2)
                btn_triangle = joystick.get_button(3)

                # 🔥 Force Button O HIGH if fire detected
                if fire_detected:
                    btn_circle = 1

                current_values = (
                    round(self.map_range(lx) / 5) * 5 if lx != 0.0 else 512,
                    round(self.map_range(ly) / 5) * 5 if ly != 0.0 else 512,
                    round(self.map_range(rx) / 5) * 5 if rx != 0.0 else 512,
                    round(self.map_range(ry) / 5) * 5 if ry != 0.0 else 512,
                    btn_cross, btn_circle, btn_square, btn_triangle
                )

                self.sock.sendto(",".join(map(str, current_values)).encode(), (self.udp_ip, self.data_port))
                print("📤 Sent:", current_values)

                time.sleep(self.update_rate)

        except Exception as e:
            print(f"[ESP8266Joystick] Error: {e}")
        finally:
            pygame.quit()
            if self.sock:
                self.sock.close()
            print("[ESP8266Joystick] stopped")

    def stop(self):
        self.stop_event.set()


# ================== ESP32-CAM Fire Detection ==================
class FrameGrabber(threading.Thread):
    def __init__(self, stream_url: str, out_queue: queue.Queue, name: str = "FrameGrabber"):
        super().__init__(daemon=True, name=name)
        self.stream_url = stream_url
        self.out_queue = out_queue
        self.stop_event = threading.Event()
        self.cap: Optional[cv2.VideoCapture] = None

    def connect(self):
        if self.cap:
            try: self.cap.release()
            except: pass
        self.cap = cv2.VideoCapture(self.stream_url)
        time.sleep(0.5)

    def run(self):
        print(f"[FrameGrabber] connecting to {self.stream_url}")
        self.connect()
        while not self.stop_event.is_set():
            if not self.cap or not self.cap.isOpened():
                print("[FrameGrabber] stream not opened, reconnecting...")
                self.connect()
                time.sleep(1)
                continue

            ret, frame = self.cap.read()
            if not ret or frame is None:
                print("[FrameGrabber] empty frame received, reconnecting...")
                self.connect()
                time.sleep(0.5)
                continue

            if not self.out_queue.empty():
                try: self.out_queue.get_nowait()
                except queue.Empty: pass
            self.out_queue.put(frame)

        print("[FrameGrabber] stopping and releasing capture")
        if self.cap: self.cap.release()

    def stop(self):
        self.stop_event.set()


class YoloDetector(threading.Thread):
    def __init__(self, model_path: str, in_queue: queue.Queue, out_queue: queue.Queue,
                 conf_thresh: float = 0.5, fire_class_name: Optional[str] = 'fire'):
        super().__init__(daemon=True, name="YoloDetector")
        self.model_path = model_path
        self.in_queue = in_queue
        self.out_queue = out_queue
        self.conf_thresh = conf_thresh
        self.fire_class_name = fire_class_name
        self.stop_event = threading.Event()
        self.model = None

    def load_model(self):
        if YOLO is None:
            raise RuntimeError("ultralytics.YOLO not available. Install the 'ultralytics' package")
        print(f"[YoloDetector] Loading model from {self.model_path} ...")
        self.model = YOLO(self.model_path)
        print("[YoloDetector] Model loaded")

    def run(self):
        global fire_detected
        try:
            self.load_model()
        except Exception as e:
            print(f"[YoloDetector] failed to load model: {e}")
            return

        while not self.stop_event.is_set():
            try:
                frame = self.in_queue.get(timeout=1.0)
            except queue.Empty:
                continue

            try:
                results = self.model(frame, imgsz=640, conf=self.conf_thresh)
            except Exception as e:
                print(f"[YoloDetector] inference error: {e}")
                continue

            annotated = frame.copy()
            fire_found = False
            for r in results:
                for box in r.boxes:
                    conf = float(box.conf[0])
                    cls_idx = int(box.cls[0])
                    cls_name = self.model.names.get(cls_idx, str(cls_idx))
                    if str(cls_name).lower() == str(self.fire_class_name).lower() and conf >= self.conf_thresh:
                        fire_found = True
                        x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                        label = f"{cls_name} {conf:.2f}"
                        cv2.rectangle(annotated, (x1, y1), (x2, y2), (0,0,255), 2)
                        cv2.putText(annotated, label, (x1, y1-4),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 1)

            fire_detected = fire_found  # 🔥 update global flag

            if not self.out_queue.empty():
                try: self.out_queue.get_nowait()
                except queue.Empty: pass
            self.out_queue.put(annotated)

        print("[YoloDetector] stopping")

    def stop(self):
        self.stop_event.set()


# ================== MAIN ==================
def main():
    STREAM_URL = "http://192.168.137.23:81/stream"
    MODEL_PATH = "C:/Users/pc/Desktop/best.pt"

    raw_q = queue.Queue(maxsize=1)
    annotated_q = queue.Queue(maxsize=1)

    joystick_thread = ESP8266Joystick()
    grabber = FrameGrabber(STREAM_URL, raw_q)
    detector = YoloDetector(MODEL_PATH, raw_q, annotated_q, conf_thresh=0.5, fire_class_name="fire")

    try:
        joystick_thread.start()
        grabber.start()
        detector.start()

        print("[Main] All threads started. Press 'q' to quit.")

        while True:
            try:
                annotated = annotated_q.get(timeout=1.0)
                cv2.imshow("ESP32 Fire Detector", cv2.resize(annotated, (640,480)))
            except queue.Empty:
                continue

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except KeyboardInterrupt:
        print("[Main] interrupted by user")
    finally:
        print("[Main] stopping threads...")
        joystick_thread.stop()
        grabber.stop()
        detector.stop()
        joystick_thread.join(2)
        grabber.join(2)
        detector.join(2)
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
