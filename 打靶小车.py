import struct
import cv2
import numpy as np
import serial
import time
import tkinter as tk
from tkinter import ttk
from PIL import Image, ImageTk
import threading

class PIDController:
    def __init__(self, kp, ki, kd, output_limit):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limit = output_limit
        self.prev_error = 0.0
        self.integral = 0.0
        
    def update(self, error, dt):
        self.integral += error * dt
        self.integral = max(-100, min(100, self.integral))
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        output = max(-self.output_limit, min(self.output_limit, output))
        self.prev_error = error
        return float(output)
    
    def reset(self):
        self.prev_error = 0.0
        self.integral = 0.0

class FrameDetector:
    def __init__(self, min_area, max_area):
        self.min_area = min_area
        self.max_area = max_area
        
    def _order_points(self, pts):
        """Sort points in the order: top-left, top-right, bottom-right, bottom-left"""
        rect = np.zeros((4, 2), dtype="float32")
        s = pts.sum(axis=1)
        rect[0] = pts[np.argmin(s)]  # top-left has smallest sum
        rect[2] = pts[np.argmax(s)]  # bottom-right has largest sum
        
        diff = np.diff(pts, axis=1)
        rect[1] = pts[np.argmin(diff)]  # top-right has smallest difference
        rect[3] = pts[np.argmax(diff)]  # bottom-left has largest difference
        return rect

    def detect_frames(self, frame):
        """Detect rectangular frames and return their perspective-transformed centers"""
        blurred = cv2.GaussianBlur(frame, (5, 5), 0)
        gray = cv2.cvtColor(blurred, cv2.COLOR_BGR2GRAY)
        mask = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 11, 2)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        frames = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if not (self.min_area <= area <= self.max_area):
                continue
                
            peri = cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, 0.02 * peri, True)
            
            if len(approx) == 4:
                rect = self._order_points(approx.reshape(4, 2))
                frames.append(rect)
        
        return frames

class App:
    def __init__(self, window, window_title):
        self.window = window
        self.window.title(window_title)

        self.cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            raise ValueError("Unable to open video source")
        
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        

        self.width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

        self.canvas = tk.Canvas(window, width=self.width, height=self.height)
        self.canvas.pack()

        # Create a frame for buttons
        button_frame = tk.Frame(window)
        button_frame.pack(fill=tk.X, side=tk.BOTTOM, pady=5)

        self.btn_mode = ttk.Button(button_frame, text="Switch Mode", command=self.switch_mode)
        self.btn_mode.pack(side=tk.LEFT, expand=True)

        self.btn_start_stop = ttk.Button(button_frame, text="Start", command=self.toggle_start_stop)
        self.btn_start_stop.pack(side=tk.LEFT, expand=True)

        self.btn_circle_scan = ttk.Button(button_frame, text="Circle Scan", command=self.toggle_circle_scan)
        self.btn_circle_scan.pack(side=tk.LEFT, expand=True)

        try:
            self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
            # Start a thread to listen for "Ready" from MCU
            self.serial_thread = threading.Thread(target=self.listen_for_ready, daemon=True)
            self.serial_thread.start()
        except serial.SerialException as e:
            print(f"Serial connection error: {e}")
            self.ser = None

        self.running = False  # Control flag
        self.control_started = False # Flag to indicate if 'start' has been sent
        self.mode = "aim"  # "aim" or "track"
        # 基于Maix版本参数，并根据树莓派帧率翻倍(dt减半)进行调整
        # Kp_pi ≈ Kp_maix, Ki_pi ≈ 2*Ki_maix, Kd_pi ≈ 0.5*Kd_maix
        self.pid_x_aim = PIDController(kp=0.037, ki=0.00033, kd=0.0033, output_limit=10)
        self.pid_y_aim = PIDController(kp=0.030, ki=0.00027, kd=0.0027, output_limit=10)
        self.pid_x_track = PIDController(kp=0.07, ki=0.012, kd=0.002, output_limit=4.0)
        self.pid_y_track = PIDController(kp=0.033, ki=0.001, kd=0.00057, output_limit=4.0)
        self.pid_x = self.pid_x_aim
        self.pid_y = self.pid_y_aim
        self.aimed = False

        self.crop_w, self.crop_h = int(self.width * 0.6), int(self.height * 0.6)
        self.crop_x, self.crop_y = (self.width - self.crop_w) // 2, (self.height - self.crop_h) // 2
        
        self.frame_detector = FrameDetector(min_area=1000, max_area=self.crop_w * self.crop_h * 0.8)
        
        self.center_delta = 25
        self.screen_center = (self.width // 2, self.height // 2 - self.center_delta)
        self.DEAD_ZONE = 2

        self.last_time = time.time()
        
        self.update()

        self.window.mainloop()

    def listen_for_ready(self):
        """Listen for 'Ready' from MCU in a background thread."""
        if not self.ser:
            return
        print("Listening for 'Ready' from MCU...")
        while not self.running:  # Only listen when not running
            try:
                line = self.ser.readline().decode('utf-8').strip()
                if "Ready" in line:
                    print("MCU is Ready. Auto-starting control.")
                    # Use after() to safely update GUI from this thread
                    self.window.after(0, self.auto_start_control)
                    break  # Exit thread once started
            except (serial.SerialException, UnicodeDecodeError) as e:
                print(f"Error reading from serial port: {e}")
                break
            except Exception as e:
                print(f"An unexpected error occurred in serial listener: {e}")
                break

    def auto_start_control(self):
        """Start the control loop automatically."""
        if not self.running:
            self.running = True
            self.btn_start_stop.config(text="Stop")
            print("Control loop started automatically.")

    def toggle_start_stop(self):
        self.running = not self.running
        if self.running:
            self.btn_start_stop.config(text="Stop")
            print("Control loop started.")
        else:
            self.btn_start_stop.config(text="Start")
            print("Control loop stopped.")
            self.control_started = False # Reset start flag
            # Reset PIDs and send stop command
            if self.pid_x: self.pid_x.reset()
            if self.pid_y: self.pid_y.reset()

    def toggle_circle_scan(self):
        self.circle_scan_active = not self.circle_scan_active
        if self.circle_scan_active:
            self.mode = "circle"
            self.btn_circle_scan.config(text="Stop Scan")
            print("Mode switched to circle")
        else:
            self.mode = "aim" # or whatever default mode you prefer
            self.btn_circle_scan.config(text="Circle Scan")
            print(f"Mode switched to {self.mode}")

    def switch_mode(self):
        if self.mode == "aim":
            self.mode = "track"
        else:
            self.mode = "aim"
        print(f"Mode switched to {self.mode}")

    def update(self):
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time
        fps = 1.0 / dt if dt > 0 else 0

        ret, frame = self.cap.read()
        if not ret:
            self.window.after(10, self.update)
            return

        frame_crop = frame[self.crop_y:self.crop_y + self.crop_h, self.crop_x:self.crop_x + self.crop_w]
        
        frames = self.frame_detector.detect_frames(frame_crop)
        rect_center = None

        if len(frames) >= 1:
            frame_pts = max(frames, key=lambda pts: cv2.contourArea(pts.astype(int)))
            dst_pts = np.array([[0, 0], [100, 0], [100, 100], [0, 100]], dtype="float32")
            M = cv2.getPerspectiveTransform(frame_pts, dst_pts)
            center_dst = np.array([[[50, 50]]], dtype="float32")
            center_src = cv2.perspectiveTransform(center_dst, cv2.invert(M)[1])
            rect_center_crop = tuple(center_src[0][0].astype(int))
            rect_center = (rect_center_crop[0] + self.crop_x, rect_center_crop[1] + self.crop_y)
            frame_pts_full = frame_pts + np.array([self.crop_x, self.crop_y])
            cv2.drawContours(frame, [frame_pts_full.astype(int)], -1, (0, 255, 0), 2)
            cv2.circle(frame, rect_center, 8, (255, 0, 0), 2)

            left_len = np.linalg.norm(frame_pts[0] - frame_pts[3])
            right_len = np.linalg.norm(frame_pts[1] - frame_pts[2])
            avg_len = (left_len + right_len) / 2
            print(f"左右边平均长度: {avg_len:.2f}")
            
            # 使用新的数据点 (96, -8) 和 (72, -4)
            k = (-4 - (-8)) / (72 - 96) 
            b = -8 - k * 9
            center_delta = int(k * avg_len + b)
            self.screen_center = (self.width // 2, self.height // 2 - center_delta)

        if self.running:
            # Send start command once when control begins
            if not self.control_started and self.ser:
                start_payload = b'start'
                header = b'\xAA\xBB'
                length = len(start_payload).to_bytes(1, 'little')
                checksum = (sum(start_payload) & 0xFF).to_bytes(1, 'little')
                data_packet = header + length + start_payload + checksum
                self.ser.write(data_packet)
                self.control_started = True
                print("Sent 'start' command to MCU.")

            if self.mode == "aim":
                if self.pid_x is not self.pid_x_aim:
                    self.pid_x = self.pid_x_aim
                    self.pid_x.reset()
                if self.pid_y is not self.pid_y_aim:
                    self.pid_y = self.pid_y_aim
                    self.pid_y.reset()
                target_point = self.screen_center
            elif self.mode == "track":
                if self.pid_x is not self.pid_x_track:
                    self.pid_x = self.pid_x_track
                    self.pid_x.reset()
                if self.pid_y is not self.pid_y_track:
                    self.pid_y = self.pid_y_track
                    self.pid_y.reset()
                target_point = self.screen_center
            elif self.mode == "circle":
                if self.pid_x is not self.pid_x_track: # 使用track的PID参数
                    self.pid_x = self.pid_x_track
                    self.pid_x.reset()
                if self.pid_y is not self.pid_y_track:
                    self.pid_y = self.pid_y_track
                    self.pid_y.reset()

                if self.circle_start_time == 0 and rect_center:
                    self.circle_start_time = time.time()
                    self.circle_center = rect_center
                    # 半径是左右两边平均长度的0.5714倍
                    self.circle_radius = avg_len * 0.5714

                if self.circle_radius > 0:
                    elapsed_time = time.time() - self.circle_start_time
                    angle = (elapsed_time * 36) % 360  # 每10秒一圈
                    rad = np.deg2rad(angle)
                    target_x = self.circle_center[0] + self.circle_radius * np.cos(rad)
                    target_y = self.circle_center[1] + self.circle_radius * np.sin(rad)
                    target_point = (int(target_x), int(target_y))
                    # 在画面上画出圆和目标点
                    cv2.circle(frame, self.circle_center, int(self.circle_radius), (255, 255, 0), 1)
                    if target_point:
                        cv2.circle(frame, target_point, 5, (0, 0, 255), -1)

            if rect_center and target_point:
                error_x = target_point[0] - rect_center[0]
                error_y = target_point[1] - rect_center[1]
                output_x = self.pid_x.update(error_x, dt)
                output_y = self.pid_y.update(error_y, dt)

                if self.ser:
                    # 瞄准完成的逻辑只在aim模式下生效
                    if self.mode == "aim" and not (abs(error_x) > self.DEAD_ZONE or abs(error_y) > self.DEAD_ZONE):
                        if not self.aimed:
                            payload = struct.pack("<dd", 500.0, 500.0)
                            self.aimed = True
                        else:
                            payload = None
                    else:
                        payload = struct.pack("<dd", float(output_x), float(output_y))
                        self.aimed = False

                    if payload:
                        # 构建带校验和的完整数据包
                        header = b'\xAA\xBB'
                        length = len(payload).to_bytes(1, 'little')
                        checksum = (sum(payload) & 0xFF).to_bytes(1, 'little')
                        data_packet = header + length + payload + checksum
                        self.ser.write(data_packet)
                        print(f"Output: ({output_x:.2f}, {output_y:.2f})")

                cv2.putText(frame, f"Real Center: {rect_center}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
                cv2.putText(frame, f"Error: ({error_x}, {error_y})", (10, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
            else:
                # 重置PID和状态
                if self.pid_x: self.pid_x.reset()
                if self.pid_y: self.pid_y.reset()
                self.aimed = False
                if self.ser and self.control_started: # Only send if control has started
                    # 丢失目标
                    payload = struct.pack("<dd", -10.0, 0.0)
                    header = b'\xAA\xBB'
                    length = len(payload).to_bytes(1, 'little')
                    checksum = (sum(payload) & 0xFF).to_bytes(1, 'little')
                    data_packet = header + length + payload + checksum
                    self.ser.write(data_packet)
                cv2.putText(frame, "No frame detected", (10, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)

        cv2.rectangle(frame, (self.crop_x, self.crop_y), (self.crop_x + self.crop_w, self.crop_y + self.crop_h), (0, 255, 255), 2)
        cv2.drawMarker(frame, self.screen_center, (0, 255, 0), cv2.MARKER_CROSS, 20, 1)

        cv2.putText(frame, f"FPS: {fps:.1f}", (10, 65), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        cv2.putText(frame, f"Current Mode: {self.mode.upper()}", (self.width - 150, self.height - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

        self.photo = ImageTk.PhotoImage(image=Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)))
        self.canvas.create_image(0, 0, image=self.photo, anchor=tk.NW)

        self.window.after(10, self.update)


if __name__ == "__main__":
    App(tk.Tk(), "Tkinter OpenCV App")