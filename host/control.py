"""
MaixCam 视觉云台控制 - 迁移自树莓派版本
支持矩形检测、筛选和PID控制
"""
from maix import image, camera, display, time, touchscreen, uart
import maix.app as maix_app
import cv2
import numpy as np
import math
import struct

# ============== PID 控制器 ==============
class PIDController:
    def __init__(self, kp, ki, kd, output_limit):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limit = output_limit
        self.prev_error = 0.0
        self.integral = 0.0
        self.last_derivative = 0.0
        self._RC = 1 / (2 * math.pi * 20)  # 微分滤波时间常数

    def update(self, error, dt):
        if dt <= 0:
            return 0.0

        # 比例项
        P_out = self.kp * error

        # 积分项
        self.integral += error * dt
        self.integral = max(-100, min(100, self.integral))
        I_out = self.ki * self.integral

        # 微分项（带低通滤波）
        derivative = (error - self.prev_error) / dt
        alpha = dt / (self._RC + dt)
        derivative = self.last_derivative + alpha * (derivative - self.last_derivative)
        D_out = self.kd * derivative
        self.last_derivative = derivative

        output = P_out + I_out + D_out
        output = max(-self.output_limit, min(self.output_limit, output))
        self.prev_error = error
        return float(output)

    def reset(self):
        self.prev_error = 0.0
        self.integral = 0.0
        self.last_derivative = 0.0


# ============== 矩形检测与筛选 ==============
class FrameDetector:
    """
    矩形检测器，支持多种筛选条件
    """
    def __init__(self, min_area=500, max_area=100000,
                 aspect_ratio_min=0.3, aspect_ratio_max=3.0,
                 angle_threshold=20, solidity_min=0.8):
        self.min_area = min_area
        self.max_area = max_area
        self.aspect_ratio_min = aspect_ratio_min    # 长宽比最小值
        self.aspect_ratio_max = aspect_ratio_max    # 长宽比最大值
        self.angle_threshold = angle_threshold      # 角度容差（度）
        self.solidity_min = solidity_min            # 凸包面积比最小值

    def _order_points(self, pts):
        """四点排序：左上，右上，右下，左下"""
        rect = np.zeros((4, 2), dtype="float32")
        s = pts.sum(axis=1)
        rect[0] = pts[np.argmin(s)]      # 左上
        rect[2] = pts[np.argmax(s)]      # 右下
        diff = np.diff(pts, axis=1)
        rect[1] = pts[np.argmin(diff)]   # 右上
        rect[3] = pts[np.argmax(diff)]   # 左下
        return rect

    def _angle(self, pt1, pt2, pt3):
        """计算pt1-pt2-pt3的角度"""
        v1 = pt1 - pt2
        v2 = pt3 - pt2
        cos_angle = np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2) + 1e-6)
        return np.degrees(np.arccos(np.clip(cos_angle, -1, 1)))

    def _is_standard_rectangle(self, approx):
        """判断是否为标准矩形（4角接近90°）"""
        if len(approx) != 4:
            return False
        for i in range(4):
            pt1 = approx[i][0]
            pt2 = approx[(i+1) % 4][0]
            pt3 = approx[(i+2) % 4][0]
            ang = self._angle(pt1, pt2, pt3)
            if abs(ang - 90) > self.angle_threshold:
                return False
        return True

    def detect_frames(self, img):
        """
        检测矩形框并返回所有符合条件的矩形
        返回: [{'points': pts, 'center': (cx,cy), 'area': area,
               'aspect_ratio': ratio, 'width': w, 'height': h, 'solidity': s}, ...]
        """
        # 预处理
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)

        # 边缘检测
        edges = cv2.Canny(blurred, 50, 150)

        # 查找轮廓
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        frames = []

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if not (self.min_area <= area <= self.max_area):
                continue

            # 多边形拟合
            peri = cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, 0.02 * peri, True)

            # 只处理四边形
            if len(approx) != 4:
                continue

            # 检查是否为凸四边形
            if not cv2.isContourConvex(approx):
                continue

            # 角度筛选（可选，更严格）
            # if not self._is_standard_rectangle(approx):
            #     continue

            pts = approx.reshape(4, 2)
            rect = self._order_points(pts)

            # 计算宽高
            width = np.linalg.norm(rect[1] - rect[0])
            height = np.linalg.norm(rect[3] - rect[0])

            if width <= 0 or height <= 0:
                continue

            # 长宽比筛选
            aspect_ratio = max(width, height) / min(width, height)
            if not (self.aspect_ratio_min <= aspect_ratio <= self.aspect_ratio_max):
                continue

            # 凸包面积比（solidity）筛选
            hull = cv2.convexHull(cnt)
            hull_area = cv2.contourArea(hull)
            solidity = area / hull_area if hull_area > 0 else 0
            if solidity < self.solidity_min:
                continue

            # 计算中心点
            M = cv2.moments(approx)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
            else:
                cx = int(np.mean(pts[:, 0]))
                cy = int(np.mean(pts[:, 1]))

            frames.append({
                'points': rect.astype(np.int32),
                'center': (cx, cy),
                'area': area,
                'aspect_ratio': aspect_ratio,
                'width': width,
                'height': height,
                'solidity': solidity,
                'contour': approx
            })

        return frames

    def select_best_frame(self, frames, screen_center, strategy='area'):
        """
        筛选最佳矩形
        strategy:
          - 'area': 选择面积最大的
          - 'center': 选择离屏幕中心最近的
          - 'combined': 综合面积和距离
          - 'largest_square': 最接近正方形的大面积
        """
        if not frames:
            return None

        if strategy == 'area':
            return max(frames, key=lambda f: f['area'])

        elif strategy == 'center':
            return min(frames, key=lambda f:
                (f['center'][0] - screen_center[0])**2 +
                (f['center'][1] - screen_center[1])**2)

        elif strategy == 'combined':
            max_area = max(f['area'] for f in frames)
            max_dist = max(math.sqrt(
                (f['center'][0] - screen_center[0])**2 +
                (f['center'][1] - screen_center[1])**2
            ) for f in frames) + 1

            def score(f):
                area_score = f['area'] / max_area if max_area > 0 else 0
                dist = math.sqrt(
                    (f['center'][0] - screen_center[0])**2 +
                    (f['center'][1] - screen_center[1])**2
                )
                dist_score = 1 - (dist / max_dist)
                return area_score * 0.7 + dist_score * 0.3

            return max(frames, key=score)

        elif strategy == 'largest_square':
            # 优先选择面积大且接近正方形的
            def square_score(f):
                # 长宽比越接近1，得分越高
                ratio_score = 1.0 / f['aspect_ratio']
                return f['area'] * ratio_score
            return max(frames, key=square_score)

        return frames[0]


# ============== 按钮类 ==============
class Button:
    def __init__(self, x, y, w, h, label, color=(255, 255, 255)):
        self.x = x
        self.y = y
        self.w = w
        self.h = h
        self.label = label
        self.color = color

    def draw(self, img, pressed=False):
        color = (0, 255, 255) if pressed else self.color
        cv2.rectangle(img, (self.x, self.y), (self.x + self.w, self.y + self.h), color, 2)
        txt_x = self.x + (self.w - len(self.label) * 8) // 2
        txt_y = self.y + self.h // 2 + 5
        cv2.putText(img, self.label, (txt_x, txt_y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

    def contains(self, x, y):
        return self.x <= x <= self.x + self.w and self.y <= y <= self.y + self.h


# ============== 串口类 ==============
class SerialComm:
    """MaixCam串口通信"""
    def __init__(self, baudrate=115200):
        self.ser = None
        try:
            devices = uart.list_devices()
            if devices:
                self.ser = uart.UART(devices[0], baudrate)
                print(f"Serial opened: {devices[0]} @ {baudrate}")
            else:
                print("No UART device found")
        except Exception as e:
            print(f"Serial open failed: {e}")

    def write(self, data):
        if self.ser:
            self.ser.write(data)

    def write_str(self, s):
        if self.ser:
            self.ser.write_str(s)

    def read(self, timeout=1000):
        if self.ser:
            return self.ser.read(timeout=timeout)
        return b''

    def close(self):
        if self.ser:
            self.ser.close()


# ============== 主应用 ==============
class App:
    def __init__(self):
        # 初始化显示
        self.disp = display.Display()
        self.scr_w = self.disp.width()
        self.scr_h = self.disp.height()

        # 摄像头分辨率
        self.cam_w, self.cam_h = 320, 240
        self.cam = camera.Camera(self.cam_w, self.cam_h, image.Format.FMT_BGR888)

        # 初始化触摸屏
        self.ts = touchscreen.TouchScreen()

        # 初始化串口
        self.ser = SerialComm(115200)
        self.ser_connected = self.ser.ser is not None

        # 控制状态
        self.running = False
        self.mode = "aim"  # "aim", "track", "circle"
        self.control_started = False

        # PID参数
        self.pid_x_aim = PIDController(kp=0.037, ki=0.00033, kd=0.0033, output_limit=10)
        self.pid_y_aim = PIDController(kp=0.030, ki=0.00027, kd=0.0027, output_limit=10)
        self.pid_x_track = PIDController(kp=0.07, ki=0.012, kd=0.002, output_limit=4.0)
        self.pid_y_track = PIDController(kp=0.033, ki=0.001, kd=0.00057, output_limit=4.0)
        self.pid_x = self.pid_x_aim
        self.pid_y = self.pid_y_aim
        self.aimed = False

        # 检测区域（中心裁剪）
        self.crop_w = int(self.cam_w * 0.6)
        self.crop_h = int(self.cam_h * 0.6)
        self.crop_x = (self.cam_w - self.crop_w) // 2
        self.crop_y = (self.cam_h - self.crop_h) // 2

        # 矩形检测器（带筛选参数）
        self.frame_detector = FrameDetector(
            min_area=500,
            max_area=self.crop_w * self.crop_h * 0.8,
            aspect_ratio_min=0.3,
            aspect_ratio_max=3.0,
            angle_threshold=20,
            solidity_min=0.8
        )
        self.filter_strategy = 'area'  # 'area', 'center', 'combined', 'largest_square'

        # 目标相关
        self.center_delta = 25
        self.screen_center = (self.cam_w // 2, self.cam_h // 2 - self.center_delta)
        self.DEAD_ZONE = 2

        # Circle scan
        self.circle_start_time = 0
        self.circle_center = (0, 0)
        self.circle_radius = 0

        # 时间
        self.last_time = time.ticks_ms()

        # FPS计算
        self.fps = 0
        self.frame_count = 0
        self.fps_time = time.ticks_ms()

        # 创建按钮
        btn_h = 25
        btn_w = 70
        self.btn_start = Button(5, 5, btn_w, btn_h, "Start", (0, 255, 0))
        self.btn_mode = Button(5, 35, btn_w, btn_h, "Mode:Aim", (255, 255, 0))
        self.btn_filter = Button(5, 65, btn_w, btn_h, "Filter:Area", (0, 255, 255))
        self.btn_exit = Button(self.cam_w - 55, 5, 50, btn_h, "Exit", (0, 0, 255))

    def send_packet(self, payload):
        """发送带校验和的数据包"""
        if not self.ser_connected:
            return
        header = b'\xAA\xBB'
        length = len(payload).to_bytes(1, 'little')
        checksum = (sum(payload) & 0xFF).to_bytes(1, 'little')
        data_packet = header + length + payload + checksum
        self.ser.write(data_packet)

    def toggle_start(self):
        self.running = not self.running
        self.btn_start.label = "Stop" if self.running else "Start"
        self.btn_start.color = (0, 0, 255) if self.running else (0, 255, 0)
        if not self.running:
            self.control_started = False
            self.pid_x.reset()
            self.pid_y.reset()

    def switch_mode(self):
        modes = ["aim", "track", "circle"]
        idx = (modes.index(self.mode) + 1) % len(modes)
        self.mode = modes[idx]
        self.btn_mode.label = f"Mode:{self.mode.capitalize()}"
        if self.mode != "circle":
            self.circle_start_time = 0
            self.circle_radius = 0

    def switch_filter(self):
        strategies = ['area', 'center', 'combined', 'largest_square']
        idx = (strategies.index(self.filter_strategy) + 1) % len(strategies)
        self.filter_strategy = strategies[idx]
        self.btn_filter.label = f"Filter:{self.filter_strategy[:6]}"

    def run(self):
        while not maix_app.need_exit():
            # 计算dt
            current_time = time.ticks_ms()
            dt = (current_time - self.last_time) / 1000.0
            self.last_time = current_time

            # FPS计算
            self.frame_count += 1
            if current_time - self.fps_time >= 1000:
                self.fps = self.frame_count
                self.frame_count = 0
                self.fps_time = current_time

            # 读取摄像头
            img_maix = self.cam.read()
            img = image.image2cv(img_maix, ensure_bgr=False, copy=False)

            # 裁剪检测区域
            crop_img = img[self.crop_y:self.crop_y + self.crop_h,
                          self.crop_x:self.crop_x + self.crop_w]

            # 检测矩形
            frames = self.frame_detector.detect_frames(crop_img)

            # 筛选最佳矩形
            rect_data = None
            rect_center = None
            avg_len = 0

            if frames:
                crop_center = (self.crop_w // 2, self.crop_h // 2)
                best = self.frame_detector.select_best_frame(frames, crop_center, self.filter_strategy)

                if best:
                    rect_data = best
                    # 转换回全图坐标
                    rect_center = (
                        best['center'][0] + self.crop_x,
                        best['center'][1] + self.crop_y
                    )
                    avg_len = (best['width'] + best['height']) / 2

                    # 绘制检测到的矩形（转换坐标到全图）
                    pts_full = best['points'] + np.array([self.crop_x, self.crop_y])
                    cv2.drawContours(img, [pts_full], -1, (0, 255, 0), 2)
                    cv2.circle(img, rect_center, 5, (255, 0, 0), -1)

                    # 计算center_delta（基于距离的动态调整）
                    k = (-4 - (-8)) / (72 - 96)
                    b = -8 - k * 96
                    center_delta = int(k * avg_len + b)
                    self.screen_center = (self.cam_w // 2, self.cam_h // 2 - center_delta)

            # 控制逻辑
            target_point = None
            if self.running:
                # 发送start命令
                if not self.control_started:
                    self.send_packet(b'start')
                    self.control_started = True
                    print("Sent 'start' command")

                # 选择PID和目标点
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
                    if self.pid_x is not self.pid_x_track:
                        self.pid_x = self.pid_x_track
                        self.pid_x.reset()
                    if self.pid_y is not self.pid_y_track:
                        self.pid_y = self.pid_y_track
                        self.pid_y.reset()

                    if self.circle_start_time == 0 and rect_center:
                        self.circle_start_time = time.ticks_ms()
                        self.circle_center = rect_center
                        self.circle_radius = avg_len * 0.5714

                    if self.circle_radius > 0:
                        elapsed = (time.ticks_ms() - self.circle_start_time) / 1000.0
                        angle = (elapsed * 36) % 360  # 每10秒一圈
                        rad = math.radians(angle)
                        target_x = self.circle_center[0] + self.circle_radius * math.cos(rad)
                        target_y = self.circle_center[1] + self.circle_radius * math.sin(rad)
                        target_point = (int(target_x), int(target_y))

                        # 绘制圆轨迹
                        cv2.circle(img, self.circle_center, int(self.circle_radius), (255, 255, 0), 1)
                        cv2.circle(img, target_point, 3, (0, 0, 255), -1)

                # 计算误差并发送
                if rect_center and target_point:
                    error_x = target_point[0] - rect_center[0]
                    error_y = target_point[1] - rect_center[1]
                    output_x = self.pid_x.update(error_x, dt)
                    output_y = self.pid_y.update(error_y, dt)

                    if self.ser_connected:
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
                            self.send_packet(payload)

                    # 显示信息
                    cv2.putText(img, f"Center:{rect_center}", (10, self.cam_h - 50),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
                    cv2.putText(img, f"Err:({error_x},{error_y})", (10, self.cam_h - 35),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
                    cv2.putText(img, f"Out:({output_x:.2f},{output_y:.2f})", (10, self.cam_h - 20),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
                else:
                    self.pid_x.reset()
                    self.pid_y.reset()
                    self.aimed = False
                    if self.ser_connected and self.control_started:
                        payload = struct.pack("<dd", -10.0, 0.0)
                        self.send_packet(payload)
                    cv2.putText(img, "No frame", (10, self.cam_h - 20),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

            # 绘制UI
            # 裁剪区域框
            cv2.rectangle(img, (self.crop_x, self.crop_y),
                         (self.crop_x + self.crop_w, self.crop_y + self.crop_h),
                         (0, 255, 255), 2)
            # 屏幕中心十字
            cv2.drawMarker(img, self.screen_center, (0, 255, 0),
                          cv2.MARKER_CROSS, 15, 1)

            # 绘制按钮
            self.btn_start.draw(img, self.running)
            self.btn_mode.draw(img)
            self.btn_filter.draw(img)
            self.btn_exit.draw(img)

            # 显示状态信息
            cv2.putText(img, f"FPS:{self.fps}", (self.cam_w - 60, self.cam_h - 5),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)
            cv2.putText(img, f"Frames:{len(frames)}", (10, self.cam_h - 5),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)

            # 处理触摸
            x, y, pressed = self.ts.read()
            # 屏幕坐标转摄像头坐标
            fx = int(x * self.cam_w / self.scr_w)
            fy = int(y * self.cam_h / self.scr_h)

            if pressed:
                if self.btn_start.contains(fx, fy):
                    self.toggle_start()
                    time.sleep_ms(200)  # 去抖
                elif self.btn_mode.contains(fx, fy):
                    self.switch_mode()
                    time.sleep_ms(200)
                elif self.btn_filter.contains(fx, fy):
                    self.switch_filter()
                    time.sleep_ms(200)
                elif self.btn_exit.contains(fx, fy):
                    maix_app.set_exit_flag(True)

            # 显示图像
            img_show = image.cv2image(img, bgr=True, copy=False)
            self.disp.show(img_show)


if __name__ == "__main__":
    app = App()
    app.run()
