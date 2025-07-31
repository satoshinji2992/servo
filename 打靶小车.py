import struct
from maix import image, display, app, time, camera
from maix import comm, touchscreen
import cv2
import numpy as np

Mode = {"aim", "track"}

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
        if error > 15:
            output = 8.0
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
        _, mask = cv2.threshold(gray, 70, 255, cv2.THRESH_BINARY_INV)
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

class TouchInterface:
    def __init__(self, disp_width, disp_height):
        self.ts = touchscreen.TouchScreen()
        self.disp_width = disp_width
        self.disp_height = disp_height
        self.pressed_already = False
        self.last_x = 0
        self.last_y = 0
        self.last_pressed = False
    def create_button(self, x, y, width, height, text, color=(255, 255, 255)):
        return {
            'x': x, 'y': y, 'width': width, 'height': height,
            'text': text, 'color': color
        }
    def draw_button(self, frame, button, is_selected=False):
        color = (0, 255, 0) if is_selected else button['color']
        cv2.rectangle(frame, 
                     (button['x'], button['y']), 
                     (button['x'] + button['width'], button['y'] + button['height']), 
                     color, 2)
        text_size = cv2.getTextSize(button['text'], cv2.FONT_HERSHEY_SIMPLEX, 0.4, 1)[0]
        text_x = button['x'] + (button['width'] - text_size[0]) // 2
        text_y = button['y'] + (button['height'] + text_size[1]) // 2
        cv2.putText(frame, button['text'], (text_x, text_y), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)
    def is_button_pressed(self, x, y, button):
        return (button['x'] <= x <= button['x'] + button['width'] and 
                button['y'] <= y <= button['y'] + button['height'])
    def check_touch(self, disp):
        x_raw, y_raw, pressed = self.ts.read()
        clicked = False
        # 检测到触摸释放时认为是点击
        if pressed:
            self.pressed_already = True
        else:
            if self.pressed_already:
                clicked = True
                self.pressed_already = False
        # 坐标映射：将触摸屏坐标映射到显示图像坐标
        x_disp, y_disp = image.resize_map_pos(
            disp.width(), disp.height(),  # 显示屏尺寸
            self.disp_width, self.disp_height,  # 图像尺寸
            image.Fit.FIT_CONTAIN,              # 适配模式
            x_raw, y_raw                        # 原始触摸坐标
        )
        # 返回映射后的显示坐标、按下状态和是否为点击事件，以及原始触摸坐标
        return x_disp, y_disp, pressed, clicked, x_raw, y_raw

def find_laser():
    disp = display.Display()
    cam = camera.Camera(400, 300, image.Format.FMT_BGR888)
    p = comm.CommProtocol(buff_size=1024)
    
    mode = "aim"  # "aim"=瞄准模式, "track"=追踪模式
    # 两套PID参数
    pid_x_aim = PIDController(kp=0.11, ki=0.001, kd=0.01, output_limit=10)
    pid_y_aim = PIDController(kp=0.085, ki=0.0008, kd=0.008, output_limit=10)
    pid_x_track = PIDController(kp=0.23, ki=0.018, kd=0.012, output_limit=4.0)
    pid_y_track = PIDController(kp=0.1, ki=0.0015, kd=0.0035, output_limit=4.0)
    pid_x = None
    pid_y = None
    aimed = False  # 是否已追中
    # 只处理中心70%区域
    crop_w, crop_h = int(400 * 0.7), int(300 * 0.7)
    crop_x, crop_y = (400 - crop_w) // 2, (300 - crop_h) // 2
    crop_center = (crop_w // 2, crop_h // 2)
    
    frame_detector = FrameDetector(min_area=1000, max_area=400*300*0.8)
    
    center_delta = 18
    screen_center = (200, 150-center_delta)  #  image center
    DEAD_ZONE = 2
    
    # 初始化触摸界面
    touch_ui = TouchInterface(320, 240)
    # 模式切换按钮
    mode_btn = touch_ui.create_button(250, 200, 60, 30, "Mode")
    mode = "aim"  # 初始为瞄准模式
    
    last_time = time.ticks_ms()
    while not app.need_exit():
        current_time = time.ticks_ms()
        dt = (current_time - last_time) / 1000.0
        last_time = current_time
        fps = 1.0 / dt if dt > 0 else 0
        
        img = cam.read()
        frame = image.image2cv(img, ensure_bgr=False, copy=False)
        # 裁剪中心70%区域
        frame_crop = frame[crop_y:crop_y+crop_h, crop_x:crop_x+crop_w]
        # 只在裁剪区域检测
        frames = frame_detector.detect_frames(frame_crop)
        rect_center = None
        if len(frames) >= 1:
            frame_pts = max(frames, key=lambda pts: cv2.contourArea(pts.astype(int)))
            dst_pts = np.array([[0, 0], [100, 0], [100, 100], [0, 100]], dtype="float32")
            M = cv2.getPerspectiveTransform(frame_pts, dst_pts)
            center_dst = np.array([[[50, 50]]], dtype="float32")
            center_src = cv2.perspectiveTransform(center_dst, cv2.invert(M)[1])
            # 坐标回到原图
            rect_center = tuple(center_src[0][0].astype(int))
            rect_center = (rect_center[0] + crop_x, rect_center[1] + crop_y)
            frame_pts_full = frame_pts + np.array([crop_x, crop_y])
            cv2.drawContours(frame, [frame_pts_full.astype(int)], -1, (0, 255, 0), 2)
            cv2.circle(frame, rect_center, 8, (255, 0, 0), 2)
        # PID控制
        # 根据模式选择PID
        if mode == "aim":
            if pid_x is not pid_x_aim:
                pid_x = pid_x_aim
                pid_x.reset()
            if pid_y is not pid_y_aim:
                pid_y = pid_y_aim
                pid_y.reset()
        else:
            if pid_x is not pid_x_track:
                pid_x = pid_x_track
                pid_x.reset()
            if pid_y is not pid_y_track:
                pid_y = pid_y_track
                pid_y.reset()
        if rect_center:
            error_x = screen_center[0] - rect_center[0]
            error_y = screen_center[1] - rect_center[1]
            output_x = pid_x.update(error_x, dt)
            output_y = pid_y.update(error_y, dt)
            if mode == "aim":
                if abs(error_x) > DEAD_ZONE or abs(error_y) > DEAD_ZONE:
                    data = struct.pack("<dd", float(output_x), float(output_y))
                    p.report(0x11, data)
                    aimed = False  # 只要偏离就重置
                else:
                    if not aimed:
                        data = struct.pack("<dd", 500.0, 500.0)
                        p.report(0x11, data)
                        aimed = True  # 只发一次
            else:  # track模式
                data = struct.pack("<dd", float(output_x), float(output_y))
                p.report(0x11, data)
            cv2.putText(frame, f"Real Center: {rect_center}", (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
            cv2.putText(frame, f"Error: ({error_x}, {error_y})", (10, 15), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
        else:
            if pid_x: pid_x.reset()
            if pid_y: pid_y.reset()
            aimed = False  # 丢失目标也重置
            # 未识别到矩形时输出 dx=20, dy=0
            data = struct.pack("<dd", -10.0, 0.0)
            p.report(0x11, data)
            cv2.putText(frame, "No frame detected", (10, 15), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)
        # 画出中心区域边框
        cv2.rectangle(frame, (crop_x, crop_y), (crop_x+crop_w, crop_y+crop_h), (0, 255, 255), 2)
        cv2.drawMarker(frame, screen_center, (0, 255, 0), cv2.MARKER_CROSS, 20, 1)
        cv2.putText(frame, f"FPS: {fps:.1f}", (10, 65), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        # 绘制模式切换按钮
        touch_ui.draw_button(frame, mode_btn)
        cv2.putText(frame, f"Current Mode: {mode.upper()}", (250, 195), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0,255,255), 1)
        # 检测模式切换按钮点击
        x_touch, y_touch, pressed, clicked, x_raw, y_raw = touch_ui.check_touch(disp)
        if clicked and touch_ui.is_button_pressed(x_touch, y_touch, mode_btn):
            mode = "track" if mode == "aim" else "aim"
            print(f"Mode switched to {mode}")
        img_show = image.cv2image(frame, bgr=True, copy=False)
        disp.show(img_show, fit=image.Fit.FIT_CONTAIN)

if __name__ == "__main__":
    find_laser()