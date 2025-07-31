import struct
from maix import image, display, app, time, camera, touchscreen
from maix import comm, protocol
import cv2
import numpy as np
import math

class PIDController:
    def __init__(self, kp, ki, kd, output_limit):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limit = output_limit
        
        self.prev_error = 0
        self.integral = 0
        
    def update(self, error, dt):
        # 积分项
        self.integral += error * dt
        # 限制积分饱和
        self.integral = max(-50, min(50, self.integral)) 
        
        # 微分项
        derivative = (error - self.prev_error) / dt if dt > 0 else 0
        
        # PID计算
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        
        # 输出限制
        output = max(-self.output_limit, min(self.output_limit, output))
        
        self.prev_error = error
        return float(output)
    
    def reset(self):
        self.prev_error = 0
        self.integral = 0

class HSVController:
    def __init__(self, initial_lower, initial_upper):
        self.lower_hsv = list(initial_lower)  # [H_min, S_min, V_min]
        self.upper_hsv = list(initial_upper)  # [H_max, S_max, V_max]
        self.current_mode = "select"  # "select", "h", "s", "v"
        self.step_size = [5, 10, 10]  # H, S, V 的调节步长
        
    def get_hsv_range(self):
        return np.array(self.lower_hsv), np.array(self.upper_hsv)

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
        # 绘制按钮边框
        cv2.rectangle(frame, 
                     (button['x'], button['y']), 
                     (button['x'] + button['width'], button['y'] + button['height']), 
                     color, 2)
        # 绘制按钮文本
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
        
        # 坐标映射：将触摸屏坐标映射到显示图像坐标
        # 参考示例代码的坐标映射方法
        x, y = image.resize_map_pos_reverse(
            self.disp_width, self.disp_height,  # 图像尺寸
            disp.width(), disp.height(),        # 显示屏尺寸
            image.Fit.FIT_CONTAIN,              # 适配模式
            x_raw, y_raw                        # 原始触摸坐标
        )
        
        # 检测到触摸释放时认为是点击
        if pressed:
            self.pressed_already = True
        else:
            if self.pressed_already:
                clicked = True
                self.pressed_already = False
        
        # 返回映射后的坐标、按下状态和是否为点击事件
        return x, y, pressed, clicked, x_raw, y_raw

def find_laser():
    # 初始化
    disp = display.Display()
    cam = camera.Camera(320, 240, image.Format.FMT_BGR888)
    
    # 初始化通信协议 (自动根据系统配置初始化UART或TCP)
    p = comm.CommProtocol(buff_size=1024)
    
    # 画面中心和死区
    # CENTER_X, CENTER_Y = 160, 120 # 这些在计算error时已经不再需要，直接依赖 rect_center
    DEAD_ZONE = 3
    
    # PID控制器 (稳定跟踪参数)
    pid_x = PIDController(kp=0.22, ki=0.002, kd=0.02, output_limit=10)
    pid_y = PIDController(kp=0.22, ki=0.002, kd=0.02, output_limit=10)
    
    # 识别405nm蓝紫色激光的HSV范围（可根据实际调整）
    # 调整V值以包含更亮的中心区域，并适当放宽S值
    blueviolet_lower = np.array([120, 30, 80])  # H约120~140，S/V适当放宽
    blueviolet_upper = np.array([140, 255, 255])
    
    # 初始化触摸界面
    touch_ui = TouchInterface(320, 240) # 注意：触摸界面代码未在此次修改中激活，请根据需要自行集成。
    
    # 时间记录
    last_time = time.ticks_ms()
    
    # 用于形态学操作的核
    kernel_morph = np.ones((5, 5), np.uint8) # 闭运算和开运算的核，可以根据光斑大小调整

    # 创建界面按钮 (目前未在主循环中使用，仅是定义)
    h_btn = touch_ui.create_button(10, 200, 30, 25, "H")
    s_btn = touch_ui.create_button(50, 200, 30, 25, "S") 
    v_btn = touch_ui.create_button(90, 200, 30, 25, "V")
    min_minus_btn = touch_ui.create_button(10, 180, 35, 20, "Min-")
    min_plus_btn = touch_ui.create_button(50, 180, 35, 20, "Min+")
    max_minus_btn = touch_ui.create_button(90, 180, 35, 20, "Max-")
    max_plus_btn = touch_ui.create_button(130, 180, 35, 20, "Max+")
    exit_btn = touch_ui.create_button(170, 180, 35, 20, "Exit")
    
    # 初始化 FrameDetector
    frame_detector = FrameDetector(
        min_area=1000,
        max_area=cam.width() * cam.height() * 0.8,
        min_aspect_ratio=0.5,
        max_aspect_ratio=2.0,
        black_lower=np.array([0, 0, 0]),
        black_upper=np.array([180, 255, 60])
    )
    
    last_rect_center = None
    # 新增变量
    red_and_green_roi = []
    center_record = []
    find_rect = True
    largest_rect = None
    corner_points = []
    while not app.need_exit():
        # 计算时间间隔
        current_time = time.ticks_ms()
        dt = (current_time - last_time) / 1000.0  # 转换为秒
        last_time = current_time
        
        img = cam.read()
        frame = image.image2cv(img, ensure_bgr=False, copy=False)
        # --- 矩形识别（用find_rect_func）---
        find_rect, largest_rect, corner_points, center_record, red_and_green_roi = find_rect_func(
            frame, center_record, find_rect, largest_rect, corner_points, red_and_green_roi)
        rect_center = None
        if not find_rect and largest_rect is not None:
            rect_center = calculate_center(corner_points)
            last_rect_center = rect_center
            # 可视化
            cv2.drawContours(frame, [largest_rect], -1, (0,255,0), 2)
            for pt in corner_points:
                cv2.circle(frame, pt, 5, (0,255,0), -1)
            cv2.putText(frame, f"Rect Center: {rect_center}", (rect_center[0]-40, rect_center[1]-10), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0,255,0), 1)
        elif last_rect_center is not None:
            rect_center = last_rect_center
            cv2.putText(frame, f"Rect Center (last): {rect_center}", (rect_center[0]-40, rect_center[1]-10), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0,200,0), 1)
        # --- 紫激光点识别（ROI内）---
        laser_center = None
        if len(red_and_green_roi) == 4:
            laser_center = find_laser_spot(frame, red_and_green_roi, blueviolet_lower, blueviolet_upper)
            if laser_center:
                cv2.circle(frame, laser_center, 5, (0, 0, 255), -1)
                cv2.putText(frame, f"Laser: {laser_center}", (laser_center[0]+10, laser_center[1]-10), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255,0,255), 1)
        # --- PID 控制和数据显示 ---
        if laser_center and rect_center:
            error_x = laser_center[0] - rect_center[0]
            error_y = laser_center[1] - rect_center[1]
            
            cv2.putText(frame, f"Error Laser->Rect: ({error_x}, {error_y})", (10, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255,255,255), 1)
            cv2.putText(frame, f"Laser: {laser_center}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255,0,255), 1)
            cv2.putText(frame, f"Rect Center: ({rect_center[0]}, {rect_center[1]})", (10, 45), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0,255,0), 1)
            
            if abs(error_x) > DEAD_ZONE or abs(error_y) > DEAD_ZONE:
                delta_x = pid_x.update(error_x, dt)
                delta_y = pid_y.update(error_y, dt)
                
                servo_delta_x = delta_x  # 舵机方向可能需要反转
                servo_delta_y = -delta_y  # 舵机方向可能需要反转
                
                data = struct.pack("<dd", float(servo_delta_x), float(servo_delta_y))
                p.report(0x11, data)
                cv2.putText(frame, f"Servo: ({servo_delta_x:.2f}, {servo_delta_y:.2f})", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0,255,255), 1)
            else:
                cv2.putText(frame, "Laser aligned with Rect Center", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0,255,0), 1)
        else:
            pid_x.reset()
            pid_y.reset()
            cv2.putText(frame, "No laser or rectangle detected", (10, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0,0,255), 1)
        
        # 将处理后的帧显示到屏幕上
        img_show = image.cv2image(frame, bgr=True, copy=False)
        disp.show(img_show, fit=image.Fit.FIT_CONTAIN)
        
        time.sleep_ms(50)

class FrameDetector:
    def __init__(self, min_area, max_area, min_aspect_ratio, max_aspect_ratio, black_lower, black_upper):
        self.min_area = min_area
        self.max_area = max_area
        self.min_aspect_ratio = min_aspect_ratio
        self.max_aspect_ratio = max_aspect_ratio
        self.black_lower = black_lower
        self.black_upper = black_upper

    def detect_frames(self, frame):
        """检测黑色方框，返回内外框信息"""
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask_black = cv2.inRange(hsv, self.black_lower, self.black_upper)
        kernel = np.ones((3, 3), np.uint8)
        mask_black = cv2.morphologyEx(mask_black, cv2.MORPH_CLOSE, kernel)
        mask_black = cv2.morphologyEx(mask_black, cv2.MORPH_OPEN, kernel)
        contours, hierarchy = cv2.findContours(mask_black, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        frames = []
        if contours and hierarchy is not None:
            for i, contour in enumerate(contours):
                area = cv2.contourArea(contour)
                if area < self.min_area or area > self.max_area:
                    continue
                x, y, w, h = cv2.boundingRect(contour)
                aspect_ratio = float(w) / h
                if aspect_ratio < self.min_aspect_ratio or aspect_ratio > self.max_aspect_ratio:
                    continue
                epsilon = 0.02 * cv2.arcLength(contour, True)
                approx = cv2.approxPolyDP(contour, epsilon, True)
                if len(approx) >= 4:
                    M = cv2.moments(contour)
                    if M["m00"] != 0:
                        center_x = int(M["m10"] / M["m00"])
                        center_y = int(M["m01"] / M["m00"])
                    else:
                        center_x = x + w // 2
                        center_y = y + h // 2
                    frame_info = {
                        'contour': contour,
                        'approx': approx,
                        'center': (center_x, center_y),
                        'bbox': (x, y, w, h),
                        'area': area,
                        'type': 'unknown',
                        'aspect_ratio': aspect_ratio
                    }
                    frames.append(frame_info)
        if len(frames) == 2:
            frames.sort(key=lambda f: f['area'])
            inner_frame = frames[0]
            outer_frame = frames[1]
            inner_frame['type'] = 'inner'
            outer_frame['type'] = 'outer'
            return [outer_frame, inner_frame], mask_black
        return frames, mask_black

def calculate_angle(p1, p2, p3):
    """计算由三个点形成的角p2-p1-p3的角度"""
    v1 = np.array([p1[0] - p2[0], p1[1] - p2[1]])
    v2 = np.array([p1[0] - p3[0], p1[1] - p3[1]])
    dot_product = np.dot(v1, v2)
    norm_v1 = np.linalg.norm(v1)
    norm_v2 = np.linalg.norm(v2)
    if norm_v1 == 0 or norm_v2 == 0:
        return 0
    cos_angle = dot_product / (norm_v1 * norm_v2)
    cos_angle = max(-1.0, min(1.0, cos_angle))
    angle = math.degrees(math.acos(cos_angle))
    return angle

def calculate_center(points):
    """计算一组点的中心"""
    return np.mean(points, axis=0).astype(int)

def points_close(center1, center2, threshold=20):
    """判断两个中心点是否足够接近"""
    return np.linalg.norm(np.array(center1) - np.array(center2)) < threshold

def find_rect_func(frame, center_record, find_rect, largest_rect, corner_points, red_and_green_roi):
    # 寻找矩形
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    thresh = cv2.adaptiveThreshold(
        blurred, 255, 
        cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
        cv2.THRESH_BINARY_INV, 11, 2
    )
    kernel = np.ones((5, 5), np.uint8)
    closed = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)
    closed = cv2.erode(closed, kernel, iterations=1)
    closed = cv2.dilate(closed, kernel, iterations=1)
    contours, _ = cv2.findContours(closed.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    max_area = 0
    for contour in contours:
        perimeter = cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, 0.02 * perimeter, True)
        if len(approx) == 4:
            contour_area = cv2.contourArea(contour)
            x, y, w, h = cv2.boundingRect(approx)
            bounding_area = w * h
            if contour_area > 30000 and contour_area < 80000 and contour_area / bounding_area > 0.5:
                is_rectangle = True
                angles = []
                for i in range(4):
                    p1 = approx[i][0]
                    p2 = approx[(i-1) % 4][0]
                    p3 = approx[(i+1) % 4][0]
                    angle = calculate_angle(p1, p2, p3)
                    angles.append(angle)
                    if not (80 <= angle <= 100):
                        is_rectangle = False
                        break
                if is_rectangle and contour_area > max_area:
                    max_area = contour_area
                    largest_rect = approx
                    corner_points = [tuple(p[0]) for p in approx]
                    if len(center_record) < 3:
                        center_record.append(calculate_center(corner_points))
                    else:
                        center_record[0] = center_record[1]
                        center_record[1] = center_record[2]
                        center_record[2] = calculate_center(corner_points)
                    if len(center_record) >= 3:
                        if (points_close(center_record[0], center_record[1]) and
                            points_close(center_record[1], center_record[2]) and
                            points_close(center_record[0], center_record[2])):
                            find_rect = False
                            xmin = min(corner_points[0][0], corner_points[1][0],corner_points[2][0], corner_points[3][0])
                            ymin = min(corner_points[0][1], corner_points[1][1],corner_points[2][1], corner_points[3][1])
                            xmax = max(corner_points[0][0], corner_points[1][0],corner_points[2][0], corner_points[3][0])
                            ymax = max(corner_points[0][1], corner_points[1][1],corner_points[2][1], corner_points[3][1])
                            red_and_green_roi.clear()
                            red_and_green_roi.extend([xmin - 100, ymin - 100, xmax + 100, ymax + 100])
    return find_rect, largest_rect, corner_points, center_record, red_and_green_roi

def find_laser_spot(frame, roi_box, hsv_lower, hsv_upper):
    """在ROI区域内识别最大激光光斑质心，返回(center_x, center_y)或None"""
    if len(roi_box) != 4:
        return None
    x1, y1, x2, y2 = roi_box
    x1 = max(0, x1)
    y1 = max(0, y1)
    x2 = min(frame.shape[1], x2)
    y2 = min(frame.shape[0], y2)
    roi = frame[y1:y2, x1:x2]
    if roi.size == 0:
        return None
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, hsv_lower, hsv_upper)
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    # 聚类合并轮廓
    if len(contours) > 1:
        centers = []
        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)
            center_x = x + w/2
            center_y = y + h/2
            centers.append((center_x, center_y))
        distance_threshold = 30
        merged_contours = []
        processed_indices = set()
        for i in range(len(contours)):
            if i in processed_indices:
                continue
            cluster_indices = [i]
            for j in range(i+1, len(contours)):
                if j in processed_indices:
                    continue
                dx = centers[i][0] - centers[j][0]
                dy = centers[i][1] - centers[j][1]
                distance = np.sqrt(dx*dx + dy*dy)
                if distance < distance_threshold:
                    cluster_indices.append(j)
                    processed_indices.add(j)
            merged_points = []
            for idx in cluster_indices:
                points = contours[idx].reshape(-1, 2)
                merged_points.extend(points)
            if merged_points:
                merged_points = np.array(merged_points).reshape(-1, 1, 2).astype(np.int32)
                merged_contours.append(merged_points)
        if merged_contours:
            contours = merged_contours
    # 筛选最大面积光斑
    max_area = 0
    best_center = None
    for contour in contours:
        area = cv2.contourArea(contour)
        if area > 10 and area > max_area:
            max_area = area
            x, y, w, h = cv2.boundingRect(contour)
            best_center = (int(x1 + x + w/2), int(y1 + y + h/2))
    return best_center

if __name__ == "__main__":
    find_laser()

