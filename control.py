import struct
from maix import image, display, app, time, camera, touchscreen
from maix import comm, protocol
import cv2
import numpy as np

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
        return int(output)
    
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

def find_green_laser():
    # 初始化
    disp = display.Display()
    cam = camera.Camera(320, 240, image.Format.FMT_BGR888)
    
    # 初始化通信协议 (自动根据系统配置初始化UART或TCP)
    p = comm.CommProtocol(buff_size=1024)
    
    # 画面中心和死区
    CENTER_X, CENTER_Y = 160, 120
    DEAD_ZONE = 3
    
    # PID控制器 (稳定跟踪参数)
    pid_x = PIDController(kp=0.15, ki=0.0012, kd=0.008, output_limit=10)
    pid_y = PIDController(kp=0.08, ki=0.001, kd=0.008, output_limit=10)
    
    # 初始化HSV控制器
    hsv_controller = HSVController([30, 40, 30], [80, 255, 255])
    
    # 初始化触摸界面
    touch_ui = TouchInterface(320, 240)
    
    # 时间记录
    last_time = time.ticks_ms()
    
    kernel = np.ones((3, 3), np.uint8)
    
    # 创建界面按钮
    # 主选择界面按钮
    h_btn = touch_ui.create_button(10, 200, 30, 25, "H")
    s_btn = touch_ui.create_button(50, 200, 30, 25, "S") 
    v_btn = touch_ui.create_button(90, 200, 30, 25, "V")
    
    # 调节界面按钮
    min_minus_btn = touch_ui.create_button(10, 180, 35, 20, "Min-")
    min_plus_btn = touch_ui.create_button(50, 180, 35, 20, "Min+")
    max_minus_btn = touch_ui.create_button(90, 180, 35, 20, "Max-")
    max_plus_btn = touch_ui.create_button(130, 180, 35, 20, "Max+")
    exit_btn = touch_ui.create_button(170, 180, 35, 20, "Exit")
    
    while not app.need_exit():
        # 计算时间间隔
        current_time = time.ticks_ms()
        dt = (current_time - last_time) / 1000.0  # 转换为秒
        last_time = current_time
        
        img = cam.read()
        frame = image.image2cv(img, ensure_bgr=False, copy=False)
        
        # 处理触摸输入
        x_touch, y_touch, pressed, clicked, x_raw, y_raw = touch_ui.check_touch(disp)
        
        # 显示触摸调试信息
        if pressed or clicked:
            cv2.putText(frame, f"Raw: ({x_raw}, {y_raw}) Mapped: ({x_touch}, {y_touch})", 
                       (10, 220), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255, 0, 255), 1)
            cv2.putText(frame, f"Pressed: {pressed} Clicked: {clicked}", 
                       (10, 235), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255, 0, 255), 1)
        
        # 处理界面状态和按钮
        if hsv_controller.current_mode == "select":
            # 主选择界面
            touch_ui.draw_button(frame, h_btn)
            touch_ui.draw_button(frame, s_btn)
            touch_ui.draw_button(frame, v_btn)
            
            # 显示当前HSV值
            lower, upper = hsv_controller.get_hsv_range()
            cv2.putText(frame, f"HSV: [{lower[0]}-{upper[0]}, {lower[1]}-{upper[1]}, {lower[2]}-{upper[2]}]", 
                       (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255, 255, 255), 1)
            
            # 显示模式
            cv2.putText(frame, "Mode: SELECT - Touch H/S/V buttons", 
                       (130, 210), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0, 255, 255), 1)
            
            # 使用pressed状态检测按钮（参考示例）
            if pressed:
                if touch_ui.is_button_pressed(x_touch, y_touch, h_btn):
                    hsv_controller.current_mode = "h"
                    print("Switched to H mode")
                elif touch_ui.is_button_pressed(x_touch, y_touch, s_btn):
                    hsv_controller.current_mode = "s"
                    print("Switched to S mode")
                elif touch_ui.is_button_pressed(x_touch, y_touch, v_btn):
                    hsv_controller.current_mode = "v"
                    print("Switched to V mode")
        
        else:  # 在调节模式 (h, s, v)
            # 显示调节按钮
            touch_ui.draw_button(frame, min_minus_btn)
            touch_ui.draw_button(frame, min_plus_btn)
            touch_ui.draw_button(frame, max_minus_btn)
            touch_ui.draw_button(frame, max_plus_btn)
            touch_ui.draw_button(frame, exit_btn)
            
            # 获取当前调节的通道索引
            channel_map = {"h": 0, "s": 1, "v": 2}
            channel_idx = channel_map[hsv_controller.current_mode]
            
            # 显示当前调节通道信息
            lower, upper = hsv_controller.get_hsv_range()
            cv2.putText(frame, f"Adjusting {hsv_controller.current_mode.upper()}: Min={lower[channel_idx]}, Max={upper[channel_idx]}", 
                       (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)
            
            # 使用clicked事件处理调节按钮（避免连续触发）
            if clicked:
                step = hsv_controller.step_size[channel_idx]
                
                if touch_ui.is_button_pressed(x_touch, y_touch, min_minus_btn):
                    hsv_controller.lower_hsv[channel_idx] = max(0, hsv_controller.lower_hsv[channel_idx] - step)
                    print(f"Min- clicked: {hsv_controller.current_mode}={hsv_controller.lower_hsv[channel_idx]}")
                elif touch_ui.is_button_pressed(x_touch, y_touch, min_plus_btn):
                    max_val = 179 if channel_idx == 0 else 255
                    hsv_controller.lower_hsv[channel_idx] = min(max_val, hsv_controller.lower_hsv[channel_idx] + step)
                    print(f"Min+ clicked: {hsv_controller.current_mode}={hsv_controller.lower_hsv[channel_idx]}")
                elif touch_ui.is_button_pressed(x_touch, y_touch, max_minus_btn):
                    hsv_controller.upper_hsv[channel_idx] = max(0, hsv_controller.upper_hsv[channel_idx] - step)
                    print(f"Max- clicked: {hsv_controller.current_mode}={hsv_controller.upper_hsv[channel_idx]}")
                elif touch_ui.is_button_pressed(x_touch, y_touch, max_plus_btn):
                    max_val = 179 if channel_idx == 0 else 255
                    hsv_controller.upper_hsv[channel_idx] = min(max_val, hsv_controller.upper_hsv[channel_idx] + step)
                    print(f"Max+ clicked: {hsv_controller.current_mode}={hsv_controller.upper_hsv[channel_idx]}")
                elif touch_ui.is_button_pressed(x_touch, y_touch, exit_btn):
                    hsv_controller.current_mode = "select"
                    print("Exit to select mode")
        
        # 绘制中心十字线
        cv2.line(frame, (CENTER_X-10, CENTER_Y), (CENTER_X+10, CENTER_Y), (255, 255, 255), 1)
        cv2.line(frame, (CENTER_X, CENTER_Y-10), (CENTER_X, CENTER_Y+10), (255, 255, 255), 1)
        
        # HSV转换和掩码
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower_green, upper_green = hsv_controller.get_hsv_range()
        mask = cv2.inRange(hsv, lower_green, upper_green)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        
        # 查找轮廓
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            (x, y), radius = cv2.minEnclosingCircle(largest_contour)
            center = (int(x), int(y))
            
            if 2 < radius < 50:
                # 绘制激光点
                cv2.circle(frame, center, int(radius), (0, 255, 0), 2)
                cv2.circle(frame, center, 2, (0, 0, 255), -1)
                cv2.putText(frame, f"{center}", (center[0] + 10, center[1] - 10), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
                
                # 计算误差 (激光点相对于画面中心的偏移)
                error_x = center[0] - CENTER_X  # 正值：激光点在右侧
                error_y = center[1] - CENTER_Y  # 正值：激光点在下方
                
                # 显示误差信息
                cv2.putText(frame, f"Error: ({error_x}, {error_y})", 
                           (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
                
                # 检查是否在死区内
                if abs(error_x) > DEAD_ZONE or abs(error_y) > DEAD_ZONE:
                    # PID计算角度增量
                    delta_x = pid_x.update(error_x, dt)  # X轴：正值向右转(减小angle)
                    delta_y = pid_y.update(error_y, dt)  # Y轴：正值向下转(减小angle)
                    
                    # 根据舵机方向调整：
                    # 激光点在右侧(error_x>0) -> 舵机向右转 -> angle减小 -> delta_x为负
                    # 激光点在下方(error_y>0) -> 舵机向下转 -> angle减小 -> delta_y为负
                    servo_delta_x = -delta_x  # 反向
                    servo_delta_y = -delta_y  # 反向
                    
                    # 直接发送两个改变值
                    data = struct.pack("<hh", servo_delta_x, servo_delta_y)
                    p.report(0x11, data)
                    
                    print(f"Laser: ({center[0]}, {center[1]}) Error: ({error_x}, {error_y}) Servo: ({servo_delta_x}, {servo_delta_y}) delta: ({delta_x}, {delta_y})")
                    
                    # 显示控制输出
                    cv2.putText(frame, f"Servo: ({servo_delta_x}, {servo_delta_y})", 
                               (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)
                else:
                    # 在死区内，不需要调整
                    cv2.putText(frame, "In Dead Zone", 
                               (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
                
        else:
            # 没有检测到激光点，重置PID
            pid_x.reset()
            pid_y.reset()
            cv2.putText(frame, "No Laser Detected", 
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)
        
        # 显示图像
        img_show = image.cv2image(frame, bgr=True, copy=False)
        disp.show(img_show, fit=image.Fit.FIT_CONTAIN)
        time.sleep_ms(50)

if __name__ == "__main__":
    find_green_laser()