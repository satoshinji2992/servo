import struct
from maix import image, display, app, time, camera
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

def find_green_laser():
    # 初始化
    disp = display.Display()
    cam = camera.Camera(320, 240, image.Format.FMT_BGR888)
    
    # 初始化通信协议 (自动根据系统配置初始化UART或TCP)
    p = comm.CommProtocol(buff_size=1024)
    
    # 定义命令
    APP_CMD_LASER_COORD = 0x10
    APP_CMD_SERVO_CONTROL = 0x11
    
    # 画面中心和死区
    CENTER_X, CENTER_Y = 160, 120
    DEAD_ZONE = 5
    
    # PID控制器 (稳定跟踪参数)
    pid_x = PIDController(kp=0.08, ki=0.002, kd=0.03, output_limit=10)
    pid_y = PIDController(kp=0.08, ki=0.002, kd=0.03, output_limit=10)
    
    # 时间记录
    last_time = time.ticks_ms()
    
    # 绿色激光HSV范围
    lower_green = np.array([40, 50, 50])
    upper_green = np.array([80, 255, 255])
    kernel = np.ones((3, 3), np.uint8)
    
    def encode_laser_coord(x, y):
        '''编码激光点坐标为字节数据'''
        return struct.pack("<HH", x, y)  # 2字节x + 2字节y (无符号，小端)
    
    def encode_servo_angles(delta_x, delta_y):
        '''编码舵机角度增量'''
        return struct.pack("<hh", delta_x, delta_y)  # 有符号16位
    
    while not app.need_exit():
        # 处理接收到的消息
        msg = p.get_msg()
        if msg and msg.is_req:
            if msg.cmd == APP_CMD_LASER_COORD:
                # 如果收到激光坐标请求，发送确认
                p.resp_ok(msg.cmd, b'\x01')
        
        # 计算时间间隔
        current_time = time.ticks_ms()
        dt = (current_time - last_time) / 1000.0  # 转换为秒
        last_time = current_time
        
        img = cam.read()
        frame = image.image2cv(img, ensure_bgr=False, copy=False)
        
        # 绘制中心十字线
        cv2.line(frame, (CENTER_X-10, CENTER_Y), (CENTER_X+10, CENTER_Y), (255, 255, 255), 1)
        cv2.line(frame, (CENTER_X, CENTER_Y-10), (CENTER_X, CENTER_Y+10), (255, 255, 255), 1)
        
        # HSV转换和掩码
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
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
                    
                    # 发送舵机控制命令
                    servo_body = encode_servo_angles(servo_delta_x, servo_delta_y)
                    p.report(APP_CMD_SERVO_CONTROL, servo_body)
                    
                    print(f"Laser: ({center[0]}, {center[1]}) Error: ({error_x}, {error_y}) Servo: ({servo_delta_x}, {servo_delta_y}) delta: ({delta_x}, {delta_y})")
                    
                    # 显示控制输出
                    cv2.putText(frame, f"Servo: ({servo_delta_x}, {servo_delta_y})", 
                               (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)
                else:
                    # 在死区内，不需要调整
                    cv2.putText(frame, "In Dead Zone", 
                               (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
                
                # 发送激光点坐标 (保留原功能)
                coord_body = encode_laser_coord(center[0], center[1])
                p.report(APP_CMD_LASER_COORD, coord_body)
        else:
            # 没有检测到激光点，重置PID
            pid_x.reset()
            pid_y.reset()
            cv2.putText(frame, "No Laser Detected", 
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)
        
        # 显示图像
        img_show = image.cv2image(frame, bgr=True, copy=False)
        disp.show(img_show)
        time.sleep_ms(50)

if __name__ == "__main__":
    find_green_laser()