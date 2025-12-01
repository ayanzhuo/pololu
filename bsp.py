import time
import math
import _thread
from pololu_3pi_2040_robot import robot
from PID import PIDController 

# --- EMA 滤波系数：0.2 强, 0.8 弱 ---
ALPHA = 0.5
CONTROL_PERIOD_MS = 10

class MotorController:
    """
    封装了 3pi+ 2040 机器人左右轮的 EMA 滤波和 PID 速度控制逻辑。
    """
    def __init__(self, Kp=7.5, Ki=0.05, Kd=0.05):
        # --- 硬件接口 ---
        self.motors = robot.Motors()
        self.encoders = robot.Encoders()
        
        
        self.target_left_speed = 0.0
        self.target_right_speed = 0.0

        # --- PID 初始化和参数 ---
        output_limit = 10000
        self.pid_left = PIDController(
            Kp=Kp, Ki=Ki, Kd=Kd, output_min=-output_limit, output_max=output_limit
        )
        self.pid_right = PIDController(
            Kp=Kp, Ki=Ki, Kd=Kd, output_min=-output_limit, output_max=output_limit
        )

        # --- 状态和滤波变量 ---
        self.filtered_left_speed = 0.0
        self.filtered_right_speed = 0.0
        self.left_output = 0.0
        self.right_output = 0.0
        
        # --- 时间和计数记录 ---
        self.last_time = time.ticks_ms()
        self.last_left_count = 0
        self.last_right_count = 0

    def set_target_speeds(self, target_L, target_R):
        """设置新的目标速度 (counts/s)。"""
        self.target_left_speed = float(target_L)
        self.target_right_speed = float(target_R)

    def update_and_get_dt(self):

        current_time = time.ticks_ms()
        dt_ms = time.ticks_diff(current_time, self.last_time)

        if dt_ms < CONTROL_PERIOD_MS:
            return None 
        
        dt_s = dt_ms / 1000.0
        
        left_count, right_count = self.encoders.get_counts()

        left_speed_current = (left_count - self.last_left_count) / dt_s if dt_s > 0 else 0
        right_speed_current = (right_count - self.last_right_count) / dt_s if dt_s > 0 else 0


        self.filtered_left_speed = ALPHA * left_speed_current + (1 - ALPHA) * self.filtered_left_speed
        self.filtered_right_speed = ALPHA * right_speed_current + (1 - ALPHA) * self.filtered_right_speed


        self.left_output = self.pid_left.calculate(
            self.target_left_speed, self.filtered_left_speed
        )
        self.right_output = self.pid_right.calculate(
            self.target_right_speed, self.filtered_right_speed
        )

        # 5. 设置电机速度
        self.motors.set_speeds(int(self.left_output), int(self.right_output))
        
        # 6. 更新记录
        self.last_time = current_time
        self.last_left_count = left_count
        self.last_right_count = right_count
        
        return dt_s 

    def get_status(self):
        """返回当前的控制状态 (滤波速度, PID 输出)。"""
        return (
            self.filtered_left_speed, 
            self.filtered_right_speed, 
            self.left_output, 
            self.right_output
        )
    
    def motors_off(self):
        """关闭电机。"""
        self.motors.off()

#===================== 屏幕 =====================#

class RobotDisplay:
    def __init__(self, drive_instance):
        self.display = robot.Display()
        self.status_message = "INIT" 

    def set_custom_message(self, message): 
        self.status_message = message

    def run(self):
        while True:
           
            self.display.fill(0)
            
            # 优先显示自定义消息
            self.display.text(self.status_message, 0, 0, 1) # 显示自定义状态
                         
            self.display.show()
            time.sleep_ms(100)


#===================== 运动学解算 =====================#
class RobotDrive:

    WHEEL_RADIUS_MM = 16.0  
    WHEEL_SEPARATION_MM = 88.0
    ENCODER_CPR = 360.0 # Counts Per Revolution
    

    COUNTS_PER_MM = ENCODER_CPR / (2.0 * math.pi * WHEEL_RADIUS_MM)

    def __init__(self, Kp=5.5, Ki=0.05, Kd=0.05):
        self.controller = MotorController(Kp=Kp, Ki=Ki, Kd=Kd)
        self.current_v = 0.0 
        self.current_w = 0.0 
        

    def set_speed(self, v, w):

        v_L = v - w * (self.WHEEL_SEPARATION_MM / 2.0)
        v_R = v + w * (self.WHEEL_SEPARATION_MM / 2.0)

        target_L_counts_s = v_L * self.COUNTS_PER_MM
        target_R_counts_s = v_R * self.COUNTS_PER_MM

        self.controller.set_target_speeds(target_L_counts_s, target_R_counts_s)


    def update(self):

        dt_s = self.controller.update_and_get_dt() 
        
        if dt_s is None:
            return False 

        filtered_L_counts_s, filtered_R_counts_s, _, _ = self.controller.get_status()


        MM_PER_COUNT_S = 1.0 / self.COUNTS_PER_MM 
        v_L_actual = filtered_L_counts_s * MM_PER_COUNT_S
        v_R_actual = filtered_R_counts_s * MM_PER_COUNT_S
        

        v_actual = (v_R_actual + v_L_actual) / 2.0
        w_actual = (v_R_actual - v_L_actual) / self.WHEEL_SEPARATION_MM
        
        self.current_v = v_actual
        self.current_w = w_actual

        return True


    def motors_off(self):
        """关闭电机。"""
        self.controller.motors_off()

    def get_current_velocity(self): 
        """返回当前机器人的实际中心线速度 (v, mm/s) 和角速度 (w, rad/s)。"""
        return self.current_v, self.current_w
    
