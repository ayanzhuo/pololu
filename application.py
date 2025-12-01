import time
import _thread
from pololu_3pi_2040_robot import robot
from bsp import RobotDrive 



class LineFollower:
    """
    实现了基于线传感器的比例(P)循迹控制逻辑。
    负责读取传感器、计算误差，并生成 v, w 速度指令。
    """
    
    # --- 循迹常量 ---
    BASE_SPEED_MM_S = 800.0   # 基础前进速度 (v)
    MAX_STEER_OMEGA = 30.0     # 最大转向角速度 (w, rad/s)

    #  校准扫动参数
    CAL_SWEEP_V = 0.0       # 前进基础速度 (mm/s)
    CAL_SWEEP_W = 5.0        # 快速转动角速度 (rad/s)
    CAL_SWEEP_DURATION = 800  # 扫描持续时间 (毫秒)
    CAL_LOOP_DELAY = 20       # 校准循环延迟 (毫秒)
    #  控制参数
    STEERING_KP_BASE = 0.12   #直线抖动较小，可以微调
    STEERING_KP_HIGH = 0.2 
    ERROR_THRESHOLD = 1000    
    SENSOR_WEIGHTS = [-2000, -1000, 0, 1000, 2000] 
    
    def __init__(self, drive_controller):

        self.drive = drive_controller
        self.line_sensors = robot.LineSensors()
        self.sensor_count = len(self.line_sensors.read_calibrated())

        
    def _calculate_error(self):

        readings = self.line_sensors.read_calibrated() # 返回 0-1000 的值
        
      
        weighted_sum = 0
        total_value = 0
        
        for i in range(self.sensor_count):
            value = 1000 - readings[i] # 反转：让 '在白线上' (低值) 成为权重计算的低贡献者
            weighted_sum += value * self.SENSOR_WEIGHTS[i]
            total_value += value
            
        if total_value == 0:
            # 如果所有传感器都在白色上（或所有值都太低），保持上次状态或直线前进
            return 0 
        
        # 误差 E 的范围大约在 -2000 到 +2000 之间 (取决于传感器灵敏度)
        error = weighted_sum / total_value
        
        return error

    def follow_line(self):

        error = self._calculate_error()
    
        abs_error = abs(error)
        steering_kp = self.STEERING_KP_BASE

        if abs_error > self.ERROR_THRESHOLD:
            steering_kp = self.STEERING_KP_HIGH
    
        steering_omega = steering_kp * error

        
        if steering_omega > self.MAX_STEER_OMEGA:
            steering_omega = self.MAX_STEER_OMEGA
        elif steering_omega < -self.MAX_STEER_OMEGA:
            steering_omega = -self.MAX_STEER_OMEGA

        self.drive.set_speed(v=self.BASE_SPEED_MM_S, w=steering_omega)
        
        return error, steering_omega
    
    def _perform_sweep(self, angular_velocity, duration_ms):
        
        self.drive.set_speed(self.CAL_SWEEP_V, angular_velocity)
        
        start_time = time.ticks_ms()
        

        while time.ticks_diff(time.ticks_ms(), start_time) < duration_ms:
            self.line_sensors.calibrate()
            
            # 持续驱动 PID 闭环，实现平滑运动
            self.drive.update() 
            time.sleep_ms(self.CAL_LOOP_DELAY)

        # 3. 扫动完成后停止，并等待片刻
        self.drive.set_speed(0, 0)
        # 确保 PID 驱动的电机有时间减速
        for _ in range(5): 
            self.drive.update()
            time.sleep_ms(self.CAL_LOOP_DELAY)
        
        time.sleep_ms(100) # 短暂休息

    def calibrate_motorsweep(self):
    
        self._perform_sweep(-self.CAL_SWEEP_W, self.CAL_SWEEP_DURATION) 
        self._perform_sweep(self.CAL_SWEEP_W, self.CAL_SWEEP_DURATION)

        self.drive.set_speed(0, 0)
        
        for _ in range(30):
            self.drive.update()
            time.sleep_ms(10)
            
        self.drive.motors_off()
