import time
import math

class ClosedLoopMotors:
    def __init__(self, motors, encoders, kp=0.5, ki=0.2, kf=0.1):
        self.motors = motors
        self.encoders = encoders
        
        self.kp = kp  # 速度域比例增益（单位：PWM/(cm/s)）
        self.ki = ki  # 速度域积分增益
        self.kf = kf  # 前馈系数（开环补偿）
        
        self.R = 1.3   # 轮子半径，单位 cm
        self.TWO_L = 9.7 # 车轮间距，单位 cm
        self.HALF_L = self.TWO_L / 2.0 # 车轮间距一半，单位 cm
        
        self.COUNTS_PER_CM = 90.0 / (math.pi * self.R * 2.0) # 每 cm 的编码器计数
        
        # 在速度域（cm/s）积分误差
        self.integral_error_L = 0.0
        self.integral_error_R = 0.0
        self.integral_error_limit = 100.0  # 防止积分饱和
        
        # 速度滤波（低通滤波系数，0-1）
        self.velocity_filter = 0.6
        self.last_real_v_L = 0.0
        self.last_real_v_R = 0.0
        
        self.last_time = time.ticks_us()
        self.last_counts = self.encoders.get_counts()
        
        self.current_v = 0.0
        self.current_omega = 0.0

    def car_move(self, v_cm_s, omega_rad_s):
        target_v_R = v_cm_s + (omega_rad_s * self.HALF_L)
        target_v_L = v_cm_s - (omega_rad_s * self.HALF_L)
        
        target_counts_R = target_v_R * self.COUNTS_PER_CM
        target_counts_L = target_v_L * self.COUNTS_PER_CM
        
        self._set_target_counts(target_counts_L, target_counts_R)

    def _set_target_counts(self, target_L, target_R):
        now = time.ticks_us()
        dt = time.ticks_diff(now, self.last_time) / 1000000.0
        if dt <= 0: return
        
        current_counts = self.encoders.get_counts()
        speed_L = (current_counts[0] - self.last_counts[0]) / dt  # counts/s
        speed_R = (current_counts[1] - self.last_counts[1]) / dt  # counts/s
        
        self.last_time = now
        self.last_counts = current_counts
        
        # 从计数速度转换到 cm/s
        real_v_L = speed_L / self.COUNTS_PER_CM
        real_v_R = speed_R / self.COUNTS_PER_CM
        
        # 低通滤波速度信号，降低噪声
        real_v_L = (real_v_L * self.velocity_filter) + (self.last_real_v_L * (1 - self.velocity_filter))
        real_v_R = (real_v_R * self.velocity_filter) + (self.last_real_v_R * (1 - self.velocity_filter))
        self.last_real_v_L = real_v_L
        self.last_real_v_R = real_v_R
        
        # 计算当前速度和角速度（用于外部读取或诊断）
        self.current_v = (real_v_L + real_v_R) / 2.0
        self.current_omega = (real_v_R - real_v_L) / self.TWO_L
        
        # 目标速度（cm/s）
        target_v_L = target_L / self.COUNTS_PER_CM
        target_v_R = target_R / self.COUNTS_PER_CM

        # 在速度域（cm/s）进行 PID 控制
        error_v_L = target_v_L - real_v_L
        self.integral_error_L += error_v_L * dt
        self.integral_error_L = max(-self.integral_error_limit, min(self.integral_error_limit, self.integral_error_L))
        pwm_L = (self.kp * error_v_L) + (self.ki * self.integral_error_L) + (self.kf * target_v_L)
        
        error_v_R = target_v_R - real_v_R
        self.integral_error_R += error_v_R * dt
        self.integral_error_R = max(-self.integral_error_limit, min(self.integral_error_limit, self.integral_error_R))
        pwm_R = (self.kp * error_v_R) + (self.ki * self.integral_error_R) + (self.kf * target_v_R)
        
        self.motors.set_speeds(int(max(-6000, min(6000, pwm_L))), 
                               int(max(-6000, min(6000, pwm_R))))

    def get_kinematics(self):
        return self.current_v, self.current_omega

    def off(self):
        self.motors.off()
        self.integral_error_L = 0
        self.integral_error_R = 0
        self.last_time = time.ticks_us()
        self.last_counts = self.encoders.get_counts()
        self.current_v = 0
        self.current_omega = 0