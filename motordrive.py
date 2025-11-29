import time
import math

class ClosedLoopMotors:
    def __init__(self, motors, encoders, kp=2.0, ki=10.0, kf=1.0):
        self.motors = motors
        self.encoders = encoders
        
        self.kp = kp
        self.ki = ki
        self.kf = kf
        
        self.R = 1.3   # 轮子半径，单位 cm
        self.TWO_L = 9.7 # 车轮间距，单位 cm
        self.HALF_L = self.TWO_L / 2.0 # 车轮间距一半，单位 cm
        
        self.COUNTS_PER_CM = 90.0 / (math.pi * self.R * 2.0) # 每 cm 的编码器计数
        
        self.integral_error_L = 0
        self.integral_error_R = 0
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
        speed_L = (current_counts[0] - self.last_counts[0]) / dt
        speed_R = (current_counts[1] - self.last_counts[1]) / dt
        
        self.last_time = now
        self.last_counts = current_counts
        
        real_v_L = speed_L / self.COUNTS_PER_CM
        real_v_R = speed_R / self.COUNTS_PER_CM
        self.current_v = (real_v_R + real_v_R) / 2.0
        self.current_omega = (real_v_R - real_v_L) / self.TWO_L

        error_L = target_L - speed_L
        self.integral_error_L += error_L * dt
        self.integral_error_L = max(-3000, min(3000, self.integral_error_L))
        pwm_L = (self.kp * error_L) + (self.ki * self.integral_error_L) + (target_L * self.kf)
        
        error_R = target_R - speed_R
        self.integral_error_R += error_R * dt
        self.integral_error_R = max(-3000, min(3000, self.integral_error_R))
        pwm_R = (self.kp * error_R) + (self.ki * self.integral_error_R) + (target_R * self.kf)
        
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