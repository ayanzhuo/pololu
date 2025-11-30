import time

class PIDController:
    """
    通用的 PID 控制器实现。
    """
    def __init__(self, Kp, Ki, Kd, output_min=None, output_max=None):
        """
        初始化 PID 控制器参数。
        """
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.output_min = output_min
        self.output_max = output_max
        
        self.integral_error = 0.0
        self.last_error = 0.0
        
        self.last_time = time.ticks_ms() 
        
        self.last_output = 0.0

    def calculate(self, setpoint, measured_value):
        """
        计算 PID 输出。
        """
        current_time = time.ticks_ms()
        
        dt_ms = time.ticks_diff(current_time, self.last_time)
        
        # 将毫秒转换为秒 (dt_s)
        dt_s = dt_ms / 1000.0
        
        # 在 PID 公式中，我们使用 dt_s
        dt = dt_s 
        
        if dt <= 0:
            # 避免除以零和不必要的重复计算
            return self.last_output

        error = setpoint - measured_value
        
        # 比例项 (P)
        p_term = self.Kp * error
        
        # 积分项 (I)
        self.integral_error += error * dt
        # 积分限幅 (防止积分饱和)
        self.integral_error = max(-1000, min(1000, self.integral_error))
        i_term = self.Ki * self.integral_error
        
        # 微分项 (D)
        derivative_error = (error - self.last_error) / dt
        d_term = self.Kd * derivative_error
        
        # 总输出
        output = p_term + i_term + d_term
        
        # 输出限幅
        if self.output_min is not None and self.output_max is not None:
            output = max(self.output_min, min(self.output_max, output))
        
        self.last_error = error
        

        self.last_time = current_time
        self.last_output = output
        
        return output

    def reset(self):
        """
        重置 PID 控制器的状态（用于停止或模式切换）。
        """
        self.integral_error = 0.0
        self.last_error = 0.0
        self.last_output = 0.0
        

        self.last_time = time.ticks_ms()
