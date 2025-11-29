import time
from pololu_3pi_2040_robot import Robot
from bsp.PID import PIDController


class RobotMotor:
    """
    封装 Pololu 3pi+ 2040 Robot 单个电机的速度控制逻辑。
    """
    def __init__(self, robot_instance: Robot, motor_side: str, encoder_counts_per_rev: int = 2700):
            
            self.robot = robot_instance
            self.side = motor_side.lower()
            
            # Pololu 电机速度范围通常是 -1200 到 1200
            OUTPUT_MIN = -1200
            OUTPUT_MAX = 1200
            
            # 初始 PID 参数 (需要根据实际调参)
            Kp = 0.6
            Ki = 0.05
            Kd = 0.01
            
            # *** 实例化 PID 控制器 ***
            self.pid = PIDController(Kp, Ki, Kd, OUTPUT_MIN, OUTPUT_MAX)
            
            self.target_speed_rpm = 0.0
            self.encoder_counts_per_rev = encoder_counts_per_rev
            self.last_counts = self._get_current_counts()
            self.last_time = time.monotonic()


    def _get_current_counts(self):
        """
        获取当前编码器计数值。
        这部分是您已有的 Pololu 库接口。
        """
        if self.side == "left":
            # 假设 Pololu 库提供了一个获取编码器计数值的方法
            return self.robot.encoders.get_counts_left()
        elif self.side == "right":
            return self.robot.encoders.get_counts_right()
        else:
            return 0

    def _calculate_current_speed_rpm(self):
        """
        计算当前电机的实际速度 (RPM)。
        """
        current_counts = self._get_current_counts()
        current_time = time.monotonic()
        
        delta_counts = current_counts - self.last_counts
        delta_time = current_time - self.last_time
        
        self.last_counts = current_counts
        self.last_time = current_time
        
        if delta_time == 0:
            return 0.0

        # 将计数值转换为 RPM
        # (计数值 / 每转计数值) = 转数
        # (转数 / delta_time_秒) = RPS (转/秒)
        # RPS * 60 = RPM (转/分钟)
        revolutions = delta_counts / self.encoder_counts_per_rev
        rps = revolutions / delta_time
        rpm = rps * 60.0
        
        return rpm

    def set_speed_rpm(self, target_rpm: float):
        """
        设置电机的目标速度 (RPM)。
        """
        self.target_speed_rpm = target_rpm
        print(f"{self.side.capitalize()} 电机目标速度设置为: {target_rpm:.2f} RPM")

    def update_control_loop(self):
        """
        控制循环的核心。在主程序中需要定时调用此方法。
        """
        # 1. 获取当前速度反馈
        current_speed_rpm = self._calculate_current_speed_rpm()
        
        # 2. 计算 PID 控制输出 (PWM 占空比值)
        control_output = self.pid.compute(self.target_speed_rpm, current_speed_rpm)
        
        # 3. 应用控制输出到 Pololu 电机驱动
        self._apply_motor_output(int(control_output))
        
        return current_speed_rpm, control_output

    def _apply_motor_output(self, output: int):
        """
        将计算出的控制输出发送给 Pololu 电机驱动。
        """
        left_speed = self.robot.motors.get_speeds()[0] if self.side == "right" else output
        right_speed = self.robot.motors.get_speeds()[1] if self.side == "left" else output
        
        # Pololu 3pi+ 2040 Robot 通过 set_speeds 方法同时设置两个电机的速度
        self.robot.motors.set_speeds(left_speed, right_speed)

def stop(self):
        """
        停止电机。
        """
        self.set_speed_rpm(0.0)
        self.update_control_loop() # 立即执行一次，确保速度设置为 0
        
        # *** 调用 PID.py 中的 reset 方法 ***
        self.pid.reset()
        
        print(f"{self.side.capitalize()} 电机已停止。")