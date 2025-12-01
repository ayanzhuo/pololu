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
    BASE_SPEED_MM_S = 600.0   # 基础前进速度 (v)
    MAX_STEER_OMEGA = 25.0     # 最大转向角速度 (w, rad/s)

    #  校准扫动参数
    CAL_SWEEP_V = 0.0       # 前进基础速度 (mm/s)
    CAL_SWEEP_W = 8.0        # 快速转动角速度 (rad/s)
    CAL_SWEEP_DURATION = 800  # 扫描持续时间 (毫秒)
    CAL_LOOP_DELAY = 20       # 校准循环延迟 (毫秒)
    #  控制参数
    STEERING_KP_BASE = 0.006
    STEERING_KP_HIGH = 0.024

    ERROR_THRESHOLD = 1000
    SENSOR_WEIGHTS = [-2000, -1000, 0, 1000, 2000]
    
    # --- 新增 IMU/转弯常量 ---
    GYRO_KP = 140.0         # 比例增益 (P)
    GYRO_KD = 4.0           # 微分增益 (D)
    MAX_TURN_SPEED = 3000   # 最大转弯速度 (counts/s)
    ANGLE_TOLERANCE = 3.0   # 停止转弯的角度容忍度 (度)
    SPECIAL_TURN_ANGLE = -90.0 # 目标左转 90 度 (终点左转)

    # === 新增 IMU 硬件和状态 ===


    def __init__(self, drive_controller):

        self.drive = drive_controller
        self.line_sensors = robot.LineSensors()
        self.sensor_count = len(self.line_sensors.read_calibrated())
        self.bump_sensors = robot.BumpSensors()
        self.bump_sensors.calibrate()
        self.imu = robot.IMU()
        self.imu.enable_default() # 启用 IMU
        
        self.robot_angle = 0.0              # 当前机器人角度 (度)
        self.target_angle = 0.0             # 目标角度 (度)
        self.last_time_gyro_reading = None  # 上次读取 IMU 的时间戳 (us)
        self.turn_rate = 0.0                # 当前转弯角速度 (度/秒)
        self.is_turning = False             # 标志是否正在执行陀螺仪转弯

        self.special_action_counter = 0

    def _calculate_error(self):

        readings = self.line_sensors.read_calibrated()

        weighted_sum = 0
        total_value = 0

        for i in range(self.sensor_count):
            value = 1000 - readings[i]  
            weighted_sum += value * self.SENSOR_WEIGHTS[i]
            total_value += value

        if total_value == 0:
            return 0

        error = weighted_sum / total_value

        return error
    def wait_for_collision_release(self):
        self.bump_sensors.read() 
        is_pressed = self.bump_sensors.left_is_pressed() or self.bump_sensors.right_is_pressed()
        while is_pressed:
            self.bump_sensors.read() 
            is_pressed = self.bump_sensors.left_is_pressed() or self.bump_sensors.right_is_pressed()
            time.sleep_ms(10) 

        time.sleep_ms(20) 

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

        time.sleep_ms(100)

    def calibrate_motorsweep(self):

        self._perform_sweep(-self.CAL_SWEEP_W, self.CAL_SWEEP_DURATION)
        self._perform_sweep(self.CAL_SWEEP_W, self.CAL_SWEEP_DURATION)

        self.drive.set_speed(0, 0)

        for _ in range(30):
            self.drive.update()
            time.sleep_ms(10)

        self.drive.motors_off()
    def update_angle(self):
        """从陀螺仪读取数据，并积分计算机器人的当前角度 (Yaw)。"""
        if self.imu.gyro.data_ready():
            self.imu.gyro.read()
            self.turn_rate = self.imu.gyro.last_reading_dps[2] 
            
            now = time.ticks_us()
            
            if self.last_time_gyro_reading:
                dt = time.ticks_diff(now, self.last_time_gyro_reading)
                self.robot_angle += self.turn_rate * dt / 1000000
                
            self.last_time_gyro_reading = now

    def start_gyro_turn(self, angle_to_add):
        """设置目标角度并启动陀螺仪转弯模式。"""
        # 重置角度追踪，从当前朝向开始计算
        self.robot_angle = 0.0 
        self.last_time_gyro_reading = None 
        
        self.target_angle = angle_to_add 
        self.is_turning = True
        
    def gyro_turn_step(self):
        """执行一个周期的 PD 转向控制。返回 True 表示转弯仍在进行中。"""
        if not self.is_turning:
            return False

        angle_error = self.target_angle - self.robot_angle
        
        # 检查是否接近目标角度 (停止条件)
        if abs(angle_error) < self.ANGLE_TOLERANCE:
            self.is_turning = False
            # 停止电机，因为我们不再需要 PID/PD 控制
            self.drive.motors_off() 
            return False # 转弯完成

        # PD 控制计算转速
        turn_speed = angle_error * self.GYRO_KP - self.turn_rate * self.GYRO_KD
        
        # 钳位速度
        if turn_speed > self.MAX_TURN_SPEED:
            turn_speed = self.MAX_TURN_SPEED
        elif turn_speed < -self.MAX_TURN_SPEED:
            turn_speed = -self.MAX_TURN_SPEED
            
        # 应用电机速度 (注意：这里需要调用 RobotDrive 底层 set_speeds 来精确控制左右轮)
        # 假设你的 RobotDrive 内部有一个 motors 对象 (robot.Motors)
        self.drive.controller.motors.set_speeds(int(-turn_speed), int(turn_speed))
        
        # 注意：这里我们依赖主循环中对 robot_drive.update() 的调用来执行速度闭环。
        # 如果你的 motors.set_speeds 是直接的 PWM 设置，则不需要 robot_drive.update()，
        # 但如果 motors.set_speeds 是目标速度，则需要。
        # 鉴于示例代码直接控制 motors，我们在这里依赖它立即生效。
        
        return True # 转弯进行中
# ==========================================
    def _speacial_black(self):
        readings = self.line_sensors.read_calibrated()

        BLACK_THRESHOLD = 800

        if (
            readings[1] > BLACK_THRESHOLD
        ):
            return True
        return False

    def _is_all_black(self):
        """
        检查中间三个循线传感器（索引 1, 2, 3）是否都检测到了黑色。
        这通常用于识别终点黑线或特定标记。
        """
        # 读取原始校准值
        readings = self.line_sensors.read_calibrated()

        BLACK_THRESHOLD = 750

        if (readings[1] > BLACK_THRESHOLD and
            readings[2] > BLACK_THRESHOLD and
                readings[3] > BLACK_THRESHOLD):
            return True
        return False

    FIXED_TURN_W = 10.0  # 固定的左转角速度 (负值表示左转，确保其足够大)
    SPECIAL_TURN_W = 4.0  # 特殊情况的左转角速度

    def lane_keep(self):
        # 1. 如果正在执行陀螺仪转弯，直接跳过所有循线逻辑
        if self.is_turning:
            # 返回 0, 0 让主循环知道当前正在执行特殊动作
            return 0.0, 0.0
        
        if self._is_all_black():

            self.drive.set_speed(v=0.0, w=self.FIXED_TURN_W)

            return 0.0, self.FIXED_TURN_W
        
        if self._speacial_black() and self.special_action_counter == 0: 
            counter += 1
            self.start_gyro_turn(self.SPECIAL_TURN_ANGLE) 
            return 0.0, 0.0
        
        if self._is_on_green_marker():
            self.drive.set_speed(v=0.0, w=self.FIXED_TURN_W)
            return 0.0, self.FIXED_TURN_W

        error = self._calculate_error()

        abs_error = abs(error)
        steering_kp = self.STEERING_KP_BASE

        # 使用分段 KP
        if abs_error > self.ERROR_THRESHOLD:
            steering_kp = self.STEERING_KP_HIGH
            
        steering_omega = steering_kp * (-error)

        if steering_omega > self.MAX_STEER_OMEGA:
            steering_omega = self.MAX_STEER_OMEGA
        elif steering_omega < -self.MAX_STEER_OMEGA:
            steering_omega = -self.MAX_STEER_OMEGA

        self.drive.set_speed(v=self.BASE_SPEED_MM_S, w=steering_omega)

        return error, steering_omega

# ===================终点检测=========================

    def _is_on_green_marker(self):
        readings = self.line_sensors.read_calibrated()

        GREEN_MIN_THRESHOLD = 300
        GREEN_MAX_THRESHOLD = 700


        middle_readings = [readings[0], readings[1],
                           readings[2], readings[3], readings[4]]


        for r in middle_readings:
            if not (GREEN_MIN_THRESHOLD <= r <= GREEN_MAX_THRESHOLD):
                return False  

        return True
