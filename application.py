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

    def __init__(self, drive_controller):

        self.drive = drive_controller
        self.line_sensors = robot.LineSensors()
        self.sensor_count = len(self.line_sensors.read_calibrated())

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
        counter = 0
        if self._is_all_black():

            self.drive.set_speed(v=0.0, w=self.FIXED_TURN_W)

            return 0.0, self.FIXED_TURN_W
        
        if self._speacial_black() and counter == 0:
            counter += 1
 
            self.drive.set_speed(v=0.0, w=self.SPECIAL_TURN_W)

            start_time = time.ticks_ms()
            while time.ticks_diff(time.ticks_ms(), start_time) < 500:
                self.drive.update()  # 关键：确保电机 PID 在转弯期间持续更新
                time.sleep_ms(5)     # 短暂等待，避免 CPU 占用过高
            # ===============================

            self.drive.set_speed(v=0.0, w=0.0)
            
            return 0.0, self.SPECIAL_TURN_W
        
        if self._is_on_green_marker():
            self.drive.set_speed(v=0.0, w=self.FIXED_TURN_W)
            return 0.0, self.FIXED_TURN_W

        error = self._calculate_error()

        abs_error = abs(error)
        steering_kp = self.STEERING_KP_BASE

        # 使用分段 KP
        if abs_error > self.ERROR_THRESHOLD:
            steering_kp = self.STEERING_KP_HIGH

        # 修正转向：error > 0 (偏右) 需要 w < 0 (左转)
        steering_omega = steering_kp * (-error)

        # 钳位，限制最大转向角速度
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
