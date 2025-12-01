from pololu_3pi_2040_robot import robot
from PID import PIDController
import time
import _thread

# --- 常量和滤波变量 ---
# EMA 滤波系数：0.2 滤波效果强，0.8 滤波效果弱
ALPHA = 0.5
CONTROL_PERIOD_MS = 10

# 滤波后的速度变量（必须在主线程外部声明）
filtered_left_speed = 0.0
filtered_right_speed = 0.0

# --- 硬件初始化 ---
display = robot.Display()
motors = robot.Motors()
encoders = robot.Encoders()

# --- PID 初始化和参数 ---
target_left_speed = 2000.0
target_right_speed = 2000.0


pid_left = PIDController(Kp=5.5, Ki=0.05, Kd=0.05,
                         output_min=-10000, output_max=10000)
pid_right = PIDController(Kp=5.5, Ki=0.05, Kd=0.05,
                          output_min=-10000, output_max=10000)

# 共享数据结构：[Filtered_L_Speed(Counts/s), Filtered_R_Speed(Counts/s), Output_L, Output_R]
pid_data = [0.0, 0.0, 0.0, 0.0]

# --- 状态记录 ---
last_time = time.ticks_ms()
last_left_count = 0
last_right_count = 0

# --- 线程函数 ---


def display_thread_function():
    while True:
        L_speed, R_speed, L_output, R_output = pid_data
        display.fill(0)
        display.text(f"L:{L_speed:.0f}", 0, 0, 1)
        display.text(f"R:{R_speed:.0f}", 0, 8, 1)
        display.text(f"LO:{L_output:.0f}", 0, 16, 1)
        display.text(f"RO:{R_output:.0f}", 0, 24, 1)
        display.show()
        time.sleep_ms(100)


_thread.start_new_thread(display_thread_function, ())


# ===================== 主控制循环 =====================#
try:

    while True:
        current_time = time.ticks_ms()
        dt_ms = time.ticks_diff(current_time, last_time)

        if dt_ms >= CONTROL_PERIOD_MS:
            dt_s = dt_ms / 1000.0

            left_count, right_count = encoders.get_counts()

            left_speed_current = (
                left_count - last_left_count) / dt_s if dt_s > 0 else 0
            right_speed_current = (
                right_count - last_right_count) / dt_s if dt_s > 0 else 0

            filtered_left_speed = ALPHA * left_speed_current + \
                (1 - ALPHA) * filtered_left_speed
            filtered_right_speed = ALPHA * right_speed_current + \
                (1 - ALPHA) * filtered_right_speed

            left_output = pid_left.calculate(
                target_left_speed, filtered_left_speed)
            right_output = pid_right.calculate(
                target_right_speed, filtered_right_speed)

            motors.set_speeds(int(left_output), int(right_output))

            pid_data[0] = filtered_left_speed
            pid_data[1] = filtered_right_speed
            pid_data[2] = left_output
            pid_data[3] = right_output

            last_time = current_time
            last_left_count = left_count
            last_right_count = right_count

        time.sleep_ms(1)

except KeyboardInterrupt:
    motors.off()
    display.fill(0)
    display.show()
