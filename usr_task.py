from pololu_3pi_2040_robot import robot
from motordrive import ClosedLoopMotors
import _thread
import time
import math

# ==========================================
# 1. 初始化与配置
# ==========================================
motors = robot.Motors()
encoders = robot.Encoders()
display = robot.Display()
bump_sensors = robot.BumpSensors()
line_sensors = robot.LineSensors()
buzzer = robot.Buzzer()
led = robot.YellowLED()
imu = robot.IMU()
imu.reset()
imu.enable_default()

driver = ClosedLoopMotors(motors, encoders, kp=0.5, ki=0.2, kf=0.1)


TARGET_VELOCITY_CM = 80.0

LQR_K1 = 0.003
LQR_K2 = 0.6

# 陀螺仪变量
gyro_offset = 0
current_gyro_rad = 0
GYRO_FILTER = 0.4
last_error = 0

# 状态机
MODE_STOP = 0
MODE_FOLLOW = 1
MODE_LANE_KEEP = 2
current_mode = MODE_STOP
play_music_flag = False
music_flag_lock = _thread.allocate_lock()  # 线程互斥锁

# 音乐
nyan_cat_melody = "T140 O5 f#8 g#8 d#8 d#8 b8 d8 c#8 b4"

# 调试模式开关
DEBUG = True  # 设置为 True 启用调试，False 禁用


def music_thread_func():
    global play_music_flag
    while True:
        with music_flag_lock:
            flag = play_music_flag
        if flag:
            if not buzzer.is_playing():
                buzzer.play(nyan_cat_melody)
            time.sleep_ms(100)
        else:
            if buzzer.is_playing():
                buzzer.off()
            time.sleep_ms(100)


_thread.start_new_thread(music_thread_func, ())


def calibrate_gyro():
    global gyro_offset
    total = 0
    count = 0
    for i in range(200):  # 增加循环次数确保采样充分
        if imu.gyro.data_ready():
            imu.gyro.read()
            total += imu.gyro.last_reading_dps[2]
            count += 1
        if count >= 100:  # 达到100个样本后停止
            break
        time.sleep_ms(2)
    gyro_offset = total / count if count > 0 else 0


def lqr_follow_step():
    global last_error, current_gyro_rad

    line = line_sensors.read_calibrated()

    # 获取实时运动学反解
    current_v, current_omega = driver.get_kinematics()

    # 更新陀螺仪滤波
    if imu.gyro.data_ready():
        imu.gyro.read()
        dps_z = imu.gyro.last_reading_dps[2] - gyro_offset
        rad_z = math.radians(dps_z)
        current_gyro_rad = (rad_z * GYRO_FILTER) + \
            (current_gyro_rad * (1 - GYRO_FILTER))

    # 线位置计算（加权平均）
    if line[1] < 700 and line[2] < 700 and line[3] < 700:
        # 线丢失：使用历史误差方向继续转
        if last_error < 0:
            pos = 0
        else:
            pos = 4000
    else:
        denom = sum(line)
        if denom == 0:
            denom = 1
        pos = (1000*line[1] + 2000*line[2] + 3000 *
               line[3] + 4000*line[4]) // denom

    error = pos - 2000  # 中心偏差（-2000 到 +2000）
    last_error = error

    # 基于偏差调整速度
    if abs(error) > 1200:
        cmd_v = TARGET_VELOCITY_CM * 0.7  # 大偏差减速
    else:
        cmd_v = TARGET_VELOCITY_CM

    # LQR 控制：位置误差 + 陀螺仪反馈
    ctrl_err = error if abs(error) > 50 else 0
    ctrl_gyro = current_gyro_rad if abs(current_gyro_rad) > 0.05 else 0

    target_omega = (LQR_K1 * ctrl_err) - (LQR_K2 * ctrl_gyro)
    target_omega = max(-6.0, min(6.0, target_omega))

    # 调用闭环控制驱动
    driver.car_move(cmd_v, target_omega)


def lane_keep_step():
    global current_gyro_rad

    line = line_sensors.read_calibrated()

    # 获取实时运动学反解
    current_v, current_omega = driver.get_kinematics()

    # 更新陀螺仪
    if imu.gyro.data_ready():
        imu.gyro.read()
        dps_z = imu.gyro.last_reading_dps[2] - gyro_offset
        rad_z = math.radians(dps_z)
        current_gyro_rad = (rad_z * GYRO_FILTER) + \
            (current_gyro_rad * (1 - GYRO_FILTER))

    v_cruise = 70.0
    w_turn = 5.0

    # 更简洁的车道保持逻辑
    if line[0] > 400:
        driver.car_move(v_cruise, -w_turn)  # 左偏
    elif line[4] > 400:
        driver.car_move(v_cruise, w_turn)   # 右偏
    else:
        driver.car_move(v_cruise, 0)        # 直行


def calibrate_line_sensors():
    motors.set_speeds(1000, -1000)
    for i in range(25):
        line_sensors.calibrate()
    motors.off()
    time.sleep_ms(200)

    motors.set_speeds(-1000, 1000)
    for i in range(50):
        line_sensors.calibrate()
    motors.off()
    time.sleep_ms(200)

    motors.set_speeds(1000, -1000)
    for i in range(25):
        line_sensors.calibrate()
    motors.off()


# ================================================
# 3. 主程序逻辑
# ================================================

display.fill(0)
display.text("Hands OFF!", 10, 20)
display.show()
time.sleep_ms(1000)

# 调试模式入口
if DEBUG:
    # 等待按键选择调试模式
    display.fill(0)
    display.text("DEBUG MODE", 20, 10)
    display.show()
    time.sleep_ms(1000)
    bump_sensors.read()
    time.sleep_ms(1000)

calibrate_gyro()

buzzer.play("c32 c32")
bump_sensors.calibrate()
calibrate_line_sensors()

led.off()
display.fill(0)
display.text("Ready", 20, 25, 1)
display.show()
buzzer.play(">L16 cdeg")
time.sleep_ms(1000)


while True:
    bump_sensors.read()

    if current_mode == MODE_STOP:
        driver.off()
        with music_flag_lock:
            play_music_flag = False
        if bump_sensors.right_is_pressed():
            current_mode = MODE_FOLLOW
            with music_flag_lock:
                play_music_flag = True
            time.sleep_ms(500)

    elif current_mode == MODE_FOLLOW:
        lqr_follow_step()

        if bump_sensors.left_is_pressed():
            driver.car_move(-40, 0)
            time.sleep_ms(150)
            current_mode = MODE_LANE_KEEP

    elif current_mode == MODE_LANE_KEEP:
        lane_keep_step()

    time.sleep_ms(2)
