from pololu_3pi_2040_robot import robot
import _thread
import time

# --- 1. 初始化硬件 ---
motors = robot.Motors()
display = robot.Display()
bump_sensors = robot.BumpSensors()
line_sensors = robot.LineSensors()
buzzer = robot.Buzzer()
led = robot.YellowLED()
imu = robot.IMU()
imu.reset()
imu.enable_default()

# --- 2. LQR 控制参数 (原 PID 参数对应转换) ---
# LQR 增益矩阵 K = [k1, k2]
# k1: 对应原 kp，惩罚位置误差 (Q矩阵的第一项)
# k2: 对应原 kd，惩罚速度误差 (Q矩阵的第二项)
lqr_k1 = 30    # Position Gain
lqr_k2 = 20  # Velocity Gain

max_speed = 4000       # 竞速最大速度
gyro_offset = 0
# --- 3. 状态定义 ---
MODE_STOP = 0
MODE_FOLLOW = 1
MODE_LANE_KEEP = 2

current_mode = MODE_STOP
play_music_flag = False

# Nyan Cat BGM
nyan_cat_melody = "T140 O5 \
    f#8 g#8 d#8 d#8 b8 d8 c#8 b4 \
    b8 c#8 d8 d8 c#8 b8 c#8 d#8 \
    f#8 g#8 d#8 f#8 c#8 d#8 b8 c#8 b8 \
    d#8 f#8 g#8 d#8 f#8 c#8 d#8 b8 d8 d#8 d8 c#8 b8 c#8 \
    d#8 b8 c#8 d#8 f#8 c#8 d#8 c#8 b8 c#8 b4 \
    f#8 g#8 d#8 d#8 b8 d8 c#8 b4 \
    b8 c#8 d8 d8 c#8 b8 c#8 d#8 \
    f#8 g#8 d#8 f#8 c#8 d#8 b8 c#8 b8 \
    d#8 f#8 g#8 d#8 f#8 c#8 d#8 b8 d8 d#8 d8 c#8 b8 c#8 \
    d#8 b8 c#8 d#8 f#8 c#8 d#8 c#8 b8 c#8 b4"

# 全局变量：上一时刻的误差，用于观测状态 x2 (error_dot)
last_error = 0

# --- 4. 核心功能函数 ---


def music_thread_func():
    global play_music_flag
    while True:
        if play_music_flag:
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
    print("Calibrating Gyro...")
    total = 0
    # 读取 100 次取平均值，作为静止时的基准零点
    for i in range(100):
        if imu.gyro.data_ready():
            imu.gyro.read()
            total += imu.gyro.last_reading_dps[2]
        time.sleep_ms(2)  # 稍微等一下
    gyro_offset = total / 100
    print("Gyro calibrated:", gyro_offset)


def lqr_follow_step():
    """
    LQR 循迹 (红外 + 陀螺仪融合版)
    """
    global last_error
    
    # --- 1. 读取红外传感器 ---
    line = line_sensors.read_calibrated()
    
    # --- 2. 计算 position_raw (之前缺失的部分) ---
    # 断线保护逻辑：如果所有传感器都读不到黑线 (<700)
    if line[1] < 700 and line[2] < 700 and line[3] < 700:
        # 根据上次的误差判断是偏左还是偏右丢的
        if last_error < 0:
            position_raw = 0     # 假装在线的最左边
        else:
            position_raw = 4000  # 假装在线的最右边
    else:
        # 正常计算加权平均位置
        denominator = sum(line)
        if denominator == 0: 
            denominator = 1
        position_raw = (1000*line[1] + 2000*line[2] + 3000*line[3] + 4000*line[4]) // denominator

    # --- 3. 计算位置误差 (P项) ---
    # 2000 是中心点
    error = position_raw - 2000
    
    # ------------------------------------------------------
    # --- 4. 读取陀螺仪 (D项 - 速度) ---
    # ------------------------------------------------------
    gyro_z = 0
    # 检查 IMU 数据是否准备好
    if imu.gyro.data_ready():
        imu.gyro.read()
        # 读取 Z 轴角速度，并减去零偏
        gyro_z = imu.gyro.last_reading_dps[2] - gyro_offset
    
    # 更新历史误差 (虽然有了陀螺仪，这个主要用于断线保护逻辑的判断)
    last_error = error 

    # --- 5. 计算控制量 u ---
    # 融合模式: P项用红外误差，D项用陀螺仪角速度
    # 注意：lqr_k2 的数值可能需要大幅减小 (比如尝试 20-50)，因为 gyro_z 数值很大
    # 如果发现车子抖动加剧，尝试把 + 改成 - 
    u = (lqr_k1 * error) + (lqr_k2 * gyro_z)

    # --- 6. 电机控制 ---
    # 开启反转 (min_speed = -max_speed) 以获得最强抓地力
    min_speed = -max_speed 
    
    left_target = int(max_speed + u)
    right_target = int(max_speed - u)
    
    left = max(min_speed, min(max_speed, left_target))
    right = max(min_speed, min(max_speed, right_target))

    motors.set_speeds(left, right)


def lane_keep_step():
    """ 车道保持逻辑 """
    line = line_sensors.read_calibrated()
    turn_speed = 2000  # 竞速保持高转速
    fwd_speed = 2000

    if line[0] > 500:
        motors.set_speeds(fwd_speed, -turn_speed)
    elif line[4] > 500:
        motors.set_speeds(-turn_speed, fwd_speed)
    else:
        motors.set_speeds(fwd_speed, fwd_speed)


# --- 5. 上电校准流程 ---
calibrate_gyro()
led.on()


buzzer.play("c32 c32")
bump_sensors.calibrate()
display.fill(0)
display.text("Calibrating ...", 10, 20, 1)

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

led.off()
display.fill(1)
display.text("Ready", 30, 35, 0)
display.show()
buzzer.play(">L16 cdeg")
time.sleep_ms(1000)


# --- 6. 主循环 ---

while True:
    bump_sensors.read()

    if current_mode == MODE_STOP:
        motors.off()
        play_music_flag = False[]
        if bump_sensors.right_is_pressed():
            current_mode = MODE_FOLLOW
            # play_music_flag = True

    elif current_mode == MODE_FOLLOW:
        # 调用 LQR 控制函数
        lqr_follow_step()

        if bump_sensors.left_is_pressed():
            buzzer.play("c32 c32")  # 听到这个声音说明传感器是灵敏的
            motors.set_speeds(2000, -2000)
            time.sleep_ms(100)
            current_mode = MODE_LANE_KEEP

    elif current_mode == MODE_LANE_KEEP:
        lane_keep_step()

    time.sleep_ms(2)
