from pololu_3pi_2040_robot import robot
import _thread
import time

# --- 1. 初始化硬件 ---
motors = robot.Motors()
bump_sensors = robot.BumpSensors()
line_sensors = robot.LineSensors()
buzzer = robot.Buzzer()
led = robot.YellowLED()

# --- 2. LQR 控制参数 (原 PID 参数对应转换) ---
# LQR 增益矩阵 K = [k1, k2]
# k1: 对应原 kp，惩罚位置误差 (Q矩阵的第一项)
# k2: 对应原 kd，惩罚速度误差 (Q矩阵的第二项)
lqr_k1 = 90    # Position Gain
lqr_k2 = 2000  # Velocity Gain

max_speed = 2000       # 竞速最大速度

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

def lqr_follow_step():
    """ 
    使用状态反馈控制律 (State Feedback Control Law) u = -Kx 
    State x = [error, error_dot]^T
    """
    global last_error
    
    # 1. 获取传感器原始数据
    line = line_sensors.read_calibrated()
    
    # 2. 状态观测 (State Observation)
    # 计算 x1: Error (位置误差)
    # 断线保护逻辑
    if line[1] < 700 and line[2] < 700 and line[3] < 700:
        if last_error < 0:
            position_raw = 0     
        else:
            position_raw = 4000  
    else:
        denominator = sum(line)
        if denominator == 0: denominator = 1
        position_raw = (1000*line[1] + 2000*line[2] + 3000*line[3] + 4000*line[4]) // denominator

    # 构建状态向量 x
    # x1: 位置误差 (Centering Error)
    error = position_raw - 2000
    
    # x2: 误差导数 (Derivative of Error) - 也就是横向速度
    error_dot = error - last_error
    
    # 更新观测器历史
    last_error = error
    
    # 3. 计算最优控制量 u (Control Input)
    # LQR Control Law: u = -K * x
    # 注意：这里的正负号取决于电机定义的旋转方向
    # 我们定义：u > 0 时，车辆应向右修正（左轮加速，右轮减速）
    
    # u = K1 * error + K2 * error_dot
    u = (lqr_k1 * error) + (lqr_k2 * error_dot)

    # 4. 执行器混合 (Actuator Mixing)
    # 将控制量 u 映射到左右电机
    min_speed = 0
    
    # 左轮 = 基准速度 + 控制量
    left_target = int(max_speed + u)
    # 右轮 = 基准速度 - 控制量
    right_target = int(max_speed - u)
    
    # 饱和限制 (Saturation) - 防止数值溢出
    left = max(min_speed, min(max_speed, left_target))
    right = max(min_speed, min(max_speed, right_target))

    motors.set_speeds(left, right)

def lane_keep_step():
    """ 车道保持逻辑 """
    line = line_sensors.read_calibrated()
    turn_speed = 2000 # 竞速保持高转速
    fwd_speed = 2000
    
    if line[0] > 500: 
        motors.set_speeds(fwd_speed, -turn_speed)
    elif line[4] > 500: 
        motors.set_speeds(-turn_speed, fwd_speed)
    else:
        motors.set_speeds(fwd_speed, fwd_speed)

# --- 5. 上电校准流程 ---
led.on()
bump_sensors.calibrate()

buzzer.play("c32 c32")
motors.set_speeds(1000, -1000)
for i in range(25): line_sensors.calibrate()
motors.off()
time.sleep_ms(200)

motors.set_speeds(-1000, 1000)
for i in range(50): line_sensors.calibrate()
motors.off()
time.sleep_ms(200)

motors.set_speeds(1000, -1000)
for i in range(25): line_sensors.calibrate()
motors.off()

led.off()
display.fill(1)
display.text("LQR Controller", 10, 20, 0)
display.text("Ready", 40, 40, 0)
display.show()
buzzer.play(">L16 cdeg") 
time.sleep_ms(1000)


# --- 6. 主循环 ---

while True:
    bump_sensors.read()
    
    if current_mode == MODE_STOP:
        motors.off()
        play_music_flag = False
        if bump_sensors.right_is_pressed(): 
            buzzer.play("a32") 
            led.on()   
            current_mode = MODE_FOLLOW 
            play_music_flag = True

    elif current_mode == MODE_FOLLOW:
            # 调用 LQR 控制函数
            lqr_follow_step()
            
            if bump_sensors.left_is_pressed():
                buzzer.play("c32 c32")
                current_mode = MODE_LANE_KEEP 

    elif current_mode == MODE_LANE_KEEP:
        lane_keep_step()
        
    time.sleep_ms(2)