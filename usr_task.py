from pololu_3pi_2040_robot import robot
import time

# --- 1. 初始化硬件 ---
motors = robot.Motors()
bump_sensors = robot.BumpSensors()
line_sensors = robot.LineSensors()
buzzer = robot.Buzzer()
led = robot.YellowLED()

# --- 2. 参数设置 ---
max_speed = 2000       # 循迹最大速度
kp = 90                # PID 参数 P
kd = 2000              # PID 参数 D

# --- 3. 状态定义 ---
MODE_STOP = 0          # 停止等待模式
MODE_FOLLOW = 1        # 循迹模式
MODE_LANE_KEEP = 2     # 车道保持模式

# 初始状态设为停止
current_mode = MODE_STOP

# 全局变量
last_p = 0    

# --- 4. 核心功能函数 ---

def line_follow_step():
    """ PID 循迹单步逻辑 """
    global last_p
    line = line_sensors.read_calibrated()
    
    # 断线保护：如果全白(<700)，根据上次位置极速转弯找线
    if line[1] < 700 and line[2] < 700 and line[3] < 700:
        if last_p < 0:
            l = 0     
        else:
            l = 4000  
    else:
        denominator = sum(line)
        if denominator == 0: denominator = 1
        l = (1000*line[1] + 2000*line[2] + 3000*line[3] + 4000*line[4]) // denominator

    p = l - 2000       
    d = p - last_p     
    last_p = p         
    
    pid = p * kp + d * kd

    min_speed = 0
    left = max(min_speed, min(max_speed, int(max_speed + pid)))
    right = max(min_speed, min(max_speed, int(max_speed - pid)))

    motors.set_speeds(left, right)

def lane_keep_step():
    """ 车道保持逻辑：撞线反弹 """
    line = line_sensors.read_calibrated()
    turn_speed = 1500
    fwd_speed = 2000
    
    # 简单的逻辑：左边看到线往右转，右边看到线往左转
    if line[0] > 500: # 左压线
        motors.set_speeds(fwd_speed, -turn_speed)
    elif line[4] > 500: # 右压线
        motors.set_speeds(-turn_speed, fwd_speed)
    else:
        motors.set_speeds(fwd_speed, fwd_speed)

# --- 5. 上电校准流程 ---

# 提示：LED常亮，开始校准碰撞传感器（此时不要触碰前方）
led.on()
bump_sensors.calibrate()

# 提示：两声短促滴滴，开始校准巡线（请左右晃动小车）
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

# 校准完成：LED熄灭，播放一段音乐，进入待机
led.off()
buzzer.play(">L16 cdeg") 
time.sleep_ms(1000)


# --- 6. 主循环 (状态机) ---

while True:
    # 必须不断读取传感器
    bump_sensors.read()
    
    # ================= 状态 1: 停止等待 =================
    if current_mode == MODE_STOP:
        motors.off()
        
        # 任务 1：等待【右侧】碰撞，进入循迹
        if bump_sensors.right_is_pressed():    
            current_mode = MODE_FOLLOW # 切换状态

    # ================= 状态 2: PID 循迹 =================
    elif current_mode == MODE_FOLLOW:
        line_follow_step()
        current_mode = MODE_LANE_KEEP # 切换状态

    # ================= 状态 3: Lane Keeping =================
    elif current_mode == MODE_LANE_KEEP:
        lane_keep_step()
        
        # 可选：如果此时再按右侧，是否要停下来？
        # 如果需要“按右侧停止”，可以取消下面几行的注释
        # if bump_sensors.right_is_pressed():
        #     buzzer.play("g32")
        #     led.off()
        #     current_mode = MODE_STOP
        #     time.sleep_ms(500)

    # 稍微延时，保持节奏
    time.sleep_ms(2)