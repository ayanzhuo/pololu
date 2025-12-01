import time
import math
from pololu_3pi_2040_robot import robot
from PID import PIDController 


# 实例化硬件对象
motors = robot.Motors()
imu = robot.IMU()
button_a = robot.ButtonA()
button_c = robot.ButtonC()
display = robot.Display()
yellow_led = robot.YellowLED()

# PD 控制参数 (用于角度闭环) - 基于 Pololu 示例的经验值
GYRO_KP = 140.0         # 比例增益 (P)
GYRO_KD = 4.0           # 微分增益 (D)
MAX_TURN_SPEED = 3000   # 最大转弯速度 (counts/s)
ANGLE_TOLERANCE = 3.0   # 停止转弯的角度容忍度 (度)
TURN_ANGLE = 90.0       # 目标转角 (度)

# --- IMU 状态变量 ---
robot_angle = 0.0              # 当前机器人角度 (度)
target_angle = 0.0             # 目标角度 (度)
last_time_gyro_reading = None  # 上次读取 IMU 的时间戳 (us)
turn_rate = 0.0                # 当前转弯角速度 (度/秒)
is_turning = False             # 标志是否正在执行陀螺仪转弯

# ==========================================
# --- 函数定义 ---
# ==========================================

def update_angle():
    """
    从陀螺仪读取数据，并积分计算机器人的当前角度 (Yaw)。
    该函数必须高频调用。
    """
    global robot_angle, last_time_gyro_reading, turn_rate
    
    if imu.gyro.data_ready():
        imu.gyro.read()
        # Z轴角速度 (度/秒)
        turn_rate = imu.gyro.last_reading_dps[2] 
        
        now = time.ticks_us()
        
        if last_time_gyro_reading:
            dt = time.ticks_diff(now, last_time_gyro_reading)
            # 角度积分: robot_angle += rate * dt / 1,000,000
            robot_angle += turn_rate * dt / 1000000
            
        last_time_gyro_reading = now

def start_gyro_turn(angle_to_add):
    """设置目标角度并启动陀螺仪转弯模式。"""
    global target_angle, robot_angle, last_time_gyro_reading, is_turning

    # 1. 重置角度追踪：从当前朝向开始计算
    robot_angle = 0.0 
    last_time_gyro_reading = None # 重置时间戳
    
    # 2. 设置目标角度
    target_angle = angle_to_add 
    is_turning = True
    
    # 3. 视觉反馈
    display.fill(1)
    display.text("Turning...", 30, 28, 0)
    display.show()
    time.sleep_ms(200) # 视觉停留

def gyro_turn_step():
    """执行一个周期的 PD 转向控制。返回 True 表示转弯仍在进行中。"""
    global is_turning, robot_angle, target_angle, turn_rate
    
    if not is_turning:
        return False

    angle_error = target_angle - robot_angle
    
    # 检查是否接近目标角度 (停止条件)
    if abs(angle_error) < ANGLE_TOLERANCE:
        is_turning = False
        motors.off()
        return False # 转弯完成

    # PD 控制计算转速
    # Turn Speed = P项 (角度误差) - D项 (角速度)
    turn_speed = angle_error * GYRO_KP - turn_rate * GYRO_KD
    
    # 钳位速度
    if turn_speed > MAX_TURN_SPEED:
        turn_speed = MAX_TURN_SPEED
    elif turn_speed < -MAX_TURN_SPEED:
        turn_speed = -MAX_TURN_SPEED
        
    # 应用电机速度 (左轮反向，实现原地转弯)
    # motors.set_speeds 接收 counts/s 或 PWM 值
    motors.set_speeds(int(-turn_speed), int(turn_speed))
    
    return True # 转弯进行中

def draw_status():
    """显示当前角度和状态。"""
    display.fill(0)
    
    if is_turning:
        display.text("TARGET:", 0, 0, 1)
        display.text(f"{target_angle:>.1f} deg", 0, 8, 1)
        display.text("ERROR:", 0, 20, 1)
        display.text(f"{target_angle - robot_angle:>.1f}", 0, 28, 1)
    else:
        display.text(f"A: Turn {TURN_ANGLE} deg (R)", 0, 0, 1)
        display.text(f"C: Turn {-TURN_ANGLE} deg (L)", 0, 8, 1)

    display.text("Angle:", 0, 40, 1)
    display.text(f"{robot_angle:>.1f}", 48, 40, 1)
    display.text(f"State: {'TURNING' if is_turning else 'READY'}", 0, 56, 1)
    display.show()

# ==========================================
# --- 主循环 ---
# ==========================================

# 初始化 IMU
display.fill(0)
display.text("IMU Init...", 0, 0, 1)
display.show()
imu.reset()
imu.enable_default()
time.sleep_ms(500)
last_time_gyro_reading = time.ticks_us() # 启动 IMU 读取计时器

try:
    while True:
        # 1. 核心：高频更新角度
        update_angle()
        
        # 2. 陀螺仪转弯步进
        gyro_turn_step()
        
        # 3. 按钮控制 (仅在不转弯时响应)
        if not is_turning:
            if button_a.check():
                start_gyro_turn(TURN_ANGLE) # 右转 90度
                while button_a.check(): pass # 等待按键释放
            
            if button_c.check():
                start_gyro_turn(-TURN_ANGLE) # 左转 90度
                while button_c.check(): pass # 等待按键释放
                
        # 4. 显示状态
        draw_status()

        # 短暂等待，保证循环稳定
        time.sleep_ms(10) 

except KeyboardInterrupt:
    motors.off()
    display.fill(0)
    display.text("STOP", 0, 0, 1)
    display.show()