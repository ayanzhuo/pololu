import time
import _thread
import math

from pololu_3pi_2040_robot import robot
from mymotor import RobotDrive, RobotDisplay 

# --- 硬件和系统初始化 ---

print("Initializing robot systems...")
led = robot.YellowLED()
buzzer = robot.Buzzer()

# --- 1. 实例化高层控制器 ---
# 使用默认 PID 参数 Kp=5.5, Ki=0.05, Kd=0.05
robot_drive = RobotDrive()

# --- 2. 启动显示线程 ---
display_manager = RobotDisplay(robot_drive)
_thread.start_new_thread(display_manager.run, ())



DURATION_MS = 20000 # 运行时间 2 秒
# vmax = 800; wmax = 30# 最大线速度和角速度
# ===================== 主程序：测试运动序列 =====================#

try:
    
    robot_drive.set_speed(v=0.0, w=50.0) 
    led.on()

    start_time = time.ticks_ms()
    end_time = start_time + DURATION_MS

    # 主控制循环：前进 20 秒
    while time.ticks_ms() < end_time:

        robot_drive.update()
        time.sleep_ms(1)


    robot_drive.motors_off() 
    led.off()


except KeyboardInterrupt:
    print("\n程序中断。停止电机。")
    robot_drive.motors_off()
    led.off()