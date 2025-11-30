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
robot_drive = RobotDrive()

# --- 2. 启动显示线程 ---
display_manager = RobotDisplay(robot_drive)
_thread.start_new_thread(display_manager.run, ())


# ===================== 主程序：测试运动序列 =====================#

try:
    
    robot_drive.set_speed(v=1000.0, w=0.0) 
    robot_drive.update()



except KeyboardInterrupt:
    robot_drive.motors_off()
