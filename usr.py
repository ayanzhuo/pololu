import time
import _thread
import math
from pololu_3pi_2040_robot import robot
from bsp import RobotDrive, RobotDisplay, MusicPlayer
from application import LineFollower

DEBUG = True
# ==========================================
# --- 硬件和常量初始化 ---
# ==========================================
button_a = robot.ButtonA()
button_b = robot.ButtonB()
button_c = robot.ButtonC()
buzzer = robot.Buzzer()
led = robot.YellowLED()
rgb_leds = robot.RGBLEDs()

rgb_leds.set_brightness(4) 
for i in range(6):
    rgb_leds.set(i, [0, 129, 226])
if not DEBUG:   
    rgb_leds.show()

# ===================状态机 =====================
MODE_WAIT_CALIB = 0
MODE_CALIBRATING = 1
MODE_FOLLOW = 2
MODE_STOP = 3
MODE_LANE_KEEP = 4
mode_lock = _thread.allocate_lock()
current_mode_ref = [MODE_WAIT_CALIB]

# ======================参数调优=====================#
# 1. 驱动控制参数 
DRIVE_KP = 5.5
DRIVE_KI = 0.05
DRIVE_KD = 0.05

# 2. 循线控制参数 
LINE_BASE_SPEED = 600.0   # 基础前进速度 (mm/s)
LINE_MAX_OMEGA = 20.0     # 最大转向角速度 (rad/s)
STEER_KP_LOW = 0.006      # 循线 KP 低增益
STEER_KP_HIGH = 0.03      # 循线 KP 高增益

# 3. IMU 陀螺仪转弯参数 
GYRO_TURN_KP = 140.0      # 陀螺仪转弯 P 增益
GYRO_TURN_KD = 4.0        # 陀螺仪转弯 D 增益
GYRO_MAX_SPEED = 3000     # 陀螺仪转弯最大电机速度
TURN_ANGLE_DEG = -90.0    # 特殊转弯角度 (度)

# ===================== 实例化控制系统 =====================#

robot_drive = RobotDrive(Kp=DRIVE_KP, Ki=DRIVE_KI, Kd=DRIVE_KD)
line_follower = LineFollower(
    robot_drive, 
    LINE_BASE_SPEED, 
    LINE_MAX_OMEGA, 
    STEER_KP_LOW, 
    STEER_KP_HIGH, 
    GYRO_TURN_KP, 
    GYRO_TURN_KD, 
    GYRO_MAX_SPEED, 
    TURN_ANGLE_DEG
)
display_manager = RobotDisplay(robot_drive)
music_player = MusicPlayer(buzzer, mode_lock, current_mode_ref)
_thread.start_new_thread(display_manager.run, ())
time.sleep_ms(200)

# ===================== 主控制循环 =====================#
try:

    while True:
        line_follower.update_angle()
        robot_drive.update()

        line_follower.bump_sensors.read()
        collision_detected = line_follower.bump_sensors.left_is_pressed(
        ) or line_follower.bump_sensors.right_is_pressed()

        if line_follower.gyro_turn_step():
            time.sleep_ms(2)
            continue

        with mode_lock:
            current_mode = current_mode_ref[0]

        #============等待校准开机模式============
        if current_mode == MODE_WAIT_CALIB:
            robot_drive.set_speed(v=0, w=0)
            display_manager.set_custom_message("Press A to CALIB")
            if button_a.is_pressed():
                with mode_lock:
                    current_mode_ref[0] = MODE_CALIBRATING
                    
            if not DEBUG:
                if collision_detected:
                    with mode_lock:
                        current_mode_ref[0] = MODE_CALIBRATING
                    line_follower.wait_for_collision_release()
        #============校准模式============
        elif current_mode == MODE_CALIBRATING:
            display_manager.set_custom_message("CALIBRATING...")
            line_follower.calibrate_motorsweep()
            with mode_lock:
                current_mode_ref[0] = MODE_STOP
        #============循线模式============
        elif current_mode == MODE_FOLLOW:
            display_manager.set_custom_message("Following Line")
            line_follower.follow_line()
            if button_a.is_pressed():
                with mode_lock:
                    current_mode_ref[0] = MODE_LANE_KEEP
            if not DEBUG:
                if collision_detected:
                    with mode_lock:
                        current_mode_ref[0] = MODE_LANE_KEEP
                    line_follower.wait_for_collision_release()
        #============车道保持模式============
        elif current_mode == MODE_LANE_KEEP:
            display_manager.set_custom_message("Lane Keep")
            line_follower.lane_keep()
            if button_a.is_pressed():
                with mode_lock:
                    current_mode_ref[0] = MODE_STOP
            if not DEBUG:
                if collision_detected:
                    with mode_lock:
                        current_mode_ref[0] = MODE_STOP
                    line_follower.wait_for_collision_release()
        #============停止模式============
        elif current_mode == MODE_STOP:
            display_manager.set_custom_message("Ready to Go!")
            robot_drive.set_speed(v=0, w=0)

            if button_a.is_pressed():
                with mode_lock:
                    current_mode_ref[0] = MODE_FOLLOW

            if not DEBUG:
                if collision_detected:
                    with mode_lock:
                        current_mode_ref[0] = MODE_FOLLOW
                    line_follower.wait_for_collision_release()
        if not DEBUG:
            music_player.music_step()
        time.sleep_ms(2)

except KeyboardInterrupt:
    robot_drive.motors_off()
