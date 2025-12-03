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
buzzer = robot.Buzzer()


rgb_leds = robot.RGBLEDs()
if not DEBUG:
    rgb_leds.set_brightness(8) 
    for i in range(6):

        rgb_leds.set(i, [0, 129, 226])
    rgb_leds.show()

# --- 状态机 ---
MODE_WAIT_CALIB = 0
MODE_CALIBRATING = 1
MODE_FOLLOW = 2
MODE_STOP = 3
MODE_LANE_KEEP = 4
MODE_RECOVERY_TURN = 5
mode_lock = _thread.allocate_lock()
current_mode_ref = [MODE_WAIT_CALIB]


# --- 实例化控制系统 ---
robot_drive = RobotDrive()
line_follower = LineFollower(robot_drive)
display_manager = RobotDisplay(robot_drive)
# ===================== 传感器校准 =====================#

music_player = MusicPlayer(buzzer, mode_lock, current_mode_ref)
_thread.start_new_thread(display_manager.run, ())
time.sleep_ms(200)

# ===================== 主控制循环 =====================#

try:
    while True:
        robot_drive.update()
        line_follower.bump_sensors.read()

        collision_detected = line_follower.bump_sensors.left_is_pressed(
        ) or line_follower.bump_sensors.right_is_pressed()

        with mode_lock:
            current_mode = current_mode_ref[0]
#======================等待校准模式=====================
        if current_mode == MODE_WAIT_CALIB:
            robot_drive.set_speed(v=0, w=0)
            display_manager.set_custom_message("Press A to CALIB")
            if button_a.check():
                with mode_lock:
                    current_mode_ref[0] = MODE_CALIBRATING
                time.sleep_ms(200)
            if not DEBUG:
                if collision_detected:
                    with mode_lock:
                        current_mode_ref[0] = MODE_CALIBRATING
                    line_follower.wait_for_collision_release()
#========================校准模式============================
        elif current_mode == MODE_CALIBRATING:
            display_manager.set_custom_message("CALIBRATING...")
            line_follower.calibrate_motorsweep()
            with mode_lock:
                current_mode_ref[0] = MODE_STOP
#=========================循迹模式=============================
        elif current_mode == MODE_FOLLOW:
            display_manager.set_custom_message("Following Line")
            line_follower.follow_line()

            if button_a.check():
                with mode_lock:
                    current_mode_ref[0] = MODE_RECOVERY_TURN
                    time.sleep_ms(200)
            # if not DEBUG:
            if collision_detected:
                with mode_lock:
                    current_mode_ref[0] = MODE_RECOVERY_TURN
                line_follower.wait_for_collision_release()
#==========================保持模式================================          
        elif current_mode == MODE_LANE_KEEP:
            display_manager.set_custom_message("Lane Keep")
            line_follower.lane_keep()
            if button_a.check():
                with mode_lock:
                    current_mode_ref[0] = MODE_STOP
                    time.sleep_ms(200)
            if not DEBUG:
                if collision_detected:
                    with mode_lock:
                        current_mode_ref[0] = MODE_STOP
                    line_follower.wait_for_collision_release()
#==========================过渡模式==============================
        elif current_mode == MODE_RECOVERY_TURN:
            display_manager.set_custom_message("RECOVERY TURN >")
            robot_drive.set_speed(v=400, w=20) 
            
            # 2. 执行阻塞式转弯计时
            start_time = time.ticks_ms()
            while time.ticks_diff(time.ticks_ms(), start_time) < 250:
                # 必须持续更新底层的 PID 速度控制
                robot_drive.update() 
                time.sleep_ms(10) # 维持高频控制

            with mode_lock:
                current_mode_ref[0] = MODE_LANE_KEEP # 转弯完成，进入 LANE_KEEP
#=============================停车模式=============================
        elif current_mode == MODE_STOP:
            display_manager.set_custom_message("Ready to Go!")
            robot_drive.set_speed(v=0, w=0)
            if button_a.check():
                with mode_lock:
                    current_mode_ref[0] = MODE_FOLLOW
                time.sleep_ms(200)
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
