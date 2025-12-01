import time
import _thread
import math
from pololu_3pi_2040_robot import robot


from bsp import RobotDrive, RobotDisplay
from application import LineFollower

# ==========================================
# --- 硬件和常量初始化 ---
# ==========================================
bump_sensors = robot.BumpSensors()
button_a = robot.ButtonA()
button_b = robot.ButtonB()
buzzer = robot.Buzzer()
led = robot.YellowLED()


# --- 状态机 ---
MODE_WAIT_CALIB = 0
MODE_CALIBRATING = 1
MODE_FOLLOW = 2
MODE_STOP = 3
MODE_LANE_KEEP = 4


mode_lock = _thread.allocate_lock()
current_mode_ref = [MODE_WAIT_CALIB]

# 音乐数据 (频率Hz, 持续时间ms)
MUSIC_NOTES = [(523, 100), (659, 100), (784, 150), (0, 50)]

# --- 实例化控制系统 ---
robot_drive = RobotDrive()
line_follower = LineFollower(robot_drive)
display_manager = RobotDisplay(robot_drive)

# ==========================================


# def music_thread_function():

#     while True:
#         play_music = False

#         with mode_lock:
#             if current_mode_ref[0] == MODE_FOLLOW:
#                 play_music = True

#         # if play_music:
#         #     for freq, duration in MUSIC_NOTES:
#         #         if freq > 0:
#         #             buzzer.frequency(freq)
#         #             buzzer.on()
#         #         else:
#         #             buzzer.off()

#                 # time.sleep_ms(duration)

#             buzzer.off()
#             time.sleep_ms(500)
#         else:
#             buzzer.off()
#             time.sleep_ms(100)

# ==========================================
bump_sensors.calibrate()

_thread.start_new_thread(display_manager.run, ())
# _thread.start_new_thread(music_thread_function, ())
time.sleep_ms(200)

# ===================== 主控制循环 =====================#

try:
    while True:
        robot_drive.update()
        bump_sensors.read()
        t = time.ticks_ms()

        with mode_lock:
            current_mode = current_mode_ref[0]

        if current_mode == MODE_WAIT_CALIB:
            robot_drive.set_speed(v=0, w=0)
            display_manager.set_custom_message("Press A to CALIB")
            if button_a.check():
                with mode_lock:
                    current_mode_ref[0] = MODE_CALIBRATING
                time.sleep_ms(200)

        elif current_mode == MODE_CALIBRATING:
            display_manager.set_custom_message("CALIBRATING...")
            line_follower.calibrate_motorsweep()
            with mode_lock:
                current_mode_ref[0] = MODE_STOP

        elif current_mode == MODE_FOLLOW:
            display_manager.set_custom_message("Following Line")
            line_follower.follow_line()

            if button_a.check():
                with mode_lock:
                    current_mode_ref[0] = MODE_LANE_KEEP
                    time.sleep_ms(200)

        elif current_mode == MODE_LANE_KEEP:
            display_manager.set_custom_message("Lane Keep")
            line_follower.lane_keep()
            if button_a.check():
                with mode_lock:
                    current_mode_ref[0] = MODE_STOP
                    time.sleep_ms(200)

        elif current_mode == MODE_STOP:
            display_manager.set_custom_message("Ready to Go!")
            robot_drive.set_speed(v=0, w=0)
            if button_a.check():
                with mode_lock:
                    current_mode_ref[0] = MODE_FOLLOW
                time.sleep_ms(200)

        time.sleep_ms(2)

except KeyboardInterrupt:
    robot_drive.motors_off()
