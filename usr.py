import time
import _thread
import math
from pololu_3pi_2040_robot import robot


from bsp import RobotDrive, RobotDisplay, MusicPlayer
from application import LineFollower

# ==========================================
# --- 硬件和常量初始化 ---
# ==========================================
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
current_mode_ref = [MODE_WAIT_CALIB]
# ======================参数调优=====================#
KP_TUNE = 5.5
KI_TUNE = 0.05
KD_TUNE = 0.05
# ===================== 实例化控制系统 =====================#
robot_drive = RobotDrive(Kp=KP_TUNE, Ki=KI_TUNE, Kd=KD_TUNE)
line_follower = LineFollower(robot_drive)
display_manager = RobotDisplay(robot_drive)
mode_lock = _thread.allocate_lock()
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

        if current_mode == MODE_WAIT_CALIB:
            robot_drive.set_speed(v=0, w=0)
            display_manager.set_custom_message("Press A to CALIB")
            if collision_detected:
                with mode_lock:
                    current_mode_ref[0] = MODE_CALIBRATING
                line_follower.wait_for_collision_release()

        elif current_mode == MODE_CALIBRATING:
            display_manager.set_custom_message("CALIBRATING...")
            line_follower.calibrate_motorsweep()
            with mode_lock:
                current_mode_ref[0] = MODE_STOP

        elif current_mode == MODE_FOLLOW:
            display_manager.set_custom_message("Following Line")
            line_follower.follow_line()

            if collision_detected:
                with mode_lock:
                    current_mode_ref[0] = MODE_LANE_KEEP
                line_follower.wait_for_collision_release()

        elif current_mode == MODE_LANE_KEEP:
            display_manager.set_custom_message("Lane Keep")
            line_follower.lane_keep()

            if collision_detected:
                with mode_lock:
                    current_mode_ref[0] = MODE_STOP
                line_follower.wait_for_collision_release()

        elif current_mode == MODE_STOP:
            display_manager.set_custom_message("Ready to Go!")
            robot_drive.set_speed(v=0, w=0)
            if collision_detected:
                with mode_lock:
                    current_mode_ref[0] = MODE_FOLLOW
                line_follower.wait_for_collision_release()

        music_player.music_step()
        time.sleep_ms(2)

except KeyboardInterrupt:
    robot_drive.motors_off()
