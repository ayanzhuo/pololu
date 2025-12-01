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

music_index = 0
note_start_time = 0
music_is_playing = False
song_finished_time = 0


#======================阴间音乐==========================
MUSIC_NOTES = [
    (330, 350), # E4 
    (330, 350), # E4 
    (330, 700), # E4 (长)
    (0, 100),   # 休息
    
    (330, 350), # E4 
    (330, 350), # E4 
    (330, 700), # E4 (长)
    (0, 100),   # 休息
    
    (392, 350), # G4 (高)
    (262, 350), # C4 (低)
    (294, 350), # D4 (中)
    (330, 1000),# E4 (结束，非常长)
    (0, 500)    # 循环间休息
]

# --- 实例化控制系统 ---
robot_drive = RobotDrive()
line_follower = LineFollower(robot_drive)
display_manager = RobotDisplay(robot_drive)

_thread.start_new_thread(display_manager.run, ()) 
# ==========================================


def music_step():

    global music_index, note_start_time, music_is_playing, song_finished_time
    global current_mode_ref, mode_lock

    with mode_lock:
      current_mode = current_mode_ref[0]

    if current_mode == MODE_FOLLOW:
        if not music_is_playing and time.ticks_diff(time.ticks_ms(), song_finished_time) > 500:
            music_is_playing = True
            music_index = 0
            note_start_time = time.ticks_ms()

    else:
        buzzer.off()
        music_is_playing = False
        return

    if music_is_playing:
        now = time.ticks_ms()
        if music_index < len(MUSIC_NOTES):
            freq, duration = MUSIC_NOTES[music_index]

            if time.ticks_diff(now, note_start_time) < duration:
                if freq > 0:
                    if buzzer.pwm.freq() != freq:
                        buzzer.pwm.freq(freq)
                        buzzer.on()

                elif freq == 0:
                    buzzer.off()
            else:
                music_index += 1
                note_start_time = now

        else:
            buzzer.off()
            music_is_playing = False
            song_finished_time = now


time.sleep_ms(200)

def wait_for_collision_release():
    while bump_sensors.left_is_pressed or bump_sensors.right_is_pressed:
        bump_sensors.read()
        time.sleep_ms(10)
    time.sleep_ms(20) # 额外的去抖动

# ===================== 主控制循环 =====================#

try:
    while True:
        line_follower.update_angle()
        robot_drive.update()

        # 陀螺仪转弯步进
        if line_follower.gyro_turn_step():
            time.sleep_ms(2)
            continue

        bump_sensors.read()
        collision_detected = bump_sensors.left_is_pressed or bump_sensors.right_is_pressed

        with mode_lock:
            current_mode = current_mode_ref[0]

        if current_mode == MODE_WAIT_CALIB:
            robot_drive.set_speed(v=0, w=0)
            display_manager.set_custom_message("Press A to CALIB")
            if collision_detected:
                with mode_lock:
                    current_mode_ref[0] = MODE_CALIBRATING
                wait_for_collision_release()

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
                wait_for_collision_release()

        elif current_mode == MODE_LANE_KEEP:
            display_manager.set_custom_message("Lane Keep")
            line_follower.lane_keep()

            if collision_detected:
                with mode_lock:
                    current_mode_ref[0] = MODE_STOP
                wait_for_collision_release()

        elif current_mode == MODE_STOP:
            display_manager.set_custom_message("Ready to Go!")
            robot_drive.set_speed(v=0, w=0)
            if collision_detected:
                with mode_lock:
                    current_mode_ref[0] = MODE_FOLLOW
                wait_for_collision_release()

        # === 辅助逻辑 (低频) ===
        music_step()     
        time.sleep_ms(2)  # 保持主循环调度间隔

except KeyboardInterrupt:
    robot_drive.motors_off()
