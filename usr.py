from pololu_3pi_2040_robot import robot
from mymotor import RobotDrive, MotorController, RobotDisplay
import _thread
import time
import math

# ==========================================
bump_sensors = robot.BumpSensors()
line_sensors = robot.LineSensors()
buzzer = robot.Buzzer()
led = robot.YellowLED()
imu = robot.IMU()
imu.reset()
imu.enable_default()

robot_drive = RobotDrive()
display_manager = RobotDisplay(robot_drive)
_thread.start_new_thread(display_manager.run, ())

# 状态机
MODE_STOP = 0
MODE_FOLLOW = 1
MODE_LANE_KEEP = 2
current_mode = MODE_STOP
play_music_flag = False
music_flag_lock = _thread.allocate_lock()


# ===================== 主控制循环 =====================#
try:
    while True:
        robot_drive.update()

        bump_sensors.read()
        if current_mode == MODE_STOP:
            robot_drive.motors_off()

            with music_flag_lock:
                play_music_flag = False
            if bump_sensors.right_is_pressed():
                current_mode = MODE_FOLLOW
                with music_flag_lock:
                    play_music_flag = True
                time.sleep_ms(500)

        elif current_mode == MODE_FOLLOW:
            # follow_line_step()

            if bump_sensors.left_is_pressed():
                # mymotor.car_move(-40, 0)
                # time.sleep_ms(150)
                current_mode = MODE_LANE_KEEP

        elif current_mode == MODE_LANE_KEEP:
            # lane_keep_step()
            pass

        time.sleep_ms(2)

except KeyboardInterrupt:
    robot_drive.motors_off()