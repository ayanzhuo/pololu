# micropython-pololu-3pi-2040-robot-v1.20
## 基础系统包层 (bsp.py)
- MotorController (内环 $\text{PID}$): 实现了基于编码器的速度闭环控制。它接收目标速度 ($\text{counts/s}$)，并使用 $\text{PID}$ 算法和 $\text{EMA}$ 滤波来计算实际的 $\text{PWM}$ 输出。
- RobotDrive: 实现差分驱动运动学，将高层级的线速度 ($v$) 和角速度 ($\omega$) 转换为左右轮的 $\text{MotorController}$ 目标速度。
- MusicPlayer: 封装了非阻塞的音乐播放逻辑，将对蜂鸣器的控制与主循环解耦。

## 应用逻辑层 (application.py)
- LineFollower (外环控制): 负责所有的状态逻辑、传感器数据处理和 $\text{IMU}$ 数据的管理。

## 主执行层 (usr.py)
- 状态机MODE\_WAIT\_CALIB,MODE\_FOLLOW等): 控制机器人的整体行为流程。
- 实时调度： 确保高优先级的任务（如 $\text{IMU}$ 积分和 $\text{PID}$ 更新）在每 $2\text{ms}$ 的循环中优先执行，并通过 if line_follower.gyro_turn_step(): continue 逻辑，在执行精确转弯时暂停状态机。
