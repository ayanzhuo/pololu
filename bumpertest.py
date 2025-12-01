from pololu_3pi_2040_robot import robot
import time
import micropython # 用于 Viper 警告处理

# ==========================================
# --- 硬件初始化 ---
# ==========================================
display = robot.Display()
bump_sensors = robot.BumpSensors()
yellow_led = robot.YellowLED()


# --- 初始设置 ---
display.fill(0)
display.text("BUMP SENSOR DEBUG", 0, 0, 1)
display.show()
time.sleep_ms(500)

# ==========================================
# --- 主数据显示循环 ---
# ==========================================

try:
    while True:
        # 1. 读取原始数据 (此调用也会更新内部的 pressed/not pressed 状态)
        # 返回的是 [左传感器原始值, 右传感器原始值]
        raw_data = bump_sensors.read()
        
        left_value = raw_data[0]
        right_value = raw_data[1]
        
        # 2. 获取布尔状态 (通过方法调用)
        left_pressed = bump_sensors.left_is_pressed()
        right_pressed = bump_sensors.right_is_pressed()

        # 清屏
        display.fill(0) 

        # 显示原始传感器数值 (Raw Values)
        display.text("L Raw:", 0, 0, 1)
        display.text(f"{left_value}", 45, 0, 1)
        
        display.text("R Raw:", 0, 10, 1)
        display.text(f"{right_value}", 45, 10, 1)
        
        # 显示布尔状态 (是否按下)
        display.text("L Pressed:", 0, 30, 1)
        display.text(f"{'YES' if left_pressed else 'NO'}", 60, 30, 1)
        
        display.text("R Pressed:", 0, 40, 1)
        display.text(f"{'YES' if right_pressed else 'NO'}", 60, 40, 1)
        
        # 状态指示灯
        yellow_led.value(left_pressed or right_pressed)

        display.show()
        
        # 更新频率
        time.sleep_ms(50)

except KeyboardInterrupt:
    display.fill(0)
    display.text("STOPPED", 0, 0, 1)
    display.show()