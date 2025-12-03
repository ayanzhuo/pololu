#include <Servo.h>  // 导入舵机库
#include <EEPROM.h> // 导入EEPROM库
#include <math.h>   // 导入数学库，用于三角函数 (sin, cos, atan2)
#include <string.h> // 导入 string.h 用于 memset 和 memcpy

// --- 宏定义 (来自官方代码) ---
#define EEPROM_START_FLAG "HIWONDER"
#define EEPROM_SERVO_OFFSET_START_ADDR 1024
#define EEPROM_SERVO_OFFSET_DATA_ADDR 1024+16

// 假设夹爪角度 (对应 limt_angles[0])
#define GRIPPER_OPEN  82
#define GRIPPER_CLOSE 0 

// --- 运动学参数定义 (单位: mm, 基于尺寸图) ---
// MiniArm 几何尺寸 (L4 为估算值，请自行验证)
const float L1 = 104.0; // 关节1到关节2的垂直高度
const float L2 = 58.0;  // 连杆 2 (肩到肘)
const float L3 = 62.0;  // 连杆 3 (肘到腕)
const float L4 = 115.0; // 连杆 4 (腕到末端夹爪中心) - 估算值

// --- 全局变量定义 ---
const static uint8_t servoPins[5] = { 7, 6, 5, 4, 3}; // 舵机引脚定义
static uint8_t eeprom_read_buf[16];
Servo servos[5];

// 关节角度限制 [min, max]
const uint8_t limt_angles[5][2] = {{0,82},{0,180},{0,180},{25,180},{0,180}}; 

static int8_t servo_offset[6] = { 0 , 0 , 0 , 0 , 0 , 0 }; // 舵机偏差值
static uint8_t servo_expect[6]; // 期望角度 (目标角度 + 偏差)
static uint8_t servo_angles[5] = { 90,90,90,90,90};  // 舵机当前角度 
// IK 计算的目标角度 (动作数据)
static uint8_t extended_func_angles[5] = { GRIPPER_OPEN, 90, 90, 90, 90}; 

// --- 函数声明 ---
static void servo_control(void); 
void user_task(void);
void read_servo_offset(void);
float degToRad(float deg);
float radToDeg(float rad);
bool inverseKinematics(float x, float y, float z, float orientation_angle, uint8_t target_angles[5]);
void goToCartesian(float x, float y, float z, uint8_t gripper_angle);
float customConstrain(float amt, float low, float high); 
uint8_t customConstrain(uint8_t amt, uint8_t low, uint8_t high); 

// --- 角度/弧度转换函数 ---
float degToRad(float deg) {
    return deg * M_PI / 180.0;
}

float radToDeg(float rad) {
    return rad * 180.0 / M_PI;
}

// 解决宏冲突：自定义的 constrain 函数
float customConstrain(float amt, float low, float high) {
    if (amt < low) return low;
    if (amt > high) return high;
    return amt;
}

uint8_t customConstrain(uint8_t amt, uint8_t low, uint8_t high) {
    if (amt < low) return low;
    if (amt > high) return high;
    return amt;
}

// --- 核心函数：逆运动学 (IK) 求解 ---
/* * 关键函数：将笛卡尔坐标转换为关节角度。
 */
bool inverseKinematics(float x, float y, float z, float orientation_angle, uint8_t target_angles[5]) {
    
    // --- 步骤 1: 投影和距离计算 ---
    float Px_plane = sqrt(x*x + y*y); // XY 平面径向距离
    float Py_z = z - L1;             // 相对肩关节的垂直高度

    // 假设将 L4 连杆视为刚性工具，计算肩关节到 L4 关节中心的距离
    float R_dist = sqrt(Px_plane*Px_plane + Py_z*Py_z); 

    float L_eff = L3 + L4;

    // 检查工作空间
    if (R_dist > (L2 + L_eff) || R_dist < fabsf(L_eff - L2)) {
        Serial.println("IK Error: Target unreachable.");
        return false;
    }
    
    // --- 步骤 2: 求解 Theta1 (Base 关节) - 对应 target_angles[4] ---
    float theta1 = radToDeg(atan2(y, x));
    
    // --- 步骤 3: 求解 Theta3 (Elbow 关节) - 对应 target_angles[2] ---
    // 使用余弦定理求解 L3 与 L2 连杆之间的角度
    float cos_theta3_val = (R_dist*R_dist - L2*L2 - L_eff*L_eff) / (2 * L2 * L_eff);
    // 确保值在 [-1, 1] 范围内
    cos_theta3_val = customConstrain(cos_theta3_val, -1.0, 1.0); 

    float theta3_calc = radToDeg(acos(cos_theta3_val)); 
    // 实际关节角 = 180° - theta3_calc (典型 Elbow-Up 配置)
    float theta3 = 180.0 - theta3_calc; 

    // --- 步骤 4: 求解 Theta2 (Shoulder 关节) - 对应 target_angles[3] ---
    
    // 1. 求解 alpha (R_dist 与 L2 之间的角度)
    float cos_alpha_val = (R_dist*R_dist + L2*L2 - L_eff*L_eff) / (2 * R_dist * L2);
    cos_alpha_val = customConstrain(cos_alpha_val, -1.0, 1.0);
    float alpha = acos(cos_alpha_val); // 弧度

    // 2. 求解 gamma (R_dist 与水平面夹角)
    float gamma = atan2(Py_z, Px_plane); // 弧度

    // 3. Theta2 的最终解 (假设为 Elbow-Up，肩关节角度为 gamma + alpha)
    float theta2 = radToDeg(gamma + alpha); 
    
    // --- 步骤 5: 求解 Theta4 (Wrist 关节) - 对应 target_angles[1] ---
    // 姿态控制：假设希望末端姿态（Theta2 + Theta3 + Theta4）等于 orientation_angle
    float theta4 = orientation_angle - theta2 - theta3; 

    // --- 步骤 6: 结果赋值和角度约束 ---
    // 确保结果在 0-180 度范围内
    target_angles[4] = (uint8_t)customConstrain(theta1, 0.0, 180.0); // Base (0~180)
    target_angles[3] = (uint8_t)customConstrain(theta2, 0.0, 180.0); // Shoulder (25~180)
    target_angles[2] = (uint8_t)customConstrain(theta3, 0.0, 180.0); // Elbow (0~180)
    target_angles[1] = (uint8_t)customConstrain(theta4, 0.0, 180.0); // Wrist (0~180)
    
    // IK 成功
    return true; 
}


// --- 运动指令封装函数 ---
void goToCartesian(float x, float y, float z, uint8_t gripper_angle) {
    uint8_t calculated_angles[5];
    // 假设末端姿态（相对于水平面）保持 90.0 度 (即垂直于地面)
    float orientation = 90.0; 
    
    if (inverseKinematics(x, y, z, orientation, calculated_angles)) {
        // 将 IK 结果写入全局目标数组 (extended_func_angles)
        extended_func_angles[4] = calculated_angles[4]; // Base
        extended_func_angles[3] = calculated_angles[3]; // Shoulder
        extended_func_angles[2] = calculated_angles[2]; // Elbow
        extended_func_angles[1] = calculated_angles[1]; // Wrist
        
        // 设置夹爪角度
        extended_func_angles[0] = gripper_angle; 
        
        Serial.print("Target [X,Y,Z]: "); Serial.print(x); Serial.print(",");
        Serial.print(y); Serial.print(","); Serial.print(z); Serial.println("");
        Serial.print("IK Angles [G,W,E,S,B]: "); // G=Gripper, W=Wrist, E=Elbow, S=Shoulder, B=Base
        Serial.print(extended_func_angles[0]); Serial.print(",");
        Serial.print(extended_func_angles[1]); Serial.print(",");
        Serial.print(extended_func_angles[2]); Serial.print(",");
        Serial.print(extended_func_angles[3]); Serial.print(",");
        Serial.print(extended_func_angles[4]); Serial.println("");
    } else {
        Serial.println("IK Solver failed. Target unreachable.");
    }
}


// --- Arduino Setup 函数 ---
void setup() {
    Serial.begin(115200);
    Serial.setTimeout(500);
    
    // 绑定舵机IO口
    for (int i = 0; i < 5; ++i) {
        servos[i].attach(servoPins[i]);
    }

    // 调用偏差读取
    read_servo_offset(); 

    // 初始姿态：运动到安全点 (X=150, Y=0, Z=150)
    goToCartesian(150.0, 0.0, 150.0, GRIPPER_OPEN); 
    
    delay(2000);
    Serial.println("start");
}

// --- Arduino Loop 函数 ---
void loop() {
    // 用户任务
    user_task();

    // 舵机控制 (自动平滑地运动到 extended_func_angles)
    servo_control();
}


// --- 抓取任务逻辑 (使用 IK 实现动作序列) ---
void user_task(void)
{
    static uint32_t last_tick = 0;
    static uint8_t sequence_state = 0;
    
    // 状态切换间隔时间
    const uint32_t STATE_DELAY_MS = 2500; // 2.5秒
    
    // 抓取目标参数 (假设在机械臂前方 Z=50mm 处)
    const float GRAB_X = 0.0;
    const float GRAB_Y = 130.0;
    const float GRAB_Z = 20.0; 
    const float APPROACH_HEIGHT = 100.0; // 抬起/接近高度

    // 状态切换计时器
    if (millis() - last_tick < STATE_DELAY_MS) {
        return;
    }
    last_tick = millis();

    switch (sequence_state) {
        case 0: // 抬升到接近点上方 (夹爪张开)
            Serial.println("Sequence 0: Moving to Approach Point...");
            goToCartesian(GRAB_X, GRAB_Y, GRAB_Z + APPROACH_HEIGHT, GRIPPER_OPEN);
            sequence_state = 1;
            break;

        case 1: // 下降到抓取点
            Serial.println("Sequence 1: Dropping to Target Point...");
            goToCartesian(GRAB_X, GRAB_Y, GRAB_Z, GRIPPER_OPEN);
            sequence_state = 2;
            break;

        case 2: // 抓取 (闭合夹爪)
            Serial.println("Sequence 2: Grabbing...");
            // 只更新夹爪角度，保持坐标不变
            extended_func_angles[0] = GRIPPER_CLOSE; 
            sequence_state = 3;
            break;

        case 3: // 提离 (回到接近点，夹爪闭合)
            Serial.println("Sequence 3: Lifting up...");
            goToCartesian(GRAB_X, GRAB_Y, GRAB_Z + APPROACH_HEIGHT, GRIPPER_CLOSE);
            sequence_state = 4;
            break;
            
        case 4: // 移动到释放点 (假设 X=100, Y=100)
            Serial.println("Sequence 4: Moving to Release Point...");
            goToCartesian(100.0, 100.0, APPROACH_HEIGHT, GRIPPER_CLOSE);
            sequence_state = 5; 
            break;
            
        case 5: // 释放 (张开夹爪)
            Serial.println("Sequence 5: Releasing...");
            extended_func_angles[0] = GRIPPER_OPEN;
            sequence_state = 6;
            break;
            
        case 6: // 运动回初始安全点，并重新开始
            Serial.println("Sequence 6: Homing and Restarting...");
            goToCartesian(150.0, 0.0, 150.0, GRIPPER_OPEN);
            sequence_state = 0; // 重新开始循环
            break;
    }
}


// --- 官方代码提供的舵机控制和EEPROM读取函数 ---

// 舵机控制任务（不需修改）
void servo_control(void) {
    static uint32_t last_tick = 0;
    if (millis() - last_tick < 20) {
        return;
    }
    last_tick = millis();

    for (int i = 0; i < 5; ++i) {
        // 应用偏差补偿
        servo_expect[i] = extended_func_angles[i] + servo_offset[i];
        
        // 平滑运动控制 (低通滤波)
        if(servo_angles[i] > servo_expect[i])
        {
            servo_angles[i] = servo_angles[i] * 0.9 + servo_expect[i] * 0.1;
            if(servo_angles[i] < servo_expect[i])
                servo_angles[i] = servo_expect[i];
        }else if(servo_angles[i] < servo_expect[i])
        {
            servo_angles[i] = servo_angles[i] * 0.9 + (servo_expect[i] * 0.1 + 1);
            if(servo_angles[i] > servo_expect[i])
                servo_angles[i] = servo_expect[i];
        }

        // 角度限制 (防止超出物理极限)
        servo_angles[i] = servo_angles[i] < limt_angles[i][0] ? limt_angles[i][0] : servo_angles[i];
        servo_angles[i] = servo_angles[i] > limt_angles[i][1] ? limt_angles[i][1] : servo_angles[i];
        
        // 写入舵机 (舵机 0 和 5 需要反向控制)
        servos[i].write(i == 0 || i == 5 ? 180 - servo_angles[i] : servo_angles[i]);
    }
}

/**
 * @brief 从 EEPROM 读取舵机偏差值
 * */
void read_servo_offset(void){
    // 读取偏差角度 
    for (int i = 0; i < 16; ++i) {
        eeprom_read_buf[i] = EEPROM.read(EEPROM_SERVO_OFFSET_START_ADDR + i);
    }
    // 强制转换为 char*
    if (strcmp((char*)eeprom_read_buf, EEPROM_START_FLAG) == 0) {
        memset(eeprom_read_buf, 0 , sizeof(eeprom_read_buf));
        Serial.println("read offset");
        for (int i = 0; i < 6; ++i) {
            eeprom_read_buf[i] = EEPROM.read(EEPROM_SERVO_OFFSET_DATA_ADDR + i);
        }
        memcpy(servo_offset , eeprom_read_buf , 6);
        Serial.print("Loaded Offset: "); 
        for (int i = 0; i < 6; ++i) {
            Serial.print(servo_offset[i]);
            Serial.print(" ");
        }
        Serial.println("");
    }
}