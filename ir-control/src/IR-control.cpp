#include "IR-control.h"
#include "motor_control.h"

// 选择8个GPIO作为红外输入（可根据实际连线调整）
const int IR_PINS[8] = {4, 5, 6, 7, 8, 9, 10, 11};

// 方向名称数组，按引脚顺序对应
const char* IR_DIRECTIONS[8] = {
    "0°",     // GPIO4
    "45°",   // GPIO5
    "90°",   // GPIO6
    "135°", // GPIO7
    "180°", // GPIO8
    "225°", // GPIO9
    "270°",   // GPIO10
    "315°"  // GPIO11
};

// 角度表，0~7号传感器对应角度
const int IR_ANGLES[8] = {0, 45, 90, 135, 180, 225, 270, 315};

 /**
 * @brief 初始化红外接收器系统
 * 
 * 该函数负责初始化8个红外传感器的GPIO引脚配置。
 * 红外传感器工作原理：
 * - 检测到红外信号时输出低电平(0)
 * - 无红外信号时输出高电平(1)
 * 
 * 传感器空间布局（以船体为中心，按角度排列）：
 * - IR_PINS[0] (GPIO4):  0°   - 正前方
 * - IR_PINS[1] (GPIO5):  45°  - 右前方
 * - IR_PINS[2] (GPIO6):  90°  - 正右方  
 * - IR_PINS[3] (GPIO7):  135° - 右后方
 * - IR_PINS[4] (GPIO8):  180° - 正后方
 * - IR_PINS[5] (GPIO9):  225° - 左后方
 * - IR_PINS[6] (GPIO10): 270° - 正左方
 * - IR_PINS[7] (GPIO11): 315° - 左前方
 * 
 * @note 确保硬件连接与上述引脚定义一致
 * @note 必须在使用红外控制功能前调用此函数
 * @note 该函数应在Arduino setup()函数中调用
 */
void setupIR() {
    // 循环配置8个红外传感器引脚为输入模式
    for (int i = 0; i < 8; ++i) {
        // 设置GPIO引脚为输入模式，用于读取红外传感器的数字信号
        // INPUT模式下引脚具有高阻抗，适合读取外部信号状态
        pinMode(IR_PINS[i], INPUT);
    }
    
    // 通过串口输出初始化完成信息，便于调试和监控
    Serial.println("IR输入引脚初始化完成");
}

// 判断主方向（前方优先，取中间，抑制反射）
int getMainIRDirection(const int irVals[8]) {
    // activeIdx数组用于存储检测到红外信号的传感器索引
    // 当传感器检测到红外信号时，其对应的irVals[i]值为0
    // 该数组记录所有激活传感器的索引号，用于后续方向判断和滤波处理
    int activeIdx[8], count = 0;
    for (int i = 0; i < 8; ++i) {
        if (irVals[i] == 0) activeIdx[count++] = i;
    }
    if (count == 0) return -1; // 无信号
    // 前方优先：0,1,7有信号时，忽略3,4,5
    bool front = (irVals[0]==0 || irVals[1]==0 || irVals[7]==0);
    if (front) {
        int filtered[8], fcount = 0;
        for (int i = 0; i < count; ++i) {
            if (activeIdx[i] != 3 && activeIdx[i] != 4 && activeIdx[i] != 5)
                filtered[fcount++] = activeIdx[i];
        }
        if (fcount > 0) {
            count = fcount;
            for (int i = 0; i < count; ++i) activeIdx[i] = filtered[i];
        }
    }
    // 取中间那个
    int mainIdx = activeIdx[count/2];
    return mainIdx;
}

// 状态变量用于方向切换防抖
void handleIRSignal() {
    static int lastMainDir = -1;
    static int stableMainDir = -1;
    static int stableCount = 0;
    const int STABLE_THRESHOLD = 3; // 连续3帧一致才判定为新方向

    int irVals[8];
    for (int i = 0; i < 8; ++i) {
        irVals[i] = digitalRead(IR_PINS[i]);
    }

    // === 优化后的自动控制部分：两次采样一致才判定 ===
    static unsigned long lastSampleTime = 0; // 上次采样时间
    static bool firstSampleDone = false;     // 是否已完成第一次采样
    static int firstSample[8] = {1,1,1,1,1,1,1,1};  // 第一次采样结果
    static int secondSample[8] = {1,1,1,1,1,1,1,1}; // 第二次采样结果

    // 检测是否有红外信号（任一传感器为0即有信号）
    bool hasSignal = false;
    for (int i = 0; i < 8; ++i) {
        if (irVals[i] == 0) {
            hasSignal = true;
            break;
        }
    }

    unsigned long now = millis(); // 获取当前系统运行的毫秒数，用于定时和时间间隔计算

    // 只有检测到红外信号时才运行采样与控制算法
    if (hasSignal) {
        if (!firstSampleDone) {
            // 第一次采样，记录当前传感器状态
            for (int i = 0; i < 8; ++i) firstSample[i] = irVals[i];
            lastSampleTime = now;
            firstSampleDone = true;
        } 
        else if (now - lastSampleTime >= 50) { // 50ms后进行第二次采样
            // 第二次采样，记录当前传感器状态
            for (int i = 0; i < 8; ++i) secondSample[i] = irVals[i];

            // 找出两次采样都检测到的光源（即两次都为0的传感器）
            int angleSum = 0, count = 0;
            for (int i = 0; i < 8; ++i) {
                if (firstSample[i] == 0 && secondSample[i] == 0) {
                    angleSum += IR_ANGLES[i];
                    count++;
                }
            }
            
            // 根据计算的平均角度控制电机
            if (count > 0) {
                int avgAngle = angleSum / count; // 计算平均角度
                Serial.print("检测角度: ");
                Serial.println(avgAngle);

                // 根据平均角度调整电机转向
                if (avgAngle == 0) {
                    // 正前方，直行
                    motor_control(0, 250);
                    motor_control(1, 250);
                } 
                else if (avgAngle > 0 && avgAngle <= 60) {
                    // 稍偏右，右转
                    motor_control(0, 240);
                    motor_control(1, 180);
                    delay(50);
                } 
                else if (avgAngle > 60 && avgAngle <= 90) {
                    // 明显偏右，右转更多
                    motor_control(0, 250);
                    motor_control(1, 150);
                    delay(50);
                } 
                else if (avgAngle > 300 && avgAngle <= 330) {
                    // 稍偏左，左转
                    motor_control(0, 180);
                    motor_control(1, 250);
                    delay(50);
                } 
                else if (avgAngle > 270 && avgAngle <= 300) {
                    // 明显偏左，左转更多
                    motor_control(0, 150);
                    motor_control(1, 250);
                    delay(50);
                }
                else if (avgAngle > 90 && avgAngle <= 270) {
                    // 后方信号，倒退
                    motor_control(0, -180);
                    motor_control(1, -180);
                    delay(50);
                }
            }
            // 重置采样状态，准备下一轮采样
            firstSampleDone = false;
        }
    } 
    else {
        // 没有信号时重置采样状态
        firstSampleDone = false;
    }

    // 新增：无信号2秒自动停止
    static unsigned long lastSignalTime = 0;
    int mainDir = getMainIRDirection(irVals);
    if (mainDir >= 0) {
        lastSignalTime = now;
    } else if (now - lastSignalTime >= 2000) {
        // 2秒无信号，停止电机
        motor_control(0, 0);
        motor_control(1, 0);
    }
}
