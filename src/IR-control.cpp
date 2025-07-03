#include "IR-control.h"
#include "motor_control.h"
#include "mqtt_manager.h"  // 添加MQTT管理头文件

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

void setupIR() {
    for (int i = 0; i < 8; ++i) {
        pinMode(IR_PINS[i], INPUT);
    }
    Serial.println("IR输入引脚初始化完成");
}

// 判断主方向（前方优先，取中间，抑制反射）
int getMainIRDirection(const int irVals[8]) {
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
    int mainDir = getMainIRDirection(irVals);

    if (mainDir >= 0) {
        if (mainDir == lastMainDir) {
            stableCount++;
        } else {
            stableCount = 1;
            lastMainDir = mainDir;
        }
        // 只有连续多帧一致且与上次输出方向不同才输出
        if (stableCount >= STABLE_THRESHOLD && mainDir != stableMainDir) {
            // 限制串口输出频率，防止卡顿
            static unsigned long lastSerialOutput = 0;
            unsigned long now = millis();
            if (now - lastSerialOutput >= 200) { // 最多每200ms输出一次
                Serial.print("主方向: ");
                Serial.print(IR_DIRECTIONS[mainDir]);
                Serial.print(" (");
                
                // 输出所有激活的传感器引脚
                bool first = true;
                for (int i = 0; i < 8; ++i) {
                    if (irVals[i] == 0) { // 如果传感器被激活
                        if (!first) Serial.print(" ");
                        Serial.print(IR_PINS[i]);
                        first = false;
                    }
                }
                
                Serial.println(")");
                lastSerialOutput = now;
            }
            
            // 方向改变时立即发送MQTT消息
            stableMainDir = mainDir;
            // 驱动导航状态机，方向已改变
            motor_control_ir_navigation(mainDir, true);
            publishIRInfo(mainDir, IR_DIRECTIONS[mainDir], IR_ANGLES[mainDir]);
        }
        // 方向未改变时，每3秒定时发送一次
        else if (stableCount >= STABLE_THRESHOLD && mainDir == stableMainDir) {
            static unsigned long lastMqttSend = 0;
            unsigned long now = millis();
            if (now - lastMqttSend >= 3000) { // 3秒间隔
                publishIRInfo(mainDir, IR_DIRECTIONS[mainDir], IR_ANGLES[mainDir]);
                // 持续驱动，方向未变
                motor_control_ir_navigation(mainDir, false);
                lastMqttSend = now;
            }
        }
        // 持续运行导航，若在未变但未到3秒也需调用驱动以维护状态
        else if (stableCount >= STABLE_THRESHOLD && mainDir == stableMainDir) {
            motor_control_ir_navigation(mainDir, false);
        }
    } else {
        // 没有信号时停止电机
        motor_control_ir_navigation(-1, false);
        // stableMainDir = -1;
    }
}
