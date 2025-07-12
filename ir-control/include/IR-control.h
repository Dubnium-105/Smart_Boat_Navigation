#ifndef IR_CONTROL_H
#define IR_CONTROL_H

// 导航状态结构体
struct NavigationStatus {
    bool hasValidSignal;           // 是否有有效信号
    int lastDirection;             // 最后一次有效的方向（角度）
    unsigned long timeSinceLastSignal; // 自上次接收到信号的时间（毫秒）
};

// IR传感器引脚声明
extern const int IR_PINS[8];

// 初始化红外接收器
void setupIR();

// 处理红外信号并控制电机
void handleIRSignal();

// 获取当前导航状态
NavigationStatus getNavigationStatus();

#endif // IR_CONTROL_H
