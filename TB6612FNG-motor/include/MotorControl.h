#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <Arduino.h>

// 电机死区设置
#define MOTOR_DEADZONE 10  // 死区范围：-10到+10之间的速度值会被设为0

class MotorControl {
public:
    static void initialize();
    static void handleKeyboardInput();
    static void setMotorASpeed(int speed);
    static void setMotorBSpeed(int speed);
    
private:
    static int applyDeadzone(int speed);  // 死区处理函数
};

#endif
