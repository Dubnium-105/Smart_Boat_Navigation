#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <Arduino.h>

// 电机控制引脚定义
#define MOTOR_A_IN1 38     // A电机控制引脚1
#define MOTOR_A_IN2 39     // A电机控制引脚2
#define MOTOR_B_IN3 40     // B电机控制引脚1
#define MOTOR_B_IN4 41     // B电机控制引脚2

/**
 * @brief 初始化电机控制所需的PWM通道和引脚。
 */
void setup_motors();

/**
 * @brief 控制指定电机的速度和方向。
 * 
 * @param motor 电机选择 (0=电机A, 1=电机B)。
 * @param speed 速度值 (-100到100，负值表示反向，0表示停止)。
 */
void motor_control(uint8_t motor, int speed);

#endif // MOTOR_CONTROL_H