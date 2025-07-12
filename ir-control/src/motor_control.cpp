#include "motor_control.h"
#include <Arduino.h>
#include <esp32-hal-ledc.h>

// 角度表，0~7号传感器对应角度
const int IR_ANGLES[8] = {0, 45, 90, 135, 180, 225, 270, 315};

/**
 * 初始化电机控制函数 - 配置PWM通道和引脚
 */
void setup_motors() {
  Serial.println("初始化电机控制...");
  
  // 设置四个PWM通道用于控制两个电机
  ledcSetup(0, 5000, 8);  // 电机A方向1，5kHz频率，8位分辨率
  ledcAttachPin(MOTOR_A_IN1, 0);
  ledcSetup(1, 5000, 8);  // 电机A方向2
  ledcAttachPin(MOTOR_A_IN2, 1);
  ledcSetup(2, 5000, 8);  // 电机B方向1
  ledcAttachPin(MOTOR_B_IN3, 2);
  ledcSetup(3, 5000, 8);  // 电机B方向2
  ledcAttachPin(MOTOR_B_IN4, 3);
  
  // 初始化时停止所有电机
  ledcWrite(0, 0);
  ledcWrite(1, 0);
  ledcWrite(2, 0);
  ledcWrite(3, 0);
  
  Serial.println("电机控制初始化完成");
}

/**
 * 控制电机运动
 * @param motor 电机选择 (0=电机A, 1=电机B)
 * @param pwm_value PWM值 (-255到255，负值表示反向)
 */
void motor_control(uint8_t motor, int pwm_value) {
  // 确保PWM值在-255到255范围内
  pwm_value = constrain(pwm_value, -255, 255);
  
  uint8_t pin1_channel, pin2_channel;
  
  // 根据motor参数选择对应的PWM通道
  if (motor == 0) { // 电机A
    pin1_channel = 0; // MOTOR_A_IN1通道
    pin2_channel = 1; // MOTOR_A_IN2通道
  } else if (motor == 1) { // 电机B
    pin1_channel = 2; // MOTOR_B_IN3通道
    pin2_channel = 3; // MOTOR_B_IN4通道
  } else {
    // 如果motor参数不是0或1，则为无效电机编号，直接返回
    return; 
  }

  if (pwm_value > 0) { // 正转
    ledcWrite(pin1_channel, pwm_value);
    ledcWrite(pin2_channel, 0);
  } else if (pwm_value < 0) { // 反转
    ledcWrite(pin1_channel, 0);
    ledcWrite(pin2_channel, abs(pwm_value));
  } else { // 停止
    ledcWrite(pin1_channel, 0);
    ledcWrite(pin2_channel, 0);
  }
}
