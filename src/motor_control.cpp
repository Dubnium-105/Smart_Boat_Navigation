#include "motor_control.h"
#include <Arduino.h>
#include <esp32-hal-ledc.h> // 显式包含ledc库

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
 * @param pwm_value PWM值 (-255到255，负值表示反向，直接使用无映射)
 */
void motor_control(uint8_t motor, int pwm_value) {
  // 确保PWM值在-255到255范围内
  pwm_value = constrain(pwm_value, -255, 255);
  
  uint8_t pin1_channel, pin2_channel;
  
  if (motor == 0) { // 电机A
    pin1_channel = 0; // MOTOR_A_IN1
    pin2_channel = 1; // MOTOR_A_IN2
  } else if (motor == 1) { // 电机B
    pin1_channel = 2; // MOTOR_B_IN3
    pin2_channel = 3; // MOTOR_B_IN4
  } else {
    return; // 无效电机编号
  }

  if (pwm_value > 0) { // 正转，直接使用PWM值
    ledcWrite(pin1_channel, pwm_value);
    ledcWrite(pin2_channel, 0);
    // Serial.printf("Motor %d Forward: PWM=%d\n", motor, pwm_value);
  } else if (pwm_value < 0) { // 反转，直接使用PWM绝对值
    ledcWrite(pin1_channel, 0);
    ledcWrite(pin2_channel, abs(pwm_value));
    // Serial.printf("Motor %d Reverse: PWM=%d\n", motor, abs(pwm_value));
  } else { // 停止
    ledcWrite(pin1_channel, 0);
    ledcWrite(pin2_channel, 0);
    // Serial.printf("Motor %d Stop\n", motor);
  }
}

// 红外自动导航差速控制（最高速150）
void motor_control_ir_auto(int mainDirIdx) {
    if (mainDirIdx < 0 || mainDirIdx > 7) return;
    int targetAngle = IR_ANGLES[mainDirIdx];
    int delta = (targetAngle + 360) % 360;
    if (delta > 180) delta -= 360; // -180~180
    int baseSpeed = 120; // 默认速度
    int maxSpeed = 150;  // 自动模式最高速
    int diff = (delta * 2) / 90; // 45°时差速最大
    int left = baseSpeed - diff * 60;
    int right = baseSpeed + diff * 60;
    left = constrain(left, -maxSpeed, maxSpeed);
    right = constrain(right, -maxSpeed, maxSpeed);
    motor_control(0, left);
    motor_control(1, right);
}

// 红外导航控制，带强制停止选项
void motor_control_ir_navigation(int mainDirIdx, bool forceStop) {
    if (forceStop) {
        motor_control(0, 0);
        motor_control(1, 0);
        return;
    }
    if (mainDirIdx < 0 || mainDirIdx > 7) return;
    int targetAngle = IR_ANGLES[mainDirIdx];
    int delta = (targetAngle + 360) % 360;
    if (delta > 180) delta -= 360; // -180~180
    int baseSpeed = 120;
    int maxSpeed = 150;
    int diff = (delta * 2) / 90;
    int left = baseSpeed - diff * 60;
    int right = baseSpeed + diff * 60;
    left = constrain(left, -maxSpeed, maxSpeed);
    right = constrain(right, -maxSpeed, maxSpeed);
    motor_control(0, left);
    motor_control(1, right);
}