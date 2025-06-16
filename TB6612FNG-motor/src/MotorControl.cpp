#include <Arduino.h>
#include "MotorControl.h"

// Motor pin definitions
#define AIN1_PIN 16
#define AIN2_PIN 17
#define BIN1_PIN 18
#define BIN2_PIN 19
#define PWMA_PIN 46
#define PWMB_PIN 45
#define STBY_PIN 21

// Motor speed variables
static int motorASpeed = 0;
static int motorBSpeed = 0;

void MotorControl::initialize() {
    // Set pin modes
    pinMode(STBY_PIN, OUTPUT);
    pinMode(AIN1_PIN, OUTPUT);
    pinMode(AIN2_PIN, OUTPUT);
    pinMode(BIN1_PIN, OUTPUT);
    pinMode(BIN2_PIN, OUTPUT);
    pinMode(PWMA_PIN, OUTPUT);    pinMode(PWMB_PIN, OUTPUT);
    
    // Enable motor driver
    digitalWrite(STBY_PIN, HIGH);
    
    // Print initialization info
    Serial.println("TB6612FNG 电机驱动初始化完成");
    Serial.printf("电机死区设置: ±%d (速度绝对值小于此值时电机停止)\n", MOTOR_DEADZONE);
    Serial.println("键盘控制: W/S-电机A, E/D-电机B");
}

void MotorControl::handleKeyboardInput() {
    if (Serial.available()) {
        char command = Serial.read();
        
        switch (command) {
            case 'w':
                motorASpeed += 10;
                break;
            case 's':
                motorASpeed -= 10;
                break;
            case 'e':
                motorBSpeed += 10;
                break;
            case 'd':
                motorBSpeed -= 10;
                break;
            default:
                Serial.println("Invalid command. Use WS for Motor A and ED for Motor B.");
                return;
        }
          // Apply speed limits
        motorASpeed = constrain(motorASpeed, -255, 255);
        motorBSpeed = constrain(motorBSpeed, -255, 255);
        
        // Update motor speeds (with deadzone applied internally)
        setMotorASpeed(motorASpeed);
        setMotorBSpeed(motorBSpeed);
        
        // Print status with deadzone info
        Serial.printf("Motor A Speed: %d (原始值: %d, 死区: ±%d)\n", 
                     applyDeadzone(motorASpeed), motorASpeed, MOTOR_DEADZONE);
        Serial.printf("Motor B Speed: %d (原始值: %d, 死区: ±%d)\n", 
                     applyDeadzone(motorBSpeed), motorBSpeed, MOTOR_DEADZONE);
    }
}

void MotorControl::setMotorASpeed(int speed) {
    // 应用死区处理
    speed = applyDeadzone(speed);
    motorASpeed = constrain(speed, -255, 255);
    
    if (motorASpeed > 0) {
        digitalWrite(AIN1_PIN, HIGH);
        digitalWrite(AIN2_PIN, LOW);
        analogWrite(PWMA_PIN, motorASpeed);
    } else if (motorASpeed < 0) {
        digitalWrite(AIN1_PIN, LOW);
        digitalWrite(AIN2_PIN, HIGH);
        analogWrite(PWMA_PIN, abs(motorASpeed));
    } else {
        // 停止电机
        digitalWrite(AIN1_PIN, LOW);
        digitalWrite(AIN2_PIN, LOW);
        analogWrite(PWMA_PIN, 0);
    }
}

void MotorControl::setMotorBSpeed(int speed) {
    // 应用死区处理
    speed = applyDeadzone(speed);
    motorBSpeed = constrain(speed, -255, 255);
    
    if (motorBSpeed > 0) {
        digitalWrite(BIN1_PIN, HIGH);
        digitalWrite(BIN2_PIN, LOW);
        analogWrite(PWMB_PIN, motorBSpeed);
    } else if (motorBSpeed < 0) {
        digitalWrite(BIN1_PIN, LOW);
        digitalWrite(BIN2_PIN, HIGH);
        analogWrite(PWMB_PIN, abs(motorBSpeed));
    } else {
        // 停止电机
        digitalWrite(BIN1_PIN, LOW);
        digitalWrite(BIN2_PIN, LOW);
        analogWrite(PWMB_PIN, 0);
    }
}

// 死区处理函数
int MotorControl::applyDeadzone(int speed) {
    if (abs(speed) < MOTOR_DEADZONE) {
        return 0;  // 在死区范围内，速度设为0
    }
    return speed;
}
