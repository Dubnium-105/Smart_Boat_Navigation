#ifndef NAVIGATION_H
#define NAVIGATION_H

#include "esp_camera.h"
#include <Arduino.h>
#include "motor_control.h" // 直接包含电机控制头文件而不是使用外部声明

// 添加详细注释，描述导航模块的功能和状态机
// navigation.h定义了导航系统的状态机、颜色检测阈值和相关函数。
// 它支持多种任务状态，例如寻找光电门、避障和寻找目标浮标。

// 任务状态定义
enum NavigationState {
  IDLE,                // 空闲状态
  FIND_GATE,           // 寻找第一个光电门
  PASS_GATE_1,         // 通过第一个光电门
  AVOID_OBSTACLES,     // 避障阶段
  FIND_GATE_2,         // 寻找第二个光电门
  PASS_GATE_2,         // 通过第二个光电门
  FIND_TARGET,         // 寻找目标浮标
  APPROACH_TARGET,     // 接近目标浮标
  HIT_TARGET,          // 撞击目标浮标
  GO_TO_DOCK,          // 前往停泊区
  FINAL_DOCKING        // 最终停泊
};

// 颜色检测HSV阈值
// 黄色HSV阈值
extern const int YELLOW_H_LOW;
extern const int YELLOW_H_HIGH;
extern const int YELLOW_S_LOW;
extern const int YELLOW_S_HIGH;
extern const int YELLOW_V_LOW;
extern const int YELLOW_V_HIGH;

// 红色HSV阈值 (考虑到红色在HSV的两端，需要两组值)
extern const int RED1_H_LOW;
extern const int RED1_H_HIGH;
extern const int RED2_H_LOW;
extern const int RED2_H_HIGH;
extern const int RED_S_LOW;
extern const int RED_S_HIGH;
extern const int RED_V_LOW;
extern const int RED_V_HIGH;

// 导航状态变量
extern volatile NavigationState currentState;
extern unsigned long stateStartTime;
extern int targetColorIndex;
extern bool targetFound;
extern bool obstacleDetected;
extern bool gateDetected;
extern int targetX;
extern int targetY;

// 函数声明
void initNavigation();
void navigationStateMachine(camera_fb_t *fb, int brightX, int brightY);
void changeState(NavigationState newState);
bool detectColor(camera_fb_t *fb, int &outX, int &outY, bool isRedTarget);
bool detectObstacles(camera_fb_t *fb, int &obstacleX, int &obstacleY);
bool detectGate(int brightX, int brightY, int &targetX, int &targetY);
void calculateSteering(int targetX, int targetY, int imageWidth, int imageHeight, 
                       int &leftSpeed, int &rightSpeed);

// 移除了对motor_control的外部声明，因为已经包含了motor_control.h

#endif // NAVIGATION_H