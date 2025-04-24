#include "navigation.h"
#include <Arduino.h>

// 当前导航状态
volatile NavigationState currentState = IDLE;

// 目标颜色设置 (0=黄色, 1=红色)
int targetColorIndex = 0;  // 默认为黄色，可通过MQTT更改

// 颜色检测HSV阈值
// 黄色HSV阈值
const int YELLOW_H_LOW = 20;
const int YELLOW_H_HIGH = 40;
const int YELLOW_S_LOW = 100;
const int YELLOW_S_HIGH = 255;
const int YELLOW_V_LOW = 100;
const int YELLOW_V_HIGH = 255;

// 红色HSV阈值 (考虑到红色在HSV的两端，需要两组值)
const int RED1_H_LOW = 0;
const int RED1_H_HIGH = 10;
const int RED2_H_LOW = 160;
const int RED2_H_HIGH = 180;
const int RED_S_LOW = 100;
const int RED_S_HIGH = 255;
const int RED_V_LOW = 100;
const int RED_V_HIGH = 255;

// 任务控制变量
bool targetFound = false;       // 是否找到目标
bool obstacleDetected = false;  // 是否检测到障碍物
bool gateDetected = false;      // 是否检测到光电门
int targetX = -1;               // 目标X坐标
int targetY = -1;               // 目标Y坐标
unsigned long stateStartTime = 0;  // 状态开始时间，用于超时控制

/**
 * 初始化导航系统
 */
void initNavigation() {
  currentState = IDLE;
  stateStartTime = millis();
  Serial.println("导航系统已初始化");
}

/**
 * 颜色检测函数 - 检测图像中指定颜色的物体
 */
bool detectColor(camera_fb_t *fb, int &outX, int &outY, bool isRedTarget) {
  if (fb == NULL || fb->format != PIXFORMAT_JPEG) {
    return false;  // 只支持JPEG格式
  }
  
  // 将JPEG解码为RGB并进行HSV转换
  // 这里使用简化版以节省资源，只在图像的关键区域进行颜色检测
  
  int targetCount = 0;
  int sumX = 0, sumY = 0;
  
  // 在图像中搜索目标颜色
  // 每隔几个像素采样以提高效率
  const int STEP = 4;
  
  for (int y = 0; y < fb->height; y += STEP) {
    for (int x = 0; x < fb->width; x += STEP) {
      // 从JPEG中提取RGB并转换为HSV
      // 简化版实现 - 实际代码需要JPEG解码库
      
      // 检查当前像素是否符合目标颜色
      bool matchesColor = false;
      
      if (isRedTarget) {
        // 检查红色 (两个H范围)
        // matchesColor = (h >= RED1_H_LOW && h <= RED1_H_HIGH) || 
        //                (h >= RED2_H_LOW && h <= RED2_H_HIGH);
        matchesColor = true; // 简化版，实际需要根据HSV值判断
      } else {
        // 检查黄色
        // matchesColor = (h >= YELLOW_H_LOW && h <= YELLOW_H_HIGH);
        matchesColor = true; // 简化版，实际需要根据HSV值判断
      }
      
      if (matchesColor) {
        sumX += x;
        sumY += y;
        targetCount++;
      }
    }
  }
  
  // 如果找到足够多的目标颜色像素，计算中心位置
  if (targetCount > 20) {
    outX = sumX / targetCount;
    outY = sumY / targetCount;
    return true;
  }
  
  return false;
}

/**
 * 障碍物检测函数 - 检测前方的黑色浮球
 */
bool detectObstacles(camera_fb_t *fb, int &obstacleX, int &obstacleY) {
  if (fb == NULL) {
    return false;
  }
  
  // 对于灰度图，直接检测暗区域作为障碍物
  if (fb->format == PIXFORMAT_GRAYSCALE) {
    const int DARK_THRESHOLD = 50;  // 暗区域阈值
    int darkCount = 0;
    int sumX = 0, sumY = 0;
    
    // 每隔几个像素采样以提高效率
    const int STEP = 4;
    
    // 只检查图像下半部分，因为障碍物通常在视野下方
    for (int y = fb->height/2; y < fb->height; y += STEP) {
      for (int x = 0; x < fb->width; x += STEP) {
        int pixelValue = fb->buf[y * fb->width + x];
        if (pixelValue < DARK_THRESHOLD) {
          sumX += x;
          sumY += y;
          darkCount++;
        }
      }
    }
    
    // 如果找到足够多的暗像素，视为障碍物
    if (darkCount > 50) {
      obstacleX = sumX / darkCount;
      obstacleY = sumY / darkCount;
      return true;
    }
  }
  
  return false;
}

/**
 * 光电门检测函数 - 使用最亮点检测算法寻找光电门
 */
bool detectGate(int brightX, int brightY, int &targetX, int &targetY) {
  // 最亮点检测方法已经在find_brightest函数中实现
  // 这里只需确认亮点是否足够亮并位于合理位置
  
  // 检查亮点是否在图像的有效范围内
  if (brightX >= 0 && brightY >= 0) {
    targetX = brightX;
    targetY = brightY;
    return true;
  }
  
  return false;
}

/**
 * 计算船只应该转向的方向和速度
 */
void calculateSteering(int targetX, int targetY, int imageWidth, int imageHeight, 
                      int &leftSpeed, int &rightSpeed) {
  // 图像中心点
  int centerX = imageWidth / 2;
  int centerY = imageHeight / 2;
  
  // 计算目标相对于中心的偏移
  int offsetX = targetX - centerX;
  
  // 将偏移转换为转向值
  // 偏移越大，转向越明显
  float steeringFactor = (float)offsetX / (float)centerX;
  
  // 基础速度
  int baseSpeed = 50;  // 适中的基础速度
  
  // 根据偏移计算左右电机速度
  if (steeringFactor > 0) {  // 目标在右侧，需要向右转
    rightSpeed = baseSpeed * (1.0 - steeringFactor);
    leftSpeed = baseSpeed;
  } else {  // 目标在左侧，需要向左转
    leftSpeed = baseSpeed * (1.0 + steeringFactor);
    rightSpeed = baseSpeed;
  }
  
  // 确保速度在合理范围内
  leftSpeed = constrain(leftSpeed, -100, 100);
  rightSpeed = constrain(rightSpeed, -100, 100);
}

/**
 * 状态切换函数
 */
void changeState(NavigationState newState) {
  currentState = newState;
  stateStartTime = millis();  // 记录状态开始时间
  
  // 状态切换时的初始化操作
  switch (newState) {
    case IDLE:
      // 停止所有电机
      motor_control(0, 0);
      motor_control(1, 0);
      break;
      
    case FIND_GATE:
      Serial.println("开始寻找第一个光电门");
      break;
      
    case PASS_GATE_1:
      Serial.println("正在通过第一个光电门");
      break;
      
    case AVOID_OBSTACLES:
      Serial.println("进入避障阶段");
      break;
      
    case FIND_GATE_2:
      Serial.println("开始寻找第二个光电门");
      break;
      
    case PASS_GATE_2:
      Serial.println("正在通过第二个光电门");
      break;
      
    case FIND_TARGET:
      Serial.println("开始寻找目标浮标");
      break;
      
    case APPROACH_TARGET:
      Serial.println("正在接近目标浮标");
      break;
      
    case HIT_TARGET:
      Serial.println("撞击目标浮标");
      break;
      
    case GO_TO_DOCK:
      Serial.println("前往停泊区");
      break;
      
    case FINAL_DOCKING:
      Serial.println("最终停泊");
      break;
  }
}

/**
 * 导航状态机 - 处理船只导航的各个状态
 */
void navigationStateMachine(camera_fb_t *fb, int brightX, int brightY) {
  // 声明在switch前的所有可能用到的变量，避免跨case初始化问题
  int obstacleX = -1, obstacleY = -1;
  int colorX = -1, colorY = -1;
  bool isRedTarget = (targetColorIndex == 1);  // 1=红色，0=黄色
  int leftSpeed = 0, rightSpeed = 0;
  int speed = 0;
  
  // 处理超时 - 如果某个状态持续太久，进入下一状态或重置
  unsigned long currentTime = millis();
  unsigned long stateElapsedTime = currentTime - stateStartTime;
  
  // 根据当前导航状态执行对应逻辑
  switch (currentState) {
    case IDLE:
      // 空闲状态，可以通过MQTT命令或按钮启动
      // 这里简化为自动启动，实际应根据比赛规则触发
      Serial.println("准备开始任务...");
      changeState(FIND_GATE);
      break;
      
    case FIND_GATE:
      // 寻找第一个光电门 - 使用红外光源检测
      if (detectGate(brightX, brightY, targetX, targetY)) {
        Serial.printf("发现光电门: x=%d, y=%d\n", targetX, targetY);
        gateDetected = true;
        
        // 调整船只朝向光电门
        calculateSteering(targetX, targetY, 320, 240, leftSpeed, rightSpeed);
        
        // 输出方向控制命令
        motor_control(0, leftSpeed);
        motor_control(1, rightSpeed);
        
        // 如果光电门在图像中央区域，表示已经对准，开始通过
        if (abs(targetX - 160) < 30) {
          changeState(PASS_GATE_1);
        }
      } else {
        // 没有检测到光电门，执行搜索模式
        Serial.println("寻找光电门中...");
        // 执行缓慢旋转，扫描周围环境
        motor_control(0, 20);
        motor_control(1, -20);
      }
      
      // 如果搜索时间过长，重置状态
      if (stateElapsedTime > 20000) {  // 20秒超时
        Serial.println("寻找光电门超时，重置状态");
        changeState(FIND_GATE);  // 重新开始寻找
      }
      break;
      
    case PASS_GATE_1:
      // 通过第一个光电门 - 直线行驶
      Serial.println("正在通过光电门...");
      
      // 保持直线通过
      motor_control(0, 50);
      motor_control(1, 50);
      
      // 假设通过需要5秒，之后进入避障状态
      if (stateElapsedTime > 5000) {
        Serial.println("已通过第一个光电门");
        changeState(AVOID_OBSTACLES);
      }
      break;
      
    case AVOID_OBSTACLES:
      // 避障阶段 - 检测黑色浮球并避开
      if (detectObstacles(fb, obstacleX, obstacleY)) {
        Serial.printf("检测到障碍物: x=%d, y=%d\n", obstacleX, obstacleY);
        
        // 确定避障方向（假设向左避障）
        if (obstacleX < 160) {  // 障碍物在左侧，向右避开
          motor_control(0, 30);
          motor_control(1, 60);
        } else {  // 障碍物在右侧，向左避开
          motor_control(0, 60);
          motor_control(1, 30);
        }
      } else {
        // 没有障碍物，继续前进
        motor_control(0, 40);
        motor_control(1, 40);
      }
      
      // 避障持续15秒后进入寻找第二个光电门状态
      if (stateElapsedTime > 15000) {
        Serial.println("避障阶段完成");
        changeState(FIND_GATE_2);
      }
      break;
      
    case FIND_GATE_2:
      // 寻找第二个光电门 - 使用红外光源检测
      if (detectGate(brightX, brightY, targetX, targetY)) {
        Serial.printf("发现第二个光电门: x=%d, y=%d\n", targetX, targetY);
        
        // 调整船只朝向光电门
        calculateSteering(targetX, targetY, 320, 240, leftSpeed, rightSpeed);
        
        motor_control(0, leftSpeed);
        motor_control(1, rightSpeed);
        
        // 如果光电门在图像中央区域，表示已经对准，开始通过
        if (abs(targetX - 160) < 30) {
          changeState(PASS_GATE_2);
        }
      } else {
        // 没有检测到光电门，执行搜索模式
        Serial.println("寻找第二个光电门中...");
        // 执行缓慢旋转，扫描周围环境
        motor_control(0, 20);
        motor_control(1, -20);
      }
      
      // 如果搜索时间过长，尝试不同方向
      if (stateElapsedTime > 20000) {  // 20秒超时
        Serial.println("寻找第二个光电门超时，尝试不同方向");
        motor_control(0, -20);
        motor_control(1, 20);
        // 重置状态时间，给予更多时间寻找
        stateStartTime = currentTime;
      }
      break;
      
    case PASS_GATE_2:
      // 通过第二个光电门
      Serial.println("正在通过第二个光电门...");
      
      // Drift correction - 保持直线通过
      motor_control(0, 50);
      motor_control(1, 50);
      
      // 假设通过需要5秒，之后进入寻找目标浮标状态
      if (stateElapsedTime > 5000) {
        Serial.println("已通过第二个光电门");
        changeState(FIND_TARGET);
      }
      break;
      
    case FIND_TARGET:
      // 寻找目标浮标 - 检测指定颜色
      if (detectColor(fb, colorX, colorY, isRedTarget)) {
        Serial.printf("发现目标浮标: x=%d, y=%d\n", colorX, colorY);
        targetX = colorX;
        targetY = colorY;
        targetFound = true;
        changeState(APPROACH_TARGET);
      } else {
        // 没有检测到目标，执行搜索模式
        Serial.println("寻找目标浮标中...");
        // 缓慢旋转，扫描周围环境
        motor_control(0, 15);
        motor_control(1, -15);
      }
      
      // 如果搜索时间过长，重置状态
      if (stateElapsedTime > 25000) {  // 25秒超时
        Serial.println("寻找浮标超时，重置状态");
        changeState(FIND_TARGET);  // 重新开始寻找
      }
      break;
      
    case APPROACH_TARGET:
      // 接近目标浮标
      if (targetFound) {
        Serial.printf("接近目标浮标: x=%d, y=%d\n", targetX, targetY);
        
        // 调整船只朝向目标
        calculateSteering(targetX, targetY, 320, 240, leftSpeed, rightSpeed);
        
        // 增加速度以便撞击
        leftSpeed = map(leftSpeed, -100, 100, -80, 80);
        rightSpeed = map(rightSpeed, -100, 100, -80, 80);
        
        motor_control(0, leftSpeed);
        motor_control(1, rightSpeed);
        
        // 目标很靠近时，全速前进撞击
        if (targetY > 180) {  // 目标在图像下方，表示很近
          changeState(HIT_TARGET);
        }
      } else {
        // 失去目标，重新寻找
        changeState(FIND_TARGET);
      }
      
      // 如果接近时间过长，直接进入撞击状态
      if (stateElapsedTime > 10000) {  // 10秒超时
        Serial.println("接近目标超时，直接撞击");
        changeState(HIT_TARGET);
      }
      break;
      
    case HIT_TARGET:
      // 撞击目标浮标 - 全速前进
      Serial.println("全速撞击目标浮标!");
      
      // 全速前进
      motor_control(0, 100);
      motor_control(1, 100);
      
      // 撞击持续3秒后进入前往停泊区状态
      if (stateElapsedTime > 3000) {
        Serial.println("撞击完成，前往停泊区");
        changeState(GO_TO_DOCK);
      }
      break;
      
    case GO_TO_DOCK:
      // 前往停泊区 - 根据场地布局调整
      Serial.println("正在前往停泊区...");
      
      // 由于停泊区的位置在比赛中是固定的
      // 可以通过预设的路线或使用最后一个光电门作为参考点
      
      // 这里使用简单的时间控制转向
      if (stateElapsedTime < 5000) {
        // 先向左转向
        motor_control(0, 20);
        motor_control(1, 40);
      } else if (stateElapsedTime < 15000) {
        // 直线行驶一段时间
        motor_control(0, 40);
        motor_control(1, 40);
      } else {
        // 进入最终停泊阶段
        changeState(FINAL_DOCKING);
      }
      break;
      
    case FINAL_DOCKING:
      // 最终停泊 - 减速直到停止
      Serial.println("准备最终停泊...");
      
      // 逐渐减速
      speed = 30 - (stateElapsedTime / 200);  // 线性减速
      if (speed < 0) speed = 0;
      
      motor_control(0, speed);
      motor_control(1, speed);
      
      // 3秒后完全停止
      if (stateElapsedTime > 3000) {
        Serial.println("任务完成，已停泊");
        motor_control(0, 0);
        motor_control(1, 0);
        // 保持FINAL_DOCKING状态，等待下一次任务
      }
      break;
  }
}