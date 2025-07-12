#include <Arduino.h>
#include "motor_control.h"
#include "IR-control.h"

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println("\n\n====================================");
  Serial.println("      红外控制系统启动中...     ");
  Serial.println("====================================");

  // 1. 初始化红外接收器
  Serial.println("初始化红外接收器...");
  setupIR();

  // 2. 初始化电机
  Serial.println("初始化电机控制...");
  setup_motors();

  // 3. 检查红外信号并进入自动控制模式
  Serial.println("进入红外自动控制模式...");
  
  Serial.println("====================================");
  Serial.println("       系统初始化完成!          ");
  Serial.println("====================================");
}

void loop() {
  // 处理红外信号并控制电机
  handleIRSignal();
  delay(2); // 保持高帧率
}