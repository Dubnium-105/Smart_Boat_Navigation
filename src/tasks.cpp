#include "tasks.h"
#include <Arduino.h>
#include <WiFi.h> // 需要访问WiFi.localIP()

// 低优先级串口监控任务函数 - 分配到核心0，降低与视频流的冲突
void serialMonitorTask(void *parameter) {
  unsigned long lastDiagTime = 0;
  unsigned long lastBrightTime = 0;
  const int BRIGHT_REFRESH_INTERVAL = 800;  // 增加刷新间隔到800ms，减少串口负担

  while (true) {
    unsigned long currentTime = millis();

    // 每10秒输出一次诊断信息
    if (currentTime - lastDiagTime >= 10000) {
      // 检查是否系统繁忙，如果繁忙则跳过此次诊断
      if (!systemIsBusy) {
        Serial.printf("摄像头运行中，当前帧率: %.2f FPS\n", currentFPS);
        // 确保WiFi已连接再获取IP
        if (WiFi.status() == WL_CONNECTED) {
            Serial.printf("当前IP: %s\n", WiFi.localIP().toString().c_str());
        } else {
            Serial.println("WiFi未连接");
        }
        // 可以添加更多诊断信息，如内存使用情况
        // Serial.printf("Free Heap: %u bytes\n", ESP.getFreeHeap());
      } else {
          Serial.println("系统繁忙，跳过诊断信息输出。");
      }
      lastDiagTime = currentTime;
    }

    // 定期输出最亮点信息，间隔增大
    if (currentTime - lastBrightTime >= BRIGHT_REFRESH_INTERVAL) {
      // 只在系统不繁忙时尝试获取互斥锁
      if (!systemIsBusy && xSemaphoreTake(frameAccessMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        if (newBrightPointAvailable) {
          // 安全复制共享数据以减少锁持有时间
          int local_x = sharedBrightX;
          int local_y = sharedBrightY;
          newBrightPointAvailable = false; // 标记为已处理
          xSemaphoreGive(frameAccessMutex); // 尽快释放锁
          
          // 锁外处理数据，减少锁竞争
          Serial.printf("[最亮点] x=%d, y=%d\n", local_x, local_y);
          // 使用更小的显示尺寸, 确保坐标转换正确
          print_ascii_frame(24, 12, local_x * 24 / 320, local_y * 12 / 240); 
        } else {
          xSemaphoreGive(frameAccessMutex); // 没有新数据也要释放锁
        }
      } else if (systemIsBusy) {
          // Serial.println("系统繁忙，跳过最亮点信息输出。"); // 可选的调试信息
      } else {
          // Serial.println("未能获取帧访问锁，跳过最亮点信息输出。"); // 可选的调试信息
      }
      lastBrightTime = currentTime;
    }

    // 更长的延时，减轻CPU负担
    vTaskDelay(pdMS_TO_TICKS(100)); // 100ms延时
  }
}

// 系统健康监视任务 - 监控堆栈和内存
void systemMonitorTask(void* parameter) {
  const UBaseType_t STACK_WARNING_THRESHOLD = 1024; // 1KB 堆栈警告阈值
  const UBaseType_t STACK_CRITICAL_THRESHOLD = 512; // 512B 堆栈危险阈值

  while(true) {
    // 检查当前任务的堆栈水位 (High Water Mark: 任务启动后堆栈指针从未低于的值)
    // 返回值是剩余堆栈空间的字数 (每个字通常4字节)
    UBaseType_t uxHighWaterMarkWords = uxTaskGetStackHighWaterMark(NULL);
    uint32_t freeStackBytes = uxHighWaterMarkWords * sizeof(StackType_t);

    // 如果堆栈剩余空间小于警告阈值，标记系统繁忙
    systemIsBusy = (freeStackBytes < STACK_WARNING_THRESHOLD); 
    
    // 如果堆栈非常低，则打印警告
    if (freeStackBytes < STACK_CRITICAL_THRESHOLD) {
      Serial.printf("警告: 系统监控任务堆栈空间严重不足: %u 字节剩余\n", freeStackBytes);
    } else if (systemIsBusy) {
       // Serial.printf("注意: 系统监控任务堆栈空间较低: %u 字节剩余\n", freeStackBytes); // 可选的调试信息
    }

    // 也可以检查总的可用堆内存
    // uint32_t freeHeap = ESP.getFreeHeap();
    // Serial.printf("Free Heap: %u bytes\n", freeHeap);
    // if (freeHeap < SOME_HEAP_THRESHOLD) {
    //    Serial.println("警告: 可用堆内存低!");
    //    systemIsBusy = true; // 也可以根据堆内存设置繁忙状态
    // }
    
    vTaskDelay(pdMS_TO_TICKS(2000));  // 每2秒检查一次
  }
}


TaskHandle_t createSerialMonitorTask(BaseType_t coreID) {
  TaskHandle_t taskHandle = NULL;
  xTaskCreatePinnedToCore(
    serialMonitorTask,    // 任务函数
    "SerialMonitor",      // 任务名称
    4096,                 // 堆栈大小 (字节) - 保持4KB以防万一
    NULL,                 // 参数
    1,                    // 低优先级 (0是最低)
    &taskHandle,          // 任务句柄
    coreID                // 指定核心
  );
  if (taskHandle == NULL) {
      Serial.println("错误: 无法创建串口监控任务!");
  } else {
      Serial.println("串口监控任务已创建");
  }
  return taskHandle;
}

TaskHandle_t createSystemMonitorTask(BaseType_t coreID) {
  TaskHandle_t taskHandle = NULL;
  xTaskCreatePinnedToCore(
    systemMonitorTask,    // 任务函数
    "SysMonitor",         // 任务名称
    2048,                 // 堆栈大小 (字节) - 2KB应该足够
    NULL,                 // 参数
    2,                    // 中等优先级
    &taskHandle,          // 任务句柄 (如果需要可以保存)
    coreID                // 指定核心
  );
   if (taskHandle == NULL) {
      Serial.println("错误: 无法创建系统监控任务!");
  } else {
      Serial.println("系统监控任务已创建");
  }
  return taskHandle;
}