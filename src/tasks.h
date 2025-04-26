#ifndef TASKS_H
#define TASKS_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

// 外部声明共享变量和互斥锁 (在main.cpp中定义)
extern volatile int sharedBrightX;
extern volatile int sharedBrightY;
extern volatile bool newBrightPointAvailable;
extern volatile bool systemIsBusy;
extern SemaphoreHandle_t frameAccessMutex;

// 外部声明流媒体相关变量 (在stream.cpp中定义)
extern float currentFPS;

// 外部声明ASCII帧打印函数 (在find_brightest.cpp中定义)
// 使用与find_brightest.h相同的链接属性
#ifdef __cplusplus
extern "C" {
#endif
extern void print_ascii_frame(int width, int height, int bx, int by);
#ifdef __cplusplus
}
#endif

/**
 * @brief 创建并启动串口监控任务。
 * 
 * @param coreID 要将任务固定到的CPU核心 (0 或 1)。
 * @return TaskHandle_t 创建的任务句柄，如果失败则返回NULL。
 */
TaskHandle_t createSerialMonitorTask(BaseType_t coreID);

/**
 * @brief 创建并启动系统健康监控任务。
 * 
 * @param coreID 要将任务固定到的CPU核心 (0 或 1)。
 * @return TaskHandle_t 创建的任务句柄，如果失败则返回NULL。
 */
TaskHandle_t createSystemMonitorTask(BaseType_t coreID);

#endif // TASKS_H