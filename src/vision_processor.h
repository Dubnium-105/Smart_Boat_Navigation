#ifndef VISION_PROCESSOR_H
#define VISION_PROCESSOR_H

#include <Arduino.h>
#include "esp_camera.h"
#include "motor_control.h"
#include "navigation.h"
#include <vector>  // 添加标准库向量支持

// 前向声明，防止循环依赖
extern SemaphoreHandle_t frameAccessMutex;
extern volatile int sharedBrightX;
extern volatile int sharedBrightY;
extern volatile bool newBrightPointAvailable;
extern int targetColorIndex; // 来自navigation.h

// 前向声明find_brightest函数
extern "C" {
    void find_brightest(const unsigned char* gray, int width, int height, int& out_x, int& out_y);
    void print_ascii_frame(int width, int height, int bx, int by);
}

/**
 * @brief 视觉处理器类，用于处理摄像头图像并提取有用信息
 * 
 * 该类提供基于原始图像数据的高级视觉处理功能：
 * - 目标检测（颜色检测、形状检测）
 * - 航线规划
 * - 障碍物检测
 */
class VisionProcessor {
private:
    // 颜色阈值 (使用navigation.h中定义的值)
    
    // 图像处理参数
    int frameWidth;
    int frameHeight;
    
    // 检测结果缓存
    struct DetectionResult {
        bool targetFound;
        int targetX;
        int targetY;
        float targetArea;
        bool gateDetected;
        int gateX;
        int gateY;
        bool obstacleDetected;
        int obstacleX;
        int obstacleY;
    } lastDetection;

public:
    /**
     * @brief 构造函数
     * 
     * @param width 图像宽度
     * @param height 图像高度
     */
    VisionProcessor(int width, int height) : 
        frameWidth(width), 
        frameHeight(height) {
        
        // 初始化检测结果
        lastDetection.targetFound = false;
        lastDetection.targetX = -1;
        lastDetection.targetY = -1;
        lastDetection.targetArea = 0;
        lastDetection.gateDetected = false;
        lastDetection.gateX = -1;
        lastDetection.gateY = -1;
        lastDetection.obstacleDetected = false;
        lastDetection.obstacleX = -1;
        lastDetection.obstacleY = -1;
    }
    
    /**
     * @brief 处理一帧图像并提取特征
     * 
     * @param fb 摄像头帧缓冲区
     * @return true 如果处理成功
     */
    bool processFrame(camera_fb_t *fb) {
        if (!fb || fb->len == 0) {
            Serial.println("无效的帧缓冲区");
            return false;
        }
        
        // 处理帧缓冲区
        // 1. 基本的最亮点查找 (使用现有find_brightest函数)
        int bx = -1, by = -1;
        bool brightPointFound = findBrightestPoint(fb, bx, by);
        
        // 2. 目标颜色检测 (根据当前任务阶段)
        bool targetIsRed = (targetColorIndex == 1); // 假设1表示红色
        int colorX = -1, colorY = -1;
        bool colorFound = findColorTarget(fb, colorX, colorY, targetIsRed);
        
        // 3. 门和障碍物检测
        int gateX = -1, gateY = -1;
        bool gateFound = findGate(fb, bx, by, gateX, gateY);
        
        int obstacleX = -1, obstacleY = -1;
        bool obstacleFound = findObstacles(fb, obstacleX, obstacleY);
        
        // 更新检测结果
        lastDetection.targetFound = colorFound;
        lastDetection.targetX = colorX;
        lastDetection.targetY = colorY;
        lastDetection.gateDetected = gateFound;
        lastDetection.gateX = gateX;
        lastDetection.gateY = gateY;
        lastDetection.obstacleDetected = obstacleFound;
        lastDetection.obstacleX = obstacleX;
        lastDetection.obstacleY = obstacleY;
        
        return true;
    }
    
    /**
     * @brief 使用现有的find_brightest函数查找最亮点
     * 
     * @param fb 摄像头帧缓冲区
     * @param outX 输出参数，最亮点的x坐标
     * @param outY 输出参数，最亮点的y坐标
     * @return true 如果找到了最亮点
     */
    bool findBrightestPoint(camera_fb_t *fb, int &outX, int &outY) {
        // 调用现有的find_brightest函数
        if (fb->format == PIXFORMAT_GRAYSCALE) {
            // 尝试获取互斥锁
            if (xSemaphoreTake(frameAccessMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                // 直接调用正确声明的函数
                find_brightest(fb->buf, fb->width, fb->height, outX, outY);
                
                // 更新共享变量
                sharedBrightX = outX;
                sharedBrightY = outY;
                newBrightPointAvailable = true;
                
                xSemaphoreGive(frameAccessMutex);
                return (outX >= 0 && outY >= 0);
            }
        }
        return false;
    }
    
    /**
     * @brief 查找特定颜色的目标
     * 
     * @param fb 摄像头帧缓冲区
     * @param outX 输出参数，目标的x坐标
     * @param outY 输出参数，目标的y坐标
     * @param isRedTarget 是否查找红色目标
     * @return true 如果找到了目标
     */
    bool findColorTarget(camera_fb_t *fb, int &outX, int &outY, bool isRedTarget) {
        // 调用现有的detectColor函数
        return detectColor(fb, outX, outY, isRedTarget);
    }
    
    /**
     * @brief 查找门
     * 
     * @param fb 摄像头帧缓冲区
     * @param brightX 最亮点的x坐标
     * @param brightY 最亮点的y坐标
     * @param outX 输出参数，门的x坐标
     * @param outY 输出参数，门的y坐标
     * @return true 如果找到了门
     */
    bool findGate(camera_fb_t *fb, int brightX, int brightY, int &outX, int &outY) {
        // 调用现有的detectGate函数
        return detectGate(brightX, brightY, outX, outY);
    }
    
    /**
     * @brief 查找障碍物
     * 
     * @param fb 摄像头帧缓冲区
     * @param outX 输出参数，障碍物的x坐标
     * @param outY 输出参数，障碍物的y坐标
     * @return true 如果找到了障碍物
     */
    bool findObstacles(camera_fb_t *fb, int &outX, int &outY) {
        // 调用现有的detectObstacles函数
        return detectObstacles(fb, outX, outY);
    }
    
    /**
     * @brief 根据视觉处理结果计算控制信号
     * 
     * @param leftSpeed 输出参数，左电机速度
     * @param rightSpeed 输出参数，右电机速度
     */
    void calculateControlSignals(int &leftSpeed, int &rightSpeed) {
        // 根据当前任务状态和检测结果计算控制信号
        
        // 默认控制信号
        leftSpeed = 0;
        rightSpeed = 0;
        
        // 根据检测结果计算控制信号
        if (lastDetection.targetFound) {
            // 目标跟踪逻辑
            calculateSteering(
                lastDetection.targetX, 
                lastDetection.targetY, 
                frameWidth, 
                frameHeight, 
                leftSpeed, 
                rightSpeed
            );
        } else if (lastDetection.gateDetected) {
            // 门通过逻辑
            calculateSteering(
                lastDetection.gateX, 
                lastDetection.gateY, 
                frameWidth, 
                frameHeight, 
                leftSpeed, 
                rightSpeed
            );
        } else if (lastDetection.obstacleDetected) {
            // 障碍物避让逻辑
            // ...同样使用现有的转向逻辑
        } else {
            // 默认行为：缓慢前进并旋转
            leftSpeed = 100;
            rightSpeed = 80;
        }
    }
    
    /**
     * @brief 获取最后一次检测结果
     * 
     * @return const DetectionResult& 检测结果
     */
    const DetectionResult& getLastDetection() const {
        return lastDetection;
    }
};

// 创建全局视觉处理器实例
extern VisionProcessor* g_visionProcessor;

// 初始化视觉处理器
bool initVisionProcessor(int width, int height);

#endif // VISION_PROCESSOR_H