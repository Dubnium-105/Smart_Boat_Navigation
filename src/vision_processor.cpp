#include "vision_processor.h"
#include "find_brightest.h"

// 全局视觉处理器实例
VisionProcessor* g_visionProcessor = nullptr;

// 初始化视觉处理器
bool initVisionProcessor(int width, int height) {
    if (g_visionProcessor != nullptr) {
        delete g_visionProcessor;
    }
    
    g_visionProcessor = new VisionProcessor(width, height);
    if (g_visionProcessor == nullptr) {
        Serial.println("无法创建视觉处理器实例！");
        return false;
    }
    
    Serial.println("视觉处理器初始化成功");
    return true;
}