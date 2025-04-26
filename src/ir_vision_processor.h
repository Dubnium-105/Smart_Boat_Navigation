#ifndef IR_VISION_PROCESSOR_H
#define IR_VISION_PROCESSOR_H

#include <Arduino.h>
#include "esp_camera.h"
#include "motor_control.h"
#include <vector>    // 添加标准库 vector 头文件
#include <algorithm> // 添加算法头文件
#include <cmath>     // 添加数学函数头文件

// OpenCV风格数据结构定义 (ESP32环境下的简化版本)
namespace cv {
    struct Point {
        int x, y;
        Point() : x(0), y(0) {}
        Point(int _x, int _y) : x(_x), y(_y) {}
    };
    
    struct Point2f {
        float x, y;
        Point2f() : x(0.0f), y(0.0f) {}
        Point2f(float _x, float _y) : x(_x), y(_y) {}
    };
    
    struct Rect {
        int x, y, width, height;
        Rect() : x(0), y(0), width(0), height(0) {}
        Rect(int _x, int _y, int _w, int _h) : x(_x), y(_y), width(_w), height(_h) {}
    };
    
    struct Scalar {
        int val[4];
        Scalar() { val[0] = 0; val[1] = 0; val[2] = 0; val[3] = 0; } // 添加默认构造函数
        Scalar(int v0, int v1, int v2) { val[0] = v0; val[1] = v1; val[2] = v2; val[3] = 0; }
    };
    
    struct Moments {
        double m00, m10, m01;
        Moments() : m00(0), m10(0), m01(0) {}
    };
    
    // Vec类型定义
    template<typename T, int n> struct Vec {
        T val[n];
    };
    
    typedef Vec<int, 4> Vec4i;
    typedef Vec<float, 3> Vec3f;
    
    // 轮廓定义为点集合
    typedef std::vector<Point> Contour;
}

// 硬件控制接口适配器 (将提供的框架与我们的硬件接口对接)
class MotorController {
public:
    void setSpeed(float left, float right) { 
        // 调用实际的电机控制函数 
        motor_control(0, (int)left);  // 假设0是左电机
        motor_control(1, (int)right); // 假设1是右电机
    }
};

class ServoController {
public:
    void setAngle(float angle) { 
        // 如果有舵机，调用舵机控制函数
        // servo_set_angle(angle);
        Serial.printf("设置舵机角度: %.1f\n", angle);
    }
};

// 增强型视觉处理器类 (基于提供的OpenCV框架)
class IRVisionProcessor {
private:
    MotorController motor;
    ServoController servo;
    
    // 图像尺寸
    int frameWidth;
    int frameHeight;
    
    // 颜色阈值 (使用navigation.h中定义的值但用OpenCV风格封装)
    cv::Scalar redLower1; // 红色HSV阈值1
    cv::Scalar redUpper1;
    cv::Scalar redLower2; // 红色HSV阈值2
    cv::Scalar redUpper2;
    cv::Scalar yellowLower; // 黄色HSV阈值
    cv::Scalar yellowUpper;

public:
    IRVisionProcessor(int width, int height);
    
    // 主处理函数
    void processFrame(camera_fb_t *fb);
    
    // 检测逻辑
    cv::Point2f detectTarget(camera_fb_t *fb, bool isRedTarget);
    std::vector<cv::Rect> detectGates(camera_fb_t *fb);
    std::vector<cv::Point2f> detectBuoys(camera_fb_t *fb);
    
    // 导航控制
    void navigate(const cv::Point2f& target, const std::vector<cv::Rect>& gates, 
                 const std::vector<cv::Point2f>& buoys);
    
    // 辅助函数                 
    cv::Moments moments(const std::vector<cv::Point>& contour);
    float contourArea(const std::vector<cv::Point>& contour);
    void findContours(camera_fb_t *fb, int threshold, 
                     std::vector<std::vector<cv::Point>>& contours);
    void HoughCircles(camera_fb_t *fb, std::vector<cv::Vec3f>& circles);
    void HoughLinesP(camera_fb_t *fb, std::vector<cv::Vec4i>& lines);
};

// 全局变量
extern IRVisionProcessor* g_irVisionProcessor;

// 初始化函数
bool initIRVisionProcessor(int width, int height);

#endif // IR_VISION_PROCESSOR_H