#include "ir_vision_processor.h"
#include "navigation.h"
#include "find_brightest.h"

// 引用外部变量
extern int targetColorIndex; // 来自navigation.h

// 确保可以访问颜色阈值常量
extern const int YELLOW_H_LOW;
extern const int YELLOW_H_HIGH;
extern const int YELLOW_S_LOW;
extern const int YELLOW_S_HIGH;
extern const int YELLOW_V_LOW;
extern const int YELLOW_V_HIGH;
extern const int RED1_H_LOW;
extern const int RED1_H_HIGH;
extern const int RED2_H_LOW;
extern const int RED2_H_HIGH;
extern const int RED_S_LOW;
extern const int RED_S_HIGH;
extern const int RED_V_LOW;
extern const int RED_V_HIGH;

// 全局实例
IRVisionProcessor* g_irVisionProcessor = nullptr;

// 构造函数实现
IRVisionProcessor::IRVisionProcessor(int width, int height) :
    frameWidth(width),
    frameHeight(height)
{
    // 初始化HSV颜色阈值 (从navigation.h中定义的值获取)
    redLower1 = cv::Scalar(RED1_H_LOW, RED_S_LOW, RED_V_LOW);
    redUpper1 = cv::Scalar(RED1_H_HIGH, RED_S_HIGH, RED_V_HIGH);
    redLower2 = cv::Scalar(RED2_H_LOW, RED_S_LOW, RED_V_LOW);
    redUpper2 = cv::Scalar(RED2_H_HIGH, RED_S_HIGH, RED_V_HIGH);
    yellowLower = cv::Scalar(YELLOW_H_LOW, YELLOW_S_LOW, YELLOW_V_LOW);
    yellowUpper = cv::Scalar(YELLOW_H_HIGH, YELLOW_S_HIGH, YELLOW_V_HIGH);
    
    Serial.println("IRVisionProcessor: 已初始化");
}

// 初始化函数实现
bool initIRVisionProcessor(int width, int height) {
    if (g_irVisionProcessor != nullptr) {
        delete g_irVisionProcessor;
    }
    
    g_irVisionProcessor = new IRVisionProcessor(width, height);
    if (g_irVisionProcessor == nullptr) {
        Serial.println("无法创建IR视觉处理器实例！");
        return false;
    }
    
    Serial.println("IR视觉处理器初始化成功");
    return true;
}

// 主处理函数实现
void IRVisionProcessor::processFrame(camera_fb_t *fb) {
    if (!fb || fb->len == 0) {
        Serial.println("无效的帧缓冲区");
        return;
    }
    
    // 1. 检测目标 (假设默认使用红色目标)
    bool isRedTarget = (targetColorIndex == 1); // 假设1表示红色
    cv::Point2f target = detectTarget(fb, isRedTarget);
    
    // 2. 检测门
    std::vector<cv::Rect> gates = detectGates(fb);
    
    // 3. 检测浮标 (S弯曲圆形浮标)
    std::vector<cv::Point2f> buoys = detectBuoys(fb);
    
    // 4. 导航控制
    navigate(target, gates, buoys);
}

// 目标检测函数实现 (颜色检测)
cv::Point2f IRVisionProcessor::detectTarget(camera_fb_t *fb, bool isRedTarget) {
    // 调用现有的颜色检测功能
    int targetX = -1, targetY = -1;
    bool found = detectColor(fb, targetX, targetY, isRedTarget);
    
    if (found) {
        return cv::Point2f(targetX, targetY);
    } else {
        return cv::Point2f(-1, -1); // 未检测到
    }
}

// 门检测函数实现 (基于霍夫线变换的概念)
std::vector<cv::Rect> IRVisionProcessor::detectGates(camera_fb_t *fb) {
    std::vector<cv::Rect> gates;
    
    // 在ESP32环境中，我们使用简化的逻辑，基于最亮点检测
    int gateX = -1, gateY = -1;
    int brightX = -1, brightY = -1;
    
    // 先获取最亮点
    find_brightest(fb->buf, fb->width, fb->height, brightX, brightY);
    
    // 然后尝试识别门
    if (detectGate(brightX, brightY, gateX, gateY)) {
        // 检测到光电门，创建一个代表门的矩形
        // 这里假设门的大小
        int gateWidth = 50;
        int gateHeight = 100;
        gates.push_back(cv::Rect(gateX - gateWidth/2, gateY - gateHeight/2, 
                                 gateWidth, gateHeight));
    }
    
    return gates;
}

// 浮标检测函数实现 (基于霍夫圆变换的概念)
std::vector<cv::Point2f> IRVisionProcessor::detectBuoys(camera_fb_t *fb) {
    std::vector<cv::Point2f> buoys;
    
    // 在ESP32环境中，我们可以使用颜色检测来近似浮标检测
    // 假设我们要检测黄色浮标 
    int buoyX = -1, buoyY = -1;
    bool isRedTarget = false; // 假设黄色不是红色
    
    if (detectColor(fb, buoyX, buoyY, isRedTarget)) {
        buoys.push_back(cv::Point2f(buoyX, buoyY));
    }
    
    return buoys;
}

// 导航控制函数实现
void IRVisionProcessor::navigate(const cv::Point2f& target, const std::vector<cv::Rect>& gates, 
                                const std::vector<cv::Point2f>& buoys) {
    // 默认控制信号
    int leftSpeed = 0;
    int rightSpeed = 0;
    
    // 导航策略：优先级 目标 > 门 > 浮标
    if (target.x != -1) {
        // 有目标，计算目标相对位置
        float error_x = target.x - frameWidth/2;
        
        // PID控制 (简化版，只用比例系数)
        float steer_angle = error_x * 0.1f;
        servo.setAngle(steer_angle);
        
        // 根据目标距离调整速度（假设y坐标越大表示越近）
        float targetSize = (target.y / frameHeight); // 0-1之间的值
        int baseSpeed = 150; // 基础速度
        
        if (targetSize > 0.8f) {
            // 目标非常近，降低速度准备撞击
            baseSpeed = 100;
        }
        
        // 计算左右电机的差速
        float speedDiff = abs(error_x) * 0.5f;
        if (error_x > 0) {
            // 目标在右侧，右转
            leftSpeed = baseSpeed;
            rightSpeed = baseSpeed - speedDiff;
        } else {
            // 目标在左侧，左转
            leftSpeed = baseSpeed - speedDiff;
            rightSpeed = baseSpeed;
        }
    } 
    else if (!gates.empty()) {
        // 有门，尝试穿过
        cv::Rect gate = gates[0]; // 取第一个门
        
        // 计算门中心相对位置
        float gateCenterX = gate.x + gate.width/2;
        float error_x = gateCenterX - frameWidth/2;
        
        // 设置舵机角度
        float steer_angle = error_x * 0.1f;
        servo.setAngle(steer_angle);
        
        // 通过门时保持中等速度
        int baseSpeed = 120;
        float speedDiff = abs(error_x) * 0.3f;
        
        if (error_x > 0) {
            leftSpeed = baseSpeed;
            rightSpeed = baseSpeed - speedDiff;
        } else {
            leftSpeed = baseSpeed - speedDiff;
            rightSpeed = baseSpeed;
        }
    }
    else if (!buoys.empty()) {
        // 有浮标，尝试绕行
        cv::Point2f buoy = buoys[0]; // 取第一个浮标
        
        // 计算浮标相对位置
        float error_x = buoy.x - frameWidth/2;
        
        // 设置舵机角度 (与前面相反，避开浮标)
        float steer_angle = -error_x * 0.15f;
        servo.setAngle(steer_angle);
        
        // 绕行浮标时使用较低速度
        int baseSpeed = 100;
        float speedDiff = abs(error_x) * 0.4f;
        
        if (error_x > 0) {
            // 浮标在右侧，向左转以避开
            leftSpeed = baseSpeed - speedDiff;
            rightSpeed = baseSpeed;
        } else {
            // 浮标在左侧，向右转以避开
            leftSpeed = baseSpeed;
            rightSpeed = baseSpeed - speedDiff;
        }
    }
    else {
        // 没有检测到任何目标，执行搜索模式
        // 缓慢转圈寻找目标
        leftSpeed = 80;
        rightSpeed = 40;
    }
    
    // 应用电机控制
    motor.setSpeed(leftSpeed, rightSpeed);
}

// 辅助函数实现
cv::Moments IRVisionProcessor::moments(const std::vector<cv::Point>& contour) {
    cv::Moments m;
    // 简化版的矩计算
    if (contour.empty()) return m;
    
    // 计算0阶矩（面积）和1阶矩
    for (const auto& pt : contour) {
        m.m00 += 1.0;
        m.m10 += pt.x;
        m.m01 += pt.y;
    }
    
    return m;
}

float IRVisionProcessor::contourArea(const std::vector<cv::Point>& contour) {
    // 简化版的轮廓面积计算
    return static_cast<float>(contour.size()); // 使用点数作为面积的近似
}

void IRVisionProcessor::findContours(camera_fb_t *fb, int threshold, 
                                    std::vector<std::vector<cv::Point>>& contours) {
    // 这是一个模拟函数，在ESP32环境中不实现完整的轮廓检测
    // 在实际的OpenCV实现中会使用cv::findContours
    
    Serial.println("轮廓检测: 在ESP32上模拟");
    
    // 为了接口完整性，添加一个假轮廓
    std::vector<cv::Point> fakeContour;
    for (int i = 0; i < 10; i++) {
        fakeContour.push_back(cv::Point(100 + i, 100 + i));
    }
    contours.push_back(fakeContour);
}

void IRVisionProcessor::HoughCircles(camera_fb_t *fb, std::vector<cv::Vec3f>& circles) {
    // 这是一个模拟函数，在ESP32环境中不实现完整的霍夫圆检测
    // 在实际的OpenCV实现中会使用cv::HoughCircles
    
    Serial.println("圆检测: 在ESP32上模拟");
    
    // 为了接口完整性，添加一个假圆
    cv::Vec3f fakeCircle;
    fakeCircle.val[0] = 100.0f; // x中心
    fakeCircle.val[1] = 100.0f; // y中心
    fakeCircle.val[2] = 30.0f;  // 半径
    circles.push_back(fakeCircle);
}

void IRVisionProcessor::HoughLinesP(camera_fb_t *fb, std::vector<cv::Vec4i>& lines) {
    // 这是一个模拟函数，在ESP32环境中不实现完整的霍夫线检测
    // 在实际的OpenCV实现中会使用cv::HoughLinesP
    
    Serial.println("线段检测: 在ESP32上模拟");
    
    // 为了接口完整性，添加一个假线段
    cv::Vec4i fakeLine;
    fakeLine.val[0] = 50;  // x1
    fakeLine.val[1] = 50;  // y1
    fakeLine.val[2] = 150; // x2
    fakeLine.val[3] = 150; // y2
    lines.push_back(fakeLine);
}