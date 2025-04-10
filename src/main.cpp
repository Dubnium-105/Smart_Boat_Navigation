#include <WiFi.h>
#include "esp_camera.h"
#include <ArduinoJson.h>
#include <vector>
#include <base64.h> // 添加Base64编码库
#include "img_converters.h" // 用于图像格式转换

// 函数前置声明
void setup_camera();
void setup_wifi();
void motor_control(uint8_t ch, int speed);
void process_command(String cmd);
void send_sensor_data();
void send_image();  // 修改为仅发送图像，不寻找最亮点
void process_image_for_coordinates();

// 网络配置
const char* networks[][2] = {
    {"room@407", "room@407"},
    {"motorola", "1145141919"},
    {"xbox", "12345678"}
};

WiFiClient espClient;
WiFiServer tcpServer(5000);
WiFiClient tcpClient;

// 硬件配置
#define MOTOR_A_IN1 38
#define MOTOR_A_IN2 39
#define MOTOR_B_IN3 40
#define MOTOR_B_IN4 41

// 摄像头配置
#define PWDN_GPIO_NUM     -1  // 无电源控制引脚
#define RESET_GPIO_NUM    -1  // 无硬件复位引脚
#define XCLK_GPIO_NUM     15  // 摄像头外部时钟
#define SIOD_GPIO_NUM     4   // I2C SDA
#define SIOC_GPIO_NUM     5   // I2C SCL
#define Y9_GPIO_NUM       16  // 数据线Y9
#define Y8_GPIO_NUM       17  // 数据线Y8
#define Y7_GPIO_NUM       18  // 数据线Y7
#define Y6_GPIO_NUM       12  // 数据线Y6
#define Y5_GPIO_NUM       10  // 数据线Y5
#define Y4_GPIO_NUM       8   // 数据线Y4
#define Y3_GPIO_NUM       9   // 数据线Y3
#define Y2_GPIO_NUM       11  // 数据线Y2
#define VSYNC_GPIO_NUM    6   // 垂直同步
#define HREF_GPIO_NUM     7   // 行同步
#define PCLK_GPIO_NUM     13  // 像素时钟

// 图像传输配置
#define IMAGE_QUALITY 30     // JPEG压缩质量 (10-63)，越低质量越差但尺寸越小
#define FRAME_SIZE FRAMESIZE_QQVGA // 图像尺寸 (QVGA=320x240, QQVGA=160x120)
#define MAX_FRAME_RATE 5     // 最大帧率限制

// 坐标传输配置
const unsigned long COORDINATES_INTERVAL = 100; // 坐标发送最小间隔(毫秒)
unsigned long last_coordinates_time = 0; // 上次发送坐标的时间

// 全局变量
unsigned long last_frame_time = 0;
bool auto_send_image = true;  // 是否自动发送图像
bool coordinates_only_mode = false;

// 添加新的全局变量
#define TCP_IMAGE_PORT 5001
#define TCP_CONTROL_PORT 5000
WiFiServer imageTcpServer(TCP_IMAGE_PORT);
WiFiClient imageTcpClient;

// 添加串口状态输出间隔
#define STATUS_INTERVAL 1000  // 1秒输出一次状态
unsigned long last_status_time = 0;

// 在全局变量区域添加
// 最亮点坐标
int brightest_x = 160;  // 默认值为图像中心
int brightest_y = 120;

// 定义通信协议
#define CMD_PING "PING"
#define CMD_MOTOR "MOTOR"
#define CMD_IMAGE "IMAGE"
#define CMD_STATUS "STATUS"

// 通信状态
struct CommStatus {
    bool wifi_connected;
    bool tcp_control_connected;
    bool tcp_image_connected;
    uint32_t last_heartbeat;
    uint32_t frame_count;
} comm_status;

// 设备状态
struct DeviceStatus {
    int brightest_x;
    int brightest_y;
    int brightness;
    uint32_t free_heap;
    float fps;
} device_status = {160, 120, 0, 0, 0};

// 初始化通信状态
void init_communication() {
    comm_status = {false, false, false, 0, 0};
    tcpServer.begin();
    imageTcpServer.begin();
    Serial.printf("TCP服务器启动在端口 %d (控制) 和 %d (图像)\n", 
                 TCP_CONTROL_PORT, TCP_IMAGE_PORT);
}

// 更新和发送状态
void update_status() {
    device_status.free_heap = ESP.getFreeHeap();
    device_status.fps = 1000.0f / (millis() - last_frame_time);
    
    StaticJsonDocument<200> doc;
    doc["heap"] = device_status.free_heap;
    doc["fps"] = device_status.fps;
    doc["x"] = device_status.brightest_x;
    doc["y"] = device_status.brightest_y;
    doc["clients"] = (tcpClient.connected() ? 1 : 0) + 
                    (imageTcpClient.connected() ? 1 : 0);
    
    String status;
    serializeJson(doc, status);
    if(tcpClient.connected()) {
        tcpClient.println(CMD_STATUS + String(" ") + status);
    }
}

// 处理图像发送
void handle_image_transmission() {
    if (!imageTcpClient.connected()) return;
    
    static uint32_t last_image_time = 0;
    uint32_t now = millis();
    if (now - last_image_time < 100) return; // 限制10fps
    
    last_image_time = now;
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) return;
    
    uint32_t size = fb->len;
    imageTcpClient.write((uint8_t*)&size, 4);
    size_t sent = imageTcpClient.write(fb->buf, fb->len);
    
    if (sent == fb->len) {
        comm_status.frame_count++;
    }
    
    esp_camera_fb_return(fb);
}

void send_status() {
    static char buf[200];
    int len = snprintf(buf, sizeof(buf), 
        "STATUS\t"
        "memory:%d\t"
        "wifi:%s\t"
        "fps:%.1f\t"
        "clients:%d\t"
        "coordinates:%d,%d\n",
        ESP.getFreeHeap(),
        WiFi.localIP().toString().c_str(), 
        1000.0f / (millis() - last_frame_time),
        (tcpClient.connected() ? 1 : 0) + (imageTcpClient.connected() ? 1 : 0),
        brightest_x, brightest_y
    );
    Serial.write(buf, len);
}

// 完整的摄像头初始化
void setup_camera() {
    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = Y2_GPIO_NUM;
    config.pin_d1 = Y3_GPIO_NUM;
    config.pin_d2 = Y4_GPIO_NUM;
    config.pin_d3 = Y5_GPIO_NUM;
    config.pin_d4 = Y6_GPIO_NUM;
    config.pin_d5 = Y7_GPIO_NUM;
    config.pin_d6 = Y8_GPIO_NUM;
    config.pin_d7 = Y9_GPIO_NUM;
    config.pin_xclk = XCLK_GPIO_NUM;
    config.pin_pclk = PCLK_GPIO_NUM;
    config.pin_vsync = VSYNC_GPIO_NUM;
    config.pin_href = HREF_GPIO_NUM;
    config.pin_sscb_sda = SIOD_GPIO_NUM;
    config.pin_sscb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn = PWDN_GPIO_NUM;
    config.pin_reset = RESET_GPIO_NUM;
    config.xclk_freq_hz = 40000000;
    config.pixel_format = PIXFORMAT_JPEG; // 直接使用JPEG格式
    config.frame_size = FRAME_SIZE;
    config.jpeg_quality = IMAGE_QUALITY;
    config.fb_count = 2; // 使用2个帧缓冲区提高性能
  
    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
      Serial.printf("Camera init failed with error 0x%x", err);
      ESP.restart();
    }
    
    // 获取传感器对象并设置参数
    sensor_t * s = esp_camera_sensor_get();
    if (s) {
        s->set_brightness(s, 0);     // -2 to 2
        s->set_contrast(s, 0);       // -2 to 2
        s->set_saturation(s, -1);    // -2 to 2 (降低饱和度有助于检测白色)
        s->set_special_effect(s, 0); // 0 = no effect
        s->set_whitebal(s, 1);       // 1 = enable
        s->set_awb_gain(s, 1);       // 1 = enable
        s->set_wb_mode(s, 0);        // 0 = auto
        s->set_exposure_ctrl(s, 1);  // 1 = enable
        s->set_aec2(s, 0);           // 0 = disable
        s->set_gain_ctrl(s, 1);      // 1 = enable
        s->set_agc_gain(s, 0);       // 0 = gain
        s->set_gainceiling(s, (gainceiling_t)0); // 0 = 2x
        s->set_bpc(s, 1);            // 1 = enable
        s->set_wpc(s, 1);            // 1 = enable
        s->set_raw_gma(s, 1);        // 1 = enable
        s->set_lenc(s, 1);           // 1 = enable
        s->set_hmirror(s, 0);        // 0 = disable
        s->set_vflip(s, 0);          // 0 = disable
        s->set_dcw(s, 1);            // 1 = enable
        
        // 优化白色检测的设置
        s->set_aec_value(s, 200);    // 适中的曝光值，避免过曝
        s->set_ae_level(s, 0);       // 自动曝光水平
    }
}

// 图像处理和发送相关函数 - 使用Base64编码
void send_image() {
    // 帧率控制
    unsigned long current_time = millis();
    if (current_time - last_frame_time < (1000 / MAX_FRAME_RATE)) {
        return; // 限制帧率
    }
    last_frame_time = current_time;
    
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
        Serial.println("获取图像失败");
        return;
    }
    
    // 如果TCP客户端已连接，发送图像
    if (imageTcpClient && imageTcpClient.connected()) {
        // 发送图像大小信息
        uint32_t size = fb->len;
        imageTcpClient.write((uint8_t*)&size, 4);
        
        // 发送图像数据
        imageTcpClient.write(fb->buf, fb->len);
    }
    
    // 释放图像缓冲区
    esp_camera_fb_return(fb);
}

void setup_wifi() {
    for(int i=0; i<3; i++){
        const char* ssid = networks[i][0];
        const char* pass = networks[i][1];
        
        WiFi.begin(ssid, pass);
        Serial.printf("Trying %s...", ssid);
        
        int retries = 0;
        while(WiFi.status() != WL_CONNECTED && retries < 15){
          delay(1000);
          Serial.print(".");
          retries++;
        }
        
        if(WiFi.status() == WL_CONNECTED){
          Serial.printf("\nConnected to %s\n", ssid);
          Serial.print("IP: ");
          Serial.println(WiFi.localIP());
          comm_status.wifi_connected = true;
          return;
        }
        Serial.println("\nFailed");
    }
    Serial.println("All networks failed!");
    comm_status.wifi_connected = false;
}

void setup_motors() {
  ledcSetup(0, 5000, 8);
  ledcAttachPin(MOTOR_A_IN1, 0);
  ledcSetup(1, 5000, 8);
  ledcAttachPin(MOTOR_A_IN2, 1);
  ledcSetup(2, 5000, 8);
  ledcAttachPin(MOTOR_B_IN3, 2);
  ledcSetup(3, 5000, 8);
  ledcAttachPin(MOTOR_B_IN4, 3);
}

void process_command(String cmd) {
    if (cmd == "PING") {
        tcpClient.print("PONG\n");
        return;
    }
    if(cmd.startsWith("MOTOR")){
      int firstComma = cmd.indexOf(',');
      int secondComma = cmd.indexOf(',', firstComma+1);
      
      int left = cmd.substring(firstComma+1, secondComma).toInt();
      int right = cmd.substring(secondComma+1).toInt();
      
      motor_control(0, left);
      motor_control(1, right);
      
      // 回传确认
      tcpClient.print("ACK,MOTOR," + String(left) + "," + String(right) + "\n");
    }
}

void motor_control(uint8_t motor, int speed) {
  speed = constrain(speed, -100, 100);
  
  if(motor == 0){ // MOTOR_A
    if(speed > 0){
      ledcWrite(0, map(speed, 0,100, 0,255));
      ledcWrite(1, 0);
    } else {
      ledcWrite(0, 0);
      ledcWrite(1, map(abs(speed),0,100,0,255));
    }
  } else { // MOTOR_B
    if(speed > 0){
      ledcWrite(2, map(speed,0,100,0,255));
      ledcWrite(3, 0);
    } else {
      ledcWrite(2, 0);
      ledcWrite(3, map(abs(speed),0,100,0,255));
    }
  }
}

void process_image_for_coordinates() {
  // 检查时间间隔
  unsigned long current_time = millis();
  if (current_time - last_coordinates_time < COORDINATES_INTERVAL) {
    return;
  }
  last_coordinates_time = current_time;

  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Camera capture failed");
    return;
  }

  // 使用更精确的坐标计算
  uint8_t *buf = fb->buf;
  size_t len = fb->len;
  int width = fb->width;
  int height = fb->height;
  int maxBrightness = 0;
  int maxX = width / 2;
  int maxY = height / 2;
  
  if (fb->format == PIXFORMAT_JPEG) {
    // 将JPEG转换为RGB565格式进行处理
    uint8_t * rgb_buf = (uint8_t *)malloc(width * height * 3);
    if (!rgb_buf) {
      Serial.println("内存分配失败");
      esp_camera_fb_return(fb);
      return;
    }

    if (fmt2rgb888(fb->buf, fb->len, PIXFORMAT_JPEG, rgb_buf) == true) {
      // 遍历RGB数据寻找最亮点
      for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
          int idx = (y * width + x) * 3;
          // 计算亮度：使用RGB加权平均
          int brightness = (rgb_buf[idx] * 30 + rgb_buf[idx + 1] * 59 + rgb_buf[idx + 2] * 11) / 100;
          if (brightness > maxBrightness) {
            maxBrightness = brightness;
            maxX = x;
            maxY = y;
          }
        }
      }
      // 更新全局变量
      brightest_x = maxX;
      brightest_y = maxY;
    }
    free(rgb_buf);
  }

  // 创建JSON对象存储坐标和亮度信息
  StaticJsonDocument<200> doc;
  doc["x"] = maxX;
  doc["y"] = maxY;
  doc["brightness"] = maxBrightness;
  doc["width"] = width;
  doc["height"] = height;
  doc["timestamp"] = millis();
  
  char buffer[200];
  serializeJson(doc, buffer);
  
  // 通过TCP发送坐标数据
  if (tcpClient.connected()) {
      tcpClient.println(buffer);
  }
  
  // 通过串口发送坐标数据
  Serial.printf("发送坐标：x=%d, y=%d, 亮度=%d\n", maxX, maxY, maxBrightness);
  
  esp_camera_fb_return(fb);
}

void send_sensor_data() {
    // 其余代码保持不变
    DynamicJsonDocument doc(512);
    doc["ax"] = 0;
    doc["ay"] = 0;
    doc["az"] = 0;
    doc["gx"] = 0;
    doc["gy"] = 0;
    doc["gz"] = 0;
    
    String jsonStr;
    serializeJson(doc, jsonStr);
    
    if(tcpClient.connected()) {
        tcpClient.println(jsonStr);
    }
    
    // 如果启用了自动发送图像，则发送图像
    if (auto_send_image) {
        send_image();
    }
    
    // 如果启用了坐标模式，在这里获取并添加最亮点信息
    if (coordinates_only_mode) {
        process_image_for_coordinates();
    }
}

void setup() {
    Serial.begin(115200);
    Serial.println("\n\n=== ESP32-CAM 启动 ===");
    delay(1000);
    
    randomSeed(analogRead(0)); // 初始化随机数生成器
    
    Serial.println("初始化摄像头...");
    setup_camera();
    Serial.println("摄像头初始化完成");
    
    Serial.println("连接WiFi...");
    setup_wifi();
    if (WiFi.status() == WL_CONNECTED) {
        Serial.printf("WiFi 已连接，IP 地址: %s\n", WiFi.localIP().toString().c_str());
    } else {
        Serial.println("WiFi 连接失败，请检查网络配置。");
    }
    
    Serial.println("启动TCP服务器...");
    tcpServer.begin();
    Serial.println("控制端口 5000 已启动");
    
    Serial.println("启动TCP图像服务器...");
    imageTcpServer.begin();
    Serial.println("图像端口 5001 已启动");
    
    Serial.println("设置完成，开始运行");
    init_communication();
}

// 添加TCP图像发送函数
void send_image_tcp() {
    if (!imageTcpClient || !imageTcpClient.connected()) {
        return;
    }

    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
        Serial.println("获取图像失败");
        return;
    }

    // 发送图像大小信息
    uint32_t size = fb->len;
    imageTcpClient.write((uint8_t*)&size, 4);
    
    // 发送图像数据
    imageTcpClient.write(fb->buf, fb->len);
    
    esp_camera_fb_return(fb);
}

void loop() {
    static unsigned long lastMsg = 0;
    static unsigned long lastReconnectAttempt = 0;
    unsigned long now = millis();
    
    // 检查WiFi连接
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("WiFi连接断开，尝试重连");
      setup_wifi();
      return;
    }
    
    // 处理TCP客户端
    if (tcpClient && !tcpClient.connected()) {
        Serial.println("TCP客户端断开连接");
        tcpClient.stop();
        delay(100);
    }

    // 接受新连接
    if (!tcpClient || !tcpClient.connected()) {
      WiFiClient newClient = tcpServer.available();
      if (newClient) {
          tcpClient = newClient;
          tcpClient.setTimeout(100);
          Serial.println("新TCP客户端已连接");
      }
    } else {
      // 处理TCP客户端数据
      if (tcpClient.available()) {
        String cmd = tcpClient.readStringUntil('\n');
        if (cmd.length() > 0) {
          Serial.printf("收到TCP命令: %s\n", cmd.c_str());
          process_command(cmd);
        }
      }
    }
    
    // 处理TCP图像客户端连接
    if (!imageTcpClient || !imageTcpClient.connected()) {
        imageTcpClient = imageTcpServer.available();
        if (imageTcpClient) {
            Serial.println("新的TCP图像客户端已连接");
            imageTcpClient.setTimeout(1);
        }
    }
    
    // 如果TCP图像客户端已连接，发送图像
    if (imageTcpClient && imageTcpClient.connected()) {
        static unsigned long lastImageTime = 0;
        unsigned long now = millis();
        if (now - lastImageTime > 100) { // 10fps
            lastImageTime = now;
            send_image_tcp();
        }
    }
    
    // 定期发送传感器数据
    if (now - lastMsg > 100) {  // 减少间隔到100毫秒，提高响应速度
      lastMsg = now;
      send_sensor_data();
    }
    
    // 定期发送状态
    if (now - last_status_time > STATUS_INTERVAL) {
        last_status_time = now;
        send_status();
    }
    
    // 更新状态
    static uint32_t last_status_time = 0;
    if (millis() - last_status_time > STATUS_INTERVAL) {
        update_status();
        last_status_time = millis();
    }
}