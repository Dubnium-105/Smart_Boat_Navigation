#include <Arduino.h>
#include "esp_camera.h"
#include <WiFi.h>

// 硬件配置
#define MOTOR_A_IN1 38
#define MOTOR_A_IN2 39
#define MOTOR_B_IN3 40
#define MOTOR_B_IN4 41

// 摄像头配置
#define PWDN_GPIO_NUM     -1
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM     15
#define SIOD_GPIO_NUM     4
#define SIOC_GPIO_NUM     5
#define Y9_GPIO_NUM       16
#define Y8_GPIO_NUM       17
#define Y7_GPIO_NUM       18
#define Y6_GPIO_NUM       12
#define Y5_GPIO_NUM       10
#define Y4_GPIO_NUM       8
#define Y3_GPIO_NUM       9
#define Y2_GPIO_NUM       11
#define VSYNC_GPIO_NUM    6
#define HREF_GPIO_NUM     7
#define PCLK_GPIO_NUM     13

#define IMAGE_QUALITY 30
#define FRAME_SIZE FRAMESIZE_QQVGA

// WiFi配置
const char* networks[][2] = {
    {"room@407", "room@407"},
    {"motorola", "1145141919"},
    {"xbox", "12345678"}
};

WiFiServer imageServer(5001);  // 图像传输服务器端口
WiFiClient imageClient;

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
    config.pixel_format = PIXFORMAT_JPEG;
    config.frame_size = FRAME_SIZE;
    config.jpeg_quality = IMAGE_QUALITY;
    config.fb_count = 2;

    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        Serial.printf("Camera init failed with error 0x%x", err);
        ESP.restart();
    }

    sensor_t * s = esp_camera_sensor_get();
    if (s) {
        s->set_brightness(s, 0);
        s->set_contrast(s, 0);
        s->set_saturation(s, -1);
        s->set_special_effect(s, 0);
        s->set_whitebal(s, 1);
        s->set_awb_gain(s, 1);
        s->set_wb_mode(s, 0);
        s->set_exposure_ctrl(s, 1);
        s->set_aec2(s, 0);
        s->set_gain_ctrl(s, 1);
        s->set_agc_gain(s, 0);
        s->set_gainceiling(s, (gainceiling_t)0);
        s->set_bpc(s, 1);
        s->set_wpc(s, 1);
        s->set_raw_gma(s, 1);
        s->set_lenc(s, 1);
        s->set_hmirror(s, 0);
        s->set_vflip(s, 0);
        s->set_dcw(s, 1);
        s->set_aec_value(s, 200);
        s->set_ae_level(s, 0);
    }
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

void motor_control(uint8_t motor, int speed) {
    speed = constrain(speed, -100, 100);
    
    if(motor == 0) { // MOTOR_A
        if(speed > 0) {
            ledcWrite(0, map(speed, 0, 100, 0, 255));
            ledcWrite(1, 0);
        } else {
            ledcWrite(0, 0);
            ledcWrite(1, map(abs(speed), 0, 100, 0, 255));
        }
    } else { // MOTOR_B
        if(speed > 0) {
            ledcWrite(2, map(speed, 0, 100, 0, 255));
            ledcWrite(3, 0);
        } else {
            ledcWrite(2, 0);
            ledcWrite(3, map(abs(speed), 0, 100, 0, 255));
        }
    }
}

void setup_wifi() {
    for(int i=0; i<3; i++){
        const char* ssid = networks[i][0];
        const char* pass = networks[i][1];
        
        WiFi.begin(ssid, pass);
        Serial.printf("正在连接 %s...", ssid);
        
        int retries = 0;
        while(WiFi.status() != WL_CONNECTED && retries < 15){
            delay(1000);
            Serial.print(".");
            retries++;
        }
        
        if(WiFi.status() == WL_CONNECTED){
            Serial.printf("\n已连接到 %s\n", ssid);
            Serial.print("IP地址: ");
            Serial.println(WiFi.localIP());
            imageServer.begin();
            Serial.println("图像服务器已启动在端口5001");
            return;
        }
        Serial.println("\n连接失败");
    }
    Serial.println("所有网络连接失败!");
}

void send_camera_frame() {
    camera_fb_t *fb = esp_camera_fb_get();
    if(!fb) {
        Serial.println("获取图像失败");
        return;
    }

    if(imageClient && imageClient.connected()) {
        // 发送图像大小(4字节)
        uint32_t size = fb->len;
        imageClient.write((uint8_t*)&size, 4);
        
        // 发送图像数据
        imageClient.write(fb->buf, fb->len);
    }
    
    esp_camera_fb_return(fb);
}

void setup() {
    Serial.begin(115200);
    Serial.println("\n=== ESP32-CAM 启动 ===");
    
    setup_camera();
    setup_motors();
    setup_wifi();  // 添加WiFi初始化
    
    Serial.println("初始化完成");
}

void loop() {
    // 检查WiFi连接状态
    if(WiFi.status() != WL_CONNECTED) {
        Serial.println("WiFi断开,尝试重连...");
        setup_wifi();
        return;
    }
    
    // 检查新的客户端连接
    if(!imageClient || !imageClient.connected()) {
        imageClient = imageServer.available();
        if(imageClient) {
            Serial.println("新的图像客户端已连接");
        }
    }
    
    // 发送摄像头画面
    if(imageClient && imageClient.connected()) {
        send_camera_frame();
        delay(100); // 限制帧率约10fps
    }
    
    // 此处可以添加测试代码
    delay(10);
}