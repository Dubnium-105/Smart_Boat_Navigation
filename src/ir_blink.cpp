// 用IRremote库控制48号引脚的IR小灯以10ms闪烁
#include <Arduino.h>
#include <IRremote.h>

#define IR_LED_PIN 48

void setup() {
    IrSender.begin(IR_LED_PIN, ENABLE_LED_FEEDBACK); // 初始化IR发送器
}

void loop() {
    digitalWrite(IR_LED_PIN, HIGH); // 点亮IR小灯
    delay(10);                      // 延时10ms
    digitalWrite(IR_LED_PIN, LOW);  // 熄灭IR小灯
    delay(10);                      // 延时10ms
}
