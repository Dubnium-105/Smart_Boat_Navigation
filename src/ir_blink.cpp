// 用IRremote库控制48号引脚的IR小灯以10ms闪烁
#include <Arduino.h>
#include "ir_blink.h"

#define IR_LED_PIN 48

void ir_blink_init() {
    pinMode(IR_LED_PIN, OUTPUT);
}

void ir_blink_task() {
    static unsigned long lastToggle = 0;
    static bool ledState = false;
    unsigned long now = millis();
    if (now - lastToggle >= 10) {
        ledState = !ledState;
        digitalWrite(IR_LED_PIN, ledState ? HIGH : LOW);
        lastToggle = now;
    }
}
