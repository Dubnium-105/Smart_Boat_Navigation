# 附录D — 源文件解析

本文档旨在对无人船舶物联网远程控制系统的关键源文件进行逐一解析，阐明其在系统中的功能、核心逻辑与依赖关系，为代码审查、二次开发与维护提供详尽参考。

## 1. 船舶终端（Smart_Boat_Navigation/）

### 1.1 项目配置文件

- **`platformio.ini`**
  - **功能**：作为 PlatformIO 项目的核心配置文件，声明了项目所使用的硬件平台（`esp32-s3-devkitm-1`）、软件框架（`arduino`）、依赖库（如 `PubSubClient`, `ArduinoJson`）以及编译、烧录和监控的相关参数（如 `upload_port`, `monitor_speed`）。
  - **核心逻辑**：定义了构建环境，确保了代码在不同开发机器上的一致性。

### 1.2 头文件 (include/)

- **`config.h`**
  - **功能**：提供一个集中的用户配置入口，所有需要用户自定义的参数（如Wi-Fi凭据、MQTT服务器地址、船舶名称等）都存放在此，方便修改与管理。
  - **核心逻辑**：通过 `const` 常量定义了系统各模块所需的配置参数。

- **`motor_control.h`**
  - **功能**：声明电机控制模块的公共接口、引脚定义与常量。
  - **核心逻辑**：提供 `setup_motors()` 初始化函数和 `motor_control()` 核心控制函数的原型，将具体的硬件引脚（如 AIN1, AIN2）与逻辑功能解耦。

- **`mqtt_manager.h`**
  - **功能**：声明 MQTT 管理模块的接口、回调函数、状态变量及相关主题。
  - **核心逻辑**：定义了 `setupMQTT()`、`mqtt_reconnect()`、`publishStatusInfo()` 等核心函数的原型，并声明了用于处理消息的 `callback()` 函数。

- **`wifi_manager.h`**
  - **功能**：声明 Wi-Fi 网络管理模块的接口。
  - **核心逻辑**：提供 `connectWiFi()` 和 `wifiAutoReconnect()` 函数原型，封装了网络连接与断线重连的逻辑。

### 1.3 源文件 (src/)

- **`main.cpp`**
  - **功能**：船舶终端程序的唯一入口点，负责系统的整体初始化与主循环逻辑。
  - **核心逻辑**：在 `setup()` 函数中，按顺序调用各模块的初始化函数（`setup_motors`, `connectWiFi`, `setupMQTT`）。在 `loop()` 函数中，持续处理 MQTT 消息 (`mqttClient.loop()`)、执行网络重连检查 (`wifiAutoReconnect`)，并将全局的速度变量应用到电机。

- **`motor_control.cpp`**
  - **功能**：实现电机控制的具体逻辑。
  - **核心逻辑**：`setup_motors()` 函数负责初始化 ESP32 的 LEDC PWM 通道，设置频率和分辨率。`motor_control()` 函数接收一个速度值（-255 到 255），并将其转换为对应引脚上的 PWM 信号，通过控制不同引脚的电平实现正转、反转和停止。

- **`mqtt_manager.cpp`**
  - **功能**：实现 MQTT 通信的核心功能，包括连接、订阅、消息发布和回调处理。其配置参数（服务器、端口）由 `config.h` 提供。
  - **核心逻辑**：`setupMQTT()` 配置 MQTT 服务器地址和端口。`mqtt_reconnect()` 实现连接重试逻辑。`callback()` 函数是消息处理的核心，它解析收到的消息主题（如 `/motor`、`/motor_control`、`/restart`、`/check_mqtt`），校验 `clientId`，并根据指令更新全局状态或调用相应的功能模块。`publishStatusInfo()` 等函数则负责将设备状态封装成 JSON 格式并发布到指定主题（如 `/ESP32_info`、`/motor/status`、`/check_mqtt_reply`）。

- **`wifi_manager.cpp`**
  - **功能**：实现 Wi-Fi 的连接与自动重连机制。其配置参数（Wi-Fi凭据、静态IP）由 `config.h` 提供。
  - **核心逻辑**：`connectWiFi()` 函数会遍历一个预定义的 Wi-Fi 凭据列表，尝试连接直到成功。`wifiAutoReconnect()` 在检测到 Wi-Fi 断开时，会以一定的退避策略尝试重新连接，保证了网络的健壮性。

## 2. 岸基用户端（MQTT网页控制端/）

- **`index.html`**
  - **功能**：定义了网页控制端的用户界面（UI）结构。
  - **核心逻辑**：包含了所有的页面元素，如滑块、按钮、状态显示区域和日志窗口。它通过 `<script>` 标签引入了 Paho MQTT 客户端库和核心的 `index.js` 逻辑文件。

- **`index.js`**
  - **功能**：实现了网页端所有的动态交互和通信逻辑。
  - **核心逻辑**：
    1.  **MQTT 连接**：使用 Paho MQTT 库创建客户端实例，连接到 MQTT Broker。
    2.  **UI 事件处理**：为滑块、按钮等元素绑定事件监听器，当用户操作时，生成对应的 JSON 载荷。
    3.  **指令发送**：通过 `client.publish()` 将控制指令（如电机速度）发送到相应主题。
    4.  **状态订阅与更新**：订阅设备状态主题（如 `/ESP32_info`），接收并解析消息，实时更新到页面的状态显示区域。
    5.  **多设备支持**：通过解析设备上报的 `clientId` 动态生成设备选择列表。
    6.  **心跳与重连**：实现了定时检查连接状态并在断开时自动重连的机制。

- **`index.css`**
  - **功能**：定义了网页界面的所有样式。
  - **核心逻辑**：包含了布局、颜色、字体、响应式设计以及虚拟摇杆等动态元素的视觉效果。

- **`网页MQTT配置修改指南.md`**
  - **功能**：提供一个简明的指导文档，说明如何修改 `index.js` 中的 MQTT Broker 连接参数。
