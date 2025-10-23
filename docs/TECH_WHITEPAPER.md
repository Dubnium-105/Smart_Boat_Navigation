# 智能小船 MQTT 远程控制与导航系统 · 技术白皮书

最后更新：2025-10-22

## 1. 项目概述

本系统实现“网页端 → 云端 MQTT 服务器 → 设备端（ESP32-S3）”的远程控制闭环：
- 网页控制端通过 WebSocket 连接云端 MQTT（Paho MQTT JS），发送电机速度等指令；
- 设备端（ESP32-S3，Arduino 框架）接入 2.4G Wi-Fi，使用 PubSubClient 收发消息，按协议驱动双电机；
- 设备端周期回传状态（IP、RSSI 等）和传感器信息，网页端实时展示。

核心能力：
- 手动控制：网页端实现便捷的人机交互控制；
- 多设备终端选择：网页端支持按 clientId 选择目标设备终端的单独控制；
- 健壮性：Wi-Fi 断线自动重连、MQTT 定时重连与心跳检测、消息节流；
- 易扩展：消息主题清晰、JSON 载荷可扩展，任务负载模块可扩展。

## 2. 系统架构与数据流

- 硬件：基于ESP32-S3-DevKitM-1作为系统主控，动力系统为双路直流电机驱动，可自定义适配各种物联网传感器。
- 设备端：
  - Wi-Fi：可多候选自动连接，自动断线重连；
  - MQTT：连接 自有/公有MQTT Broker，可自定义的订阅/发布主题；
  - 电机：LEDC 4 通道 PWM控制动力系统；
  - 任务逻辑: 可自定义扩展的传感器信息采集/负载组件任务等。;
- 网页端：
  - 用户登录网页，通过在线网站进行人机交互从而控制设备终端；
  - UI：设备检测、滑块/摇杆人机交互控制、实时信息回传监控；
  - 多船：基于设备上报 clientId 区分目标。；
  
数据流概览：
1) 设备上线：设备连接成功后发布 /ESP32_info 与 /check_mqtt {cmd:"device_online", clientId}；任意网页端收到后发布 /check_mqtt_reply {msg:"web_ready", webId, to:clientId}，设备串口打印 webId。
2) 前端发布 → /motor: {speedA,speedB,clientId} → 设备匹配自身 g_clientId 收到后驱动电机并反馈 /motor/status。
3) 前端发布 → /motor_control: {key:"e"|"shutdown"} → 设备接收后归零/停止并反馈。
4) 设备定期/事件发布 → /ESP32_info、/motor/status。
5) 健康检查：前端发布 /check_mqtt {cmd:"ping"} → 设备回复 /check_mqtt_reply {msg:"pong"}，并补充一次 /ESP32_info。

## 3. 主题与消息协议

所有 JSON 字段按需要可扩展，建议前后端保持向后兼容。

- 指令类（前端 → 设备）
  - /motor
    - 载荷：{ speedA: int[-255,255], speedB: int[-255,255], clientId: string }
    - 语义：直接设定两路 PWM 值（方向含在正负）。设备会 constrain 到 [-255,255] 并立即生效。
  - /motor_control
    - 载荷：{ key: "e" | "shutdown" }
    - 语义：开/关控制。shutdown 会将 speedA/speedB 归零并停机。
  
  - /restart
    - 载荷：{ cmd: "restart" }（前端实现为字符串亦可）
    - 语义：远程重启设备。
  - /check_mqtt
    - 载荷：
      - { cmd: "ping" }（前端健康检查）
      - { cmd: "device_online", clientId }（设备上线广播）
    - 语义：
      - ping：设备响应 pong，并补充一次状态广播。
      - device_online：网页端收到后应回执至 /check_mqtt_reply，带上 to=clientId 与自身 webId。

- 状态类（设备 → 前端）
  - /motor/status
    - 载荷：{ speedA:int, speedB:int, pwm_direct:true, timestamp:ms }
    - 语义：确认当前电机 PWM 输出。
  - /ESP32_info
    - 载荷：{ msg, ip, wifi_ssid, wifi_rssi, wifi_connected, clientId, timestamp }
    - 语义：设备健康/网络信息。
  
  - /check_mqtt_reply
    - 载荷：
      - {"msg":"pong"}
      - {"msg":"web_ready", "webId":"webClient_xxx", "to":"ESP32-<SHIP_NAME>"}

备注：设备侧对 /motor 指令要求带 clientId 且与自身 g_clientId 匹配，否则忽略，避免多船串扰。

## 4. 设备端实现细节（Arduino/ESP32）

- 平台配置：见 platformio.ini
  - board: esp32-s3-devkitm-1, framework: arduino, PSRAM 启用，串口 115200。
  - 依赖：PubSubClient、ArduinoJson。可按需引入其他扩展库（相机/传感器等）。
- 入口 `src/main.cpp`
  - 初始化：Serial → setup_motors() → connectWiFi() → setupMQTT() → mqtt_reconnect()。
  - 循环：wifiAutoReconnect()；MQTT 每 5s 尝试重连；若连接则 mqttClient.loop()；将全局 speedA/speedB 应用到 motor_control()。
  - 全局速度：int speedA/speedB，默认 0，由 MQTT /motor 指令更新。
- Wi-Fi 管理 `include/wifi_manager.h` 与 `src/wifi_manager.cpp`
  - 多账号轮询连接，3s/账号，成功后输出 IP；支持 USE_FIXED_IP 切换固定 IP（默认 false）。
  - wifiAutoReconnect()：断线尝试重连，避免频繁重试（1s backoff）。
- 电机控制 `include/motor_control.h` 与 `src/motor_control.cpp`
  - 引脚：A(38,39)，B(40,41)；LEDC 通道 0..3，5kHz，8bit。
  - motor_control(motor, pwm)：pwm∈[-255,255]，正/反向写入对应通道，0 停止。
- MQTT 管理 `include/mqtt_manager.h` 与 `src/mqtt_manager.cpp`
  - 服务器：如 自定义域名:端口；缓冲区 1KB。
  - clientId：形如 "ESP32-<SHIP_NAME>"（在 `include/config.h` 中通过 SHIP_NAME 配置）。
  - 订阅：/motor, /restart, /check_mqtt, /motor_control。
  - 回调：
    - /motor：解析 speedA/speedB，校验 clientId 一致再执行；反馈 /motor/status。
    
    - /check_mqtt：回复 /check_mqtt_reply: {msg:"pong"}，并 publishStatusInfo()。
    - /restart：远程重启设备。
    - /motor_control：key=="shutdown" → 停机并反馈。
  - 状态上报：publishStatusInfo() 发布 /ESP32_info。
  

- 参考实现 `reference/`
  - `main.cpp`

## 5. 网页控制端实现

- 入口文件：`MQTT网页控制端/index.html` + `index.js` + `index.css`
- 连接：host=Broker服务器域名/IP, port=目标端口, path=/mqtt（可按《网页MQTT配置修改指南》修改）
- UI：
  - 控制方式：滑块与摇杆两种本地 UI 切换；
  - 电机开关：/motor_control, key: e|shutdown；
  - 速度设定：/motor, {speedA,speedB,clientId}；节流 50ms；
  - 监控：/ESP32_info、/motor/status、/check_mqtt、/check_mqtt_reply；
  - 显示：页面左侧显示本机网页端 ID（webId）。
  - 多船：记录设备上报的 clientId 生成下拉框，选择后发送时带上 targetClientId；
  - 心跳：每 60s 检查连接，断开自动重连；
  - 快捷键：方向键/Shift，e 开机，q 关机。

建议：
- 在局域网或公网部署前端文件，可直接本地打开 index.html 使用；
- 与设备端共用同一 MQTT 实例或可达的公网实例；
- 如需鉴权/SSL，请在 MQTT 侧启用用户名密码/证书，并在前端 connect 选项中配置。

## 6. 部署与运行

- 设备端（Windows + PlatformIO）
  1) 安装 VS Code + PlatformIO 插件。
  2) 连接 ESP32-S3-DevKitM-1（16MB Flash, PSRAM）。
  3) 修改 `include/config.h` 文件，填入你的 Wi-Fi 列表、MQTT 服务器地址等信息。
  4) 编译烧录，串口 115200 观察日志。

- 网页端
  1) 打开 `MQTT网页控制端/index.html`（本地或任意静态服务器）。
  2) 若 MQTT 服务器不同，参考《网页MQTT配置修改指南》修改 `index.js` 中 host/port/path。
  3) 选择目标船只（clientId），开启电机并用滑块/摇杆控制。

- 命令行（Windows cmd，供参考）
  - 可在 VS Code 的 PlatformIO 按钮操作；或在终端使用 pio：

```
# 构建
pio run
# 烧录（串口端口按实际替换）
pio run -t upload --upload-port COM5
# 串口监视
pio device monitor -b 115200
```

## 7. 安全与健壮性

- clientId 过滤：/motor 指令必须带 clientId，设备侧仅执行匹配的指令。
- 断线恢复：Wi-Fi 自动重连、MQTT 定时重连；前端心跳检测与重连。
- 节流与反抖：前端速度指令 50ms 节流，降低带宽与抖动。
- 资源限制：PubSubClient buffer 已增至 1KB；避免一次性过大消息。
- 可选鉴权：实际部署建议启用 MQTT 用户名/密码或 TLS；前端 Paho 支持 wss，设备端支持 SSL 需替换为支持 TLS 的客户端库或自签证书。

## 8. 调试与常见问题

- 无法连接 MQTT：
  - 确认服务器、端口与是否启用 WebSocket（前端）/TCP（设备端）；
  - 检查防火墙/端口映射；
  - 设备端串口查看 mqttClient.state()；
  - 前端 F12 控制台查看错误。
- 指令无效：
  - 确认已选择正确的 targetClientId；
  - /motor 载荷包含 clientId 且与设备端 g_clientId 一致（设备端上报在 /ESP32_info）。
- 电机不转：
  - 确认供电与引脚正确；
  - 速度值范围 [-255,255]，开关未 shutdown；
  - 设备端串口是否收到 /motor 并反馈 /motor/status。

## 9. 文件逐个说明

项目根 Smart_Boat_Navigation/
- platformio.ini：PlatformIO 构建/烧录配置与依赖库声明。
- include/
  - config.h：用户配置入口（Wi-Fi、MQTT、船名等）。
  - wifi_manager.h：Wi-Fi 连接与重连模块声明。
  - mqtt_manager.h：MQTT 客户端与回调等声明。
  - motor_control.h：电机引脚、初始化与 PWM 控制声明。
- src/
  - main.cpp：设备主入口，初始化各模块与循环逻辑，应用速度到电机。
  - wifi_manager.cpp：多账号 Wi-Fi 连接与自动重连实现，配置由 config.h 提供。
  - mqtt_manager.cpp：MQTT 连接/重连/订阅与回调、状态上报，配置由 config.h 提供。
  - motor_control.cpp：LEDC PWM 初始化与双向速度控制。
网页端 MQTT网页控制端/
- index.html：网页 UI 布局与入口，加载 Paho MQTT 与 index.js。
- index.js：核心逻辑（连接、订阅、指令发送、UI 交互、多船选择、心跳、快捷键）。
- index.css：UI 样式与响应式适配、摇杆视觉效果。
- 网页MQTT配置修改指南.md：前端 MQTT host/port/path 修改指引。
## 10. 后续扩展建议
- 导航算法接入：以模块化方式接入自动导航算法（如路径规划/避障），并与 motor_control_* 控制接口整合为设备端“自动模式”；
- 安全：启用 MQTT 账号或 JWT，或迁移到 TLS（wss/ssl）；
- 日志：设备侧持久化关键事件到 SPIFFS；
- 配置：将 Wi-Fi/MQTT/电机参数持久化至 NVS，并添加前端配置面板；
- 多船编排：在服务器侧做 ACL（按 clientId topic 级别权限）与离线消息保留。
---
如需变更服务器或主题，请更新设备端 `include/config.h`（MQTT_SERVER/MQTT_PORT/SHIP_NAME 等）与网页端 `MQTT网页控制端/index.js` 的对应配置，并通过串口与浏览器控制台确认连接状态。