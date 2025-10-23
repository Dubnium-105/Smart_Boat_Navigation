# 附录A — 系统配置参数说明

本文档列出无人船舶物联网远程控制系统中涉及的关键配置参数，包含参数含义、数据类型、建议默认值、允许取值范围与说明。

1. 网络参数（Network）
-- wifi_ssid_list
   - 类型：字符串数组
   - 默认：[]
   - 说明：设备尝试连接的优先 Wi‑Fi SSID 列表，按顺序轮询尝试连接。
-- wifi_password_list
   - 类型：字符串数组（与 `wifi_ssid_list` 索引对应）
   - 默认：[]
   - 说明：对应 SSID 的密码。
-- use_fixed_ip
   - 类型：布尔
   - 默认：false
   - 说明：是否使用静态 IP，true 表示自定义 ip/address/netmask/gateway/dns。
-- fixed_ip / netmask / gateway / dns
   - 类型：字符串（IPv4）
   - 默认：null（仅在 use_fixed_ip=true 时有效）

2. MQTT 通信参数（MQTT）
-- mqtt_server
   - 类型：字符串（域名或 IP）
   - 默认："broker.example.com"
   - 说明：MQTT Broker 地址。
-- mqtt_port
   - 类型：整数
   - 默认：1883
   - 说明：MQTT 端口；若启用 WebSocket（前端）通常使用 8083 或 9001；若启用 TLS，使用 8883 / 443 等。
-- mqtt_use_tls
   - 类型：布尔
   - 默认：false
   - 说明：是否使用 TLS/SSL 加密连接。
-- mqtt_username / mqtt_password
   - 类型：字符串
   - 默认：""
   - 说明：可选的鉴权用户名/密码。
-- mqtt_client_id
   - 类型：字符串
   - 默认："ESP32-<MAC>"
   - 说明：设备在 Broker 中的唯一标识（应当可配置）。
-- mqtt_keepalive
   - 类型：整数（秒）
   - 默认：60
   - 说明：MQTT Keepalive，建议 30-120 之间。
-- mqtt_qos_default
   - 类型：整数（0/1/2）
   - 默认：0
   - 说明：消息发布与订阅默认 QoS，状态类建议 0 或 1，关键控制消息可选 1。
-- mqtt_buffer_size
   - 类型：整数（字节）
   - 默认：1024
   - 说明：库缓冲区大小，需覆盖最大单条消息长度。

3. 消息主题与命名空间
-- topics_prefix
   - 类型：字符串
   - 默认：""（空字符串）
   - 说明：系统主题前缀（可选）。当前实现采用根级主题（例如 "/motor"）。如需分层命名，可设置为 "usv" 并相应调整前后端主题。
-- clientId_template
   - 类型：字符串 模板
   - 默认："ESP32-<SHIP_NAME>"

4. 控制与执行参数（Control）
-- speed_range
   - 类型：整数区间
   - 默认：[-255, 255]
   - 说明：电机速度（PWM）范围，正/负代表正反转。
-- pwm_frequency
   - 类型：整数（Hz）
   - 默认：5000
   - 说明：LEDC PWM 频率，需与驱动硬件匹配。
-- pwm_resolution_bits
   - 类型：整数（bits）
   - 默认：8
   - 说明：PWM 分辨率（8-bit 对应 0-255）。
-- max_rudder_angle
   - 类型：整数（度）
   - 默认：45
   - 说明：舵机最大安全舵角，上位机与设备端都应校验。
-- control_rate
   - 类型：整数（ms）
   - 默认：50
   - 说明：控制指令节流周期（前端与设备端均推荐节流以避免抖动），示例：50ms。

5. 导航/控制算法参数（可选）
-- pid_speed_kp / pid_speed_ki / pid_speed_kd
   - 类型：浮点
   - 默认：kp:1.0, ki:0.0, kd:0.0
   - 说明：速度环 PID 参数，留作可配置项。
-- pid_heading_kp / pid_heading_ki / pid_heading_kd
   - 类型：浮点
   - 默认：kp:2.0, ki:0.0, kd:0.1
   - 说明：航向控制 PID 参数。

6. 状态上报与心跳（Telemetry）
-- status_publish_interval
   - 类型：整数（ms）
   - 默认：2000
   - 说明：定期上报 /ESP32_info 的周期，可按需动态调整。
-- event_publish_on_change
   - 类型：布尔
   - 默认：true
   - 说明：是否在重要状态变化（如网络连接变化、关键异常）时立即上报。

7. 重连与退避策略（Robustness）

9. 安全与访问控制
-- acl_restrict_by_clientid
   - 类型：布尔
   - 默认：true
   - 说明：设备仅接受带有匹配 clientId 的控制主题消息。
-- timestamp_tolerance_ms
   - 类型：整数（ms）
   - 默认：5000
   - 说明：接收控制消息时允许的时间差（防止历史消息被误执行）。

10. 日志与诊断
-- serial_baudrate
   - 类型：整数
   - 默认：115200
   - 说明：串口日志波特率。
-- enable_file_logging
   - 类型：布尔
   - 默认：false
   - 说明：若设备支持 SD 卡 或 SPIFFS，可开启持久化日志记录。

11. 说明与扩展
- 所有参数建议在设备侧提供 NVS（非易失存储）或配置文件持久化接口，并支持远程下发与恢复出厂设置。
- 所有默认值为建议值，实际部署时应根据硬件平台、网络环境与安全策略调整。
