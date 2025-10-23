# 附录B — 接口规范文档（MQTT 主题与消息载荷）

本文档详细定义系统中使用的 MQTT 主题命名规范、消息载荷（JSON）结构、字段类型、示例载荷以及交互语义，作为开发者实现参考。

1. 命名与主题规则
- 主题前缀：当前实现默认不使用前缀（根级主题，如 "/motor"、"/ESP32_info"），也可根据需要配置前缀（如 `usv`）。
- 主题格式：`{prefix}/{功能}/{clientId}` 或 `{prefix}/{功能}`（广播）。
- clientId：设备端唯一标识符，建议不含斜杠(`/`)或空白字符。客户端 ID 生成形如 `ESP32-<SHIP_NAME>`（参见 `include/config.h` 的 SHIP_NAME）。

2. 主题列表与载荷

- 下行控制（岸基 → 设备）
  - {prefix}/motor
    - 说明：设定电机输出（需要在载荷中指定 clientId）。
    - QoS：0 或 1（建议 1 用于关键控制）
    - JSON Schema：
      {
        "clientId": "string",
        "speedA": "integer",   // -255..255
        "speedB": "integer",   // -255..255
        "timestamp": "integer" // ms since epoch, 可选
      }
    - 示例：
      {"clientId":"ESP32-BOAT-001","speedA":120,"speedB":120,"timestamp":1690000000000}

  - {prefix}/motor_control
    - 说明：紧急停止/开关等控制。
    - JSON Schema：
      {"key": "string"} // "e" | "shutdown"


  - {prefix}/restart
    - 说明：远程重启（谨慎使用）。
    - JSON Schema：{ "cmd":"restart", "clientId":"string" }

  - {prefix}/check_mqtt
     - 说明：健康检查请求（岸基→设备）。
     - JSON Schema：{ "cmd":"ping", "clientId":"string" }

- 上行状态（设备 → 岸基）
  - {prefix}/check_mqtt
    - 说明：设备上线广播。
    - JSON Schema：{ "cmd":"device_online", "clientId":"string", "timestamp":"integer" }
    - 语义：设备连接成功后广播上线事件，网页端应回执至 {prefix}/check_mqtt_reply，带上 to=clientId 与自身 webId。

  - {prefix}/motor/status
    - 说明：电机状态确认。
    - JSON Schema：
      {"clientId":"string","speedA":"integer","speedB":"integer","pwm_direct":"boolean","timestamp":"integer"}

  - {prefix}/ESP32_info
    - 说明：设备健康信息与网络状态上报。
    - JSON Schema：
      {
        "clientId":"string",
        "msg":"string",
        "ip":"string",
        "wifi_ssid":"string",
        "wifi_rssi":"integer",
        "wifi_connected":"boolean",
        "timestamp":"integer"
      }


  - {prefix}/check_mqtt_reply
    - 说明：健康检查回复与网页端回执。
    - JSON Schema（两种常见载荷）：
      1) 健康检查回复（设备→网页）：{ "msg":"pong", "timestamp":"integer" }
      2) 网页端回执（网页→设备）：{ "msg":"web_ready", "webId":"string", "to":"string", "timestamp":"integer" }
    - 语义：
      - pong：响应 {prefix}/check_mqtt 中 cmd=="ping" 的请求。
      - web_ready：网页端在收到设备 {prefix}/check_mqtt 的 cmd=="device_online" 后回执，to 为目标设备 clientId。

3. 时间戳规范
- 使用 Unix epoch 毫秒（ms since 1970-01-01T00:00:00Z）。
- 时钟不同步时，接收端应使用 `timestamp_tolerance_ms`（见附录A）来判断是否接受控制消息。

4. 字段通用约定
- clientId：字符串，长度建议 < 64 字符，唯一；设备上报时包含 clientId，以便前端生成设备列表。
- timestamp：整数（ms），可选但强烈建议由发送端附加。
- error/code：在失败/异常反馈中使用，整数 code 与可读 msg。

5. 质量保证（QoS）与保留消息
- 控制消息：建议 QoS=1，重要消息（如 shutdown）可要求 ACK 机制。
- 状态广播：QoS=0 或 1，根据实时性与可靠性权衡。
- 保留（retain）：对于设备上线/身份信息，建议使用 retain=true 的 /ESP32_info 以便前端可以在订阅后立即获得最新设备状态（注意隐私/安全）。

6. 安全建议
- 主题隔离：在 Broker 侧通过 ACL 限定哪个 clientId 可以发布/订阅哪些主题（按 prefix/功能/ID 级别）。
- 鉴权：启用用户名/密码或 TLS，生产环境优先。
- 时间戳/重放防护：检查 timestamp，拒绝老旧消息。

7. JSON Schema（可机器验证）
- 建议维护一份 JSON Schema 文件（例如 schemas/motor.json、schemas/esp32_info.json），供设备端与前端在开发时校验与自动生成文档。

8. 示例交互序列
0) 设备连接成功后：
  - 发布 {prefix}/ESP32_info（设备信息）
  - 发布 {prefix}/check_mqtt {"cmd":"device_online","clientId":"ESP32-<SHIP_NAME>","timestamp":...}
  - 网页端收到后回执 {prefix}/check_mqtt_reply {"msg":"web_ready","webId":"webClient_xxx","to":"ESP32-<SHIP_NAME>","timestamp":...}
1) 前端选择 deviceA 后发送 {prefix}/check_mqtt {"cmd":"ping","clientId":"deviceA"}
2) 设备回复 {prefix}/check_mqtt_reply {"msg":"pong","timestamp":...} 并额外发布 {prefix}/ESP32_info
3) 用户发送 {prefix}/motor {"clientId":"deviceA","speedA":120,"speedB":120}
4) 设备收到并校验 clientId 后设置 PWM 并发布 {prefix}/motor/status

9. 版本控制与兼容性
- 所有消息应向后兼容：新增可选字段应当不会破坏现有解析。建议在主题或消息体中加入 `version` 字段用于重大变更管理。

