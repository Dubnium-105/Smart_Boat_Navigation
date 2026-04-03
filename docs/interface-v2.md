# Smart Boat Navigation 通信协议规范（V2.0）

适用范围：Smart_Boat_Navigation 项目中的控制指令、状态回执、遥测上报。
版本：v2.0（工程化）
日期：2026-04-03

## 1. 目标

1. 统一 command / event / state 消息模型。
2. 支持 v1/v2 共存，便于设备与控制端平滑升级。
3. 提供 ACK/NACK 与端到端超时重试策略。
4. 提供 client_id/token 鉴权和命令权限控制。
5. 对高频遥测进行批量打包、二进制化与压缩传输。

## 2. 版本与兼容策略

1. 当前主版本：v2。
2. 最低兼容版本：v1。
3. 设备同时订阅：
1. /sbn/v1/{device_id}/cmd/*
2. /sbn/v2/{device_id}/cmd/*
4. 旧 Topic 兼容保留：
1. /motor
2. /navigation
3. /ota
4. /restart
5. /ESP32_info
6. /motor/status

迁移建议：
1. 阶段A：控制端双读，命令优先发 v2。
2. 阶段B：仅发 v2，保留旧 Topic 兜底。
3. 阶段C：关闭旧 Topic 下行，仅留上行兼容窗口。

## 3. 统一消息模型

v2 统一结构采用顶层基础字段 + data 业务负载。

示例：command

{
  "protocol_ver": 2,
  "schema_ver": 2,
  "compat_min_ver": 1,
  "msg_type": "command",
  "msg_name": "motor",
  "msg_id": "5a2f2d2f-8f51-4c67-b8f5-a4b5a095b89e",
  "trace_id": "trace-5a2f2d2f",
  "device_id": "boat-ABC123",
  "client_id": "python-panel-1712123123",
  "token": "sbn-operator-token",
  "ttl_ms": 1500,
  "ts_unix_ms": 1770000000123,
  "ts_ms": 123456,
  "source": "control_panel",
  "data": {
    "mode": "direct_pwm",
    "speedA": 80,
    "speedB": 75
  }
}

字段约束：
1. protocol_ver：协议主版本。
2. schema_ver：当前 schema 版本。
3. msg_type：command | state | event。
4. msg_name：业务消息名，如 motor / nav_mode / ack。
5. msg_id：命令唯一 ID，建议 UUID。
6. device_id：目标设备。
7. client_id：调用方 ID。
8. token：鉴权令牌。
9. ttl_ms：命令有效期。
10. data：业务载荷。

## 4. Topic 规范

v2 主题前缀：

/sbn/v2/{device_id}/{channel}/{name}

channel 取值：
1. cmd
2. state
3. telemetry
4. ack

关键主题：
1. /sbn/v2/{device_id}/cmd/motor
2. /sbn/v2/{device_id}/cmd/nav_mode
3. /sbn/v2/{device_id}/cmd/mission
4. /sbn/v2/{device_id}/cmd/ota
5. /sbn/v2/{device_id}/cmd/restart
6. /sbn/v2/{device_id}/cmd/protocol_cfg
7. /sbn/v2/{device_id}/ack/cmd
8. /sbn/v2/{device_id}/state/device
9. /sbn/v2/{device_id}/state/motor
10. /sbn/v2/{device_id}/state/mission
11. /sbn/v2/{device_id}/telemetry/perception
12. /sbn/v2/{device_id}/telemetry/decision
13. /sbn/v2/{device_id}/telemetry/binary

## 5. 命令与 ACK/NACK

### 5.1 ACK 结构

{
  "protocol_ver": 2,
  "schema_ver": 2,
  "msg_type": "event",
  "msg_name": "ack",
  "msg_id": "5a2f2d2f-8f51-4c67-b8f5-a4b5a095b89e",
  "device_id": "boat-ABC123",
  "client_id": "python-panel-1712123123",
  "ok": true,
  "code": "OK",
  "detail": "motor command applied",
  "duplicate": false,
  "retryable": false,
  "ts_ms": 123789,
  "data": {
    "ok": true,
    "code": "OK",
    "detail": "motor command applied"
  }
}

### 5.2 错误码建议

1. OK
2. BAD_JSON
3. BAD_DEVICE_ID
4. BAD_PARAM_RANGE
5. CMD_EXPIRED
6. AUTH_REQUIRED
7. AUTH_FAILED
8. PERMISSION_DENIED
9. MODE_CONFLICT
10. UNSUPPORTED_MODE
11. INTERNAL_ERROR

### 5.3 控制端重试策略

1. 每个命令必须带 msg_id。
2. 发送后等待 ACK，超时则重发。
3. 建议默认：timeout=1.2s，retry=2（总 3 次）。
4. 若收到 NACK 且 retryable=true，可重试。
5. 若收到 NACK 且 retryable=false，立即失败。

### 5.4 设备端幂等策略

1. 设备缓存最近 N 条 msg_id（建议 20 条）。
2. 重复 msg_id 不重复执行业务，只回放上次 ACK。
3. 缓存建议 TTL：120s。

## 6. 鉴权与权限

### 6.1 鉴权字段

1. client_id：调用方身份。
2. token：预共享令牌。

### 6.2 角色建议

1. viewer
2. operator
3. admin

### 6.3 权限矩阵

1. motor：operator/admin
2. nav_mode：operator/admin
3. mission：admin
4. ota：admin
5. restart：admin
6. protocol_cfg：admin

## 7. 二进制遥测协议

### 7.1 设计

1. 高频 telemetry 样本先进入设备侧环形队列。
2. 按 batch_ms 或 max_samples 触发打包。
3. 样本序列编码为固定长度二进制。
4. 采用 RLE 压缩（可退化为原始模式）。
5. 通过 MQTT telemetry/binary 上报。
6. 可选并行 HTTP 上传（application/octet-stream）。

### 7.2 包头格式（24字节，小端）

1. magic[4]：固定为 SBN2
2. protocol_ver[1]
3. codec[1]：0=raw, 1=rle
4. sample_count[2]
5. raw_len[2]
6. payload_len[2]
7. batch_seq[4]
8. crc32_raw[4]
9. batch_ts_ms[4]

### 7.3 样本格式（12字节，小端）

1. kind[1]
2. nav_mode[1]
3. value1[1]（int8）
4. value2[1]（int8）
5. ts_ms[4]（uint32）
6. extra[4]（int32）

kind 建议：
1. 1：IR/perception
2. 2：decision
3. 3：motor
4. 4：health

### 7.4 RLE 规则

1. 输出按二元组编码：[count][byte]。
2. count 范围 1~255。
3. 当压缩后长度不小于原始长度时，自动回退 codec=0。

### 7.5 HTTP 上传重试

1. 当 HTTP 上报失败时缓存最近 1 包。
2. 重试退避：1s, 2s, 4s, 8s, 16s, 32s。
3. 超过最大重试次数后丢弃并记录 decision 事件。

## 8. protocol_cfg 动态配置

Topic：/sbn/v2/{device_id}/cmd/protocol_cfg

示例：

{
  "protocol_ver": 2,
  "schema_ver": 2,
  "msg_type": "command",
  "msg_name": "protocol_cfg",
  "msg_id": "9bf3fcb1-1d10-4029-b308-f3956588b504",
  "device_id": "boat-ABC123",
  "client_id": "python-panel-1712123123",
  "token": "sbn-admin-token",
  "ttl_ms": 2000,
  "ts_unix_ms": 1770000000123,
  "data": {
    "telemetry": {
      "batch_ms": 1500,
      "max_samples": 24,
      "http_enable": true,
      "http_url": "http://example.com/telemetry/upload"
    }
  }
}

## 9. 安全建议

1. 生产环境应替换默认 token。
2. 建议引入 TLS MQTT（8883）与证书校验。
3. 建议 token 轮换机制与过期时间。
4. 建议按 client_id 建立白名单。

## 10. 实施清单

1. 设备端：完成 v2 订阅、鉴权、ACK 幂等、二进制遥测。
2. 控制端：完成 v2 发包、ACK 超时重试、二进制解码展示。
3. 文档：保留 v1 文档，新增 v2 文档与迁移建议。
4. 联调：验证 v1/v2 同时在线的互操作与回退路径。
