# Smart Boat Navigation MQTT 接口文档（V1.0）

> 适用范围：`Smart_Boat_Navigation` 项目中基于 MQTT 的设备控制、状态上报、任务调度与告警闭环。  
> 版本：`v1.0`（建议稿，可直接实现）  
> 日期：2026-03-27

---

## 1. 设计目标

1. 在保持现有 `/motor`、`/navigation`、`/ESP32_info` 等旧 Topic 可用的前提下，统一协议结构。
2. 将“电机控制”扩展为“任务级控制（Mission）+ 融合决策遥测（Decision）”。
3. 提供完整的可追踪链路（`msg_id`、`trace_id`、`ack`、错误码）。
4. 支持可观测性（状态/事件/告警）与后续 A/B 算法验证。

---

## 2. 协议约定

### 2.1 协议与编码

- 协议：MQTT 3.1.1（兼容 MQTT 5）
- 负载格式：UTF-8 JSON
- 时间字段：
  - `ts_ms`: 设备侧毫秒时间（例如 `millis`）
  - `ts_unix_ms`: 可选，Unix 毫秒时间戳（上位机/云端补充）

### 2.2 主题命名

统一前缀：

```text
/sbn/v1/{device_id}/{channel}/{name}
```

- `device_id`: 设备唯一标识（例：`boat-001`）
- `channel`: `cmd | state | telemetry | event | ack`
- `name`: 具体业务名（如 `motor`、`mission`）

示例：

```text
/sbn/v1/boat-001/cmd/motor
/sbn/v1/boat-001/state/device
```

### 2.3 QoS 与 Retain 建议

- 控制命令（`cmd/*`）：QoS 1，retain=false
- 高频遥测（`telemetry/*`）：QoS 0，retain=false
- 关键状态（`state/*`）：QoS 1，retain=true（尤其 `state/device`、`state/mission`）
- 告警（`event/alert`）：QoS 1，retain=false

### 2.4 通用字段约束

#### 命令类通用字段

| 字段 | 类型 | 必填 | 说明 |
|---|---|---:|---|
| `msg_id` | string | 是 | 指令唯一 ID（UUID） |
| `device_id` | string | 是 | 目标设备 |
| `ts_unix_ms` | number | 否 | 上位机发送时间 |
| `ttl_ms` | number | 否 | 指令有效期（毫秒） |
| `trace_id` | string | 否 | 链路追踪 ID |
| `schema_ver` | number | 否 | Schema 版本，默认 1 |

#### 上行状态/遥测通用字段

| 字段 | 类型 | 必填 | 说明 |
|---|---|---:|---|
| `device_id` | string | 是 | 设备 ID |
| `ts_ms` | number | 是 | 设备时间 |
| `schema_ver` | number | 否 | Schema 版本 |
| `source` | string | 否 | 数据来源（`esp32`/`fusion_v1`） |

---

## 3. Topic 详细定义

## 3.1 控制下行（C2D）

### 3.1.1 电机控制

- **Topic**: `/sbn/v1/{device_id}/cmd/motor`
- **QoS**: 1
- **频率建议**: ≤ 20Hz

#### 请求示例（direct PWM）

```json
{
  "msg_id": "1c53b1de-c5ef-4b7f-9d95-3b2ae8f68331",
  "device_id": "boat-001",
  "mode": "direct_pwm",
  "speedA": 120,
  "speedB": 110,
  "ttl_ms": 500,
  "ts_unix_ms": 1770000000000,
  "schema_ver": 1
}
```

#### 字段说明

| 字段 | 类型 | 必填 | 范围/枚举 | 说明 |
|---|---|---:|---|---|
| `mode` | string | 是 | `direct_pwm`/`intent_control` | 控制模式 |
| `speedA` | number | 条件 | `-255..255` | 电机 A 目标 |
| `speedB` | number | 条件 | `-255..255` | 电机 B 目标 |
| `intent` | string | 条件 | 见 3.2.2 | 仅 `intent_control` 时可用 |

> 约束：`mode=direct_pwm` 时，`speedA/speedB` 必填。

#### 应答

设备应发布：

- `/sbn/v1/{device_id}/ack/cmd`
- `/sbn/v1/{device_id}/state/motor`

---

### 3.1.2 导航模式切换

- **Topic**: `/sbn/v1/{device_id}/cmd/nav_mode`
- **QoS**: 1

#### 请求示例

```json
{
  "msg_id": "693bd284-f853-42f1-b6c5-5d662cc1af95",
  "device_id": "boat-001",
  "mode": "navigate",
  "ttl_ms": 1000,
  "schema_ver": 1
}
```

`mode` 枚举：
- `manual`：手动
- `navigate`：自动导航
- `mission`：任务模式（新增）

---

### 3.1.3 任务控制（Mission）

- **Topic**: `/sbn/v1/{device_id}/cmd/mission`
- **QoS**: 1

#### 请求示例（启动任务）

```json
{
  "msg_id": "398d3ea4-218e-4e17-a6fb-f6f8f44f0f0d",
  "device_id": "boat-001",
  "action": "start",
  "mission": {
    "mission_id": "M-20260327-001",
    "version": 1,
    "steps": [
      {"type": "goto_heading", "heading_deg": 45, "timeout_s": 20},
      {"type": "avoid_obstacle", "min_clearance_m": 1.5, "duration_s": 30},
      {"type": "hold", "duration_s": 10}
    ],
    "failsafe": {
      "on_link_loss": "hold",
      "on_sensor_fault": "safe_stop"
    }
  },
  "ttl_ms": 3000,
  "schema_ver": 1
}
```

#### 字段说明

| 字段 | 类型 | 必填 | 枚举/范围 | 说明 |
|---|---|---:|---|---|
| `action` | string | 是 | `start`/`pause`/`resume`/`abort` | 任务控制动作 |
| `mission` | object | 条件 | - | `start` 时必填 |
| `mission.steps[].type` | string | 是 | `goto_heading`/`avoid_obstacle`/`hold` | 任务步骤 |

---

### 3.1.4 OTA 升级

- **Topic**: `/sbn/v1/{device_id}/cmd/ota`
- **QoS**: 1

```json
{
  "msg_id": "f77a09f1-e64e-4e98-b6d2-2ca58e8f7de3",
  "device_id": "boat-001",
  "url": "https://example.com/fw.bin",
  "sha256": "a41f...",
  "force": false,
  "schema_ver": 1
}
```

> 建议：设备先校验 `sha256` 再升级，失败返回 `OTA_VERIFY_FAIL`。

---

### 3.1.5 重启指令

- **Topic**: `/sbn/v1/{device_id}/cmd/restart`
- **QoS**: 1

```json
{
  "msg_id": "1a8f43d0-42f7-4f0f-a2c6-9de9cb44b97f",
  "device_id": "boat-001",
  "delay_ms": 1000,
  "schema_ver": 1
}
```

---

## 3.2 状态与遥测上行（D2C）

### 3.2.1 设备状态

- **Topic**: `/sbn/v1/{device_id}/state/device`
- **QoS**: 1
- **retain**: true
- **建议频率**: 1Hz 或状态变化触发

```json
{
  "device_id": "boat-001",
  "fw_version": "1.2.0",
  "uptime_ms": 1234567,
  "wifi": {"connected": true, "ssid": "lab-ap", "rssi": -56},
  "mqtt": {"connected": true},
  "stream_url": "http://192.168.1.8/stream",
  "nav_mode": "navigate",
  "health": {"camera_ok": true, "ir_ok": true, "imu_ok": true},
  "ts_ms": 1234567,
  "schema_ver": 1
}
```

---

### 3.2.2 电机状态

- **Topic**: `/sbn/v1/{device_id}/state/motor`
- **QoS**: 1
- **retain**: false
- **建议频率**: 5~10Hz 或变化触发

```json
{
  "device_id": "boat-001",
  "speedA_cmd": 120,
  "speedB_cmd": 110,
  "speedA_applied": 118,
  "speedB_applied": 109,
  "pwm_direct": true,
  "ts_ms": 1234567,
  "schema_ver": 1
}
```

---

### 3.2.3 感知遥测（Perception）

- **Topic**: `/sbn/v1/{device_id}/telemetry/perception`
- **QoS**: 0
- **建议频率**: 10~20Hz

```json
{
  "device_id": "boat-001",
  "vision": {"target_dx": -0.21, "target_conf": 0.74},
  "ir": {"main_dir_idx": 7, "main_dir_deg": 315, "strength": 0.62, "stable_frames": 4},
  "imu": {"yaw_rate": 3.2, "acc_norm": 1.04},
  "ts_ms": 1234567,
  "schema_ver": 1
}
```

---

### 3.2.4 融合决策遥测（Decision）

- **Topic**: `/sbn/v1/{device_id}/telemetry/decision`
- **QoS**: 0
- **建议频率**: 5~10Hz

```json
{
  "device_id": "boat-001",
  "intent": "avoid_left",
  "confidence": 0.81,
  "risk": {"collision": 0.72, "target_loss": 0.12, "instability": 0.08},
  "speedA": 85,
  "speedB": 40,
  "source": "fusion_v1",
  "ts_ms": 1234567,
  "schema_ver": 1
}
```

#### `intent` 枚举建议

- `follow_target`
- `avoid_left`
- `avoid_right`
- `slow_down`
- `hold_position`
- `safe_stop`

---

### 3.2.5 任务状态

- **Topic**: `/sbn/v1/{device_id}/state/mission`
- **QoS**: 1
- **retain**: true
- **建议频率**: 状态变化触发（必要时 1Hz 心跳）

```json
{
  "device_id": "boat-001",
  "mission_id": "M-20260327-001",
  "state": "EXEC",
  "step_index": 1,
  "progress": 0.42,
  "last_event": "enter_step",
  "ts_ms": 1234567,
  "schema_ver": 1
}
```

`state` 枚举：`IDLE | PLAN | EXEC | RECOVER | DONE | ABORT`

---

### 3.2.6 告警事件

- **Topic**: `/sbn/v1/{device_id}/event/alert`
- **QoS**: 1
- **retain**: false

```json
{
  "device_id": "boat-001",
  "level": "WARN",
  "code": "SENSOR_IR_NOISE",
  "detail": "stable_count<3 for 1200ms",
  "action": "degrade_to_vision",
  "ts_ms": 1234567,
  "schema_ver": 1
}
```

`level`：`INFO | WARN | ERROR | FATAL`

---

### 3.2.7 命令 ACK

- **Topic**: `/sbn/v1/{device_id}/ack/cmd`
- **QoS**: 1

```json
{
  "msg_id": "1c53b1de-c5ef-4b7f-9d95-3b2ae8f68331",
  "device_id": "boat-001",
  "ok": true,
  "code": "OK",
  "detail": "motor command applied",
  "ts_ms": 1234567,
  "schema_ver": 1
}
```

---

## 4. 错误码规范

| Code | 含义 | 说明 |
|---|---|---|
| `OK` | 成功 | 指令执行成功 |
| `BAD_JSON` | JSON 解析失败 | 格式错误 |
| `BAD_DEVICE_ID` | 设备 ID 不匹配 | 非目标设备 |
| `BAD_PARAM_RANGE` | 参数越界 | 如速度超出 `-255..255` |
| `CMD_EXPIRED` | 指令已过期 | `ttl_ms` 校验失败 |
| `MODE_CONFLICT` | 模式冲突 | 例如 manual 下执行 mission |
| `SENSOR_FAULT` | 传感器异常 | 可触发降级或停机 |
| `OTA_VERIFY_FAIL` | OTA 校验失败 | hash 不一致 |
| `INTERNAL_ERROR` | 内部错误 | 未分类异常 |

---

## 5. 时序建议

### 5.1 设备上线时序

1. 连接 MQTT
2. 发布 `/state/device`（retain）
3. 发布 `/state/motor`
4. 发布 `event/alert`（`level=INFO`, `code=ONLINE`）可选

### 5.2 电机命令时序

1. 上位机发布 `/cmd/motor`
2. 设备做 `device_id`、`ttl_ms`、参数范围校验
3. 执行电机控制
4. 设备发布 `/ack/cmd` + `/state/motor`

### 5.3 任务控制时序

1. `/cmd/mission action=start`
2. `/state/mission state=PLAN`
3. `/state/mission state=EXEC`
4. 异常时 `/event/alert` + `state=RECOVER`
5. 完成：`DONE`；中止：`ABORT`

---

## 6. 与现有 Topic 的兼容映射

| 现有 Topic | 新 Topic（建议） | 兼容策略 |
|---|---|---|
| `/motor` | `/sbn/v1/{id}/cmd/motor` | 设备同时订阅两者 |
| `/navigation` | `/sbn/v1/{id}/cmd/nav_mode` | 旧命令映射到 `mode` |
| `/ota` | `/sbn/v1/{id}/cmd/ota` | 保留旧 Topic 一段时间 |
| `/restart` | `/sbn/v1/{id}/cmd/restart` | 并行支持 |
| `/ESP32_info` | `/sbn/v1/{id}/state/device` | 新格式结构化字段 |
| `/motor/status` | `/sbn/v1/{id}/state/motor` | 双发过渡 |
| `/ir_info` | `/sbn/v1/{id}/telemetry/perception` | 合并入 `ir` 子字段 |

**迁移建议：**
- Phase 1（1~2周）：双写双读
- Phase 2（1周）：上位机切换为新 Topic
- Phase 3（1周）：旧 Topic 仅保留只读/兼容

---

## 7. 校验规则（建议实现）

1. `device_id` 不匹配：忽略并回 ACK（`BAD_DEVICE_ID`）可选。
2. `ttl_ms` 超时：拒绝执行（`CMD_EXPIRED`）。
3. `speedA/speedB` 超范围：拒绝或钳制（建议拒绝+明确 ACK）。
4. `mission.steps` 空：`BAD_PARAM_RANGE`。
5. 任务模式外收到任务控制：`MODE_CONFLICT`。

---

## 8. 最小实现清单（开发任务）

### 固件侧
- [ ] 增加新 Topic 订阅与路由
- [ ] 增加统一 ACK 发布函数
- [ ] 增加 `state/device` 与 `state/mission`
- [ ] 增加 `telemetry/perception` 与 `telemetry/decision`
- [ ] 保留旧 Topic 兼容路径

### 控制面板侧
- [ ] 增加 `device_id` 输入
- [ ] 命令携带 `msg_id`、`ttl_ms`
- [ ] 新增 ACK 订阅与展示区
- [ ] 新增 Mission 面板（start/pause/resume/abort）

---

## 9. 示例：失败 ACK

```json
{
  "msg_id": "693bd284-f853-42f1-b6c5-5d662cc1af95",
  "device_id": "boat-001",
  "ok": false,
  "code": "BAD_PARAM_RANGE",
  "detail": "speedA out of range: 400",
  "ts_ms": 2233445,
  "schema_ver": 1
}
```

---

## 10. 版本策略

- `schema_ver` 按 major 兼容。
- `v1.x` 仅新增字段，不删除字段。
- 破坏性变更使用 `v2` 前缀（如 `/sbn/v2/...`）。

---

## 11. 附录：推荐保留字段（便于实验分析）

- `experiment_id`: 试验编号
- `policy_id`: 当前策略版本
- `env_tag`: 场景标签（indoor_pool / outdoor_lake）
- `latency_ms`: 指令到执行延迟
- `drop_rate`: 遥测丢包率估计

这些字段可大幅降低后期“问题复现与参数回归”的成本。
