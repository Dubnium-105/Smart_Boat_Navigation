# 可靠性与安全机制说明

本分支补充船端 Fail-safe 安全保护框架，面向 ESP32-S3 智能船舶控制系统已有的 WiFi 重连、4G 兜底、MQTT 指令、重启与 OTA 基础能力，新增统一安全管理模块 `safety_manager`，并将安全总闸接入主循环、电机输出和自动导航差速控制。

## 已补充内容

### 1. 心跳超时停车

`safety_manager` 维护 `lastHeartbeatMs` 和 `SAFETY_HEARTBEAT_TIMEOUT_MS`。当控制端心跳超时后，系统会进入 fail-safe 状态并立即调用电机停车逻辑，将左右电机 PWM 输出置零。该机制用于处理上位机断开、MQTT 掉线、WiFi/4G 链路异常、控制端程序崩溃等情况。

默认超时时间为 3000 ms，可通过 PlatformIO build flags 覆盖：

```ini
-DSAFETY_HEARTBEAT_TIMEOUT_MS=3000
```

### 2. 地理围栏越界保护

新增圆形地理围栏接口：

- `safetySetHome(latitude, longitude)`：设置返航/安全中心点；
- `safetySetGeoFence(centerLatitude, centerLongitude, radiusMeters)`：设置围栏中心和半径；
- `safetyEnableGeoFence(true/false)`：启停围栏保护；
- `safetyUpdateTelemetry(latitude, longitude, batteryVoltage)`：更新定位和电池遥测。

当定位有效且设备距离围栏中心超过半径时，系统会触发 `geofence_violation`，停车并进入返航标记状态。未接 GPS 时默认不误触发，便于分阶段接入真实定位模块。

### 3. 电量低返航保护

安全模块预留电源监测入口，默认低电阈值为 6.80 V：

```ini
-DSAFETY_LOW_BATTERY_VOLTAGE=6.80
```

当 `safetyUpdateTelemetry()` 传入的电压低于阈值后，系统会触发 `low_battery_return_home`，停止当前电机输出并设置 `returningHome=true`。目前代码层面完成低电保护与返航状态切换，后续可在任务执行器中根据 `returningHome` 接入 GPS 返航路径规划。

### 4. OTA 灰度发布与回滚镜像

新增 OTA 灰度判断和回滚接口：

- `safetyShouldAcceptOta(rolloutPercent)`：按设备 ID 稳定散列到 0~99 分桶，仅允许命中灰度比例的设备升级；
- `safetyPrepareOta(rollbackAllowed)`：升级前停车并记录是否允许回滚；
- `safetyMarkBootSuccessful()`：新固件启动后标记镜像有效，取消回滚挂起；
- `safetyRequestRollback(reason)`：请求回滚并重启。

该部分基于 ESP-IDF OTA 回滚接口实现，实际回滚效果依赖分区表和 bootloader 是否启用 app rollback 支持。

### 5. 指令白名单与速率限制

`safetyAcceptCommand()` 提供统一指令白名单和限速判断，当前内置白名单包括：

- `heartbeat`
- `legacy_motor`
- `legacy_navigation`
- `legacy_ota`
- `legacy_restart`
- `legacy_shutdown`
- `motor`
- `nav_mode`
- `mission`
- `ota`
- `restart`
- `safety`

默认最小指令间隔为 80 ms，可通过 build flags 覆盖：

```ini
-DSAFETY_COMMAND_MIN_INTERVAL_MS=80
```

### 6. 电机输出安全总闸

`motor_control()` 中加入了非零 PWM 的安全总闸：只要 `safetyAllowsMotion()` 返回 false，所有非零电机输出都会被钳制为 0。停车指令本身不会被拦截，避免 fail-safe 触发停车时出现递归阻塞。

红外自动导航 `motor_control_ir_auto()` 和 `motor_control_ir_navigation()` 也在差速计算后调用 `safetyGuardMotorCommand()`，确保自动导航输出同样受心跳、围栏和电池保护约束。

## 状态上报

安全模块提供 `safetyStatusJson()`，可生成以下关键字段：

- `heartbeat_ok`
- `geofence_ok`
- `battery_ok`
- `command_ok`
- `fail_safe_active`
- `returning_home`
- `battery_voltage`
- `latitude`
- `longitude`
- `last_stop_reason`
- `geofence_enabled`
- `geofence_radius_m`

可在 MQTT 侧发布到 `/sbn/v1/{device_id}/state/safety`，供上位机展示。

## 当前接入点

- `src/main.cpp`：启动阶段初始化安全模块，循环阶段执行 `safetyLoop()`；
- `src/motor_control.cpp`：所有非零电机输出经过安全总闸；
- `include/safety_manager.h` / `src/safety_manager.cpp`：实现 Fail-safe 状态机、围栏、低电、灰度 OTA、回滚和指令限速。

