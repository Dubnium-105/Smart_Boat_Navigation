import json
import struct
import threading
import time
import uuid
import webbrowser
import zlib
from collections import deque
from dataclasses import dataclass
from typing import Any, Dict, Optional, Tuple

import paho.mqtt.client as mqtt
import tkinter as tk
from tkinter import messagebox
from tkinter import ttk

MQTT_BROKER = "broker.emqx.io"
MQTT_PORT = 1883

MQTT_TOPIC_MOTOR_CMD_LEGACY = "/motor"
MQTT_TOPIC_MOTOR_STATUS = "/motor/status"
MQTT_TOPIC_ESP32_INFO = "/ESP32_info"
MQTT_TOPIC_ESP32_STATUS = "/esp32/status"
MQTT_TOPIC_BRIGHT_POINT = "/esp32/bright_point"

PROTOCOL_VERSION = 2
SCHEMA_VERSION = 2
VIDEO_STREAM_URL_FORMAT = "http://{}/stream"

mqtt_client = None
is_connected = False

last_motor_status = "未知"
last_esp32_info = "离线"
last_esp32_status = {"fps": 0.0, "busy": False}
last_bright_point = {"x": -1, "y": -1}
last_ack_message = "尚未收到ACK"
last_telemetry_summary = "尚未收到二进制遥测"

esp32_ip = "未知"
legacy_target_client_id = ""

pending_lock = threading.Lock()
pending_acks = {}
ack_history = deque(maxlen=30)


@dataclass
class PendingAck:
    msg_id: str
    event: threading.Event
    ack_payload: Optional[Dict[str, Any]] = None


def now_unix_ms() -> int:
    return int(time.time() * 1000)


def parse_topic_device_id(topic: str) -> str:
    parts = topic.split("/")
    if len(parts) >= 4 and parts[1] == "sbn":
        return parts[3]
    return ""


def v2_topic(device_id: str, channel: str, name: str) -> str:
    return f"/sbn/v2/{device_id}/{channel}/{name}"


def v1_topic(device_id: str, channel: str, name: str) -> str:
    return f"/sbn/v1/{device_id}/{channel}/{name}"


def make_command_envelope(device_id: str, client_id: str, token: str, msg_name: str,
                          msg_id: str, ttl_ms: int, data: dict) -> dict:
    return {
        "protocol_ver": PROTOCOL_VERSION,
        "schema_ver": SCHEMA_VERSION,
        "msg_type": "command",
        "msg_name": msg_name,
        "msg_id": msg_id,
        "trace_id": f"trace-{msg_id[:8]}",
        "device_id": device_id,
        "client_id": client_id,
        "token": token,
        "ttl_ms": ttl_ms,
        "ts_unix_ms": now_unix_ms(),
        "data": data,
    }


def update_detected_device(device_id: str):
    if not device_id:
        return
    if device_id_var.get().strip() == "":
        device_id_var.set(device_id)


def append_ack_history(line: str):
    ack_history.appendleft(line)
    ack_history_var.set("\n".join(list(ack_history)))


def decode_binary_telemetry(payload: bytes) -> str:
    if len(payload) < 24:
        return "遥测包长度不足"

    magic, protocol_ver, codec, sample_count, raw_len, payload_len, seq, crc_raw, batch_ts = struct.unpack_from(
        "<4sBBHHHIII", payload, 0
    )
    if magic != b"SBN2":
        return "遥测包魔数不匹配"

    body = payload[24:24 + payload_len]
    if len(body) != payload_len:
        return "遥测包负载长度不一致"

    if codec == 0:
        raw = body
    elif codec == 1:
        raw_buf = bytearray()
        if len(body) % 2 != 0:
            return "RLE负载长度非法"
        for i in range(0, len(body), 2):
            count = body[i]
            value = body[i + 1]
            raw_buf.extend(bytes([value]) * count)
        raw = bytes(raw_buf)
    else:
        return f"未知压缩编码: {codec}"

    if len(raw) != raw_len:
        return f"解压后长度不匹配 raw={len(raw)} expected={raw_len}"

    check_crc = zlib.crc32(raw) & 0xFFFFFFFF
    if check_crc != crc_raw:
        return f"CRC校验失败 got={check_crc} expected={crc_raw}"

    sample_bytes = 12
    if raw_len % sample_bytes != 0:
        return "样本字节长度非法"

    parsed_count = raw_len // sample_bytes
    if parsed_count != sample_count:
        return f"样本数量不一致 parsed={parsed_count} expected={sample_count}"

    kind_counters = {}
    for offset in range(0, raw_len, sample_bytes):
        kind, nav_mode, value1, value2, ts_ms, extra = struct.unpack_from("<BBbbIi", raw, offset)
        kind_counters[kind] = kind_counters.get(kind, 0) + 1

    ratio = payload_len / raw_len if raw_len else 1.0
    kind_info = ", ".join([f"k{kind}:{count}" for kind, count in sorted(kind_counters.items())])
    return (
        f"seq={seq} samples={sample_count} codec={codec} ratio={ratio:.2f} "
        f"batch_ts={batch_ts} kinds={kind_info}"
    )


def on_connect(client, userdata, flags, rc, properties=None):
    global is_connected
    if rc == 0:
        is_connected = True
        root.after(0, update_mqtt_status, "已连接")

        client.subscribe(MQTT_TOPIC_MOTOR_CMD_LEGACY)
        client.subscribe(MQTT_TOPIC_MOTOR_STATUS)
        client.subscribe(MQTT_TOPIC_ESP32_INFO)
        client.subscribe(MQTT_TOPIC_ESP32_STATUS)
        client.subscribe(MQTT_TOPIC_BRIGHT_POINT)

        client.subscribe("/sbn/+/+/ack/cmd")
        client.subscribe("/sbn/+/+/state/#")
        client.subscribe("/sbn/+/+/telemetry/binary")

        root.after(0, update_esp32_info_label, "等待设备状态...")
    else:
        is_connected = False
        root.after(0, update_mqtt_status, f"连接失败 code={rc}")


def on_disconnect(client, userdata, rc, properties=None):
    global is_connected
    is_connected = False
    root.after(0, update_mqtt_status, "已断开")


def on_message(client, userdata, msg):
    global last_motor_status, last_esp32_info, last_esp32_status, last_bright_point
    global esp32_ip, legacy_target_client_id, last_ack_message, last_telemetry_summary

    topic = msg.topic
    payload_raw = msg.payload

    if topic.endswith("/ack/cmd"):
        try:
            data = json.loads(payload_raw.decode("utf-8"))
            msg_id = data.get("msg_id", "")
            if msg_id:
                with pending_lock:
                    pending = pending_acks.get(msg_id)
                    if pending is not None:
                        pending.ack_payload = data
                        pending.event.set()

            ack_ok = data.get("ok", False)
            code = data.get("code", "UNKNOWN")
            detail = data.get("detail", "")
            duplicate = data.get("duplicate", False)
            retryable = data.get("retryable", False)
            last_ack_message = (
                f"msg_id={msg_id} ok={ack_ok} code={code} duplicate={duplicate} "
                f"retryable={retryable} detail={detail}"
            )
            root.after(0, update_ack_label, last_ack_message)
            root.after(0, append_ack_history, last_ack_message)
        except Exception as exc:
            root.after(0, update_ack_label, f"ACK解析失败: {exc}")
        return

    if topic.endswith("/telemetry/binary"):
        summary = decode_binary_telemetry(payload_raw)
        last_telemetry_summary = summary
        root.after(0, update_telemetry_label, summary)
        return

    try:
        payload_str = payload_raw.decode("utf-8")
    except UnicodeDecodeError:
        payload_str = "<binary>"

    if topic == MQTT_TOPIC_MOTOR_STATUS:
        last_motor_status = payload_str
        root.after(0, update_motor_status_label, last_motor_status)
        return

    if topic == MQTT_TOPIC_ESP32_INFO:
        last_esp32_info = payload_str
        root.after(0, update_esp32_info_label, payload_str)
        try:
            info = json.loads(payload_str)
            if isinstance(info, dict):
                if "ip" in info:
                    esp32_ip = str(info["ip"])
                    root.after(0, update_esp32_ip_label, esp32_ip)
                if "clientId" in info:
                    legacy_target_client_id = str(info["clientId"])
                    root.after(0, update_legacy_client_label, legacy_target_client_id)
                if "device_id" in info and info["device_id"]:
                    update_detected_device(str(info["device_id"]))
        except json.JSONDecodeError:
            if "IP:" in payload_str:
                try:
                    ip_part = payload_str.split("IP:")[1].strip()
                    esp32_ip = ip_part
                    root.after(0, update_esp32_ip_label, esp32_ip)
                except IndexError:
                    pass
        return

    if topic == MQTT_TOPIC_ESP32_STATUS:
        try:
            data = json.loads(payload_str)
            last_esp32_status["fps"] = data.get("fps", last_esp32_status["fps"])
            last_esp32_status["busy"] = data.get("busy", last_esp32_status["busy"])
            if "ip" in data:
                esp32_ip = str(data["ip"])
                root.after(0, update_esp32_ip_label, esp32_ip)
            root.after(0, update_esp32_status_labels)
        except json.JSONDecodeError:
            pass
        return

    if topic == MQTT_TOPIC_BRIGHT_POINT:
        try:
            data = json.loads(payload_str)
            last_bright_point["x"] = data.get("x", last_bright_point["x"])
            last_bright_point["y"] = data.get("y", last_bright_point["y"])
            root.after(0, update_bright_point_label)
        except json.JSONDecodeError:
            pass
        return

    if "/state/device" in topic:
        try:
            data = json.loads(payload_str)
            if isinstance(data, dict):
                device_id = data.get("device_id") or parse_topic_device_id(topic)
                if device_id:
                    update_detected_device(str(device_id))
                stream_url = data.get("stream_url", "")
                if stream_url.startswith("http://"):
                    try:
                        ip_part = stream_url.split("http://", 1)[1].split("/", 1)[0]
                        esp32_ip = ip_part
                        root.after(0, update_esp32_ip_label, esp32_ip)
                    except Exception:
                        pass
                root.after(0, update_esp32_info_label, json.dumps(data, ensure_ascii=False))
        except Exception:
            pass


def connect_mqtt():
    global mqtt_client
    client_id = f"python-mqtt-control-panel-{int(time.time())}"
    client_id_var.set(client_id)

    mqtt_client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=client_id)
    mqtt_client.on_connect = on_connect
    mqtt_client.on_disconnect = on_disconnect
    mqtt_client.on_message = on_message

    try:
        mqtt_client.connect(MQTT_BROKER, MQTT_PORT, 60)
        threading.Thread(target=mqtt_client.loop_forever, daemon=True).start()
    except Exception as exc:
        messagebox.showerror("MQTT错误", f"无法连接MQTT: {exc}")
        update_mqtt_status("连接失败")


def publish_json(topic: str, payload: dict) -> bool:
    if mqtt_client is None:
        return False
    body = json.dumps(payload, ensure_ascii=False)
    result = mqtt_client.publish(topic, body)
    result.wait_for_publish(timeout=3.0)
    return result.rc == mqtt.MQTT_ERR_SUCCESS


def send_command_with_retry(command_name: str, data: dict, ttl_ms: int) -> Tuple[bool, Optional[Dict[str, Any]], str]:
    if not is_connected:
        return False, None, "MQTT未连接"

    device_id = device_id_var.get().strip()
    client_id = client_id_var.get().strip()
    token = token_var.get().strip()

    if not device_id:
        return False, None, "device_id 不能为空"
    if not client_id:
        return False, None, "client_id 不能为空"
    if not token:
        return False, None, "token 不能为空"

    timeout_s = float(timeout_var.get())
    retries = int(retry_var.get())

    topic = v2_topic(device_id, "cmd", command_name)
    msg_id = str(uuid.uuid4())
    envelope = make_command_envelope(device_id, client_id, token, command_name, msg_id, ttl_ms, data)

    pending = PendingAck(msg_id=msg_id, event=threading.Event(), ack_payload=None)
    with pending_lock:
        pending_acks[msg_id] = pending

    try:
        for attempt in range(1, retries + 2):
            pending.event.clear()
            pending.ack_payload = None

            if not publish_json(topic, envelope):
                time.sleep(0.2)
                continue

            root.after(0, update_send_state_label, f"等待ACK: {msg_id} attempt={attempt}/{retries + 1}")
            got_ack = pending.event.wait(timeout=timeout_s)

            if got_ack and pending.ack_payload is not None:
                ack = pending.ack_payload
                ok = bool(ack.get("ok", False))
                if ok:
                    return True, ack, "执行成功"

                retryable = bool(ack.get("retryable", False))
                code = ack.get("code", "UNKNOWN")
                detail = ack.get("detail", "")
                if retryable and attempt <= retries:
                    root.after(0, update_send_state_label, f"收到可重试NACK code={code}，准备重试")
                    continue
                return False, ack, f"NACK code={code} detail={detail}"

            if attempt <= retries:
                root.after(0, update_send_state_label, f"ACK超时，开始重试 {attempt}/{retries}")
                continue

            return False, None, "ACK超时"
    finally:
        with pending_lock:
            pending_acks.pop(msg_id, None)


def send_legacy_motor_command(speed_a: int, speed_b: int) -> bool:
    if not legacy_target_client_id:
        return False
    payload = {
        "clientId": legacy_target_client_id,
        "speedA": speed_a,
        "speedB": speed_b,
    }
    return publish_json(MQTT_TOPIC_MOTOR_CMD_LEGACY, payload)


def send_motor_command():
    def worker():
        try:
            speed_a = int(motor_a_speed_scale.get())
            speed_b = int(motor_b_speed_scale.get())
            ttl_ms = int(ttl_var.get())
        except ValueError:
            root.after(0, lambda: messagebox.showerror("输入错误", "速度/TTL 需为整数"))
            return

        ok, ack, message = send_command_with_retry(
            "motor",
            {
                "mode": "direct_pwm",
                "speedA": speed_a,
                "speedB": speed_b,
            },
            ttl_ms=ttl_ms,
        )

        if not ok and legacy_fallback_var.get():
            fallback_ok = send_legacy_motor_command(speed_a, speed_b)
            if fallback_ok:
                message = f"新协议失败，旧协议已发送: {message}"
                ok = True

        if ok:
            root.after(0, update_send_state_label, f"发送完成: {message}")
        else:
            root.after(0, update_send_state_label, f"发送失败: {message}")
            root.after(0, lambda: messagebox.showwarning("命令失败", message))

    threading.Thread(target=worker, daemon=True).start()


def send_nav_mode(mode: str):
    def worker():
        ttl_ms = int(ttl_var.get())
        ok, _, message = send_command_with_retry("nav_mode", {"mode": mode}, ttl_ms=ttl_ms)
        if ok:
            root.after(0, update_send_state_label, f"导航模式已切换: {mode}")
        else:
            root.after(0, update_send_state_label, f"导航切换失败: {message}")

    threading.Thread(target=worker, daemon=True).start()


def send_protocol_config():
    def worker():
        try:
            batch_ms = int(batch_ms_var.get())
            max_samples = int(max_samples_var.get())
            ttl_ms = int(ttl_var.get())
        except ValueError:
            root.after(0, lambda: messagebox.showerror("输入错误", "batch/max/ttl 需为整数"))
            return

        data = {
            "telemetry": {
                "batch_ms": batch_ms,
                "max_samples": max_samples,
                "http_enable": bool(http_enable_var.get()),
                "http_url": http_url_var.get().strip(),
            }
        }
        ok, _, message = send_command_with_retry("protocol_cfg", data, ttl_ms=ttl_ms)
        if ok:
            root.after(0, update_send_state_label, "协议配置下发成功")
        else:
            root.after(0, update_send_state_label, f"协议配置下发失败: {message}")

    threading.Thread(target=worker, daemon=True).start()


def stop_motors():
    motor_a_speed_scale.set(0)
    motor_b_speed_scale.set(0)
    send_motor_command()


def open_video_stream():
    if esp32_ip == "未知":
        messagebox.showwarning("无IP地址", "当前未获取设备IP")
        return
    url = VIDEO_STREAM_URL_FORMAT.format(esp32_ip)
    try:
        webbrowser.open(url)
    except Exception as exc:
        messagebox.showerror("打开失败", str(exc))


def update_mqtt_status(status: str):
    mqtt_status_var.set(f"MQTT状态: {status}")


def update_motor_status_label(status: str):
    motor_status_var.set(f"电机状态: {status}")


def update_esp32_info_label(info: str):
    esp32_info_var.set(f"ESP32信息: {info}")


def update_esp32_ip_label(ip: str):
    esp32_ip_var.set(f"ESP32 IP: {ip}")
    video_stream_button.config(state=tk.NORMAL if ip != "未知" else tk.DISABLED)


def update_esp32_status_labels():
    fps_var.set(f"FPS: {last_esp32_status.get('fps', 0.0):.2f}")
    busy_status = "是" if last_esp32_status.get("busy", False) else "否"
    busy_var.set(f"系统繁忙: {busy_status}")


def update_bright_point_label():
    x = last_bright_point.get("x", -1)
    y = last_bright_point.get("y", -1)
    bright_point_var.set(f"最亮点: (X={x}, Y={y})")


def update_ack_label(text: str):
    ack_var.set(f"最后ACK: {text}")


def update_telemetry_label(text: str):
    telemetry_var.set(f"二进制遥测: {text}")


def update_send_state_label(text: str):
    send_state_var.set(f"发送状态: {text}")


def update_legacy_client_label(text: str):
    legacy_client_var.set(f"旧协议 clientId: {text if text else '未知'}")


root = tk.Tk()
root.title("Smart Boat 协议控制面板")
root.geometry("840x780")

main_frame = ttk.Frame(root, padding=(10, 10))
main_frame.pack(fill="both", expand=True)

config_frame = ttk.LabelFrame(main_frame, text="连接与鉴权配置", padding=(8, 8))
config_frame.pack(fill="x", pady=6)

mqtt_status_var = tk.StringVar(value="MQTT状态: 未连接")
ack_var = tk.StringVar(value="最后ACK: 尚未收到ACK")
telemetry_var = tk.StringVar(value="二进制遥测: 尚未收到")
send_state_var = tk.StringVar(value="发送状态: 空闲")
legacy_client_var = tk.StringVar(value="旧协议 clientId: 未知")

client_id_var = tk.StringVar(value=f"python-panel-{int(time.time())}")
device_id_var = tk.StringVar(value="")
token_var = tk.StringVar(value="sbn-operator-token")
ttl_var = tk.StringVar(value="1500")
timeout_var = tk.StringVar(value="1.2")
retry_var = tk.StringVar(value="2")
legacy_fallback_var = tk.BooleanVar(value=True)

batch_ms_var = tk.StringVar(value="1500")
max_samples_var = tk.StringVar(value="24")
http_enable_var = tk.BooleanVar(value=False)
http_url_var = tk.StringVar(value="")

row = 0
ttk.Label(config_frame, text="device_id:").grid(row=row, column=0, sticky="w", padx=4, pady=4)
ttk.Entry(config_frame, textvariable=device_id_var, width=24).grid(row=row, column=1, sticky="w", padx=4, pady=4)
ttk.Label(config_frame, text="client_id:").grid(row=row, column=2, sticky="w", padx=4, pady=4)
ttk.Entry(config_frame, textvariable=client_id_var, width=24).grid(row=row, column=3, sticky="w", padx=4, pady=4)

row += 1
ttk.Label(config_frame, text="token:").grid(row=row, column=0, sticky="w", padx=4, pady=4)
ttk.Entry(config_frame, textvariable=token_var, width=24, show="*").grid(row=row, column=1, sticky="w", padx=4, pady=4)
ttk.Label(config_frame, text="ttl_ms:").grid(row=row, column=2, sticky="w", padx=4, pady=4)
ttk.Entry(config_frame, textvariable=ttl_var, width=10).grid(row=row, column=3, sticky="w", padx=4, pady=4)

row += 1
ttk.Label(config_frame, text="ack_timeout(s):").grid(row=row, column=0, sticky="w", padx=4, pady=4)
ttk.Entry(config_frame, textvariable=timeout_var, width=10).grid(row=row, column=1, sticky="w", padx=4, pady=4)
ttk.Label(config_frame, text="retry_count:").grid(row=row, column=2, sticky="w", padx=4, pady=4)
ttk.Entry(config_frame, textvariable=retry_var, width=10).grid(row=row, column=3, sticky="w", padx=4, pady=4)

row += 1
ttk.Checkbutton(config_frame, text="ACK失败时尝试旧协议电机Topic", variable=legacy_fallback_var).grid(
    row=row, column=0, columnspan=4, sticky="w", padx=4, pady=4
)

row += 1
ttk.Button(config_frame, text="下发协议配置(Admin)", command=send_protocol_config).grid(
    row=row, column=3, sticky="e", padx=4, pady=4
)

status_frame = ttk.LabelFrame(main_frame, text="状态信息", padding=(8, 8))
status_frame.pack(fill="x", pady=6)

esp32_info_var = tk.StringVar(value="ESP32信息: 离线")
esp32_ip_var = tk.StringVar(value="ESP32 IP: 未知")
fps_var = tk.StringVar(value="FPS: 0.00")
busy_var = tk.StringVar(value="系统繁忙: 否")
bright_point_var = tk.StringVar(value="最亮点: (X=-1, Y=-1)")
motor_status_var = tk.StringVar(value="电机状态: 未知")
ack_history_var = tk.StringVar(value="")

for var in [
    mqtt_status_var,
    esp32_info_var,
    esp32_ip_var,
    fps_var,
    busy_var,
    bright_point_var,
    motor_status_var,
    legacy_client_var,
    ack_var,
    telemetry_var,
    send_state_var,
]:
    ttk.Label(status_frame, textvariable=var, anchor="w").pack(fill="x", pady=1)

ack_frame = ttk.LabelFrame(main_frame, text="ACK历史", padding=(8, 8))
ack_frame.pack(fill="x", pady=6)

ttk.Label(ack_frame, textvariable=ack_history_var, justify="left").pack(fill="x")

motor_frame = ttk.LabelFrame(main_frame, text="电机控制", padding=(8, 8))
motor_frame.pack(fill="x", pady=6)

ttk.Label(motor_frame, text="电机A速度 (-100~100):").grid(row=0, column=0, sticky="w", padx=4, pady=4)
motor_a_speed_scale = ttk.Scale(motor_frame, from_=-100, to=100, orient=tk.HORIZONTAL, length=420)
motor_a_speed_scale.set(0)
motor_a_speed_scale.grid(row=0, column=1, sticky="w", padx=4, pady=4)
motor_a_value = ttk.Label(motor_frame, text="0")
motor_a_value.grid(row=0, column=2, sticky="w", padx=4, pady=4)
motor_a_speed_scale.config(command=lambda v: motor_a_value.config(text=f"{int(float(v))}"))

ttk.Label(motor_frame, text="电机B速度 (-100~100):").grid(row=1, column=0, sticky="w", padx=4, pady=4)
motor_b_speed_scale = ttk.Scale(motor_frame, from_=-100, to=100, orient=tk.HORIZONTAL, length=420)
motor_b_speed_scale.set(0)
motor_b_speed_scale.grid(row=1, column=1, sticky="w", padx=4, pady=4)
motor_b_value = ttk.Label(motor_frame, text="0")
motor_b_value.grid(row=1, column=2, sticky="w", padx=4, pady=4)
motor_b_speed_scale.config(command=lambda v: motor_b_value.config(text=f"{int(float(v))}"))

button_row = ttk.Frame(motor_frame)
button_row.grid(row=2, column=0, columnspan=3, sticky="w", padx=4, pady=8)

ttk.Button(button_row, text="发送电机命令", command=send_motor_command).pack(side=tk.LEFT, padx=4)
ttk.Button(button_row, text="停止电机", command=stop_motors).pack(side=tk.LEFT, padx=4)

auto_frame = ttk.LabelFrame(main_frame, text="导航模式", padding=(8, 8))
auto_frame.pack(fill="x", pady=6)

ttk.Button(auto_frame, text="手动", command=lambda: send_nav_mode("manual")).pack(side=tk.LEFT, padx=4)
ttk.Button(auto_frame, text="导航", command=lambda: send_nav_mode("navigate")).pack(side=tk.LEFT, padx=4)
ttk.Button(auto_frame, text="任务", command=lambda: send_nav_mode("mission")).pack(side=tk.LEFT, padx=4)

cfg_frame = ttk.LabelFrame(main_frame, text="协议配置下发 (需要Admin Token)", padding=(8, 8))
cfg_frame.pack(fill="x", pady=6)

ttk.Label(cfg_frame, text="batch_ms:").grid(row=0, column=0, sticky="w", padx=4, pady=4)
ttk.Entry(cfg_frame, textvariable=batch_ms_var, width=10).grid(row=0, column=1, sticky="w", padx=4, pady=4)
ttk.Label(cfg_frame, text="max_samples:").grid(row=0, column=2, sticky="w", padx=4, pady=4)
ttk.Entry(cfg_frame, textvariable=max_samples_var, width=10).grid(row=0, column=3, sticky="w", padx=4, pady=4)

ttk.Checkbutton(cfg_frame, text="启用HTTP上传", variable=http_enable_var).grid(row=1, column=0, sticky="w", padx=4, pady=4)
ttk.Label(cfg_frame, text="http_url:").grid(row=1, column=1, sticky="w", padx=4, pady=4)
ttk.Entry(cfg_frame, textvariable=http_url_var, width=44).grid(row=1, column=2, columnspan=2, sticky="w", padx=4, pady=4)

ttk.Button(cfg_frame, text="下发协议配置", command=send_protocol_config).grid(row=2, column=0, padx=4, pady=6, sticky="w")

video_frame = ttk.LabelFrame(main_frame, text="视频流", padding=(8, 8))
video_frame.pack(fill="x", pady=6)

video_stream_button = ttk.Button(video_frame, text="打开视频流", command=open_video_stream, state=tk.DISABLED)
video_stream_button.pack(anchor="w")


if __name__ == "__main__":
    connect_mqtt()
    root.mainloop()

    if mqtt_client is not None:
        try:
            mqtt_client.disconnect()
        except Exception:
            pass
