import tkinter as tk
from tkinter import ttk
from tkinter import messagebox
import paho.mqtt.client as mqtt
import json
import threading
import time
import webbrowser

# --- 配置 ---
MQTT_BROKER = "emqx.link2you.top"
MQTT_PORT = 1883
MQTT_TOPIC_MOTOR_CMD = "/motor"
MQTT_TOPIC_MOTOR_STATUS = "/motor/status" # 假设 ESP32 会发布电机状态
MQTT_TOPIC_ESP32_INFO = "/ESP32_info"    # ESP32 上线信息
MQTT_TOPIC_ESP32_STATUS = "/esp32/status" # 假设 ESP32 发布状态 (IP, FPS, Busy)
MQTT_TOPIC_BRIGHT_POINT = "/esp32/bright_point" # 假设 ESP32 发布最亮点

ESP32_IP = "未知" # 将通过 MQTT 更新
VIDEO_STREAM_URL_FORMAT = "http://{}/" # 假设的视频流 URL 格式

# --- 全局变量 ---
mqtt_client = None
is_connected = False
last_motor_status = "未知"
last_esp32_info = "离线"
last_esp32_status = {"fps": 0.0, "busy": False}
last_bright_point = {"x": -1, "y": -1}

# --- MQTT 函数 ---

def on_connect(client, userdata, flags, rc, properties=None):
    global is_connected, root
    if rc == 0:
        print("成功连接到 MQTT Broker")
        is_connected = True
        client.subscribe(MQTT_TOPIC_MOTOR_CMD) # 监听自己发出的命令（可选）
        client.subscribe(MQTT_TOPIC_MOTOR_STATUS)
        client.subscribe(MQTT_TOPIC_ESP32_INFO)
        client.subscribe(MQTT_TOPIC_ESP32_STATUS)
        client.subscribe(MQTT_TOPIC_BRIGHT_POINT)
        # 安全地更新 GUI
        root.after(0, update_mqtt_status, "已连接")
        root.after(0, update_esp32_info_label, "等待 ESP32 上线...")
    else:
        print(f"连接失败, 返回码: {rc}")
        is_connected = False
        # 安全地更新 GUI
        root.after(0, update_mqtt_status, f"连接失败 (Code: {rc})")

def on_disconnect(client, userdata, rc, properties=None):
    global is_connected, root
    print(f"与 MQTT Broker 断开连接, 返回码: {rc}")
    is_connected = False
    # 安全地更新 GUI
    root.after(0, update_mqtt_status, "已断开")
    root.after(0, update_esp32_info_label, "离线")
    root.after(0, update_esp32_ip_label, "未知")
    # 尝试重连
    # time.sleep(5)
    # connect_mqtt() # 简单重连逻辑，可能需要更健壮的实现

def on_message(client, userdata, msg):
    global root, last_motor_status, last_esp32_info, last_esp32_status, last_bright_point, ESP32_IP
    topic = msg.topic
    try:
        payload_str = msg.payload.decode("utf-8")
        print(f"收到消息: 主题='{topic}', 内容='{payload_str}'")

        if topic == MQTT_TOPIC_MOTOR_STATUS:
            last_motor_status = payload_str
            root.after(0, update_motor_status_label, last_motor_status)
        elif topic == MQTT_TOPIC_ESP32_INFO:
            last_esp32_info = payload_str
            root.after(0, update_esp32_info_label, last_esp32_info)
            # 尝试从上线消息中提取IP (如果格式固定)
            if "IP:" in payload_str:
                 try:
                     ip_part = payload_str.split("IP:")[1].strip()
                     ESP32_IP = ip_part
                     root.after(0, update_esp32_ip_label, ESP32_IP)
                 except IndexError:
                     print("无法从 ESP32_info 消息中解析 IP")

        elif topic == MQTT_TOPIC_ESP32_STATUS:
            try:
                data = json.loads(payload_str)
                last_esp32_status["fps"] = data.get("fps", last_esp32_status["fps"])
                last_esp32_status["busy"] = data.get("busy", last_esp32_status["busy"])
                # 如果状态消息包含 IP 地址
                if "ip" in data:
                    ESP32_IP = data["ip"]
                    root.after(0, update_esp32_ip_label, ESP32_IP)
                root.after(0, update_esp32_status_labels)
            except json.JSONDecodeError:
                print(f"无法解析 ESP32 状态 JSON: {payload_str}")
        elif topic == MQTT_TOPIC_BRIGHT_POINT:
             try:
                data = json.loads(payload_str)
                last_bright_point["x"] = data.get("x", last_bright_point["x"])
                last_bright_point["y"] = data.get("y", last_bright_point["y"])
                root.after(0, update_bright_point_label)
             except json.JSONDecodeError:
                print(f"无法解析最亮点 JSON: {payload_str}")

    except Exception as e:
        print(f"处理消息时出错: {e}")
        print(f"原始 Payload: {msg.payload}")


def connect_mqtt():
    global mqtt_client
    client_id = f'python-mqtt-control-panel-{time.time()}'
    mqtt_client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=client_id)
    mqtt_client.on_connect = on_connect
    mqtt_client.on_disconnect = on_disconnect
    mqtt_client.on_message = on_message
    try:
        print(f"尝试连接到 MQTT Broker: {MQTT_BROKER}:{MQTT_PORT}")
        mqtt_client.connect(MQTT_BROKER, MQTT_PORT, 60)
        # 在单独的线程中运行 MQTT 循环
        thread = threading.Thread(target=mqtt_client.loop_forever)
        thread.daemon = True # 允许主程序退出时线程也退出
        thread.start()
    except Exception as e:
        print(f"MQTT 连接失败: {e}")
        messagebox.showerror("MQTT 错误", f"无法连接到 MQTT Broker: {e}")
        update_mqtt_status(f"连接失败")


def send_motor_command():
    if not is_connected:
        messagebox.showwarning("MQTT 未连接", "请先连接到 MQTT Broker。")
        return

    try:
        speed_a = int(motor_a_speed_scale.get())
        speed_b = int(motor_b_speed_scale.get())

        command = {"speedA": speed_a, "speedB": speed_b}
        payload = json.dumps(command)

        result = mqtt_client.publish(MQTT_TOPIC_MOTOR_CMD, payload)
        result.wait_for_publish(timeout=5.0) # 等待发布完成或超时

        if result.rc == mqtt.MQTT_ERR_SUCCESS:
             print(f"成功发送电机命令: {payload}")
        else:
             print(f"发送电机命令失败, rc={result.rc}")
             messagebox.showerror("发送失败", f"无法发送电机命令 (Code: {result.rc})")

    except ValueError:
        messagebox.showerror("输入错误", "电机速度必须是整数。")
    except Exception as e:
        messagebox.showerror("发送错误", f"发送电机命令时出错: {e}")
        print(f"发送电机命令时出错: {e}")

# --- GUI 更新函数 ---

def update_mqtt_status(status):
    mqtt_status_var.set(f"MQTT 状态: {status}")

def update_motor_status_label(status):
    motor_status_var.set(f"电机状态: {status}")

def update_esp32_info_label(info):
    esp32_info_var.set(f"ESP32 信息: {info}")

def update_esp32_ip_label(ip):
    esp32_ip_var.set(f"ESP32 IP: {ip}")
    # 更新视频流按钮状态
    if ip != "未知":
        video_stream_button.config(state=tk.NORMAL)
    else:
        video_stream_button.config(state=tk.DISABLED)


def update_esp32_status_labels():
    fps_var.set(f"FPS: {last_esp32_status.get('fps', 0.0):.2f}")
    busy_status = "是" if last_esp32_status.get('busy', False) else "否"
    busy_var.set(f"系统繁忙: {busy_status}")

def update_bright_point_label():
    x = last_bright_point.get('x', -1)
    y = last_bright_point.get('y', -1)
    bright_point_var.set(f"最亮点: (X={x}, Y={y})")

# --- GUI 辅助函数 ---
def open_video_stream():
    if ESP32_IP != "未知":
        url = VIDEO_STREAM_URL_FORMAT.format(ESP32_IP)
        print(f"尝试打开视频流: {url}")
        try:
            webbrowser.open(url)
        except Exception as e:
            messagebox.showerror("打开失败", f"无法打开浏览器: {e}")
    else:
        messagebox.showwarning("无 IP 地址", "无法获取 ESP32 的 IP 地址。")

def stop_motors():
     motor_a_speed_scale.set(0)
     motor_b_speed_scale.set(0)
     send_motor_command()

# --- 创建 GUI ---
root = tk.Tk()
root.title("ESP32-CAM 智能船控制面板")
root.geometry("500x550") # 调整窗口大小

# --- 状态显示区 ---
status_frame = ttk.LabelFrame(root, text="状态信息", padding=(10, 5))
status_frame.pack(pady=10, padx=10, fill="x")

mqtt_status_var = tk.StringVar(value="MQTT 状态: 未连接")
mqtt_status_label = ttk.Label(status_frame, textvariable=mqtt_status_var)
mqtt_status_label.pack(anchor="w")

esp32_info_var = tk.StringVar(value="ESP32 信息: 离线")
esp32_info_label = ttk.Label(status_frame, textvariable=esp32_info_var)
esp32_info_label.pack(anchor="w")

esp32_ip_var = tk.StringVar(value="ESP32 IP: 未知")
esp32_ip_label = ttk.Label(status_frame, textvariable=esp32_ip_var)
esp32_ip_label.pack(anchor="w")

fps_var = tk.StringVar(value="FPS: 0.00")
fps_label = ttk.Label(status_frame, textvariable=fps_var)
fps_label.pack(anchor="w")

busy_var = tk.StringVar(value="系统繁忙: 否")
busy_label = ttk.Label(status_frame, textvariable=busy_var)
busy_label.pack(anchor="w")

bright_point_var = tk.StringVar(value="最亮点: (X=-1, Y=-1)")
bright_point_label = ttk.Label(status_frame, textvariable=bright_point_var)
bright_point_label.pack(anchor="w")

motor_status_var = tk.StringVar(value="电机状态: 未知")
motor_status_label = ttk.Label(status_frame, textvariable=motor_status_var)
motor_status_label.pack(anchor="w")


# --- 电机控制区 ---
motor_frame = ttk.LabelFrame(root, text="电机控制", padding=(10, 5))
motor_frame.pack(pady=10, padx=10, fill="x")

# 电机 A
motor_a_label = ttk.Label(motor_frame, text="电机 A 速度 (-100 to 100):")
motor_a_label.grid(row=0, column=0, padx=5, pady=5, sticky="w")
motor_a_speed_scale = ttk.Scale(motor_frame, from_=-100, to=100, orient=tk.HORIZONTAL, length=300)
motor_a_speed_scale.set(0)
motor_a_speed_scale.grid(row=0, column=1, padx=5, pady=5)
motor_a_speed_label = ttk.Label(motor_frame, text="0") # 显示当前值
motor_a_speed_label.grid(row=0, column=2, padx=5, pady=5)
motor_a_speed_scale.config(command=lambda val: motor_a_speed_label.config(text=f"{int(float(val))}"))


# 电机 B
motor_b_label = ttk.Label(motor_frame, text="电机 B 速度 (-100 to 100):")
motor_b_label.grid(row=1, column=0, padx=5, pady=5, sticky="w")
motor_b_speed_scale = ttk.Scale(motor_frame, from_=-100, to=100, orient=tk.HORIZONTAL, length=300)
motor_b_speed_scale.set(0)
motor_b_speed_scale.grid(row=1, column=1, padx=5, pady=5)
motor_b_speed_label = ttk.Label(motor_frame, text="0") # 显示当前值
motor_b_speed_label.grid(row=1, column=2, padx=5, pady=5)
motor_b_speed_scale.config(command=lambda val: motor_b_speed_label.config(text=f"{int(float(val))}"))


# 控制按钮
button_frame = ttk.Frame(motor_frame)
button_frame.grid(row=2, column=0, columnspan=3, pady=10)

send_button = ttk.Button(button_frame, text="发送电机命令", command=send_motor_command)
send_button.pack(side=tk.LEFT, padx=5)

stop_button = ttk.Button(button_frame, text="停止所有电机", command=stop_motors)
stop_button.pack(side=tk.LEFT, padx=5)

# --- 视频流 ---
video_frame = ttk.LabelFrame(root, text="视频流", padding=(10, 5))
video_frame.pack(pady=10, padx=10, fill="x")

video_stream_button = ttk.Button(video_frame, text="打开视频流", command=open_video_stream, state=tk.DISABLED)
video_stream_button.pack(pady=5)


# --- 主程序 ---
if __name__ == "__main__":
    connect_mqtt() # 启动时连接 MQTT
    root.mainloop()

    # 清理 MQTT 连接
    if mqtt_client and is_connected:
        mqtt_client.loop_stop()
        mqtt_client.disconnect()
        print("MQTT 已断开连接")
