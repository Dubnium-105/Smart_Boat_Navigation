import tkinter as tk
from tkinter import ttk, messagebox, Scale
import paho.mqtt.client as mqtt
import json
import time
from PIL import Image, ImageTk, ImageDraw
import io
import base64
import os
import sys
import traceback
import threading
import queue
import numpy as np
import cv2
import socket
from flask import Flask, render_template, Response, send_from_directory
from flask_socketio import SocketIO
import logging

# 禁用Flask的默认日志
log = logging.getLogger('werkzeug')
log.setLevel(logging.ERROR)

app = Flask(__name__)
socketio = SocketIO(app)

# 创建必要的目录
if not os.path.exists('templates'):
    os.makedirs('templates')
if not os.path.exists('static'):
    os.makedirs('static')

# 创建一个基本的favicon.ico
def create_favicon():
    img = Image.new('RGB', (16, 16), color='red')
    img_io = io.BytesIO()
    img.save(img_io, 'ICO')
    with open('static/favicon.ico', 'wb') as f:
        f.write(img_io.getvalue())

create_favicon()

# 创建HTML模板
html_template = """
<!DOCTYPE html>
<html>
<head>
    <title>摄像头画面</title>
    <link rel="shortcut icon" href="{{ url_for('static', filename='favicon.ico') }}">
    <style>
        body {
            margin: 0;
            padding: 20px;
            display: flex;
            flex-direction: column;
            align-items: center;
            background-color: #f0f0f0;
            font-family: Arial, sans-serif;
        }
        #videoFeed {
            max-width: 800px;
            width: 100%;
            border: 2px solid #333;
            border-radius: 8px;
            margin-bottom: 20px;
        }
        .info {
            background-color: white;
            padding: 10px 20px;
            border-radius: 5px;
            margin: 10px 0;
            box-shadow: 0 2px 5px rgba(0,0,0,0.1);
        }
    </style>
</head>
<body>
    <h1>实时摄像头画面</h1>
    <img id="videoFeed" src="/video_feed">
    <div class="info" id="brightnessInfo">最亮点: 等待数据...</div>
    <script src="https://cdnjs.cloudflare.com/ajax/libs/socket.io/4.0.1/socket.io.js"></script>
    <script>
        var socket = io();
        socket.on('brightness_update', function(data) {
            document.getElementById('brightnessInfo').innerHTML = 
                `最亮点: x=${data.x}, y=${data.y}, 亮度=${data.val}`;
        });
    </script>
</body>
</html>
"""

# 保存HTML模板
with open('templates/index.html', 'w', encoding='utf-8') as f:
    f.write(html_template)

# Flask路由
@app.route('/favicon.ico')
def favicon():
    return send_from_directory('static', 'favicon.ico')

class MQTTControlApp:
    def __init__(self):
        self.window = tk.Tk()
        self.window.title("电机控制器")
        self.window.geometry("800x600")
        
        # TCP连接配置
        self.tcp_client = None
        self.tcp_connected = False
        self.tcp_host = "192.168.4.1"  # ESP32的IP地址
        self.tcp_port = 5000
        
        # 图像接收相关变量
        self.image_queue = queue.Queue(maxsize=5)
        self.image_processing_thread = None
        self.streaming_enabled = True
        self.last_image_time = 0
        
        # 添加调试输出区域
        self.create_debug_area()
        
        # MQTT配置
        self.debug_print("初始化MQTT客户端...")
        self.client = mqtt.Client()
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        self.client.on_disconnect = self.on_disconnect
        
        # 图像接收相关变量
        self.image_chunks = {}
        self.total_chunks = 0
        self.image_info = {}
        self.image_processing_thread = None
        
        # 最亮点位置（从C++端接收）
        self.brightest_x = 160
        self.brightest_y = 120
        self.brightest_val = 0
        
        # 自动控制相关变量
        self.auto_control = False
        self.last_control_time = 0
        
        # 电机状态
        self.motor_a_speed = 0
        self.motor_b_speed = 0
        
        # 亮度过滤阈值
        self.brightness_threshold = 100
        
        # 添加Web服务器相关变量
        self.latest_frame = None
        self.frame_lock = threading.Lock()
        
        self.create_widgets()
        
        # 启动TCP连接
        self.connect_tcp()
        
        # 启动图像处理线程
        self.start_image_processing_thread()
        
        # 连接MQTT服务器
        self.connect_mqtt()
        
        # 启动Web服务器
        self.start_web_server()
        
    def create_debug_area(self):
        # 创建调试输出区域
        debug_frame = ttk.LabelFrame(self.window, text="调试输出")
        debug_frame.pack(side=tk.BOTTOM, fill=tk.X, padx=10, pady=5)
        
        self.debug_text = tk.Text(debug_frame, height=5, width=80)
        self.debug_text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        
        scrollbar = ttk.Scrollbar(debug_frame, command=self.debug_text.yview)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        self.debug_text.config(yscrollcommand=scrollbar.set)
        
    def debug_print(self, message):
        """向调试区域添加消息并打印到控制台"""
        timestamp = time.strftime("%H:%M:%S", time.localtime())
        formatted_msg = f"[{timestamp}] {message}\n"
        print(formatted_msg, end="")
        
        # 如果UI已经初始化，则更新UI
        if hasattr(self, 'debug_text'):
            self.debug_text.insert(tk.END, formatted_msg)
            self.debug_text.see(tk.END)  # 自动滚动到最新消息
            self.window.update_idletasks()  # 强制更新UI
        
    def connect_mqtt(self):
        """连接到MQTT服务器"""
        try:
            self.debug_print(f"连接到MQTT服务器: emqx.link2you.top:1883")
            self.client.connect("emqx.link2you.top", 1883, 60)
            self.client.loop_start()
            self.debug_print("MQTT客户端已启动")
        except Exception as e:
            self.debug_print(f"MQTT连接错误: {str(e)}")
            messagebox.showerror("连接错误", f"无法连接到MQTT服务器: {str(e)}")
        
    def connect_tcp(self):
        """连接到TCP服务器"""
        try:
            self.tcp_client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.tcp_client.connect((self.tcp_host, self.tcp_port))
            self.tcp_client.settimeout(1.0)  # 设置超时
            self.tcp_connected = True
            self.debug_print("TCP连接成功")
            
            # 启动TCP接收线程
            threading.Thread(target=self.tcp_receive_loop, daemon=True).start()
            
        except Exception as e:
            self.debug_print(f"TCP连接失败: {str(e)}")
            self.tcp_connected = False
            
    def tcp_receive_loop(self):
        """TCP数据接收循环"""
        while True:
            try:
                if not self.tcp_connected:
                    time.sleep(1)
                    continue
                    
                # 读取帧头
                header = self.tcp_client.recv(12)
                if len(header) < 12:
                    continue
                    
                # 检查帧起始标记
                if header[0] != 0xFF or header[1] != 0xAA:
                    continue
                    
                # 解析图像信息
                width = int.from_bytes(header[2:4], byteorder='little')
                height = int.from_bytes(header[4:6], byteorder='little')
                data_len = int.from_bytes(header[6:10], byteorder='little')
                img_format = header[10]
                
                # 接收图像数据
                image_data = bytearray()
                remaining = data_len
                
                while remaining > 0:
                    chunk = self.tcp_client.recv(min(4096, remaining))
                    if not chunk:
                        break
                    image_data.extend(chunk)
                    remaining -= len(chunk)
                
                # 检查是否接收完整
                if len(image_data) == data_len:
                    # 读取帧结束标记
                    footer = self.tcp_client.recv(2)
                    if footer[0] == 0xFF and footer[1] == 0x55:
                        # 将图像数据放入队列
                        image_info = {
                            'width': width,
                            'height': height,
                            'format': img_format
                        }
                        if not self.image_queue.full():
                            # 直接传递字节数据，不需要解码
                            self.image_queue.put((bytes(image_data), image_info))
                
            except socket.timeout:
                continue
            except Exception as e:
                self.debug_print(f"TCP接收错误: {str(e)}")
                self.tcp_connected = False
                time.sleep(1)
                self.connect_tcp()
                
    def create_widgets(self):
        main_frame = ttk.Frame(self.window, padding=10)
        main_frame.pack(fill=tk.BOTH, expand=True)
        
        # 添加图像显示区域
        self.image_frame = ttk.LabelFrame(main_frame, text="摄像头画面")
        self.image_frame.pack(pady=5, fill=tk.BOTH, expand=True)
        
        self.image_label = ttk.Label(self.image_frame, background="black")
        self.image_label.pack(pady=5, padx=5, fill=tk.BOTH, expand=True)
        
        # 图像状态显示
        self.image_status = ttk.Label(self.image_frame, text="等待图像...")
        self.image_status.pack(pady=2)
        
        # 添加控制按钮框架
        control_buttons_frame = ttk.Frame(self.image_frame)
        control_buttons_frame.pack(pady=5)
        
        # 添加手动刷新按钮
        refresh_button = ttk.Button(control_buttons_frame, text="刷新图像", command=self.request_image)
        refresh_button.pack(side=tk.LEFT, padx=5)
        
        # 添加流控制按钮
        self.stream_var = tk.BooleanVar(value=True)
        self.stream_button = ttk.Checkbutton(
            control_buttons_frame, 
            text="启用视频流", 
            variable=self.stream_var,
            command=self.toggle_stream
        )
        self.stream_button.pack(side=tk.LEFT, padx=5)
        
        # 电机控制面板
        control_frame = ttk.LabelFrame(main_frame, text="电机控制")
        control_frame.pack(pady=5, fill=tk.X)
        
        self.slider_L = ttk.Scale(control_frame, from_=-100, to=100, command=self.on_slider)
        self.slider_R = ttk.Scale(control_frame, from_=-100, to=100, command=self.on_slider)
        self.slider_L.grid(row=0, column=0, padx=5, sticky="ew")
        self.slider_R.grid(row=0, column=1, padx=5, sticky="ew")
        
        # 左右电机标签
        ttk.Label(control_frame, text="左电机").grid(row=1, column=0)
        ttk.Label(control_frame, text="右电机").grid(row=1, column=1)
        
        # 电机状态显示
        self.motor_a_label = ttk.Label(control_frame, text="左电机速度: 0")
        self.motor_a_label.grid(row=2, column=0, pady=5)
        self.motor_b_label = ttk.Label(control_frame, text="右电机速度: 0")
        self.motor_b_label.grid(row=2, column=1, pady=5)
        
        # 设置列权重，使滑块能够扩展
        control_frame.columnconfigure(0, weight=1)
        control_frame.columnconfigure(1, weight=1)
        
        # 亮度阈值控制
        threshold_frame = ttk.LabelFrame(main_frame, text="亮度阈值设置")
        threshold_frame.pack(pady=5, fill=tk.X)
        
        ttk.Label(threshold_frame, text="最小亮度阈值:").pack(side=tk.LEFT, padx=5)
        self.threshold_slider = Scale(threshold_frame, from_=0, to=255, orient=tk.HORIZONTAL, 
                                     command=self.on_threshold_change)
        self.threshold_slider.set(self.brightness_threshold)
        self.threshold_slider.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)
        self.threshold_value_label = ttk.Label(threshold_frame, text=f"当前值: {self.brightness_threshold}")
        self.threshold_value_label.pack(side=tk.LEFT, padx=5)
        
        # 自动控制开关
        self.auto_var = tk.BooleanVar()
        self.auto_check = ttk.Checkbutton(main_frame, text="自动追踪光源",
                                        variable=self.auto_var,
                                        command=self.toggle_auto_control)
        self.auto_check.pack(pady=5)
        
        # 状态显示
        self.status_label = ttk.Label(main_frame, text="状态: 未连接")
        self.status_label.pack(pady=5)
        
        # 帧率显示
        self.fps_label = ttk.Label(main_frame, text="帧率: 0 FPS")
        self.fps_label.pack(pady=2)
        
        # 最亮点位置显示
        self.brightest_label = ttk.Label(main_frame, text="最亮点: x=160, y=120")
        self.brightest_label.pack(pady=2)
        
    def toggle_stream(self):
        """切换视频流状态"""
        self.streaming_enabled = self.stream_var.get()
        self.debug_print(f"视频流: {'开启' if self.streaming_enabled else '关闭'}")
        self.client.publish("/camera/stream", "1" if self.streaming_enabled else "0")
        
    def start_image_processing_thread(self):
        """启动图像处理线程"""
        self.image_processing_thread = threading.Thread(target=self.process_image_queue, daemon=True)
        self.image_processing_thread.start()
        
    def find_brightest_point(self, image):
        """寻找图像中最亮的点"""
        try:
            # 转换为OpenCV格式
            cv_image = np.array(image)
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)
            
            # 转换为灰度图
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            
            # 应用亮度阈值过滤
            _, thresholded = cv2.threshold(gray, self.brightness_threshold, 255, cv2.THRESH_TOZERO)
            
            # 如果所有像素都被过滤掉，返回图像中心
            if cv2.countNonZero(thresholded) == 0:
                return image.width // 2, image.height // 2, 0
            
            # 寻找最亮点
            (min_val, max_val, min_loc, max_loc) = cv2.minMaxLoc(thresholded)
            
            # 返回最亮点坐标和亮度值
            return max_loc[0], max_loc[1], max_val
            
        except Exception as e:
            self.debug_print(f"寻找最亮点错误: {str(e)}")
            return image.width // 2, image.height // 2, 0
            
    def find_whitest_point(self, image):
        """寻找图像中最接近白色的点"""
        try:
            # 转换为OpenCV格式
            cv_image = np.array(image)
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)
            
            # 转换为HSV格式
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            
            # 定义白色范围
            lower_white = np.array([0, 0, 200])
            upper_white = np.array([180, 30, 255])
            
            # 创建白色掩码
            mask = cv2.inRange(hsv, lower_white, upper_white)
            
            # 应用亮度阈值过滤
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            _, brightness_mask = cv2.threshold(gray, self.brightness_threshold, 255, cv2.THRESH_BINARY)
            
            # 结合白色掩码和亮度掩码
            combined_mask = cv2.bitwise_and(mask, brightness_mask)
            
            # 如果所有像素都被过滤掉，返回图像中心
            if cv2.countNonZero(combined_mask) == 0:
                return image.width // 2, image.height // 2, 0
            
            # 寻找白色区域的轮廓
            contours, _ = cv2.findContours(combined_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            if contours:
                # 找到最大的白色区域
                max_contour = max(contours, key=cv2.contourArea)
                M = cv2.moments(max_contour)
                
                if M["m00"] > 0:
                    # 计算质心
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    return cx, cy, cv2.contourArea(max_contour)
            
            # 如果没有找到白色区域，使用最亮点
            return self.find_brightest_point(image)
        except Exception as e:
            self.debug_print(f"寻找白色点错误: {str(e)}")
            return image.width // 2, image.height // 2, 0
        
    def mark_brightest_point(self, image, x, y, brightness):
        """在图像上标记最亮点"""
        try:
            # 转换为OpenCV格式
            cv_image = np.array(image)
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)
            
            # 标记参数
            marker_size = 15
            marker_thickness = 3
            color = (0, 0, 255)  # 红色
            
            # 画圆
            cv2.circle(cv_image, (x, y), marker_size, color, marker_thickness)
            
            # 画十字线
            cv2.line(cv_image, (x - marker_size, y), (x + marker_size, y), color, marker_thickness)
            cv2.line(cv_image, (x, y - marker_size), (x, y + marker_size), color, marker_thickness)
            
            # 添加亮度值文本
            cv2.putText(cv_image, f"亮度: {brightness}", (x + 20, y - 20),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
            
            # 转换回PIL格式
            marked_image = Image.fromarray(cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB))
            return marked_image
            
        except Exception as e:
            self.debug_print(f"标记最亮点错误: {str(e)}")
            return image
        
    def process_image_queue(self):
        """处理图像队列"""
        last_fps_update = time.time()
        frame_count = 0
        
        while True:
            try:
                # 从队列获取图像数据
                image_data, image_info = self.image_queue.get(timeout=1.0)
                
                try:
                    # 确保image_data是字节类型
                    if isinstance(image_data, str):
                        try:
                            # 如果是Base64编码的字符串，先解码
                            image_data = base64.b64decode(image_data)
                        except Exception as e:
                            # 如果不是Base64，尝试直接编码为字节
                            image_data = image_data.encode('latin1')
                    elif not isinstance(image_data, (bytes, bytearray)):
                        image_data = bytes(image_data)
                    
                    # 将图像数据转换为PIL图像
                    image = Image.open(io.BytesIO(image_data))
                    
                    # 调整图像大小以适应显示
                    display_width = 640
                    display_height = 480
                    image = image.resize((display_width, display_height), Image.LANCZOS)
                    
                    # 寻找最亮点
                    x, y, val = self.find_brightest_point(image)
                    self.brightest_x = x
                    self.brightest_y = y
                    self.brightest_val = val
                    
                    # 标记最亮点
                    marked_image = self.mark_brightest_point(image, x, y, val)
                    
                    # 转换为Tkinter可显示的格式
                    photo = ImageTk.PhotoImage(marked_image)
                    
                    # 更新显示
                    self.window.after(0, self.update_image_display, photo, image_info, x, y, val)
                    
                    # 如果启用了自动控制，控制电机
                    if self.auto_control:
                        self.window.after(0, self.auto_track_light)
                    
                    # 更新帧率
                    frame_count += 1
                    current_time = time.time()
                    if current_time - last_fps_update >= 1.0:
                        fps = frame_count / (current_time - last_fps_update)
                        self.window.after(0, self.update_fps_display, fps)
                        frame_count = 0
                        last_fps_update = current_time
                        
                except Exception as e:
                    self.debug_print(f"图像处理错误: {str(e)}")
                    self.debug_print(traceback.format_exc())
                    self.debug_print(f"图像数据类型: {type(image_data)}")
                
                self.image_queue.task_done()
                
            except queue.Empty:
                # 队列为空，继续等待
                pass
                
            if self.streaming_enabled and time.time() - self.last_image_time > 0.2:
                # 每200ms请求一次新图像
                self.request_image()
        
    def update_image_display(self, photo, image_info, x, y, val):
        """更新图像显示（在主线程中调用）"""
        self.image_label.configure(image=photo)
        self.image_label.image = photo  # 保持引用以防止垃圾回收
        
        # 更新图像状态
        width = image_info.get('width', 0)
        height = image_info.get('height', 0)
        self.image_status.config(text=f"图像已更新: {width}x{height}")
        
        # 更新最亮点位置显示
        self.brightest_label.config(text=f"最亮点: x={x}, y={y}, 值={val}")
        
        # 更新最后图像时间
        self.last_image_time = time.time()
        
        # 更新Web显示用的帧
        with self.frame_lock:
            # 将PIL图像转换为字节流
            img_byte_arr = io.BytesIO()
            photo._PhotoImage__photo.write(img_byte_arr, format='PNG')
            self.latest_frame = img_byte_arr.getvalue()
            
            # 发送最亮点信息到Web客户端
            socketio.emit('brightness_update', {
                'x': x,
                'y': y,
                'val': val
            })
        
    def update_fps_display(self, fps):
        """更新帧率显示（在主线程中调用）"""
        self.fps_label.config(text=f"帧率: {fps:.1f} FPS")
        
    def request_image(self):
        """请求图像更新"""
        if self.tcp_connected:
            try:
                self.tcp_client.send(b"IMAGE\n")
                self.debug_print("已发送图像请求")
            except Exception as e:
                self.debug_print(f"发送图像请求失败: {str(e)}")
                self.tcp_connected = False
                
    def on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.debug_print("已成功连接到MQTT服务器")
            self.status_label.config(text="状态: 已连接")
            
            # 订阅主题
            topics = [
                ("/motor/status", 0),
                ("/sensor/data", 0),
                ("/camera/image/#", 0),  # 使用通配符订阅所有图像块
                ("/camera/info", 0),
                ("/ESP32_info", 0)
            ]
            
            for topic, qos in topics:
                self.client.subscribe(topic, qos)
                self.debug_print(f"已订阅主题: {topic}")
                
            # 发送一条消息通知设备UI已连接
            self.client.publish("/UI_info", "UI已连接")
            
            # 请求初始图像
            self.request_image()
        else:
            self.debug_print(f"连接失败，返回码: {rc}")
            self.status_label.config(text=f"状态: 连接失败 (代码: {rc})")
    
    def on_disconnect(self, client, userdata, rc):
        self.debug_print(f"与MQTT服务器断开连接，代码: {rc}")
        self.status_label.config(text="状态: 已断开连接")
        
        # 如果是意外断开，尝试重新连接
        if rc != 0:
            self.window.after(5000, self.connect_mqtt)
        
    def on_message(self, client, userdata, msg):
        try:
            if msg.topic == "/ESP32_info":
                info = msg.payload.decode()
                self.debug_print(f"设备信息: {info}")
                
            elif msg.topic == "/motor/status":
                status = msg.payload.decode()
                self.debug_print(f"电机状态: {status}")
                self.status_label.config(text=f"状态: {status}")
            
            elif msg.topic == "/sensor/data":
                try:
                    data = json.loads(msg.payload.decode())
                    # 更新最亮点信息（如果有）
                    if 'brightest_x' in data and 'brightest_y' in data:
                        self.brightest_x = data['brightest_x']
                        self.brightest_y = data['brightest_y']
                        self.brightest_val = data.get('brightest_val', 0)
                except Exception as e:
                    self.debug_print(f"处理传感器数据错误: {str(e)}")
            
            elif msg.topic == "/camera/info":
                try:
                    self.image_info = json.loads(msg.payload.decode())
                    # self.debug_print(f"收到图像信息: {self.image_info}")
                except Exception as e:
                    self.debug_print(f"处理图像信息错误: {str(e)}")
            
            elif msg.topic.startswith("/camera/image/"):
                if not self.streaming_enabled:
                    return  # 如果流已禁用，忽略图像数据
                    
                # 解析主题格式: /camera/image/[chunk_index]/[total_chunks]
                parts = msg.topic.split("/")
                if len(parts) == 5:
                    try:
                        chunk_index = int(parts[3])
                        total_chunks = int(parts[4])
                        
                        # 存储块数据
                        if total_chunks > 0:
                            self.total_chunks = total_chunks
                            self.image_chunks[chunk_index] = msg.payload.decode()
                            
                            # 检查是否收到了所有块
                            if len(self.image_chunks) == total_chunks:
                                # 按顺序合并所有块
                                base64_data = ""
                                for i in range(total_chunks):
                                    if i in self.image_chunks:
                                        base64_data += self.image_chunks[i]
                                
                                # 清空块缓冲区
                                self.image_chunks = {}
                                
                                # 如果图像队列未满，则添加到队列
                                if not self.image_queue.full():
                                    self.image_queue.put((base64_data, self.image_info))
                    except Exception as e:
                        self.debug_print(f"处理图像块错误: {str(e)}")
                
        except Exception as e:
            self.debug_print(f"处理消息错误: {str(e)}")
            self.debug_print(traceback.format_exc())
                
    def on_slider(self, value):
        if not self.auto_control:  # 只在手动模式下响应滑块
            self.motor_a_speed = int(self.slider_L.get())
            self.motor_b_speed = int(self.slider_R.get())
            
            cmd = {
                "speedA": self.motor_a_speed,
                "speedB": self.motor_b_speed
            }
            self.client.publish("/motor", json.dumps(cmd))
            
            # 更新电机状态显示
            self.motor_a_label.config(text=f"左电机速度: {self.motor_a_speed}")
            self.motor_b_label.config(text=f"右电机速度: {self.motor_b_speed}")
            
    def on_threshold_change(self, value):
        """处理亮度阈值变化"""
        self.brightness_threshold = int(value)
        self.threshold_value_label.config(text=f"当前值: {self.brightness_threshold}")
        
    def toggle_auto_control(self):
        self.auto_control = self.auto_var.get()
        self.debug_print(f"自动控制: {'开启' if self.auto_control else '关闭'}")
        
        if not self.auto_control:
            # 停止电机
            self.motor_a_speed = 0
            self.motor_b_speed = 0
            cmd = {"speedA": 0, "speedB": 0}
            self.client.publish("/motor", json.dumps(cmd))
            self.debug_print("停止电机")
            
            # 更新电机状态显示
            self.motor_a_label.config(text=f"左电机速度: {self.motor_a_speed}")
            self.motor_b_label.config(text=f"右电机速度: {self.motor_b_speed}")
            
    def auto_track_light(self):
        current_time = time.time()
        if current_time - self.last_control_time < 0.1:  # 限制控制频率
            return
            
        # 计算偏差
        error = self.brightest_x - 320  # 相对于中心点的偏差
        
        # 基础速度
        base_speed = 30
        
        # 根据偏差计算差速
        if abs(error) < 20:  # 死区
            left_speed = right_speed = 0
        else:
            # 比例控制
            k = 0.5  # 控制增益
            speed_diff = error * k
            
            left_speed = base_speed - speed_diff
            right_speed = base_speed + speed_diff
            
            # 限制速度范围
            left_speed = max(-100, min(100, left_speed))
            right_speed = max(-100, min(100, right_speed))
        
        # 更新电机状态
        self.motor_a_speed = int(left_speed)
        self.motor_b_speed = int(right_speed)
        
        # 发送控制命令
        cmd = {
            "speedA": self.motor_a_speed,
            "speedB": self.motor_b_speed
        }
        self.client.publish("/motor", json.dumps(cmd))
        
        # 更新电机状态显示
        self.motor_a_label.config(text=f"左电机速度: {self.motor_a_speed}")
        self.motor_b_label.config(text=f"右电机速度: {self.motor_b_speed}")
        
        self.last_control_time = current_time
        
    def check_image_timeout(self):
        """检查图像是否超时，如果超时则请求新图像"""
        current_time = time.time()
        if self.streaming_enabled and current_time - self.last_image_time > 5:  # 5秒没有收到新图像
            self.debug_print("图像接收超时，请求新图像")
            self.request_image()
        
        # 每2秒检查一次
        self.window.after(2000, self.check_image_timeout)
        
    def start_web_server(self):
        """启动Web服务器"""
        def run_server():
            socketio.run(app, host='0.0.0.0', port=8080)
        
        threading.Thread(target=run_server, daemon=True).start()
        self.debug_print("Web服务器已启动，访问 http://localhost:8080 查看画面")

    def run(self):
        """运行应用程序"""
        # 启动图像超时检查
        self.last_image_time = time.time()
        self.window.after(2000, self.check_image_timeout)
        
        # 启动主循环
        self.window.mainloop()
        
        # 清理资源
        self.client.loop_stop()
        self.client.disconnect()

# Flask路由
@app.route('/')
def index():
    return render_template('index.html')

def get_frame():
    """生成器函数，用于流式传输图像"""
    app_instance = None
    while app_instance is None:
        # 等待应用程序实例创建
        for obj in gc.get_objects():
            if isinstance(obj, MQTTControlApp):
                app_instance = obj
                break
        time.sleep(0.1)
    
    while True:
        with app_instance.frame_lock:
            if app_instance.latest_frame is not None:
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + app_instance.latest_frame + b'\r\n')
        time.sleep(0.1)

@app.route('/video_feed')
def video_feed():
    return Response(get_frame(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == "__main__":
    app = MQTTControlApp()
    try:
        app.run()
    except KeyboardInterrupt:
        print("\n程序被用户中断")
    except Exception as e:
        print(f"程序异常退出: {str(e)}")
        traceback.print_exc()
    finally:
        # 确保资源被正确清理
        if hasattr(app, 'client'):
            app.client.loop_stop()
            app.client.disconnect()