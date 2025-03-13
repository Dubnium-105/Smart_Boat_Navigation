import tkinter as tk
from tkinter import ttk, messagebox
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

class MQTTControlApp:
    def __init__(self):
        self.window = tk.Tk()
        self.window.title("MQTT电机控制器")
        self.window.geometry("800x600")  # 设置窗口大小
        
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
        self.last_image_time = 0
        self.image_queue = queue.Queue(maxsize=5)  # 图像队列，限制大小为5
        self.image_processing_thread = None
        self.streaming_enabled = True
        
        # 最亮点位置
        self.brightest_x = 160
        self.brightest_y = 120
        
        # 自动控制相关变量
        self.auto_control = False
        self.last_control_time = 0
        
        self.create_widgets()
        
        # 启动图像处理线程
        self.start_image_processing_thread()
        
        # 连接MQTT服务器
        self.connect_mqtt()
        
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
        
        # 设置列权重，使滑块能够扩展
        control_frame.columnconfigure(0, weight=1)
        control_frame.columnconfigure(1, weight=1)
        
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
            
            # 寻找最亮点
            (min_val, max_val, min_loc, max_loc) = cv2.minMaxLoc(gray)
            
            # 返回最亮点坐标
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
            
            # 寻找白色区域的轮廓
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
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
        
    def process_image_queue(self):
        """处理图像队列的线程函数"""
        last_fps_update = time.time()
        frame_count = 0
        
        while True:
            try:
                # 从队列获取图像数据
                base64_data, image_info = self.image_queue.get(timeout=1.0)
                
                # 处理图像
                try:
                    # 解码Base64数据
                    image_data = base64.b64decode(base64_data)
                    
                    # 将图像数据转换为PIL图像
                    image = Image.open(io.BytesIO(image_data))
                    
                    # 调整图像大小以适应显示
                    display_width = 640
                    display_height = 480
                    image = image.resize((display_width, display_height), Image.LANCZOS)
                    
                    # 寻找最亮点/最白点
                    x, y, val = self.find_whitest_point(image)
                    self.brightest_x = x
                    self.brightest_y = y
                    
                    # 在图像上标记最亮点
                    draw = ImageDraw.Draw(image)
                    r = 10  # 标记半径
                    draw.ellipse((x-r, y-r, x+r, y+r), outline='red', width=2)
                    
                    # 转换为Tkinter可显示的格式
                    photo = ImageTk.PhotoImage(image)
                    
                    # 更新显示（使用主线程）
                    self.window.after(0, self.update_image_display, photo, image_info, x, y, val)
                    
                    # 如果启用了自动控制，控制电机
                    if self.auto_control:
                        self.window.after(0, self.auto_track_light)
                    
                    # 更新帧率计数
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
                
                # 标记任务完成
                self.image_queue.task_done()
                
            except queue.Empty:
                # 队列为空，继续等待
                pass
                
    def update_image_display(self, photo, image_info, x, y, val):
        """更新图像显示（在主线程中调用）"""
        self.image_label.configure(image=photo)
        self.image_label.image = photo  # 保持引用以防止垃圾回收
        
        # 更新图像状态
        width = image_info.get('width', 0)
        height = image_info.get('height', 0)
        self.image_status.config(text=f"图像已更新: {width}x{height}")
        
        # 更新最亮点位置显示
        self.brightest_label.config(text=f"最亮点: x={x}, y={y}, 值={val:.1f}")
        
        # 更新最后图像时间
        self.last_image_time = time.time()
        
    def update_fps_display(self, fps):
        """更新帧率显示（在主线程中调用）"""
        self.fps_label.config(text=f"帧率: {fps:.1f} FPS")
        
    def request_image(self):
        """手动请求图像刷新"""
        self.debug_print("手动请求图像刷新")
        self.client.publish("/camera/request", "refresh")
        self.image_status.config(text="请求图像中...")
        
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
                    # 只在调试区域显示传感器数据，避免刷屏
                    # self.debug_print(f"传感器数据: {data}")
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
            cmd = {
                "speedA": int(self.slider_L.get()),
                "speedB": int(self.slider_R.get())
            }
            self.client.publish("/motor", json.dumps(cmd))
            # self.debug_print(f"发送电机控制: A={cmd['speedA']}, B={cmd['speedB']}")
            
    def toggle_auto_control(self):
        self.auto_control = self.auto_var.get()
        self.debug_print(f"自动控制: {'开启' if self.auto_control else '关闭'}")
        
        if not self.auto_control:
            # 停止电机
            cmd = {"speedA": 0, "speedB": 0}
            self.client.publish("/motor", json.dumps(cmd))
            self.debug_print("停止电机")
            
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
        
        # 发送控制命令
        cmd = {
            "speedA": int(left_speed),
            "speedB": int(right_speed)
        }
        self.client.publish("/motor", json.dumps(cmd))
        
        self.last_control_time = current_time
        
    def check_image_timeout(self):
        """检查图像是否超时，如果超时则请求新图像"""
        current_time = time.time()
        if self.streaming_enabled and current_time - self.last_image_time > 5:  # 5秒没有收到新图像
            self.debug_print("图像接收超时，请求新图像")
            self.request_image()
        
        # 每2秒检查一次
        self.window.after(2000, self.check_image_timeout)
        
    def run(self):
        # 启动图像超时检查
        self.last_image_time = time.time()
        self.window.after(2000, self.check_image_timeout)
        
        # 启动主循环
        self.window.mainloop()
        
        # 清理资源
        self.client.loop_stop()
        self.client.disconnect()

if __name__ == "__main__":
    app = MQTTControlApp()
    app.run()