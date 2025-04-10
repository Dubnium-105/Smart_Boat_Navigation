import tkinter as tk
from tkinter import ttk, messagebox, Canvas
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
import serial
import serial.tools.list_ports
import subprocess

class ControlApp:
    def __init__(self):
        self.window = tk.Tk()
        self.window.title("设备控制器")
        self.window.geometry("800x600")  # 设置窗口大小
        
        # 添加调试输出区域
        self.create_debug_area()
        
        # 串口相关变量
        self.serial_port = None
        self.serial_connected = False
        self.available_ports = []
        self.baud_rate = 115200
        
        # TCP相关变量
        self.tcp_image_socket = None
        self.tcp_control_socket = None
        self.tcp_connected = False
        self.tcp_image_thread = None
        self.tcp_host = None
        self.tcp_control_port = 5000
        self.tcp_image_port = 5001
        
        # 图像接收相关变量
        self.image_queue = queue.Queue(maxsize=5)  # 图像队列，限制大小为5
        self.image_processing_thread = None
        
        # 初始化消息队列
        self.message_queue = queue.Queue()
        
        self.create_widgets()
        
        # 启动图像处理线程
        self.start_image_processing_thread()
        
        # 启动UI更新定时器
        self.window.after(100, self._update_debug_text)
        
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
        
        # 将消息添加到队列中，以便在主线程中更新UI
        if hasattr(self, 'message_queue'):
            self.message_queue.put(("debug_text", formatted_msg))
        
    def _update_debug_text(self):
        """从队列中处理UI更新消息（在主线程中定期调用）"""
        try:
            while not self.message_queue.empty():
                msg_type, msg = self.message_queue.get_nowait()
                
                if msg_type == "debug_text":
                    if hasattr(self, 'debug_text'):
                        self.debug_text.insert(tk.END, msg)
                        self.debug_text.see(tk.END)  # 自动滚动到最新消息
        except Exception as e:
            print(f"UI更新错误: {str(e)}")
        
        # 每100ms检查一次队列
        self.window.after(100, self._update_debug_text)
        
    def create_widgets(self):
        main_frame = ttk.Frame(self.window, padding=10)
        main_frame.pack(fill=tk.BOTH, expand=True)
        
        # 添加图像显示区域
        self.image_frame = ttk.LabelFrame(main_frame, text="摄像头画面")
        self.image_frame.pack(pady=5, fill=tk.BOTH, expand=True)
        
        # 图像显示标签
        self.image_label = ttk.Label(self.image_frame, background="black")
        self.image_label.pack(pady=5, padx=5, fill=tk.BOTH, expand=True)
        
        # 图像状态显示
        self.image_status = ttk.Label(self.image_frame, text="等待图像...")
        self.image_status.pack(pady=2)
        
        # 添加控制按钮框架
        control_buttons_frame = ttk.Frame(self.image_frame)
        control_buttons_frame.pack(pady=5)
        
        # 添加TCP连接框架
        tcp_frame = ttk.LabelFrame(main_frame, text="TCP连接")
        tcp_frame.pack(pady=5, fill=tk.X)
        
        # 搜索设备按钮
        search_button = ttk.Button(tcp_frame, text="搜索设备", command=self.search_devices)
        search_button.pack(side=tk.LEFT, padx=5)
        
        # 设备选择下拉框
        self.device_var = tk.StringVar()
        self.device_combo = ttk.Combobox(tcp_frame, textvariable=self.device_var, width=30)
        self.device_combo.pack(side=tk.LEFT, padx=5)
        
        # 手动输入IP地址
        self.manual_ip_var = tk.StringVar()
        self.manual_ip_entry = ttk.Entry(tcp_frame, textvariable=self.manual_ip_var, width=20)
        self.manual_ip_entry.pack(side=tk.LEFT, padx=5)
        self.manual_ip_entry.insert(0, "手动输入IP地址")
        
        # 连接按钮
        self.connect_btn = ttk.Button(tcp_frame, text="连接", command=self.connect_tcp)
        self.connect_btn.pack(side=tk.LEFT, padx=5)
        
    def search_devices(self):
        """搜索局域网内以“esp32s3-”开头的设备"""
        self.debug_print("正在搜索设备...")
        try:
            result = subprocess.run(["arp", "-a"], capture_output=True, text=True)
            devices = []
            for line in result.stdout.splitlines():
                if "esp32s3-" in line:
                    parts = line.split()
                    if len(parts) > 1:  # 确保有IP地址
                        devices.append(parts[1].strip("()"))  # 提取IP地址
            self.device_combo['values'] = devices
            if devices:
                self.device_combo.set(devices[0])
                self.debug_print(f"找到设备: {', '.join(devices)}")
            else:
                self.debug_print("未找到设备")
        except Exception as e:
            self.debug_print(f"搜索设备失败: {str(e)}")
        
    def connect_tcp(self):
        """建立TCP连接"""
        try:
            selected_device = self.device_var.get()
            manual_ip = self.manual_ip_var.get()
            
            if manual_ip and manual_ip != "手动输入IP地址":
                self.tcp_host = manual_ip
                self.debug_print(f"尝试连接到手动输入的IP地址: {manual_ip}")
            elif selected_device:
                self.tcp_host = selected_device
                self.debug_print(f"尝试连接到自动发现的设备: {selected_device}")
            else:
                self.debug_print("请选择一个设备或手动输入IP地址")
                return
            
            # 连接控制socket
            self.tcp_control_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.tcp_control_socket.connect((self.tcp_host, self.tcp_control_port))
            
            # 连接图像socket
            self.tcp_image_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.tcp_image_socket.connect((self.tcp_host, self.tcp_image_port))
            
            self.tcp_connected = True
            self.debug_print(f"成功连接到设备: {self.tcp_host}")
            
            # 启动TCP图像接收线程
            self.tcp_image_thread = threading.Thread(target=self.receive_tcp_image, daemon=True)
            self.tcp_image_thread.start()
        except Exception as e:
            self.debug_print(f"TCP连接失败: {str(e)}")
            self.tcp_connected = False
            
    def receive_tcp_image(self):
        """接收TCP图像数据的线程函数"""
        while self.tcp_connected:
            try:
                # 读取图像大小
                size_data = self.tcp_image_socket.recv(4)
                if not size_data:
                    break
                    
                size = int.from_bytes(size_data, byteorder='little')
                
                # 读取图像数据
                image_data = b''
                while len(image_data) < size:
                    chunk = self.tcp_image_socket.recv(min(size - len(image_data), 8192))
                    if not chunk:
                        break
                    image_data += chunk
                    
                if len(image_data) == size:
                    # 处理图像数据
                    image = Image.open(io.BytesIO(image_data))
                    photo = ImageTk.PhotoImage(image)
                    self.window.after(0, self.update_image_display, photo)
                    
            except Exception as e:
                self.debug_print(f"TCP图像接收错误: {str(e)}")
                break
        self.tcp_connected = False
        self.debug_print("TCP图像接收线程结束")
        
    def update_image_display(self, photo):
        """更新图像显示（在主线程中调用）"""
        self.image_label.configure(image=photo)
        self.image_label.image = photo  # 保持引用以防止垃圾回收
        self.image_status.config(text="图像已更新")
        
    def start_image_processing_thread(self):
        """启动图像处理线程"""
        self.image_processing_thread = threading.Thread(target=self.process_image_queue, daemon=True)
        self.image_processing_thread.start()
        
    def process_image_queue(self):
        """处理图像队列的线程函数"""
        while True:
            try:
                # 从队列获取图像数据
                image_data = self.image_queue.get(timeout=1.0)
                # 处理图像逻辑（此处省略）
                self.image_queue.task_done()
            except queue.Empty:
                pass
            
    def run(self):
        self.window.mainloop()

if __name__ == "__main__":
    app = ControlApp()
    app.run()