import tkinter as tk
from tkinter import ttk, messagebox, Scale
import paho.mqtt.client as mqtt
import json
import time
import threading
import math
from PIL import Image, ImageTk, ImageDraw

class IRControlApp:
    def __init__(self):
        self.window = tk.Tk()
        self.window.title("红外追踪控制系统")
        self.window.geometry("800x700")
        
        # 传感器数据 - 改为8个传感器
        self.ir_sensor_status = [False] * 8
        self.target_angle = -1.0
        self.target_strength = 0.0
        self.target_vector_x = 0.0
        self.target_vector_y = 0.0
        
        # 推荐的电机速度
        self.recommended_left = 0
        self.recommended_right = 0
        
        # 实际的电机速度
        self.motor_a_speed = 0
        self.motor_b_speed = 0
        
        # 传感器方向矢量（8个方向），根据实际安装顺序（0 1 7 6 5 4 3 2）
        # PB0 -> 0度, PB1 -> 45度, PB7 -> 90度, PB6 -> 135度
        # PB5 -> 180度, PB4 -> 225度, PB3 -> 270度, PB2 -> 315度
        self.ir_vectors = [
            (1.0, 0.0),        # 0度 - 正前方 - PB0
            (0.7071, 0.7071),  # 45度 - 右前方 - PB1
            (0.7071, -0.7071), # 315度 - 左前方 - PB2
            (0.0, -1.0),       # 270度 - 正左方 - PB3
            (-0.7071, -0.7071),# 225度 - 左后方 - PB4
            (-1.0, 0.0),       # 180度 - 正后方 - PB5
            (-0.7071, 0.7071), # 135度 - 右后方 - PB6
            (0.0, 1.0),        # 90度 - 正右方 - PB7
        ]
        
        # 添加调试输出区域
        self.create_debug_area()
        
        # MQTT配置
        self.debug_print("初始化MQTT客户端...")
        self.client = mqtt.Client()
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        self.client.on_disconnect = self.on_disconnect
        
        # 自动控制相关变量
        self.auto_control = False
        self.last_control_time = 0
        
        self.create_widgets()
        
        # 连接MQTT服务器
        self.connect_mqtt()
        
    def create_debug_area(self):
        """创建调试输出区域"""
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
        
        if hasattr(self, 'debug_text'):
            self.debug_text.insert(tk.END, formatted_msg)
            self.debug_text.see(tk.END)
            self.window.update_idletasks()
        
    def create_widgets(self):
        main_frame = ttk.Frame(self.window, padding=10)
        main_frame.pack(fill=tk.BOTH, expand=True)
        
        # 传感器可视化区域
        self.sensor_frame = ttk.LabelFrame(main_frame, text="红外传感器状态")
        self.sensor_frame.pack(pady=5, fill=tk.BOTH, expand=True)
        
        self.canvas = tk.Canvas(self.sensor_frame, width=400, height=400, bg="black")
        self.canvas.pack(pady=5, padx=5, fill=tk.BOTH, expand=True)
        
        # 目标信息显示
        self.target_info = ttk.Label(self.sensor_frame, text="目标: 未检测到", font=("Arial", 12))
        self.target_info.pack(pady=2)
        
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
        
        control_frame.columnconfigure(0, weight=1)
        control_frame.columnconfigure(1, weight=1)
        
        # 自动控制开关
        self.auto_var = tk.BooleanVar()
        self.auto_check = ttk.Checkbutton(main_frame, text="自动追踪目标",
                                        variable=self.auto_var,
                                        command=self.toggle_auto_control)
        self.auto_check.pack(pady=5)
        
        # 状态显示
        self.status_label = ttk.Label(main_frame, text="状态: 未连接")
        self.status_label.pack(pady=5)
        
        # 推荐控制值显示
        self.recommendation_label = ttk.Label(main_frame, text="推荐控制: 左=0, 右=0")
        self.recommendation_label.pack(pady=2)
        
        # 应用推荐控制按钮
        self.apply_button = ttk.Button(main_frame, text="应用推荐控制", command=self.apply_recommended_control)
        self.apply_button.pack(pady=5)
        
        # 定时更新UI
        self.window.after(100, self.update_ui)
        
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
        
    def update_ui(self):
        """更新UI显示"""
        self.update_sensor_visualization()
        self.window.after(100, self.update_ui)
        
    def update_sensor_visualization(self):
        """更新传感器可视化"""
        self.canvas.delete("all")
        
        width = self.canvas.winfo_width()
        height = self.canvas.winfo_height()
        
        center_x = width / 2
        center_y = height / 2
        radius = min(width, height) * 0.4
        
        self.canvas.create_oval(
            center_x - radius, center_y - radius,
            center_x + radius, center_y + radius,
            outline="gray", width=2
        )
        
        for angle in range(0, 360, 45):
            rad = math.radians(angle)
            x1 = center_x
            y1 = center_y
            x2 = center_x + radius * math.cos(rad)
            y2 = center_y - radius * math.sin(rad)
            self.canvas.create_line(x1, y1, x2, y2, fill="gray", dash=(4, 4))
            self.canvas.create_text(
                center_x + (radius + 20) * math.cos(rad),
                center_y - (radius + 20) * math.sin(rad),
                text=f"{angle}°", fill="white"
            )
        
        for i, (vx, vy) in enumerate(self.ir_vectors):
            angle = 360 - (i * 45)
            rad = math.radians(angle)
            sensor_x = center_x + radius * math.cos(rad)
            sensor_y = center_y - radius * math.sin(rad)
            
            color = "red" if self.ir_sensor_status[i] else "gray"
            
            self.canvas.create_oval(
                sensor_x - 5, sensor_y - 5,
                sensor_x + 5, sensor_y + 5,
                fill=color, outline=color
            )
            
        if self.target_angle >= 0 and self.target_strength > 0:
            target_rad = math.radians(self.target_angle)
            vector_length = radius * self.target_strength
            if vector_length > radius:
                vector_length = radius
            
            target_x = center_x + vector_length * math.cos(target_rad)
            target_y = center_y - vector_length * math.sin(target_rad)
            
            self.canvas.create_line(
                center_x, center_y, target_x, target_y,
                fill="yellow", width=3, arrow=tk.LAST
            )
            
            self.target_info.config(
                text=f"目标: 方向={self.target_angle:.1f}°, 强度={self.target_strength:.2f}"
            )
        else:
            self.target_info.config(text="目标: 未检测到")
    
    def process_sensor_data(self, data):
        """处理传感器数据"""
        if 'ir_sensors' in data:
            self.ir_sensor_status = data['ir_sensors']
            
        if 'target_angle' in data:
            self.target_angle = data['target_angle']
            
        if 'target_strength' in data:
            self.target_strength = data['target_strength']
            
        if 'target_x' in data and 'target_y' in data:
            self.target_vector_x = data['target_x']
            self.target_vector_y = data['target_y']
            
        if 'recommended_left' in data and 'recommended_right' in data:
            self.recommended_left = data['recommended_left']
            self.recommended_right = data['recommended_right']
            self.recommendation_label.config(
                text=f"推荐控制: 左={self.recommended_left}, 右={self.recommended_right}"
            )
            
        if self.auto_control:
            self.apply_recommended_control()
    
    def apply_recommended_control(self):
        """应用推荐的控制值"""
        if self.recommended_left != 0 or self.recommended_right != 0:
            self.motor_a_speed = self.recommended_left
            self.motor_b_speed = self.recommended_right
            
            self.slider_L.set(self.motor_a_speed)
            self.slider_R.set(self.motor_b_speed)
            
            self.send_motor_command()
            
    def on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.debug_print("已成功连接到MQTT服务器")
            self.status_label.config(text="状态: 已连接")
            
            topics = [
                ("/motor/status", 0),
                ("/sensor/data", 0),
                ("/ESP32_info", 0)
            ]
            
            for topic, qos in topics:
                self.client.subscribe(topic, qos)
                self.debug_print(f"已订阅主题: {topic}")
                
            self.client.publish("/UI_info", "UI已连接")
        else:
            self.debug_print(f"连接失败，返回码: {rc}")
            self.status_label.config(text=f"状态: 连接失败 (代码: {rc})")
    
    def on_disconnect(self, client, userdata, rc):
        self.debug_print(f"与MQTT服务器断开连接，代码: {rc}")
        self.status_label.config(text="状态: 已断开连接")
        
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
                    self.process_sensor_data(data)
                except Exception as e:
                    self.debug_print(f"处理传感器数据错误: {str(e)}")
                
        except Exception as e:
            self.debug_print(f"处理消息错误: {str(e)}")
                
    def on_slider(self, value):
        if not self.auto_control:
            self.motor_a_speed = int(self.slider_L.get())
            self.motor_b_speed = int(self.slider_R.get())
            
            self.send_motor_command()
    
    def send_motor_command(self):
        """发送电机控制命令"""
        cmd = {
            "speedA": self.motor_a_speed,
            "speedB": self.motor_b_speed
        }
        self.client.publish("/motor", json.dumps(cmd))
        
        self.motor_a_label.config(text=f"左电机速度: {self.motor_a_speed}")
        self.motor_b_label.config(text=f"右电机速度: {self.motor_b_speed}")
        
    def toggle_auto_control(self):
        self.auto_control = self.auto_var.get()
        self.debug_print(f"自动控制: {'开启' if self.auto_control else '关闭'}")
        
        if not self.auto_control:
            self.motor_a_speed = 0
            self.motor_b_speed = 0
            cmd = {"speedA": 0, "speedB": 0}
            self.client.publish("/motor", json.dumps(cmd))
            self.debug_print("停止电机")
            
            self.motor_a_label.config(text=f"左电机速度: {self.motor_a_speed}")
            self.motor_b_label.config(text=f"右电机速度: {self.motor_b_speed}")
    
    def run(self):
        self.window.mainloop()
        self.client.loop_stop()
        self.client.disconnect()

if __name__ == "__main__":
    app = IRControlApp()
    app.run()