import tkinter as tk
from tkinter import ttk, messagebox
import paho.mqtt.client as mqtt
import json
import time
import math

class SimpleBoatControl:
    def __init__(self):
        self.window = tk.Tk()
        self.window.title("智能船舶导航控制")
        self.window.geometry("800x850")
        
        # 传感器数据显示
        self.ir_sensor_status = [False] * 8
        self.target_angle = -1.0
        self.target_strength = 0.0
        
        # 电机控制
        self.motor_left_speed = 0
        self.motor_right_speed = 0
        
        # 推荐电机速度（由系统计算）
        self.recommended_left_speed = 0
        self.recommended_right_speed = 0
        
        # 本地计算的推荐速度（使用ESP32同样的算法）
        self.local_recommended_left = 0
        self.local_recommended_right = 0
        
        # 导航状态
        self.nav_state = "standby"
        self.state_duration = 0
        
        # 状态映射
        self.nav_states_map = {
            "standby": "待机",
            "navigating": "自动导航",
            "arrived": "已到达",
            "returning": "返航中",
            "manual": "手动控制",
            "error": "错误状态"
        }
        
        # MQTT客户端
        self.client = mqtt.Client()
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        self.client.on_disconnect = self.on_disconnect
        
        self.create_widgets()
        self.connect_mqtt()
        
    def create_widgets(self):
        main_frame = ttk.Frame(self.window, padding=10)
        main_frame.pack(fill=tk.BOTH, expand=True)
        
        # 导航控制面板
        nav_frame = ttk.LabelFrame(main_frame, text="导航控制")
        nav_frame.pack(pady=5, fill=tk.X)
        
        # 导航状态显示
        self.nav_state_label = ttk.Label(nav_frame, text="导航状态: 待机 (0秒)", font=("Arial", 14, "bold"))
        self.nav_state_label.pack(pady=10)
        
        # 导航控制按钮
        nav_buttons_frame = ttk.Frame(nav_frame)
        nav_buttons_frame.pack(fill=tk.X, padx=10, pady=10)
        
        self.standby_btn = ttk.Button(nav_buttons_frame, text="待机", 
                                     command=lambda: self.send_navigation_command("standby"))
        self.standby_btn.grid(row=0, column=0, padx=5, pady=5, sticky="ew")
        
        self.navigate_btn = ttk.Button(nav_buttons_frame, text="开始自动导航", 
                                      command=lambda: self.send_navigation_command("navigate"))
        self.navigate_btn.grid(row=0, column=1, padx=5, pady=5, sticky="ew")
        
        self.return_btn = ttk.Button(nav_buttons_frame, text="返航", 
                                    command=lambda: self.send_navigation_command("return"))
        self.return_btn.grid(row=1, column=0, padx=5, pady=5, sticky="ew")
        
        self.manual_btn = ttk.Button(nav_buttons_frame, text="手动控制", 
                                    command=lambda: self.send_navigation_command("manual"))
        self.manual_btn.grid(row=1, column=1, padx=5, pady=5, sticky="ew")
        
        for i in range(2):
            nav_buttons_frame.columnconfigure(i, weight=1)
        
        # 传感器状态显示
        sensor_frame = ttk.LabelFrame(main_frame, text="传感器状态")
        sensor_frame.pack(pady=5, fill=tk.X)
        
        self.target_info = ttk.Label(sensor_frame, text="目标: 未检测到", font=("Arial", 12))
        self.target_info.pack(pady=5)
        
        # 简化的传感器状态显示
        sensor_status_frame = ttk.Frame(sensor_frame)
        sensor_status_frame.pack(pady=5)
        
        ttk.Label(sensor_status_frame, text="红外传感器: ").pack(side=tk.LEFT)
        self.sensor_status_label = ttk.Label(sensor_status_frame, text="[0,0,0,0,0,0,0,0]", 
                                           font=("Courier", 10))
        self.sensor_status_label.pack(side=tk.LEFT)
        
        # 推荐电机速度显示
        recommended_frame = ttk.LabelFrame(main_frame, text="推荐电机速度")
        recommended_frame.pack(pady=5, fill=tk.X)
        
        # ESP32推荐速度（来自MQTT）
        esp32_rec_frame = ttk.Frame(recommended_frame)
        esp32_rec_frame.pack(pady=5, fill=tk.X)
        
        ttk.Label(esp32_rec_frame, text="ESP32推荐速度:", font=("Arial", 10, "bold")).pack()
        
        esp32_speed_frame = ttk.Frame(esp32_rec_frame)
        esp32_speed_frame.pack(pady=5)
        
        # ESP32推荐左电机速度
        esp32_left_frame = ttk.Frame(esp32_speed_frame)
        esp32_left_frame.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=20)
        
        ttk.Label(esp32_left_frame, text="左电机:", font=("Arial", 9)).pack()
        self.rec_left_speed_label = ttk.Label(esp32_left_frame, text="0", 
                                            font=("Arial", 14), foreground="blue")
        self.rec_left_speed_label.pack()
        
        # ESP32推荐右电机速度
        esp32_right_frame = ttk.Frame(esp32_speed_frame)
        esp32_right_frame.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=20)
        
        ttk.Label(esp32_right_frame, text="右电机:", font=("Arial", 9)).pack()
        self.rec_right_speed_label = ttk.Label(esp32_right_frame, text="0", 
                                             font=("Arial", 14), foreground="blue")
        self.rec_right_speed_label.pack()
        
        # 本地计算推荐速度
        local_rec_frame = ttk.Frame(recommended_frame)
        local_rec_frame.pack(pady=5, fill=tk.X)
        
        ttk.Label(local_rec_frame, text="本地计算推荐速度:", font=("Arial", 10, "bold")).pack()
        
        local_speed_frame = ttk.Frame(local_rec_frame)
        local_speed_frame.pack(pady=5)
        
        # 本地推荐左电机速度
        local_left_frame = ttk.Frame(local_speed_frame)
        local_left_frame.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=20)
        
        ttk.Label(local_left_frame, text="左电机:", font=("Arial", 9)).pack()
        self.local_left_speed_label = ttk.Label(local_left_frame, text="0", 
                                               font=("Arial", 14), foreground="green")
        self.local_left_speed_label.pack()
        
        # 本地推荐右电机速度
        local_right_frame = ttk.Frame(local_speed_frame)
        local_right_frame.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=20)
        
        ttk.Label(local_right_frame, text="右电机:", font=("Arial", 9)).pack()
        self.local_right_speed_label = ttk.Label(local_right_frame, text="0", 
                                                font=("Arial", 14), foreground="green")
        self.local_right_speed_label.pack()
        
        # 算法选择和应用按钮
        button_frame = ttk.Frame(recommended_frame)
        button_frame.pack(pady=10, fill=tk.X)
        
        # 算法选择
        algo_frame = ttk.Frame(button_frame)
        algo_frame.pack(side=tk.LEFT, padx=10)
        
        ttk.Label(algo_frame, text="算法选择:").pack(side=tk.LEFT)
        self.algorithm_var = tk.StringVar(value="basic")
        basic_radio = ttk.Radiobutton(algo_frame, text="基础算法", variable=self.algorithm_var, 
                                     value="basic", command=self.on_algorithm_change)
        basic_radio.pack(side=tk.LEFT, padx=5)
        
        advanced_radio = ttk.Radiobutton(algo_frame, text="高级算法", variable=self.algorithm_var, 
                                        value="advanced", command=self.on_algorithm_change)
        advanced_radio.pack(side=tk.LEFT, padx=5)
        
        # 应用按钮
        apply_frame = ttk.Frame(button_frame)
        apply_frame.pack(side=tk.RIGHT, padx=10)
        
        apply_esp32_btn = ttk.Button(apply_frame, text="应用ESP32推荐速度", 
                                    command=self.apply_esp32_recommended_speed)
        apply_esp32_btn.pack(side=tk.LEFT, padx=5)
        
        apply_local_btn = ttk.Button(apply_frame, text="应用本地推荐速度", 
                                    command=self.apply_local_recommended_speed)
        apply_local_btn.pack(side=tk.LEFT, padx=5)
        
        # 手动控制面板（仅在手动模式下使用）
        manual_frame = ttk.LabelFrame(main_frame, text="手动控制（仅手动模式下有效）")
        manual_frame.pack(pady=5, fill=tk.X)
        
        # 电机控制滑块
        motor_control_frame = ttk.Frame(manual_frame)
        motor_control_frame.pack(fill=tk.X, padx=10, pady=10)
        
        # 左电机控制
        left_frame = ttk.Frame(motor_control_frame)
        left_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=5)
        
        ttk.Label(left_frame, text="左电机").pack()
        self.left_scale = ttk.Scale(left_frame, from_=-100, to=100, orient=tk.VERTICAL,
                                   command=self.on_manual_control)
        self.left_scale.pack(fill=tk.Y, expand=True)
        self.left_speed_label = ttk.Label(left_frame, text="速度: 0")
        self.left_speed_label.pack()
        
        # 右电机控制
        right_frame = ttk.Frame(motor_control_frame)
        right_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=5)
        
        ttk.Label(right_frame, text="右电机").pack()
        self.right_scale = ttk.Scale(right_frame, from_=-100, to=100, orient=tk.VERTICAL,
                                    command=self.on_manual_control)
        self.right_scale.pack(fill=tk.Y, expand=True)
        self.right_speed_label = ttk.Label(right_frame, text="速度: 0")
        self.right_speed_label.pack()
        
        # 快速控制按钮
        quick_control_frame = ttk.Frame(manual_frame)
        quick_control_frame.pack(pady=10)
        
        ttk.Button(quick_control_frame, text="停止", 
                  command=self.stop_motors).pack(side=tk.LEFT, padx=5)
        ttk.Button(quick_control_frame, text="前进", 
                  command=lambda: self.quick_move(50, 50)).pack(side=tk.LEFT, padx=5)
        ttk.Button(quick_control_frame, text="后退", 
                  command=lambda: self.quick_move(-50, -50)).pack(side=tk.LEFT, padx=5)
        ttk.Button(quick_control_frame, text="左转", 
                  command=lambda: self.quick_move(-30, 30)).pack(side=tk.LEFT, padx=5)
        ttk.Button(quick_control_frame, text="右转", 
                  command=lambda: self.quick_move(30, -30)).pack(side=tk.LEFT, padx=5)
        
        # 连接状态和日志
        status_frame = ttk.LabelFrame(main_frame, text="系统状态")
        status_frame.pack(pady=5, fill=tk.BOTH, expand=True)
        
        self.status_label = ttk.Label(status_frame, text="状态: 未连接")
        self.status_label.pack(pady=5)
        
        # 简化的日志显示
        self.log_text = tk.Text(status_frame, height=6, width=80)
        self.log_text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        log_scrollbar = ttk.Scrollbar(status_frame, command=self.log_text.yview)
        log_scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        self.log_text.config(yscrollcommand=log_scrollbar.set)
        
        # 定时更新UI
        self.window.after(100, self.update_ui)
        
    def log_message(self, message):
        """记录日志消息"""
        timestamp = time.strftime("%H:%M:%S", time.localtime())
        formatted_msg = f"[{timestamp}] {message}\n"
        print(formatted_msg, end="")
        
        if hasattr(self, 'log_text'):
            self.log_text.insert(tk.END, formatted_msg)
            self.log_text.see(tk.END)
            
    def connect_mqtt(self):
        """连接到MQTT服务器"""
        try:
            self.log_message("正在连接到MQTT服务器...")
            self.client.connect("emqx.link2you.top", 1883, 60)
            self.client.loop_start()
        except Exception as e:
            self.log_message(f"MQTT连接错误: {str(e)}")
            messagebox.showerror("连接错误", f"无法连接到MQTT服务器: {str(e)}")
        
    def on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.log_message("已成功连接到MQTT服务器")
            self.status_label.config(text="状态: 已连接")
            
            # 订阅必要的主题
            topics = [
                ("/sensor/data", 0),
                ("/navigation/status", 0),
                ("/motor/status", 0)
            ]
            
            for topic, qos in topics:
                self.client.subscribe(topic, qos)
                self.log_message(f"已订阅主题: {topic}")
                
        else:
            self.log_message(f"连接失败，返回码: {rc}")
            self.status_label.config(text=f"状态: 连接失败 (代码: {rc})")
    
    def on_disconnect(self, client, userdata, rc):
        self.log_message(f"与MQTT服务器断开连接")
        self.status_label.config(text="状态: 已断开连接")
        
    def on_message(self, client, userdata, msg):
        try:
            if msg.topic == "/sensor/data":
                data = json.loads(msg.payload.decode())
                self.process_sensor_data(data)
                
            elif msg.topic == "/navigation/status":
                data = json.loads(msg.payload.decode())
                if 'state' in data:
                    self.nav_state = data['state']
                    
            elif msg.topic == "/motor/status":
                status = msg.payload.decode()
                self.log_message(f"电机状态: {status}")
                
        except Exception as e:
            self.log_message(f"处理消息错误: {str(e)}")
                
    def process_sensor_data(self, data):
        """处理传感器数据"""
        # 更新传感器状态
        if 'ir_sensors' in data:
            self.ir_sensor_status = data['ir_sensors']
            sensor_display = [1 if status else 0 for status in self.ir_sensor_status]
            self.sensor_status_label.config(text=f"{sensor_display}")
            
        # 更新目标信息
        if 'target_angle' in data:
            self.target_angle = data['target_angle']
            
        if 'target_strength' in data:
            self.target_strength = data['target_strength']
            
        # 更新推荐电机速度
        if 'recommended_left' in data:
            self.recommended_left_speed = data['recommended_left']
            
        if 'recommended_right' in data:
            self.recommended_right_speed = data['recommended_right']
            
        # 更新导航状态
        if 'nav_state' in data:
            self.nav_state = data['nav_state']
            
        if 'state_duration' in data:
            self.state_duration = data['state_duration']
        
        # 每次收到传感器数据时，也计算本地推荐速度
        self.calculate_local_recommended_speed()
            
    def calculate_local_recommended_speed(self):
        """
        使用main.cpp中相同的算法在本地计算推荐电机速度
        """
        left_speed = 0
        right_speed = 0
        
        # 只有当有效目标存在时才计算电机速度（信号强度大于阈值）
        if self.target_strength > 0.1:
            # 根据选择的算法计算推荐速度
            if self.algorithm_var.get() == "basic":
                left_speed, right_speed = self._calculate_basic_algorithm()
            else:
                left_speed, right_speed = self._calculate_advanced_algorithm()
        
        self.local_recommended_left = left_speed
        self.local_recommended_right = right_speed
        
        return left_speed, right_speed
    
    def _calculate_basic_algorithm(self):
        """基础算法（与ESP32的send_sensor_data中的算法对应）"""
        # 算法参数（与ESP32保持一致）
        base_speed = 50  # 基础速度
        max_turn_ratio = 1.8  # 最大转向比率
        pivot_threshold = 0.9  # 原地转向阈值
        
        # 计算目标方向与船头朝向的夹角
        relative_angle = self.target_angle
        
        # 计算转向因子（-1.0到1.0）
        # 正值表示向右转，负值表示向左转，绝对值越大表示转向越急
        turn_factor = 0.0
        
        # 前方区域（-90到90度）
        if ((relative_angle >= 0 and relative_angle <= 90) or 
            (relative_angle >= 270 and relative_angle <= 360)):
            # 调整角度到-90到90范围
            adjusted_angle = relative_angle - 360 if relative_angle > 180 else relative_angle
            # 角度越大，转向系数越大
            turn_factor = adjusted_angle / 90.0
        # 后方区域（90到270度）
        else:
            # 调整角度到-180到180范围
            adjusted_angle = relative_angle - 360 if relative_angle > 270 else relative_angle
            adjusted_angle = 360 - adjusted_angle if adjusted_angle > 180 else adjusted_angle
            # 后方区域的转向系数正负取决于最近的转向方向
            if relative_angle > 90 and relative_angle < 180:
                turn_factor = 1.0  # 向右转最快到达
            elif relative_angle > 180 and relative_angle < 270:
                turn_factor = -1.0  # 向左转最快到达
            elif relative_angle == 180:
                # 正后方可以随机选择一个方向
                turn_factor = 1.0 if (int(time.time() * 1000) % 2 == 0) else -1.0
        
        # 根据转向因子调整左右电机速度
        # 使用平滑的速度曲线来实现更精确的转向控制
        turn_strength = abs(turn_factor)
        turn_direction = 1.0 if turn_factor >= 0 else -1.0
        
        # 前方区域使用差速转向
        if ((relative_angle >= 315 or relative_angle < 45) or
            (relative_angle >= 135 and relative_angle < 225)):
            # 前进或后退，根据方向调整差速
            is_forward = (relative_angle >= 315 or relative_angle < 45)
            direction = 1 if is_forward else -1
            
            # 应用非线性转向曲线，提高小角度修正的精度
            turn_adjustment = base_speed * turn_strength * max_turn_ratio
            
            if turn_strength > pivot_threshold:
                # 大角度时考虑原地转向
                pivot_factor = (turn_strength - pivot_threshold) / (1.0 - pivot_threshold)
                reduced_speed = base_speed * (1.0 - pivot_factor * 0.8)
                
                if turn_direction > 0:
                    # 向右转
                    left_speed = direction * reduced_speed
                    right_speed = direction * reduced_speed * (1.0 - 2.0 * pivot_factor)
                    if pivot_factor > 0.5:
                        right_speed = -right_speed  # 原地右转
                else:
                    # 向左转
                    right_speed = direction * reduced_speed
                    left_speed = direction * reduced_speed * (1.0 - 2.0 * pivot_factor)
                    if pivot_factor > 0.5:
                        left_speed = -left_speed  # 原地左转
            else:
                # 小角度时使用传统差速
                if turn_direction > 0:
                    # 向右转
                    left_speed = direction * base_speed
                    right_speed = direction * (base_speed - turn_adjustment)
                else:
                    # 向左转
                    right_speed = direction * base_speed
                    left_speed = direction * (base_speed - turn_adjustment)
        # 侧面区域使用原地旋转 + 前进相结合
        else:
            # 计算侧向程度（0 = 正前/正后，1 = 正左/正右）
            side_factor = 0.0
            if relative_angle >= 45 and relative_angle < 135:
                # 右侧
                side_factor = math.sin(math.radians(relative_angle))
            elif relative_angle >= 225 and relative_angle < 315:
                # 左侧
                side_factor = math.sin(math.radians(relative_angle))
            
            # 使用平滑过渡从差速转向到原地旋转
            if turn_direction > 0:
                # 向右转
                left_speed = base_speed * (1.0 - side_factor * 0.5)
                right_speed = base_speed * (1.0 - side_factor * 1.5)
                if side_factor > 0.7:
                    right_speed = -right_speed * (side_factor - 0.7) / 0.3
            else:
                # 向左转
                right_speed = base_speed * (1.0 - side_factor * 0.5)
                left_speed = base_speed * (1.0 - side_factor * 1.5)
                if side_factor > 0.7:
                    left_speed = -left_speed * (side_factor - 0.7) / 0.3
        
        # 确保速度值在有效范围内
        left_speed = max(-100, min(100, int(left_speed)))
        right_speed = max(-100, min(100, int(right_speed)))
        
        return left_speed, right_speed
    
    def _calculate_advanced_algorithm(self):
        """
        高级自动控制算法 - 改进的精确追踪算法（与ESP32的calculate_advanced_control函数对应）
        """
        if self.target_strength <= 0.1 or self.target_angle < 0:
            # 无目标时的搜索行为
            return 20, -20  # 缓慢右转搜索
        
        # 动态速度参数
        base_speed = 55
        max_turn_ratio = 1.5
        pivot_threshold = 0.85
        approach_threshold = 0.7
        
        # 计算目标方向的转向系数
        angle_rad = math.radians(self.target_angle)
        
        # 计算最短转向角度
        shortest_angle = self.target_angle
        if shortest_angle > 180:
            shortest_angle = 360 - shortest_angle
            turn_factor = -math.sin(angle_rad)  # 左转
        else:
            turn_factor = math.sin(angle_rad)   # 右转
        
        # 根据信号强度调整基础速度
        dynamic_speed = base_speed
        if self.target_strength > approach_threshold:
            # 接近目标时减速
            dynamic_speed = base_speed * (1.0 - (self.target_strength - approach_threshold) / (1.0 - approach_threshold) * 0.4)
        
        # 精确的转向控制
        turn_strength = abs(turn_factor)
        
        if turn_strength > pivot_threshold:
            # 大角度时原地转向
            pivot_speed = dynamic_speed * 0.6
            if turn_factor > 0:  # 右转
                left_speed = pivot_speed
                right_speed = -pivot_speed / 2
            else:  # 左转
                left_speed = -pivot_speed / 2
                right_speed = pivot_speed
        else:
            # 小角度时差速转向
            speed_diff = dynamic_speed * turn_strength * max_turn_ratio
            
            if self.target_angle >= 315 or self.target_angle < 45:
                # 前方区域 - 直行为主，微调转向
                left_speed = dynamic_speed
                right_speed = dynamic_speed
                
                if turn_factor > 0.1:  # 需要右转
                    right_speed -= speed_diff * 0.5
                elif turn_factor < -0.1:  # 需要左转
                    left_speed -= speed_diff * 0.5
            elif self.target_angle >= 45 and self.target_angle < 135:
                # 右侧区域
                left_speed = dynamic_speed
                right_speed = dynamic_speed - speed_diff
            elif self.target_angle >= 135 and self.target_angle < 225:
                # 后方区域 - 根据哪侧更近选择转向方向
                pivot_speed = dynamic_speed * 0.7
                if self.target_angle < 180:  # 右后方，右转
                    left_speed = pivot_speed
                    right_speed = -pivot_speed / 3
                else:  # 左后方，左转
                    left_speed = -pivot_speed / 3
                    right_speed = pivot_speed
            else:
                # 左侧区域
                left_speed = dynamic_speed - speed_diff
                right_speed = dynamic_speed
        
        # 限制速度范围
        left_speed = max(-100, min(100, int(left_speed)))
        right_speed = max(-100, min(100, int(right_speed)))
        
        return left_speed, right_speed
            
    def update_ui(self):
        """更新UI显示"""
        # 更新导航状态显示
        state_text = self.nav_states_map.get(self.nav_state, "未知状态")
        self.nav_state_label.config(
            text=f"导航状态: {state_text} ({self.state_duration}秒)"
        )
        
        # 更新目标信息显示
        if self.target_angle >= 0 and self.target_strength > 0:
            self.target_info.config(
                text=f"目标: 方向={self.target_angle:.1f}°, 强度={self.target_strength:.2f}"
            )
        else:
            self.target_info.config(text="目标: 未检测到")
        
        # 更新推荐速度显示
        self.rec_left_speed_label.config(text=str(self.recommended_left_speed))
        self.rec_right_speed_label.config(text=str(self.recommended_right_speed))
        self.local_left_speed_label.config(text=str(self.local_recommended_left))
        self.local_right_speed_label.config(text=str(self.local_recommended_right))
            
        # 根据当前状态启用/禁用手动控制
        manual_enabled = (self.nav_state == "manual")
        self.left_scale.config(state="normal" if manual_enabled else "disabled")
        self.right_scale.config(state="normal" if manual_enabled else "disabled")
        
        self.window.after(100, self.update_ui)
    
    def on_algorithm_change(self):
        """算法选择变化时重新计算推荐速度"""
        self.calculate_local_recommended_speed()
        self.log_message(f"切换到{self.algorithm_var.get()}算法")
        
    def send_navigation_command(self, command):
        """发送导航控制命令"""
        cmd = {"command": command}
        self.client.publish("/navigation", json.dumps(cmd))
        self.log_message(f"已发送导航命令: {command}")
        
        # 如果切换到手动模式，停止电机
        if command == "manual":
            self.stop_motors()
    
    def on_manual_control(self, value=None):
        """手动控制回调（仅在手动模式下有效）"""
        if self.nav_state == "manual":
            self.motor_left_speed = int(self.left_scale.get())
            self.motor_right_speed = int(self.right_scale.get())
            
            self.send_motor_command()
            
    def send_motor_command(self):
        """发送电机控制命令"""
        cmd = {
            "speedA": self.motor_left_speed,
            "speedB": self.motor_right_speed
        }
        self.client.publish("/motor", json.dumps(cmd))
        
        self.left_speed_label.config(text=f"速度: {self.motor_left_speed}")
        self.right_speed_label.config(text=f"速度: {self.motor_right_speed}")
    
    def apply_esp32_recommended_speed(self):
        """应用ESP32推荐的电机速度（仅在手动模式下有效）"""
        if self.nav_state == "manual":
            self.left_scale.set(self.recommended_left_speed)
            self.right_scale.set(self.recommended_right_speed)
            self.motor_left_speed = self.recommended_left_speed
            self.motor_right_speed = self.recommended_right_speed
            self.send_motor_command()
            self.log_message(f"已应用ESP32推荐速度: 左={self.recommended_left_speed}, 右={self.recommended_right_speed}")
        else:
            self.log_message("只能在手动模式下应用推荐速度")
    
    def apply_local_recommended_speed(self):
        """应用本地计算的推荐电机速度（仅在手动模式下有效）"""
        if self.nav_state == "manual":
            self.left_scale.set(self.local_recommended_left)
            self.right_scale.set(self.local_recommended_right)
            self.motor_left_speed = self.local_recommended_left
            self.motor_right_speed = self.local_recommended_right
            self.send_motor_command()
            self.log_message(f"已应用本地推荐速度: 左={self.local_recommended_left}, 右={self.local_recommended_right}")
        else:
            self.log_message("只能在手动模式下应用推荐速度")
        
    def quick_move(self, left, right):
        """快速移动控制"""
        if self.nav_state == "manual":
            self.left_scale.set(left)
            self.right_scale.set(right)
            self.motor_left_speed = left
            self.motor_right_speed = right
            self.send_motor_command()
        
    def stop_motors(self):
        """停止所有电机"""
        if self.nav_state == "manual":
            self.left_scale.set(0)
            self.right_scale.set(0)
            self.motor_left_speed = 0
            self.motor_right_speed = 0
            self.send_motor_command()
        
    def run(self):
        self.window.mainloop()
        if hasattr(self, 'client'):
            self.client.loop_stop()
            self.client.disconnect()

if __name__ == "__main__":
    app = SimpleBoatControl()
    app.run()
