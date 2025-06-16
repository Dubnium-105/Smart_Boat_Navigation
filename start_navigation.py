#!/usr/bin/env python3
"""
船舶导航系统启动脚本
用于发送MQTT命令启动自动导航
"""

import paho.mqtt.client as mqtt
import json
import time
import argparse

# MQTT配置
MQTT_BROKER = "emqx.link2you.top"
MQTT_PORT = 1883
MQTT_TOPIC_NAVIGATION = "/navigation"

def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("✅ 成功连接到MQTT服务器")
        client.subscribe("/navigation/status")
    else:
        print(f"❌ 连接失败，错误码: {rc}")

def on_message(client, userdata, msg):
    if msg.topic == "/navigation/status":
        try:
            status = json.loads(msg.payload.decode())
            print(f"📊 导航状态更新: {status}")
        except:
            print(f"📊 导航状态: {msg.payload.decode()}")

def send_navigation_command(client, command):
    """发送导航命令"""
    payload = json.dumps({"command": command})
    client.publish(MQTT_TOPIC_NAVIGATION, payload)
    print(f"🚢 已发送导航命令: {command}")

def main():
    parser = argparse.ArgumentParser(description='船舶导航系统控制')
    parser.add_argument('command', choices=['standby', 'navigate', 'return', 'manual', 'clear_history'], 
                       help='导航命令')
    args = parser.parse_args()

    # 创建MQTT客户端
    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message

    try:
        print(f"🔌 正在连接到MQTT服务器 {MQTT_BROKER}:{MQTT_PORT}")
        client.connect(MQTT_BROKER, MQTT_PORT, 60)
        
        # 启动后台处理
        client.loop_start()
        
        # 等待连接
        time.sleep(2)
        
        # 发送命令
        send_navigation_command(client, args.command)
        
        # 等待状态反馈
        print("⏳ 等待状态反馈...")
        time.sleep(5)
        
    except KeyboardInterrupt:
        print("\n👋 用户中断")
    except Exception as e:
        print(f"❌ 错误: {e}")
    finally:
        client.loop_stop()
        client.disconnect()
        print("🔌 已断开MQTT连接")

if __name__ == "__main__":
    main()
