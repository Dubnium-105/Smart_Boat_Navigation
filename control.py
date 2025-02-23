import paho.mqtt.client as mqtt
import time

class BoatController:
    def __init__(self):
        self.client = mqtt.Client()
        self.client.on_connect = self.on_connect
        self.client.connect("8.210.4.26", 1883, 60)  # 本地服务器IP
        self.client.loop_start()
        
    def on_connect(self, client, userdata, flags, rc):
        print(f"Connected with result code {rc}")
        
    def send_control(self, left, right):
        payload = f"{left},{right}"
        self.client.publish("/control", payload)
        print(f"Sent: L={left}% R={right}%")

if __name__ == "__main__":
    controller = BoatController()
    try:
        while True:
            cmd = input("Enter speed (L,R): ")
            left, right = map(int, cmd.split(','))
            controller.send_control(left, right)
            time.sleep(0.1)
    except KeyboardInterrupt:
        controller.client.disconnect()
