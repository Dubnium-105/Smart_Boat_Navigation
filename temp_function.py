    def send_navigation_command(self, command):
        """发送导航控制命令"""
        cmd = {
            "command": command
        }
        self.client.publish("/navigation", json.dumps(cmd))
        self.debug_print(f"已发送导航命令: {command}")
        
        # 如果是手动控制模式，确保滑块控制可用
        if command == "manual":
            self.auto_var.set(False)
            self.auto_control = False
