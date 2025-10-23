# 附录C — 开发环境搭建指南

本附录为开发者提供完整的基线环境搭建、编译与调试流程，覆盖设备端（ESP32 + PlatformIO / Arduino）与岸基网页端（静态网页或本地服务器）在 Windows 环境下的常见步骤、示例命令与常见问题排查。所有命令以 Windows cmd.exe 为准。

目录
- 设备端（ESP32-S3）
  - 开发环境安装
  - 配置与编译
  - 烧录与串口调试
- 岸基网页端
  - 本地运行与调试
  - 连接配置
- 常见问题与排查流程

设备端（ESP32-S3 + PlatformIO）

1. 前提
- 操作系统：Windows 10/11
- 硬件：ESP32-S3 开发板（如 DevKitM-1），USB 数据线

2. 安装工具
- 安装 VS Code：从 https://code.visualstudio.com/ 下载并安装。
- 安装 PlatformIO 插件：在 VS Code 扩展市场搜索 PlatformIO IDE 并安装，或在扩展中安装 PlatformIO。

3. 获取仓库
- 将本项目克隆或复制到本地，例如：

    cd H:\Code
    git clone <your-repo-url> Smart_Boat_Navigation

4. 配置 PlatformIO
- 打开 `Smart_Boat_Navigation` 文件夹（VS Code -> File -> Open Folder）。
- 打开 `platformio.ini`，检查板卡（board）配置是否匹配你的 ESP32 模块（esp32-s3-devkitm-1）。

5. 修改 Wi‑Fi 与 MQTT 配置
- 编辑 `include/config.h` 文件，填入你的 Wi-Fi 凭据、MQTT 服务器地址等信息。

6. 构建与烧录（示例：CMD）

    REM 在仓库根目录执行
    pio run
    pio run -t upload --upload-port COM5
    pio device monitor -b 115200

  - 说明：将 `COM5` 替换为实际设备端口，串口监视可用于查看设备日志。

7. 调试要点
- 检查串口日志（115200），关注 Wi‑Fi 连接、MQTT 连接状态与回调日志。
- 若编译失败，检查 `platformio.ini` 的依赖库声明（PubSubClient、ArduinoJson 等）。

岸基网页端（静态网页）

1. 浏览器直接打开
- 将 `MQTT网页控制端/index.html` 直接在浏览器中打开（file://）可以在内网测试，但某些浏览器可能限制 WebSocket 或资源加载。

2. 本地静态服务器（推荐）
- 使用简单的 HTTP 静态服务器（例如 Node.js 的 http-server 或 Python 的 http.server）。

  - Node.js（需先安装 Node.js）：

      npm install -g http-server
      cd "H:\Code\Smart_Boat_Navigation\MQTT网页控制端"
      http-server -p 8080

  - 或 Python（Windows 自带或安装）：

      cd "H:\Code\Smart_Boat_Navigation\MQTT网页控制端"
      python -m http.server 8080

  - 访问：在浏览器打开 `http://localhost:8080/index.html`

3. 配置前端 MQTT 连接
- 编辑 `MQTT网页控制端/index.js` 中的 `host`, `port`, `path` 配置，参见 `网页MQTT配置修改指南.md`。

4. 调试与浏览器控制台
- 打开 F12 控制台查看连接状态、错误与日志。

常见问题与排查
- 无法连接 MQTT：检查 Broker 是否允许 WebSocket，端口与防火墙；前端使用 ws/wss，设备端使用 TCP 或 TLS（端口对应）。
- 指令无响应：确认 `clientId` 一致，设备端是否订阅相应主题，查看串口日志是否收到了消息。
- 网络问题：Wi‑Fi 信号弱或路由器对局域网隔离，建议使用热点或路由器做桥接测试。

安全建议
- 生产环境建议启用 Broker 鉴权（用户名/密码）或 TLS（wss、ssl），并在前端与设备端都配置证书或鉴权信息。

附录：关键文件路径
- 设备端：`platformio.ini`, `include/*.h`, `src/*.cpp`
- 网页端：`MQTT网页控制端/index.html`, `index.js`, `index.css`
