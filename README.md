# i.MX6ULL PTZ Vision Monitor

基于 **i.MX6ULL Linux 主控 + STM32F103 底层控制 + Qt 界面 + YOLO/MOSSE 视觉跟踪 + 阿里云 MQTT + RTMP 推流** 的智能安防视觉跟踪云台系统。

本项目实现了摄像头视频采集、人体检测、目标跟踪、云台舵机控制、环境传感器采集、云端物模型上报、RTMP 视频推流，以及基于人体红外模块的应用级休眠和系统级唤醒功能。

## 1. 项目功能

- i.MX6ULL 运行 Qt 图形界面，显示摄像头画面、传感器数据和系统状态。
- USB 摄像头采集视频流，使用 YOLOv8n 进行人体检测。
- YOLO 检测到人体后，使用 MOSSE 跟踪器进行连续跟踪。
- 根据目标框中心与图像中心的像素误差，通过 PID 控制舵机云台转动。
- STM32 负责舵机 PWM、步进电机、DHT11、MQ2、HC-SR501 人体红外模块。
- i.MX6ULL 与 STM32 之间通过 UART 串口通信。
- 通过 MQTT 接入阿里云物联网平台，实现属性上报和云端控制。
- 通过 FFmpeg 将本地图像推流到 Ubuntu 上的 nginx-rtmp 服务器。
- 无人时进入应用级 IDLE，关闭摄像头、YOLO、RTMP 等高负载模块。
- 长时间无人时进入 Linux 系统级 suspend，由 STM32 通过 GPIO 唤醒 i.MX6ULL。

## 2. 系统架构

```text
HC-SR501 / DHT11 / MQ2 / 舵机 / 步进电机
        ↓
STM32F103C8T6
        ↓ UART
 i.MX6ULL Linux 开发板
        ↓
Qt 应用程序
        ├── SensorWorker：串口通信
        ├── VisionWorker：摄像头采集 + MOSSE 跟踪
        ├── DetectorWorker：YOLO 人体检测
        ├── MqttWorker：阿里云 MQTT
        └── RtmpStreamer：FFmpeg RTMP 推流
        ↓
LCD 显示 / 阿里云物联网平台 / nginx-rtmp
```

## 3. 目录结构

```text
imx6ull-ptz-vision-monitor/
├── security_panel.pro
├── main.cpp
├── mainwindow.cpp
├── mainwindow.h
├── sensorworker.cpp
├── sensorworker.h
├── visionworker.cpp
├── visionworker.h
├── detectorworker.cpp
├── detectorworker.h
├── mqttworker.cpp
├── mqttworker.h
├── rtmpstreamer.cpp
├── rtmpstreamer.h
├── stm32_ptz_driver/
└── docs/
    ├── hardware_connection.md
    ├── uart_protocol.md
    ├── aliyun_iot_model.md
    └── suspend_wakeup.md
```

## 4. 软件环境

### i.MX6ULL 端

- Linux：100ASK i.MX6ULL Buildroot 系统
- Qt：Qt Widgets
- OpenCV：用于图像处理和 MOSSE 跟踪
- ncnn：用于 YOLOv8n 推理
- FFmpeg：用于 RTMP 推流
- MQTT：用于接入阿里云物联网平台

### STM32 端

- MCU：STM32F103C8T6
- 开发方式：STM32CubeMX + Keil / HAL 库
- 外设：USART、GPIO、TIM PWM、DHT11、MQ2、HC-SR501

## 5. Qt 编译方法

```bash
export QT_QMAKE=/home/book/100ask_imx6ull-sdk/ToolChain/arm-buildroot-linux-gnueabihf_sdk-buildroot/bin/qmake
cd /home/book/project_backup_20260714
$QT_QMAKE security_panel.pro
make clean
make -j4
adb push security_panel /root/project/bin/
```

## 6. 开发板运行方式

```bash
cd /root/project/bin
chmod +x security_panel
export LD_LIBRARY_PATH=/root/project/lib:/usr/lib:$LD_LIBRARY_PATH
export QT_QPA_PLATFORM=linuxfb:fb=/dev/fb0
export QT_PLUGIN_PATH=/usr/lib/qt/plugins
export QT_QPA_GENERIC_PLUGINS=evdevtouch,evdevmouse,evdevkeyboard
export QT_QPA_EVDEV_TOUCHSCREEN_PARAMETERS=/dev/input/event1
export QT_QPA_EVDEV_MOUSE_PARAMETERS=/dev/input/event3
./security_panel
```

也可以使用启动脚本 `run_security_panel.sh`，在程序启动前先恢复网络，再启动 Qt 应用。

## 7. 网络与 RTMP

开发板固定 IP 示例：

```text
i.MX6ULL eth0：192.168.5.9
Ubuntu 网关：192.168.5.11
RTMP 地址：rtmp://192.168.5.11/live/home001
```

Ubuntu 端确认 RTMP 服务：

```bash
sudo ss -lntp | grep 1935
```

## 8. 阿里云物联网平台

主要物模型属性：

```text
Temp, Humi, Mq2Adc, Mq2Ppm, GasAlarm,
PersonDetected, Tracked, Yaw, Pitch, AutoTrack
```

## 9. 休眠唤醒功能

系统支持两级休眠：

```text
应用级 IDLE：关闭摄像头、YOLO、RTMP、自动跟踪，保留串口线程等待 PIR 唤醒。
系统级 suspend：Qt 通知 STM32 准备休眠，Linux 执行 echo mem > /sys/power/state；STM32 检测到人体后通过 WAKE_OUT GPIO 唤醒 i.MX6ULL。
```

详细说明见 `docs/suspend_wakeup.md`。

## 10. 注意事项

- 如果摄像头打开失败，先检查 `/dev/video*` 是否存在。
- 如果 RTMP 失败，先检查开发板是否能 ping 通 Ubuntu 的 `192.168.5.11`。
- 如果 MQTT 失败，检查 Ubuntu NAT、DNS 和公网连接。
