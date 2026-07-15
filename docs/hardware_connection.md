# 硬件连接说明

本文档说明 i.MX6ULL、STM32F103、传感器、舵机、步进电机和人体红外模块之间的硬件连接关系。

## 1. 硬件组成

```text
1. i.MX6ULL Linux 开发板
2. STM32F103C8T6 控制板
3. USB 摄像头
4. LCD 显示屏
5. 双舵机云台
6. 步进电机
7. DHT11 温湿度传感器
8. MQ2 烟雾/可燃气体传感器
9. HC-SR501 人体红外感应模块
10. Ubuntu 主机，用于 RTMP 服务器和开发调试
```

## 2. 总体连接关系

```text
USB 摄像头 → i.MX6ULL USB
LCD → i.MX6ULL LCD framebuffer
STM32 USART → i.MX6ULL UART /dev/ttymxc5
STM32 PWM → 舵机云台
STM32 GPIO → 步进电机驱动
STM32 GPIO → DHT11
STM32 ADC → MQ2
HC-SR501 OUT → STM32 GPIO 输入
STM32 WAKE_OUT → i.MX6ULL GPIO4_IO19
```

## 3. i.MX6ULL 与 STM32 串口连接

```text
i.MX6ULL TX → STM32 RX
i.MX6ULL RX → STM32 TX
i.MX6ULL GND → STM32 GND
```

串口参数：

```text
波特率：115200
数据位：8
停止位：1
校验位：无
```

Qt 程序中串口设备：`/dev/ttymxc5`。

## 4. 舵机连接

```text
水平舵机 PWM → STM32 定时器 PWM 通道
俯仰舵机 PWM → STM32 定时器 PWM 通道
舵机 VCC → 外部 5V 电源
舵机 GND → STM32 GND / 电源 GND 共地
```

注意：舵机不要直接由 STM32 3.3V 供电；舵机电源和 STM32 必须共地；标准舵机 PWM 周期通常为 20ms。

## 5. 步进电机连接

```text
STM32 IN1 → 驱动模块 IN1
STM32 IN2 → 驱动模块 IN2
STM32 IN3 → 驱动模块 IN3
STM32 IN4 → 驱动模块 IN4
```

步进电机执行时尽量避免长时间阻塞主循环。如果运行时影响串口或 PIR 响应，应改成非阻塞任务。

## 6. DHT11 温湿度传感器

```text
DHT11 VCC → 3.3V 或 5V
DHT11 GND → GND
DHT11 DATA → STM32 GPIO
```

DHT11 读取周期不要太短，一般 1s 以上。

## 7. MQ2 气体传感器

```text
MQ2 AO → STM32 ADC 输入
MQ2 VCC → 5V
MQ2 GND → GND
```

Qt 上报字段示例：`Mq2Adc`、`Mq2Ppm`、`GasAlarm`。

## 8. HC-SR501 人体红外模块

```text
HC-SR501 VCC → 5V
HC-SR501 GND → GND
HC-SR501 OUT → STM32 GPIO 输入
```

工作逻辑：无人时 OUT 为低电平；检测到人体移动时 OUT 为高电平。

当前 STM32 程序主要采用 GPIO 轮询方式读取 OUT 引脚，并在确认有效后通过 UART 向 i.MX6ULL 发送：

```text
PIR,1
PIR,0
```

调试信息可能包括：

```text
PIR_PIN:1
PIR_PIN:0
```

## 9. 系统级唤醒连接

```text
STM32 WAKE_OUT → i.MX6ULL GPIO4_IO19
STM32 GND → i.MX6ULL GND
```

i.MX6ULL 侧设备树中将 GPIO4_IO19 配置为 `gpio-keys` 唤醒源。

唤醒逻辑：

```text
i.MX6ULL 进入系统级 suspend
STM32 继续运行
HC-SR501 检测到人体
STM32 拉高 WAKE_OUT
i.MX6ULL GPIO4_IO19 触发唤醒
Linux resume
Qt 恢复摄像头、检测、推流和网络
```

## 10. 注意事项

- 所有模块必须共地。
- 舵机和步进电机建议使用独立电源。
- 摄像头设备节点可能是 `/dev/video0` 或 `/dev/video1`。
- 如果系统唤醒失败，检查 WAKE_OUT 电平和设备树 `wakeup-source`。
- 如果 PIR 有电平变化但 Qt 不唤醒，检查 STM32 是否发送 `PIR,1`。
