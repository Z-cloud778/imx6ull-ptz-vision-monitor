# UART 串口通信协议

本文档说明 i.MX6ULL Qt 应用与 STM32F103 之间的 UART 文本协议。

## 1. 串口参数

```text
设备：/dev/ttymxc5
波特率：115200
数据位：8
停止位：1
校验位：无
```

## 2. 协议设计思想

本项目采用简单文本协议，典型格式为：

```text
命令,参数1,参数2
```

每条命令以换行结束，便于串口助手调试、STM32 字符串解析和 Qt 日志打印。

## 3. i.MX6ULL 发送给 STM32 的命令

### 3.1 获取传感器数据

```text
GET
```

返回示例：

```text
T:29.4 H:43.0 MQ2_ADC:1464 MQ2_V:1.18 MQ2_RS:2.27 MQ2_PPM:426.72 ALARM:0
```

### 3.2 控制舵机角度

```text
S,yaw,pitch
```

示例：

```text
S,90,90
```

STM32 回复示例：

```text
ACK:TARGET,90,90 CUR:90,90
```

### 3.3 控制步进电机

```text
M,angle,dir,delay
```

示例：

```text
M,90,1,2
```

### 3.4 系统级休眠准备

```text
HOST,SLEEP_PREPARE
```

STM32 收到后检查当前是否有人体触发。如果允许休眠，回复：

```text
HOST,SLEEP_READY
```

如果当前 PIR 正在触发或不适合休眠，回复：

```text
HOST,SLEEP_DENY
```

### 3.5 系统唤醒确认

```text
HOST,AWAKE
```

STM32 收到后清除 `host_sleep_armed` 状态，拉低 WAKE_OUT，并回复：

```text
HOST,AWAKE_ACK
```

## 4. STM32 发送给 i.MX6ULL 的消息

### 4.1 传感器数据

```text
T:29.4 H:43.0 MQ2_ADC:1464 MQ2_V:1.18 MQ2_RS:2.27 MQ2_PPM:426.72 ALARM:0
```

字段说明：

```text
T：温度
H：湿度
MQ2_ADC：MQ2 ADC 原始值
MQ2_V：MQ2 电压
MQ2_RS：MQ2 电阻比
MQ2_PPM：估算气体浓度
ALARM：报警状态，0 表示正常，1 表示报警
```

### 4.2 人体红外触发

```text
PIR,1
```

Qt 收到后进入 PIR 视觉验证模式。如果处于应用级 IDLE，则调用 `leaveIdleMode()`，重新启动摄像头、YOLO 和 RTMP。

### 4.3 人体红外恢复

```text
PIR,0
```

`PIR,0` 不应立即表示画面中无人，是否有人应由 YOLO/MOSSE 视觉结果判断。

### 4.4 PIR 调试电平

```text
PIR_PIN:1
PIR_PIN:0
```

这是 STM32 直接打印 HC-SR501 OUT 引脚电平变化，用于调试 PIR 模块是否有电平变化。

### 4.5 休眠握手消息

```text
HOST,SLEEP_READY
HOST,SLEEP_DENY
HOST,AWAKE_ACK
```

这些消息用于 Qt 与 STM32 进行系统级休眠和唤醒握手。

## 5. Qt 中的串口处理逻辑

Qt 中由 `SensorWorker` 负责串口通信，主要职责：

```text
1. 打开 /dev/ttymxc5。
2. 定时发送 GET。
3. 解析 STM32 返回的温湿度、MQ2、PIR、休眠握手消息。
4. 通过 signal 通知 MainWindow。
5. 接收 MainWindow 的 invokeMethod 调用，发送舵机、步进、休眠相关命令。
```

典型信号：

```text
sensorUpdated(...)
pirStateChanged(bool)
hostSleepReady()
hostSleepDenied()
hostAwakeAck()
serialStatus(QString)
```

## 6. STM32 中的串口处理逻辑

STM32 通常使用 USART 接收中断接收数据：

```text
USART 中断逐字节接收
遇到换行符认为一条命令接收完成
设置 uart_cmd_ready = 1
主循环中调用 Process_Uart_Command()
```

## 7. 注意事项

- 串口两端波特率必须一致。
- i.MX6ULL 与 STM32 必须共地。
- GET 频率不要太高，否则会影响舵机命令和休眠握手。
- 准备系统级休眠时，建议暂停普通 GET 查询。
- STM32 不应在中断回调里执行耗时任务。
