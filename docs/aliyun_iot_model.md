# 阿里云物联网平台物模型说明

本文档说明本项目接入阿里云物联网平台时使用的物模型属性和 MQTT 数据格式。

## 1. 接入方式

本项目使用阿里云物联网平台 MQTT 接入方式。设备端是 i.MX6ULL Qt 应用，云端是阿里云物联网平台 IoT Hub。

主要功能：

```text
1. 上报温湿度、MQ2、人体检测、舵机角度等属性。
2. 接收云端下发的云台控制命令。
3. 接收云端下发的自动跟踪开关。
```

## 2. 物模型属性设计

| 属性标识符 | 类型 | 读写 | 说明 |
|---|---|---|---|
| Temp | float | 只读 | 温度 |
| Humi | float | 只读 | 湿度 |
| Mq2Adc | int32 | 只读 | MQ2 ADC 原始值 |
| Mq2Ppm | float | 只读 | MQ2 估算浓度 |
| GasAlarm | bool/int | 只读 | 气体报警状态 |
| PersonDetected | bool/int | 只读 | 是否检测到人体 |
| Tracked | bool/int | 只读 | 是否正在跟踪目标 |
| Yaw | int32 | 读写 | 水平舵机角度 |
| Pitch | int32 | 读写 | 俯仰舵机角度 |
| AutoTrack | bool/int | 读写 | 自动跟踪开关 |

## 3. 属性上报 Topic

```text
/sys/${ProductKey}/${DeviceName}/thing/event/property/post
```

## 4. 属性上报 JSON 示例

```json
{
  "id": "1",
  "version": "1.0",
  "params": {
    "Temp": 29.4,
    "Humi": 43.0,
    "Mq2Adc": 1464,
    "Mq2Ppm": 426.72,
    "GasAlarm": 0,
    "PersonDetected": 1,
    "Tracked": 1,
    "Yaw": 90,
    "Pitch": 90,
    "AutoTrack": 1
  },
  "method": "thing.event.property.post"
}
```

## 5. 属性设置 Topic

```text
/sys/${ProductKey}/${DeviceName}/thing/service/property/set
```

Qt 程序订阅该 Topic 后，可以接收云端控制命令。

## 6. 云端控制 JSON 示例

### 6.1 开启自动跟踪

```json
{
  "id": "1001",
  "version": "1.0",
  "params": {
    "AutoTrack": 1
  },
  "method": "thing.service.property.set"
}
```

### 6.2 关闭自动跟踪

```json
{
  "id": "1002",
  "version": "1.0",
  "params": {
    "AutoTrack": 0
  },
  "method": "thing.service.property.set"
}
```

### 6.3 设置舵机角度

```json
{
  "id": "1003",
  "version": "1.0",
  "params": {
    "Yaw": 100,
    "Pitch": 85
  },
  "method": "thing.service.property.set"
}
```

Qt 收到后关闭自动跟踪，限制 Yaw/Pitch 范围，并通过 SensorWorker 发送 `S,yaw,pitch` 给 STM32。

## 7. 属性设置回复

```text
/sys/${ProductKey}/${DeviceName}/thing/service/property/set_reply
```

JSON 示例：

```json
{
  "id": "1003",
  "version": "1.0",
  "code": 200,
  "message": "success",
  "data": {}
}
```

## 8. Qt 中的 MQTT 线程

项目中使用 `MqttWorker` 处理 MQTT，主要职责：连接阿里云 MQTT Broker、订阅 property set Topic、周期性上报属性、接收云端控制消息并发送给 MainWindow。

## 9. 常见问题

### MQTT 报 Host not found

可能是开发板 DNS 不正常、Ubuntu NAT 没配置、开发板不能访问公网。

### 云端控制没有反应

检查 Topic 是否订阅成功、属性标识符是否和物模型一致、`onMqttControl()` 是否解析对应字段、舵机命令是否通过 UART 发送给 STM32。
