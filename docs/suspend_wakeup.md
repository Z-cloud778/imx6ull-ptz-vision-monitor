# 休眠与唤醒功能说明

本文档说明本项目中应用级休眠、Linux 系统级 suspend、STM32 人体红外唤醒和网络恢复的整体逻辑。

## 1. 功能目标

长时间无人时，关闭摄像头、YOLO、RTMP 等高负载模块，降低 CPU 占用和功耗；当 HC-SR501 检测到人体红外变化后，STM32 通知或唤醒 i.MX6ULL，Qt 恢复视频采集、人体检测、推流和云端通信。

## 2. 两级休眠

```text
1. 应用级 IDLE
2. Linux 系统级 suspend
```

## 3. 应用级 IDLE

应用级 IDLE 不是 Linux 真正休眠，而是 Qt 应用关闭高负载模块。

进入 IDLE 后关闭：

```text
VisionWorker 摄像头采集
DetectorWorker YOLO 检测
RtmpStreamer RTMP 推流
AutoTrack 自动跟踪
MOSSE 跟踪状态
```

保留：

```text
SensorWorker 串口线程
PowerManager 定时器
PIR 串口解析
MQTT 状态上报可选
```

## 4. Qt 电源状态机

```cpp
enum PowerState
{
    POWER_ACTIVE = 0,
    POWER_IDLE,
    POWER_SUSPEND_PENDING
};
```

含义：

```text
POWER_ACTIVE：正常运行，摄像头、检测、跟踪、推流可以工作。
POWER_IDLE：应用级休眠，等待 PIR 唤醒。
POWER_SUSPEND_PENDING：准备进入系统级休眠，等待 STM32 回复 HOST,SLEEP_READY。
```

## 5. 关键变量

```text
lastHumanActivityMs：最近一次确认有人活动的时间。
pirWakeVerifyMode：PIR 已触发，正在等待视觉检测确认画面中是否真的有人。
pirWakeStartMs：PIR 唤醒开始时间。
suspendRequested：已经请求系统级休眠，避免重复发送休眠准备命令。
currentPersonDetected：YOLO 或视觉逻辑是否确认有人。
haveTrackTarget：MOSSE 是否正在跟踪目标。
pirMotion：PIR 当前是否触发。
```

## 6. 应用级 IDLE 进入逻辑

Qt 中 `powerTick()` 每秒检查一次：

```text
inactiveMs = now - lastHumanActivityMs
```

如果满足：

```text
powerState == POWER_ACTIVE
inactiveMs >= IDLE_TIMEOUT_MS
当前不处于 pirWakeVerifyMode
```

则调用 `enterIdleMode()`。典型时间：

```text
IDLE_TIMEOUT_MS = 30000
```

## 7. 应用级 IDLE 唤醒逻辑

STM32 检测到人体红外后，通过 UART 发送：

```text
PIR,1
```

Qt 的 `SensorWorker` 解析后发出 `pirStateChanged(true)`，MainWindow 执行 `onPirStateChanged(true)`。

逻辑：

```text
1. 设置 pirWakeVerifyMode = true
2. 设置 pirWakeStartMs = 当前时间
3. 刷新 lastHumanActivityMs，避免刚唤醒又进入 IDLE
4. 如果 powerState != POWER_ACTIVE，则调用 leaveIdleMode()
```

`leaveIdleMode()` 恢复 VisionWorker、DetectorWorker、RtmpStreamer 和状态显示。

## 8. PIR 与视觉验证

HC-SR501 只能说明有红外变化，不能证明摄像头画面中一定有人。因此 PIR 只负责唤醒，视觉负责确认。

正确逻辑：

```text
PIR 触发 → 唤醒应用 → 启动摄像头 → 启动 YOLO → YOLO 检测到 person → 初始化 MOSSE 跟踪
```

如果在 `PIR_VERIFY_TIMEOUT_MS` 内没有视觉确认，则认为 PIR 可能是误触发，重新进入应用级 IDLE。

## 9. 系统级 suspend 进入逻辑

当系统已经处于 `POWER_IDLE`，并且继续无人超过 `SUSPEND_TIMEOUT_MS`，调用 `requestSystemSuspend()`。

流程：

```text
Qt 设置 powerState = POWER_SUSPEND_PENDING
Qt 暂停普通 GET 查询
Qt 发送 HOST,SLEEP_PREPARE 给 STM32
STM32 检查当前 PIR 状态
STM32 回复 HOST,SLEEP_READY 或 HOST,SLEEP_DENY
```

## 10. STM32 休眠准备逻辑

STM32 收到 `HOST,SLEEP_PREPARE` 后检查：当前 PIR_OUT 是否为高电平、最近几秒是否刚触发过 PIR、WAKE_OUT 是否已经拉低。

如果不允许休眠：

```text
HOST,SLEEP_DENY
```

如果允许休眠：

```text
HOST,SLEEP_READY
```

并设置：

```text
host_sleep_armed = 1
WAKE_OUT = LOW
```

## 11. i.MX6ULL 进入 Linux suspend

Qt 收到 `HOST,SLEEP_READY` 后执行 `system_suspend.sh`，核心命令：

```bash
echo mem > /sys/power/state
```

这会让 Linux 进入 suspend-to-RAM。

## 12. 系统级唤醒逻辑

系统级 suspend 后，Qt 程序暂停运行，串口线程也不再处理应用层消息。此时依靠 STM32：

```text
HC-SR501 检测到人体
STM32 PIR_Task 确认高电平
如果 host_sleep_armed == 1
STM32 拉高 WAKE_OUT
i.MX6ULL GPIO4_IO19 产生唤醒事件
Linux 从 suspend 恢复
```

## 13. 设备树唤醒源

关键字段：

```dts
compatible = "gpio-keys";
gpios = <&gpio4 19 GPIO_ACTIVE_HIGH>;
linux,code = <KEY_WAKEUP>;
wakeup-source;
debounce-interval = <20>;
```

含义：`gpio-keys` 使用 Linux 内核 GPIO 按键驱动；`GPIO_ACTIVE_HIGH` 表示 STM32 拉高 WAKE_OUT 有效；`wakeup-source` 声明该 GPIO 可唤醒系统。

## 14. 唤醒后的恢复流程

Linux 被唤醒后，`echo mem` 返回，`system_suspend.sh` 继续执行。建议在脚本中恢复网络：

```bash
/root/project/bin/resume_network.sh
```

Qt 的 `afterSystemResume()` 中执行：

```text
1. powerState = POWER_ACTIVE
2. suspendRequested = false
3. 发送 HOST,AWAKE 给 STM32
4. STM32 回复 HOST,AWAKE_ACK 并拉低 WAKE_OUT
5. 延时启动 VisionWorker
6. 延时启动 DetectorWorker
7. 延时启动 RtmpStreamer
8. MQTT 继续上报状态
```

## 15. 网络恢复

系统级休眠唤醒后，开发板网络可能异常，因此需要恢复 eth0、IP 地址、默认路由和 DNS。

示例：

```bash
ifconfig eth0 down
sleep 1
ifconfig eth0 up
sleep 2
ifconfig eth0 192.168.5.9 netmask 255.255.255.0
route del default 2>/dev/null
route add default gw 192.168.5.11 eth0
echo "nameserver 223.5.5.5" > /etc/resolv.conf
```

Ubuntu 端需要配置 NAT：

```bash
sudo sysctl -w net.ipv4.ip_forward=1
sudo iptables -t nat -A POSTROUTING -s 192.168.5.0/24 -o ens33 -j MASQUERADE
sudo iptables -A FORWARD -i ens36 -o ens33 -s 192.168.5.0/24 -j ACCEPT
sudo iptables -A FORWARD -i ens33 -o ens36 -d 192.168.5.0/24 -m state --state RELATED,ESTABLISHED -j ACCEPT
```

## 16. 常见问题

### PIR 有电平变化，但无法唤醒应用

检查 STM32 是否发送 `PIR,1`。如果只看到 `PIR_PIN:1`，说明只是调试电平打印，Qt 可能没有把它当作唤醒信号。

### PIR true 了，但屏幕一直 WAIT

检查摄像头是否打开成功：

```bash
ls -l /dev/video*
```

如果 Qt 日志有 `open camera failed: No such file or directory`，说明摄像头设备节点不存在或编号变化。

### 唤醒后 RTMP 不推流

检查：

```bash
ping -c 3 192.168.5.11
sudo ss -lntp | grep 1935
```

如果网络刚恢复，RTMP 应延时 6~8 秒再启动。

### 系统刚唤醒又进入 IDLE

PIR 触发后视觉还没检测到人，inactiveMs 又超时。解决方法是 pirWakeVerifyMode 期间不要进入 IDLE，并在 `onPirStateChanged(true)` 中刷新 `lastHumanActivityMs`。
