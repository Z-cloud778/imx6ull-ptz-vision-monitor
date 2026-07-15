/*************************************************
 * @File: mqttworker.cpp
 * @Brief: 自定义MQTT客户端，基于Socket实现MQTT3.1.1基础功能
 * @Function: 阿里云IoT连接、数据上报、指令订阅、心跳保活、断线重连
 * @Note: 已脱敏，使用者自行替换个人IoT设备密钥信息
 * @Author: Z-cloud778
 * @Date: 2026-07-14
*************************************************/
#include "mqttworker.h"

#include <QDebug>

// 简易MQTT客户端，适配阿里云IoT平台
MqttWorker::MqttWorker(QObject *parent)
    : QObject(parent),
      socket(nullptr),
      pingTimer(nullptr),

      // 开源模板地址，需自行替换为个人IoT实例域名
      brokerHost("your-iot-instance.mqtt.iothub.aliyuncs.com"),
          brokerPort(1883),

      // 脱敏密钥模板，自行生成替换
          clientId("your_device_client2,signmethod=hmacsha256|"),
          username(evice_name&your_product_key"),
          password("your_dqtt_password"),

      mqttConnected(false),
      packetId(1)
{
    // 阿里云IoT通用标准主题
    propertyPostTopic = "/sys/{productKey}/{deviceName}/thing/event/property/post";
 propertyPostReplyTopic = "/sys/{productKey}/{deviceName}/thing/event/property/post_reply";
       tySetTopic = "/sys/{productKey}/{deviceName}/thing/service/property/set";
    pertySetReplyTopic = "/sys/{productKey}/{deviceName}/thing/service/property/set_reply";
}

MqttWorker::~MqttWorker()
{
}

// 启动MQTT连接，初始化套接字与心跳定时器
void MqttWorker::start()
{
    if (socket)
        return;

    socket = new QTcpSocket(this);

    connect(socket, &QTcpSocket::connected,
            this, &MqttWorker::onConnected);
    connect(socket, &QTcpSocket::readyRead,
            this, &MqttWorker::onReadyRead);
    connect(socket, &QTcpSocket::disconnected,
            this, &MqttWorker::onDisconnected);
    connect(socket, SIGNAL(error(QAbstractSocket::SocketError)),
            this, SLOT(onSocketError(QAbstractSocket::SocketError)));

    pingTimer = new QTimer(this);
    connect(pingTimer, &QTimer::timeout, this, &MqttWorker::sendPing);

    emit mqttStatus(QString("MQTT connecting to %1:%2").arg(brokerHost).arg(brokerPort));
    socket->connectToHost(brokerHost, brokerPort);
}

// 断开MQTT连接，停止心跳
void MqttWorker::stop()
{
    mqttConnected = false;

    if (pingTimer)
        pingTimer->stop();
    if (socket)
        socket->disconnectFromHost();
}

// TCP连接成功，发起MQTT注册
void MqttWorker::onConnected()
{
    emit mqttStatus("MQTT TCP connected");
    sendConnect();
}

// 连接断开，3秒自动重连
void MqttWorker::onDisconnected()
{
    mqttConnected = false;
    emit mqttStatus("MQTT disconnected");

    if (pingTimer)
        pingTimer->stop();

    QTimer::singleShot(3000, this, [this]() {
        if (!socket) return;
        if (socket->state() == QAbstractSocket::UnconnectedState)
        {
            emit mqttStatus("MQTT reconnecting...");
            socket->connectToHost(brokerHost, brokerPort);
        }
    });
}

// 套接字异常回调
void MqttWorker::onSocketError(QAbstractSocket::SocketError error)
{
    Q_UNUSED(error);
    if (socket)
        emit mqttStatus("MQTT socket error: " + socket->errorString());
}

// MQTT字符串格式编码
QByteArray MqttWorker::encodeString(const QString& text)
{
    QByteArray utf8 = text.toUtf8();
    QByteArray out;

    out.append(char((utf8.size() >> 8) & 0xff));
    out.append(char(utf8.size() & 0xff));
    out.append(utf8);

    return out;
}

// MQTT变长长度编码
QByteArray MqttWorker::encodeRemainingLength(int length)
{
    QByteArray out;
    do
    {
        char encodedByte = length % 128;
        length = length / 128;
        if (length > 0) encodedByte |= 128;
        out.append(encodedByte);
    } while (length > 0);
    return out;
}

// 发送MQTT连接鉴权报文
void MqttWorker::sendConnect()
{
    if (!socket) return;

    QByteArray variableHeader, payload;

    // 配置MQTT3.1.1协议参数
    variableHeader.append(encodeString("MQTT"));
    variableHeader.append(char(4));

    quint8 flags = 0x02;
    if (!username.isEmpty()) flags |= 0x80;
    if (!password.isEmpty()) flags |= 0x40;
    variableHeader.append(char(flags));

    // 30秒心跳间隔
    variableHeader.append(char(0));
    variableHeader.append(char(30));

    // 拼接鉴权信息
    payload.append(encodeString(clientId));
    if (!username.isEmpty()) payload.append(encodeString(username));
    if (!password.isEmpty()) payload.append(encodeString(password));

    QByteArray packet;
    packet.append(char(0x10));
    packet.append(encodeRemainingLength(variableHeader.size() + payload.size()));
    packet.append(variableHeader);
    packet.append(payload);

    socket->write(packet);
    socket->flush();
    emit mqttStatus("MQTT CONNECT sent");
}

// 订阅指定MQTT主题
void MqttWorker::sendSubscribe(const QString& topic)
{
    if (!socket || !mqttConnected) return;

    QByteArray payload;
    quint16 id = packetId++;
    payload.append(char((id >> 8) & 0xff));
    payload.append(char(id & 0xff));
    payload.append(encodeString(topic));
    payload.append(char(0)); // QoS0

    QByteArray packet;
    packet.append(char(0x82));
    packet.append(encodeRemainingLength(payload.size()));
    packet.append(payload);

    socket->write(packet);
    socket->flush();
    emit mqttStatus("MQTT SUBSCRIBE: " + topic);
}

// 发布MQTT消息（QoS0）
void MqttWorker::sendPublish(const QString& topic, const QString& payloadText)
{
    if (!socket || !mqttConnected) return;

    QByteArray payload;
    payload.append(encodeString(topic));
    payload.append(payloadText.toUtf8());

    QByteArray packet;
    packet.append(char(0x30));
    packet.append(encodeRemainingLength(payload.size()));
    packet.append(payload);

    socket->write(packet);
    socket->flush();
}

// 上报设备状态数据
void MqttWorker::publishStatusJson(const QString& json)
{
    sendPublish(statusTopic, json);
}

// 发送心跳包维持长连接
void MqttWorker::sendPing()
{
    if (!socket || !mqttConnected) return;
    QByteArray packet;
    packet.append(char(0xC0));
    packet.append(char(0x00));
    socket->write(packet);
    socket->flush();
}

// 接收套接字数据
void MqttWorker::onReadyRead()
{
    if (!socket) return;
    rxBuffer.append(socket->readAll());
    while (readOnePacket());
}

// 解析单条MQTT报文
bool MqttWorker::readOnePacket()
{
    if (rxBuffer.size() < 2) return false;

    quint8 firstByte = quint8(rxBuffer[0]);
    int multiplier = 1, value = 0, pos = 1;
    quint8 encodedByte = 0;

    do
    {
        if (pos >= rxBuffer.size()) return false;
        encodedByte = quint8(rxBuffer[pos++]);
        value += (encodedByte & 127) * multiplier;
        multiplier *= 128;

        if (multiplier > 128 * 128 * 128)
        {
            rxBuffer.clear();
            emit mqttStatus("MQTT bad remaining length");
            return false;
        }
    } while ((encodedByte & 128) != 0);

    int packetSize = pos + value;
    if (rxBuffer.size() < packetSize) return false;

    QByteArray payload = rxBuffer.mid(pos, value);
    rxBuffer.remove(0, packetSize);
    handlePacket(firstByte, payload);

    return true;
}

// 分发处理各类MQTT报文
void MqttWorker::handlePacket(quint8 firstByte, const QByteArray& packetPayload)
{
    quint8 packetType = firstByte >> 4;

    // 连接应答
    if (packetType == 2)
    {
        if (packetPayload.size() >= 2 && quint8(packetPayload[1]) == 0)
        {
            mqttConnected = true;
            emit mqttStatus("MQTT connected OK");
            sendSubscribe(propertySetTopic);
            if (pingTimer) pingTimer->start(10000);
        }
        else
        {
            emit mqttStatus("MQTT connect refused");
        }
    }
    // 云端消息下发
    else if (packetType == 3)
    {
        if (packetPayload.size() < 2) return;
        int topicLen = (quint8(packetPayload[0]) << 8) | quint8(packetPayload[1]);
        if (packetPayload.size() < 2 + topicLen) return;

        QString topic = QString::fromUtf8(packetPayload.mid(2, topicLen));
        int payloadPos = 2 + topicLen;
        int qos = (firstByte & 0x06) >> 1;
        if (qos > 0) payloadPos += 2;
        if (payloadPos > packetPayload.size()) return;

        QString payload = QString::fromUtf8(packetPayload.mid(payloadPos));
        emit controlMessage(topic, payload);
    }
    // 订阅应答
    else if (packetType == 9)
    {
        emit mqttStatus("MQTT SUBACK received");
    }
}

// 上报设备属性数据至云端
void MqttWorker::publishPropertyJson(const QString& json)
{
    if (!mqttConnected) return;
    sendPublish(propertyPostTopic, json);
}

// 应答云端设备控制指令
void MqttWorker::publishPropertySetReplyJson(const QString& json)
{
    if (!mqttConnected) return;
    sendPublish(propertySetReplyTopic, json);
}
    pro proper       evice_m"your_did|securemode=