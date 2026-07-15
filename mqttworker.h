/*************************************************
 * @File: mqttworker.h
 * @Author: Z-cloud778
 * @Date: 2026-07-14
*************************************************/
#ifndef MQTTWORKER_H
#define MQTTWORKER_H

#include <QObject>
#include <QString>
#include <QTcpSocket>
#include <QTimer>
#include <QByteArray>
#include <QAbstractSocket>

class MqttWorker : public QObject
{
    Q_OBJECT

public:
    explicit MqttWorker(QObject *parent = nullptr);
    ~MqttWorker();

public slots:
    void start();
    void stop();
    void publishStatusJson(const QString& json);
    void publishPropertyJson(const QString& json);
    void publishPropertySetReplyJson(const QString& json);

signals:
    void mqttStatus(const QString& text);
    void controlMessage(const QString& topic, const QString& payload);

private slots:
    void onConnected();
    void onReadyRead();
    void onDisconnected();
    void onSocketError(QAbstractSocket::SocketError error);
    void sendPing();

private:
    QByteArray encodeString(const QString& text);
    QByteArray encodeRemainingLength(int length);

    void sendConnect();
    void sendSubscribe(const QString& topic);
    void sendPublish(const QString& topic, const QString& payload);

    bool readOnePacket();
    void handlePacket(quint8 firstByte, const QByteArray& packetPayload);

private:
    QTcpSocket *socket;
    QTimer *pingTimer;
    QByteArray rxBuffer;

    QString brokerHost;
    quint16 brokerPort;

    QString clientId;
    QString username;
    QString password;

    QString statusTopic;
    QString controlTopic;

    bool mqttConnected;
    quint16 packetId;

    QString propertyPostTopic;
    QString propertyPostReplyTopic;
    QString propertySetTopic;
    QString propertySetReplyTopic;
};

#endif