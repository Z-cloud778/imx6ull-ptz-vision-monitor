/*************************************************
 * @File: sensorworker.h
 * @Author: Z-cloud778
 * @Date: 2026-07-14
*************************************************/
#ifndef SENSORWORKER_H
#define SENSORWORKER_H

#include <QObject>
#include <QString>
#include <QByteArray>
#include <QElapsedTimer>

class QTimer;

class SensorWorker : public QObject
{
    Q_OBJECT

public:
    explicit SensorWorker(const QString& dev = "/dev/ttymxc5",
                          int baud = 115200,
                          QObject *parent = nullptr);
    ~SensorWorker();

public slots:
    void start();
    void stop();

    void sendServoCenter();
    void sendStepperOpen();
    void sendRawCommand(const QString& cmd);
    void sendServoAngle(int yaw, int pitch);
    void sendHostSleepPrepare();
    void sendHostAwake();
    void setSleepPrepareMode(bool enable);

signals:
    void sensorUpdated(double temperature,
                       double humidity,
                       int mq2Adc,
                       double mq2Voltage,
                       double mq2Rs,
                       double mq2Ppm,
                       bool gasAlarm);

    void serialStatus(const QString& text);

    void pirStateChanged(bool motion);
    void hostSleepReady();
    void hostSleepDenied();
    void hostAwakeAck();


private:
    bool openSerial();
    void closeSerial();
    void pollOnce();

    bool parseSensorLine(const QByteArray& line,
                         double& temperature,
                         double& humidity,
                         int& mq2Adc,
                         double& mq2Voltage,
                         double& mq2Rs,
                         double& mq2Ppm);

    void updateAlarmByAdc(int adc);

private:
    QString devName;
    int baudRate;
    int fd;

    QTimer *pollTimer;
    QByteArray rxBuffer;
    QElapsedTimer getTimer;

    bool gasAlarmState;
    bool sleepPrepareMode;
};

#endif