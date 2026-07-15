/*************************************************
 * @File: sensorworker.cpp
 * @Author: Z-cloud778
 * @Date: 2026-07-14
*************************************************/
#define _DEFAULT_SOURCE

#include "sensorworker.h"

#include <QTimer>
#include <QDebug>

#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cstdio>
#include <cstring>
#include <errno.h>

static const int MQ2_ADC_ALARM_TH = 1850;
static const int MQ2_ADC_RECOVER_TH = 1750;

SensorWorker::SensorWorker(const QString& dev, int baud, QObject *parent)
    : QObject(parent),
      devName(dev),
      baudRate(baud),
      fd(-1),
      pollTimer(nullptr),
      gasAlarmState(false)
{
    sleepPrepareMode = false;
}

SensorWorker::~SensorWorker()
{
    closeSerial();
}

bool SensorWorker::openSerial()
{
    fd = ::open(devName.toLocal8Bit().constData(), O_RDWR | O_NOCTTY | O_NONBLOCK);

    if (fd < 0)
    {
        emit serialStatus(QString("open %1 failed: %2").arg(devName).arg(strerror(errno)));
        return false;
    }

    struct termios tty;
    memset(&tty, 0, sizeof(tty));

    if (tcgetattr(fd, &tty) != 0)
    {
        emit serialStatus(QString("tcgetattr failed: %1").arg(strerror(errno)));
        closeSerial();
        return false;
    }

    cfmakeraw(&tty);

    speed_t speed = B115200;

    if (baudRate != 115200)
    {
        emit serialStatus("Only 115200 is configured now, use B115200");
    }

    cfsetispeed(&tty, speed);
    cfsetospeed(&tty, speed);

    tty.c_cflag |= CLOCAL | CREAD;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;

    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;

    tcflush(fd, TCIOFLUSH);

    if (tcsetattr(fd, TCSANOW, &tty) != 0)
    {
        emit serialStatus(QString("tcsetattr failed: %1").arg(strerror(errno)));
        closeSerial();
        return false;
    }

    emit serialStatus(QString("serial open OK: %1").arg(devName));
    return true;
}

void SensorWorker::closeSerial()
{
    if (fd >= 0)
    {
        ::close(fd);
        fd = -1;
    }
}

void SensorWorker::start()
{
    if (!openSerial())
        return;

    getTimer.start();

    pollTimer = new QTimer(this);
    connect(pollTimer, &QTimer::timeout, this, &SensorWorker::pollOnce);
    pollTimer->start(100);

    emit serialStatus("sensor worker started");
}

void SensorWorker::stop()
{
    if (pollTimer)
    {
        pollTimer->stop();
        pollTimer->deleteLater();
        pollTimer = nullptr;
    }

    closeSerial();
    emit serialStatus("sensor worker stopped");
}

void SensorWorker::sendRawCommand(const QString& cmd)
{
    if (fd < 0)
    {
        emit serialStatus("TX failed: serial not open");
        qDebug() << "[UART_TX] failed, serial not open";
        return;
    }

    QByteArray data = cmd.toUtf8();

    if (!data.endsWith("\n"))
        data.append("\r\n");

    int n = ::write(fd, data.constData(), data.size());

    if (n < 0)
    {
        emit serialStatus(QString("write failed: %1").arg(strerror(errno)));
        qDebug() << "[UART_TX] write failed:" << strerror(errno);
    }
    else
    {
        tcdrain(fd);

        QString txText = QString::fromUtf8(data).trimmed();

        emit serialStatus(QString("TX: %1").arg(txText));
        qDebug() << "[UART_TX]" << txText << "bytes =" << n;
    }
}

void SensorWorker::sendServoAngle(int yaw, int pitch)
{
    if (yaw < 0)
        yaw = 0;
    if (yaw > 180)
        yaw = 180;

    if (pitch < 0)
        pitch = 0;
    if (pitch > 180)
        pitch = 180;

    QString cmd = QString("S,%1,%2").arg(yaw).arg(pitch);

    sendRawCommand(cmd);
}

void SensorWorker::sendServoCenter()
{
    qDebug() << "[SLOT] sendServoCenter";
    sendServoAngle(90, 90);
}


void SensorWorker::sendStepperOpen()
{
    qDebug() << "[SLOT] sendStepperOpen";
    sendRawCommand("M,90,1,2");
}

void SensorWorker::pollOnce()
{
    if (fd < 0)
        return;

    if (getTimer.elapsed() >= 1000)
    {
        if (sleepPrepareMode)
	   {
   		 qDebug() << "[SERIAL] skip GET during sleep prepare";
    		 return;
	   }

	   sendRawCommand("GET");
        getTimer.restart();
    }

    char buf[256];

    while (1)
    {
        int n = ::read(fd, buf, sizeof(buf));

        if (n > 0)
        {
            rxBuffer.append(buf, n);
        }
        else
        {
            break;
        }
    }

    while (1)
    {
        int idx = rxBuffer.indexOf('\n');

        if (idx < 0)
            break;

        QByteArray line = rxBuffer.left(idx);
        rxBuffer.remove(0, idx + 1);

        line = line.trimmed();

        if (line.isEmpty())
            continue;

        emit serialStatus(QString("RX: %1").arg(QString::fromLocal8Bit(line.constData(), line.size())));

        double t = 0.0;
        double h = 0.0;
        int adc = 0;
        double v = 0.0;
        double rs = 0.0;
        double ppm = 0.0;

        if (parseSensorLine(line, t, h, adc, v, rs, ppm))
        {
            updateAlarmByAdc(adc);

            emit sensorUpdated(t,
                               h,
                               adc,
                               v,
                               rs,
                               ppm,
                               gasAlarmState);
        }
    }
}

bool SensorWorker::parseSensorLine(const QByteArray& line,
                                   double& temperature,
                                   double& humidity,
                                   int& mq2Adc,
                                   double& mq2Voltage,
                                   double& mq2Rs,
                                   double& mq2Ppm)
{
    QString lineText = QString::fromLocal8Bit(line.constData(), line.size()).trimmed();

    if (lineText == "HOST,SLEEP_READY")
    {
        qDebug() << "[SERIAL] HOST,SLEEP_READY";
        emit hostSleepReady();
        return false;
    } 

    if (lineText == "HOST,SLEEP_DENY")
    {
        sleepPrepareMode = false;
        emit hostSleepDenied();
        return false;
    }

    if (lineText == "HOST,AWAKE_ACK")
    {
        sleepPrepareMode = false;
        emit hostAwakeAck();
        return false;
    }   

    if (lineText == "PIR,1")
    {
        qDebug() << "[SERIAL] PIR,1";
        emit pirStateChanged(true);
        return false;
    }

    if (lineText == "PIR,0")
    {
        qDebug() << "[SERIAL] PIR,0";
        emit pirStateChanged(false);
        return false;
    }

    QByteArray data = line.trimmed();
    const char *p = data.constData();

    double t = 0.0;
    double h = 0.0;
    int adc = 0;
    double v = 0.0;
    double rs = 0.0;
    double ppm = 0.0;
    int alarm = -1;

    int ret = std::sscanf(p,
                          "T:%lf H:%lf MQ2_ADC:%d MQ2_V:%lf MQ2_RS:%lf MQ2_PPM:%lf ALARM:%d",
                          &t,
                          &h,
                          &adc,
                          &v,
                          &rs,
                          &ppm,
                          &alarm);

    if (ret >= 6)
    {
        temperature = t;
        humidity = h;
        mq2Adc = adc;
        mq2Voltage = v;
        mq2Rs = rs;
        mq2Ppm = ppm;
        return true;
    }

    ret = std::sscanf(p,
                      "T:%lf H:%lf MQ2_ADC:%d MQ2_V:%lf MQ2_RS:%lf MQ2_PPM:%lf",
                      &t,
                      &h,
                      &adc,
                      &v,
                      &rs,
                      &ppm);

    if (ret == 6)
    {
        temperature = t;
        humidity = h;
        mq2Adc = adc;
        mq2Voltage = v;
        mq2Rs = rs;
        mq2Ppm = ppm;
        return true;
    }

    ret = std::sscanf(p,
                      "T:%lf H:%lf MQ2_ADC:%d",
                      &t,
                      &h,
                      &adc);

    if (ret == 3)
    {
        temperature = t;
        humidity = h;
        mq2Adc = adc;
        mq2Voltage = 0.0;
        mq2Rs = 0.0;
        mq2Ppm = 0.0;
        return true;
    }

    return false;
}

void SensorWorker::updateAlarmByAdc(int adc)
{
    if (!gasAlarmState)
    {
        if (adc >= MQ2_ADC_ALARM_TH)
        {
            gasAlarmState = true;
            emit serialStatus(QString("ALARM ON, adc=%1").arg(adc));
        }
    }
    else
    {
        if (adc <= MQ2_ADC_RECOVER_TH)
        {
            gasAlarmState = false;
            emit serialStatus(QString("ALARM OFF, adc=%1").arg(adc));
        }
    }
}

void SensorWorker::sendHostSleepPrepare()
{
    sleepPrepareMode = true;
    sendRawCommand("HOST,SLEEP_PREPARE");
}

void SensorWorker::sendHostAwake()
{
    sendRawCommand("HOST,AWAKE");
}

void SensorWorker::setSleepPrepareMode(bool enable)
{
    sleepPrepareMode = enable;
    qDebug() << "[SERIAL] sleepPrepareMode =" << sleepPrepareMode;
}