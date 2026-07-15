/*************************************************
 * @File: mainwindow.h
 * @Author: Z-cloud778
 * @Date: 2026-07-14
*************************************************/
#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QLabel>
#include <QPushButton>
#include <QThread>
#include <QImage>

#include "visionworker.h"
#include "sensorworker.h"
#include "detectorworker.h"
#include <QTimer>
#include <QElapsedTimer>
#include "mqttworker.h"
#include "rtmpstreamer.h"
#include <opencv2/core.hpp>

//硬件配置常量
#define SERIAL_DEV      "/dev/ttymxc5"
#define SERIAL_BAUD     115200
#define CAMERA_DEV      "/dev/video1"
#define CAM_WIDTH       320
#define CAM_HEIGHT      240
#define VIDEO_LABEL_W   640
#define VIDEO_LABEL_H   480

enum PowerState
{
    POWER_ACTIVE,//正常工作
    POWER_IDLE,//应用级休眠
    POWER_SUSPEND_PENDING//准备系统级休眠
};

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

signals:
    void streamFrameReady(cv::Mat frame);

private slots:
    void onSensorUpdated(double temperature,
                         double humidity,
                         int mq2Adc,
                         double mq2Voltage,
                         double mq2Rs,
                         double mq2Ppm,
                         bool gasAlarm);

    void onSerialStatus(const QString& text);

    void onVideoFrame(const QImage& image);
    void onVisionStatus(const QString& text);
    void onPersonDetected(bool detected);

    void onDetectorFrameCandidate(const QImage& image);
    void onDetectionResult(bool detected,
                       int x,
                       int y,
                       int w,
                       int h,
                       float score);
    void onDetectorStatus(const QString& text);
    void onDetectorIdle();
    void onTrackingResult(bool tracked,
                      int x,
                      int y,
                      int w,
                      int h,
                      float score);

    void servoControlTick();
    void onMqttStatus(const QString& text);
    void onMqttControl(const QString& topic,
                      const QString& payload);
    void publishMqttStatus();

    void powerTick();
    void onPirStateChanged(bool motion);
    void onHostSleepReady();
    void onHostSleepDenied();
    void onHostAwakeAck();

private:
    //界面文字标签
    QLabel *titleLabel;
    QLabel *tempLabel;
    QLabel *humiLabel;
    QLabel *mq2AdcLabel;
    QLabel *mq2PpmLabel;
    QLabel *gasAlarmLabel;
    QLabel *personLabel;
    QLabel *systemStateLabel;
    QLabel *serialStatusLabel;
    QLabel *visionStatusLabel;

    //摄像头画面控件
    QLabel *videoLabel;

    //视觉识别子线程
    QThread *visionThread;
    VisionWorker *visionWorker;

    //功能按钮
    QPushButton *servoCenterButton;
    QPushButton *stepperOpenButton;
    QPushButton *alarmResetButton;
    QPushButton *exitButton;
    QPushButton *autoTrackButton;

    // 串口传感器子线程
    QThread *sensorThread;
    SensorWorker *sensorWorker;

    // 传感器缓存数据
    double currentTemp;
    double currentHumi;
    int currentMq2Adc;
    double currentMq2Voltage;
    double currentMq2Rs;
    double currentMq2Ppm;
    bool currentGasAlarm;
    bool currentPersonDetected;
    bool sensorValid;

    QThread *detectorThread;
    DetectorWorker *detectorWorker;

    bool detectorBusy;

    QElapsedTimer uiClock;
    qint64 lastPersonDetectMs;

    int personBoxX;
    int personBoxY;
    int personBoxW;
    int personBoxH;
    float personScore;

    bool autoTrackEnabled;

    int servoYawAngle;
    int servoPitchAngle;

    int lastFrameW;
    int lastFrameH;

    QElapsedTimer controlClock;
    qint64 lastServoCmdMs;

    QTimer *servoControlTimer;

    bool haveTrackTarget;
    bool errorFilterInited;

    double filteredErrorX;
    double filteredErrorY;

    double servoYawCmd;
    double servoPitchCmd;

    int lastSentYaw;
    int lastSentPitch;

    qint64 lastServoTickMs;

    QThread *mqttThread;
    MqttWorker *mqttWorker;
    QTimer *mqttPublishTimer;

    PowerState powerState;

    QElapsedTimer powerClock;

    qint64 lastHumanActivityMs;
    qint64 pirWakeStartMs;

    bool suspendRequested;
    bool pirMotion;
    bool pirWakeVerifyMode;

private:
    void initUi();
    void startSensorWorker();
    void updateDisplay();
    void startVisionWorker();
    void startDetectorWorker();

    void startServoControlTimer();

    double applyDeadZone(double error, double deadZone);
    double clampDouble(double value, double minValue, double maxValue);

    int clampYawAngle(int angle);
    int clampPitchAngle(int angle);
    void startMqttWorker();
    void publishPropertySetReply(const QString& id, bool ok);
    
    void startRtmpStreamer();
    void stopRtmpStreamer();

    QThread *streamThread;
    RtmpStreamer *rtmpStreamer;

    void startPowerManager();

    void enterIdleMode();
    void leaveIdleMode();

    void requestSystemSuspend();
    void afterSystemResume();

    void stopVisionWorker();
    void stopDetectorWorker();

    QTimer *powerTimer;

};

#endif