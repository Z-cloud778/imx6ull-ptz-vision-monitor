/*************************************************
 * @File: mainwindow.cpp
 * @Author: Z-cloud778
 * @Date: 2026-07-14
 * @Note: 项目主窗口与核心业务逻辑文件
*************************************************/
#include "mainwindow.h"

#include <QWidget>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QFont>
#include <QDebug>
#include <QApplication>
#include <QMetaObject>
#include <QTimer>
#include <QPixmap>
#include <QMetaType>

#include <QPainter>
#include <QPen>
#include <QMetaType>
#include <cmath>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonParseError>

#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>
#include <cstdio>

#include <QElapsedTimer>
#include <QProcess>

static const int SERVO_YAW_MIN_ANGLE = 0;
static const int SERVO_YAW_MAX_ANGLE = 180;

static const int SERVO_PITCH_MIN_ANGLE = 0;
static const int SERVO_PITCH_MAX_ANGLE = 180;

static const int SERVO_CENTER_YAW = 90;
static const int SERVO_CENTER_PITCH = 90;

/*
 * 舵机闭环控制周期。
 * 80ms 比 120ms 响应更快，但不会像 50ms 那样过于频繁。
 */
static const int SERVO_SMOOTH_INTERVAL_MS = 50;

/*
 * 图像中心死区。
 * 误差在死区内不控制，防止 MOSSE 小抖动导致舵机乱动。
 */
static const double TRACK_DEAD_ZONE_X = 18.0;
static const double TRACK_DEAD_ZONE_Y = 14.0;

/*
 * 目标误差低通滤波系数。
 * 越大响应越快，越小越平滑。
 */
static const double ERROR_FILTER_ALPHA = 0.50;

/*
 * 增量式 PID 参数。
 * 输入：目标框中心与图像中心的像素误差。
 * 输出：本周期舵机目标角度增量，单位为度。
 *
 * 先把 D 项设得很小，避免 MOSSE 框抖动被放大。
 */
static const double YAW_PID_KP = 0.16;
static const double YAW_PID_KI = 0.055;
static const double YAW_PID_KD = 0.001;

static const double PITCH_PID_KP = 0.085;
static const double PITCH_PID_KI = 0.030;
static const double PITCH_PID_KD = 0.002;

/*
 * 单次控制最大角度增量。
 * 限幅可以防止舵机突然大幅跳动。
 */
static const double YAW_DELTA_LIMIT = 4.0;
static const double PITCH_DELTA_LIMIT = 3.0;

/*
 * 最小角度增量。
 * 只要目标已经偏出死区，就至少给一个明显修正量，
 * 避免 PID 输出太小，舵机半天不动。
 */
static const double YAW_MIN_DELTA = 1.0;
static const double PITCH_MIN_DELTA = 0.8;

/*
 * 方向系数。
 * 如果水平反向，改 YAW_SIGN。
 * 如果俯仰反向，改 PITCH_SIGN。
 */
static const int YAW_SIGN = -1;
static const int PITCH_SIGN = 1;


/*
要保证：

SUSPEND_TIMEOUT_MS > IDLE_TIMEOUT_MS

否则逻辑上容易混乱
*/
static const qint64 IDLE_TIMEOUT_MS = 30000;
static const qint64 SUSPEND_TIMEOUT_MS = 120000;
static const qint64 PIR_VERIFY_TIMEOUT_MS = 30000;


/*
 * 第一阶段先不做真正系统 suspend。
 * 等应用级休眠稳定后，再改成 true。
 */
static const bool ENABLE_SYSTEM_SUSPEND = true;


void MainWindow::requestSystemSuspend()
{
    if (powerState != POWER_IDLE)
        return;

    if (suspendRequested)
        return;

    qDebug() << "[POWER] request system suspend";

    suspendRequested = true;
    powerState = POWER_SUSPEND_PENDING;

    if (sensorWorker)
    {
        QMetaObject::invokeMethod(sensorWorker,
                                  "setSleepPrepareMode",
                                  Qt::BlockingQueuedConnection,
                                  Q_ARG(bool, true));
    }

    QTimer::singleShot(300, this, [this]() {
        if (sensorWorker &&
            powerState == POWER_SUSPEND_PENDING &&
            suspendRequested)
        {
            QMetaObject::invokeMethod(sensorWorker,
                                      "sendHostSleepPrepare",
                                      Qt::QueuedConnection);
        }
    });

    QTimer::singleShot(5000, this, [this]() {
        if (powerState == POWER_SUSPEND_PENDING && suspendRequested)
        {
            qDebug() << "[POWER] HOST,SLEEP_READY timeout, back to IDLE";

            suspendRequested = false;
            powerState = POWER_IDLE;

            if (sensorWorker)
            {
                QMetaObject::invokeMethod(sensorWorker,
                                          "setSleepPrepareMode",
                                          Qt::QueuedConnection,
                                          Q_ARG(bool, false));
            }
        }
    });
}

void MainWindow::startPowerManager()
{
    if (powerTimer)
        return;

    powerTimer = new QTimer(this);

    connect(powerTimer, &QTimer::timeout,
            this, &MainWindow::powerTick);

    powerTimer->start(1000);
}

void MainWindow::powerTick()
{
    if (!powerClock.isValid())
        return;

    qint64 now = powerClock.elapsed();

    /*
     * 只有视觉确认有人，才刷新活动时间。
     * 不要用 pirMotion 刷新 lastHumanActivityMs。
     * PIR 只负责唤醒，不负责证明真的有人。
     */
    if (currentPersonDetected || haveTrackTarget)
    {
        lastHumanActivityMs = now;

        if (pirWakeVerifyMode)
        {
            qDebug() << "[POWER] PIR wake verified by vision";
            pirWakeVerifyMode = false;
        }
    }

    /*
     * PIR 唤醒后，给视觉一段时间确认。
     * 如果摄像头没有确认到人，认为 PIR 是误触发，重新进入 IDLE。
     */
    if (pirWakeVerifyMode)
    {
        if (now - pirWakeStartMs >= PIR_VERIFY_TIMEOUT_MS)
        {
            qDebug() << "[POWER] PIR wake not verified, back to IDLE";

            pirWakeVerifyMode = false;
            pirMotion = false;

            if (powerState == POWER_ACTIVE)
            {
                enterIdleMode();
            }

            return;
        }
    }

    qint64 inactiveMs = now - lastHumanActivityMs;

    static qint64 lastPowerDebugMs = 0;

    if (now - lastPowerDebugMs >= 3000)
    {
        lastPowerDebugMs = now;

        qDebug() << "[POWER_DEBUG]"
                 << "state=" << powerState
                 << "inactiveMs=" << inactiveMs
                 << "idleTimeout=" << IDLE_TIMEOUT_MS
                 << "suspendTimeout=" << SUSPEND_TIMEOUT_MS
                 << "enableSuspend=" << ENABLE_SYSTEM_SUSPEND
                 << "suspendRequested=" << suspendRequested
                 << "person=" << currentPersonDetected
                 << "track=" << haveTrackTarget
                 << "pir=" << pirMotion
                 << "verify=" << pirWakeVerifyMode;
    }

    /*
     * 长时间无人，先进入应用级 IDLE。
     */
    if (powerState == POWER_ACTIVE &&
        inactiveMs >= IDLE_TIMEOUT_MS)
    {
        enterIdleMode();
        return;
    }

    /*
     * IDLE 状态继续无人，再进入系统级 suspend。
     */
    if (ENABLE_SYSTEM_SUSPEND &&
        powerState == POWER_IDLE &&
        inactiveMs >= SUSPEND_TIMEOUT_MS)
    {
        requestSystemSuspend();
        return;
    }
}

void MainWindow::onPirStateChanged(bool motion)
{
    pirMotion = motion;

    qDebug() << "[PIR]" << motion;

    if (motion)
    {
        if (!powerClock.isValid())
            powerClock.start();

        qint64 now = powerClock.elapsed();

        pirWakeVerifyMode = true;
        pirWakeStartMs = now;

        /*
         * 这里不是把 PIR 当成“确认有人”，
         * 而是给 YOLO/MOSSE 一个验证窗口，避免刚触发就重新 IDLE。
         */
        lastHumanActivityMs = now;

        if (powerState != POWER_ACTIVE)
        {
            leaveIdleMode();
        }
    }
    else
    {
        pirMotion = false;
    }
}


static cv::Mat qImageToBgrMat(const QImage& image)
{
    if (image.isNull())
        return cv::Mat();

    QImage rgbImage = image.convertToFormat(QImage::Format_RGB888);

    cv::Mat rgbMat(rgbImage.height(),
                   rgbImage.width(),
                   CV_8UC3,
                   const_cast<uchar*>(rgbImage.bits()),
                   rgbImage.bytesPerLine());

    cv::Mat bgrMat;
    cv::cvtColor(rgbMat, bgrMat, cv::COLOR_RGB2BGR);

    return bgrMat.clone();
}

static void drawStreamOverlay(cv::Mat& frame,
                              double temp,
                              double humi,
                              double mq2ppm,
                              bool gasAlarm,
                              bool personDetected,
                              int boxX,
                              int boxY,
                              int boxW,
                              int boxH,
                              float score,
                              int yaw,
                              int pitch,
                              bool autoTrack)
{
    if (frame.empty())
        return;

    /*
     * 1. 画人体框
     */
    if (personDetected && boxW > 0 && boxH > 0)
    {
        cv::Rect box(boxX, boxY, boxW, boxH);
        cv::Rect imageRect(0, 0, frame.cols, frame.rows);
        box = box & imageRect;

        if (box.width > 0 && box.height > 0)
        {
            cv::rectangle(frame,
                          box,
                          cv::Scalar(0, 0, 255),
                          2);

            char personText[64];
            snprintf(personText,
                     sizeof(personText),
                     "Person %.2f",
                     score);

            int textY = box.y - 8;
            if (textY < 20)
                textY = box.y + 20;

            cv::putText(frame,
                        personText,
                        cv::Point(box.x, textY),
                        cv::FONT_HERSHEY_SIMPLEX,
                        0.55,
                        cv::Scalar(0, 0, 255),
                        2);
        }
    }

    /*
     * 2. 左上角画传感器和状态数据
     */
    char line1[128];
    char line2[128];
    char line3[128];

    snprintf(line1,
             sizeof(line1),
             "Temp: %.1f C  Humi: %.1f %%",
             temp,
             humi);

    snprintf(line2,
             sizeof(line2),
             "MQ2: %.1f ppm  Alarm: %s",
             mq2ppm,
             gasAlarm ? "YES" : "NO");

    snprintf(line3,
             sizeof(line3),
             "Yaw: %d  Pitch: %d  Auto: %s",
             yaw,
             pitch,
             autoTrack ? "ON" : "OFF");

    cv::putText(frame,
                line1,
                cv::Point(10, 25),
                cv::FONT_HERSHEY_SIMPLEX,
                0.55,
                cv::Scalar(0, 255, 255),
                2);

    cv::putText(frame,
                line2,
                cv::Point(10, 50),
                cv::FONT_HERSHEY_SIMPLEX,
                0.55,
                cv::Scalar(0, 255, 255),
                2);

    cv::putText(frame,
                line3,
                cv::Point(10, 75),
                cv::FONT_HERSHEY_SIMPLEX,
                0.55,
                cv::Scalar(0, 255, 255),
                2);
}

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent),
      titleLabel(nullptr),
      tempLabel(nullptr),
      humiLabel(nullptr),
      mq2AdcLabel(nullptr),
      mq2PpmLabel(nullptr),
      gasAlarmLabel(nullptr),
      personLabel(nullptr),
      systemStateLabel(nullptr),
      serialStatusLabel(nullptr),
      visionStatusLabel(nullptr),

      videoLabel(nullptr),

      visionThread(nullptr),
      visionWorker(nullptr),

      servoCenterButton(nullptr),
      stepperOpenButton(nullptr),
      alarmResetButton(nullptr),
      exitButton(nullptr),

      sensorThread(nullptr),
      sensorWorker(nullptr),

      currentTemp(0.0),
      currentHumi(0.0),
      currentMq2Adc(0),
      currentMq2Voltage(0.0),
      currentMq2Rs(0.0),
      currentMq2Ppm(0.0),
      currentGasAlarm(false),
      currentPersonDetected(false),
      sensorValid(false),

      detectorThread(nullptr),
      detectorWorker(nullptr),

      detectorBusy(false),
      lastPersonDetectMs(-100000),
      personBoxX(0),
      personBoxY(0),
      personBoxW(0),
      personBoxH(0),
      personScore(0.0f),

      autoTrackEnabled(false),
	 servoYawAngle(SERVO_CENTER_YAW),
	 servoPitchAngle(SERVO_CENTER_PITCH),
	 lastFrameW(320),
	 lastFrameH(240),
	 lastServoCmdMs(-100000),
	 servoControlTimer(nullptr),
      haveTrackTarget(false),
	 errorFilterInited(false),
	 filteredErrorX(0.0),
	 filteredErrorY(0.0),
	 servoYawCmd(SERVO_CENTER_YAW),
	 servoPitchCmd(SERVO_CENTER_PITCH),
	 lastSentYaw(SERVO_CENTER_YAW),
	 lastSentPitch(SERVO_CENTER_PITCH),
	 lastServoTickMs(0),
	 mqttThread(nullptr),
      mqttWorker(nullptr),
      mqttPublishTimer(nullptr),
      streamThread(nullptr),
	 rtmpStreamer(nullptr),
	 powerState(POWER_ACTIVE),
	 powerTimer(nullptr),
	 lastHumanActivityMs(0),
	 pirMotion(false),
	 suspendRequested(false),
	 pirWakeVerifyMode(false),
      pirWakeStartMs(0)
{
    qRegisterMetaType<QImage>("QImage");

    uiClock.start();

    initUi();
    startSensorWorker();
    updateDisplay();
    startVisionWorker();
    startDetectorWorker();
    startServoControlTimer();
    startMqttWorker();

    qRegisterMetaType<cv::Mat>("cv::Mat");
    
    startRtmpStreamer();
    
    powerClock.start();
    lastHumanActivityMs = powerClock.elapsed();

    startPowerManager();

    QTimer *powerTimer = new QTimer(this);
    connect(powerTimer, &QTimer::timeout,
            this, &MainWindow::powerTick);
    powerTimer->start(1000);
    
}

MainWindow::~MainWindow()
{
	stopRtmpStreamer();
	
    if (visionWorker && visionThread && visionThread->isRunning())
    {
        QMetaObject::invokeMethod(visionWorker, "stop", Qt::BlockingQueuedConnection);
    }

    if (visionThread)
    {
        visionThread->quit();
        visionThread->wait();
    }

    if (detectorWorker && detectorThread && detectorThread->isRunning())
    {
        QMetaObject::invokeMethod(detectorWorker, "stop", Qt::BlockingQueuedConnection);
    }

    if (detectorThread)
    {
        detectorThread->quit();
        detectorThread->wait();
    }

    if (sensorWorker && sensorThread && sensorThread->isRunning())
    {
        QMetaObject::invokeMethod(sensorWorker, "stop", Qt::BlockingQueuedConnection);
    }

    if (sensorThread)
    {
        sensorThread->quit();
        sensorThread->wait();
    }
}

void MainWindow::initUi()
{
    QWidget *central = new QWidget(this);
    setCentralWidget(central);

    QVBoxLayout *mainLayout = new QVBoxLayout(central);
    mainLayout->setContentsMargins(8, 8, 8, 8);
    mainLayout->setSpacing(6);

    titleLabel = new QLabel("Smart Security Monitor", this);
    titleLabel->setAlignment(Qt::AlignCenter);
    titleLabel->setFixedHeight(38);

    QFont titleFont;
    titleFont.setPointSize(20);
    titleFont.setBold(true);
    titleLabel->setFont(titleFont);

    mainLayout->addWidget(titleLabel);

    /*
     * 主体区域：左边视频，右边数据显示和按钮
     */
    QHBoxLayout *bodyLayout = new QHBoxLayout();
    bodyLayout->setSpacing(10);
    mainLayout->addLayout(bodyLayout);

    /*
     * 左侧：摄像头画面
     */
    videoLabel = new QLabel(this);
    videoLabel->setAlignment(Qt::AlignCenter);
    videoLabel->setFixedSize(640, 480);
    videoLabel->setText("Camera: WAIT");
    videoLabel->setStyleSheet(
        "background-color: black;"
        "color: white;"
        "font-size: 20px;"
        "border: 2px solid #606060;"
    );

    bodyLayout->addWidget(videoLabel);

    /*
     * 右侧：数据 + 状态 + 按钮
     */
    QVBoxLayout *rightLayout = new QVBoxLayout();
    rightLayout->setSpacing(8);
    bodyLayout->addLayout(rightLayout);

    QFont labelFont;
    labelFont.setPointSize(14);

    QFont statusFont;
    statusFont.setPointSize(11);

    tempLabel = new QLabel(this);
    humiLabel = new QLabel(this);
    mq2AdcLabel = new QLabel(this);
    mq2PpmLabel = new QLabel(this);
    gasAlarmLabel = new QLabel(this);
    personLabel = new QLabel(this);
    systemStateLabel = new QLabel(this);
    serialStatusLabel = new QLabel(this);
    visionStatusLabel = new QLabel(this);

    tempLabel->setFont(labelFont);
    humiLabel->setFont(labelFont);
    mq2AdcLabel->setFont(labelFont);
    mq2PpmLabel->setFont(labelFont);
    gasAlarmLabel->setFont(labelFont);
    personLabel->setFont(labelFont);
    systemStateLabel->setFont(labelFont);

    serialStatusLabel->setFont(statusFont);
    visionStatusLabel->setFont(statusFont);

    serialStatusLabel->setWordWrap(true);
    visionStatusLabel->setWordWrap(true);

    serialStatusLabel->setFixedHeight(42);
    visionStatusLabel->setFixedHeight(42);

    QGridLayout *dataLayout = new QGridLayout();
    dataLayout->setHorizontalSpacing(8);
    dataLayout->setVerticalSpacing(6);

    dataLayout->addWidget(tempLabel, 0, 0);
    dataLayout->addWidget(humiLabel, 0, 1);

    dataLayout->addWidget(mq2AdcLabel, 1, 0);
    dataLayout->addWidget(mq2PpmLabel, 1, 1);

    dataLayout->addWidget(gasAlarmLabel, 2, 0);
    dataLayout->addWidget(personLabel, 2, 1);

    dataLayout->addWidget(systemStateLabel, 3, 0, 1, 2);
    dataLayout->addWidget(serialStatusLabel, 4, 0, 1, 2);
    dataLayout->addWidget(visionStatusLabel, 5, 0, 1, 2);

    rightLayout->addLayout(dataLayout);

    /*
     * 按钮区域：2 × 2 排列，避免横向太挤，也方便触摸
     */
    QGridLayout *buttonLayout = new QGridLayout();
    buttonLayout->setSpacing(10);

    servoCenterButton = new QPushButton("Servo\nCenter", this);
    stepperOpenButton = new QPushButton("Stepper\nOpen", this);
    alarmResetButton = new QPushButton("Alarm\nReset", this);
    exitButton = new QPushButton("Exit", this);
    autoTrackButton = new QPushButton("Auto\nOFF", this);

    QFont buttonFont;
    buttonFont.setPointSize(14);
    buttonFont.setBold(true);

    servoCenterButton->setFont(buttonFont);
    stepperOpenButton->setFont(buttonFont);
    alarmResetButton->setFont(buttonFont);
    exitButton->setFont(buttonFont);
    autoTrackButton->setFont(buttonFont);

    servoCenterButton->setFixedHeight(70);
    stepperOpenButton->setFixedHeight(70);
    alarmResetButton->setFixedHeight(70);
    exitButton->setFixedHeight(70);
    autoTrackButton->setFixedHeight(60);

    buttonLayout->addWidget(servoCenterButton, 0, 0);
    buttonLayout->addWidget(stepperOpenButton, 0, 1);
    buttonLayout->addWidget(autoTrackButton, 1, 0);
    buttonLayout->addWidget(alarmResetButton, 1, 1);
    buttonLayout->addWidget(exitButton, 2, 0, 1, 2);

    rightLayout->addLayout(buttonLayout);
    rightLayout->addStretch();

    connect(alarmResetButton, &QPushButton::clicked, this, [this]() {
        qDebug() << "[BTN] Alarm reset clicked";
        currentGasAlarm = false;
        updateDisplay();
    });

    connect(exitButton, &QPushButton::clicked, this, []() {
        qDebug() << "[BTN] Exit clicked";
        QApplication::quit();
    });

    connect(autoTrackButton, &QPushButton::clicked, this, [this]() {
        autoTrackEnabled = !autoTrackEnabled;

        if (autoTrackEnabled)
        {
            autoTrackButton->setText("Auto\nON");
            qDebug() << "[BTN] Auto Track ON";

            servoYawCmd = servoYawAngle;
            servoPitchCmd = servoPitchAngle;

            lastSentYaw = servoYawAngle;
            lastSentPitch = servoPitchAngle;

            haveTrackTarget = false;
            errorFilterInited = false;
            filteredErrorX = 0.0;
            filteredErrorY = 0.0;

            if (!controlClock.isValid())
                controlClock.start();

            lastServoTickMs = controlClock.elapsed();
        }
        else
        {
            autoTrackButton->setText("Auto\nOFF");
            qDebug() << "[BTN] Auto Track OFF";

            haveTrackTarget = false;
            errorFilterInited = false;
        }
    });

    this->setStyleSheet(
        "QMainWindow { background-color: #202020; }"
        "QLabel { color: white; }"
        "QPushButton {"
        "    font-size: 16px;"
        "    background-color: #404040;"
        "    color: white;"
        "    border: 2px solid #808080;"
        "    border-radius: 8px;"
        "}"
        "QPushButton:pressed {"
        "    background-color: #606060;"
        "}"
    );
}

void MainWindow::startDetectorWorker()
{
    if (detectorThread || detectorWorker)
        return;
    
    detectorThread = new QThread(this);
    detectorWorker = new DetectorWorker();

    detectorWorker->moveToThread(detectorThread);

    connect(detectorThread, &QThread::started,
            detectorWorker, &DetectorWorker::start);

    connect(detectorThread, &QThread::finished,
            detectorWorker, &QObject::deleteLater);

    connect(detectorWorker, &DetectorWorker::detectionResult,
            this, &MainWindow::onDetectionResult);

    connect(detectorWorker, &DetectorWorker::detectorStatus,
            this, &MainWindow::onDetectorStatus);

    connect(detectorWorker, &DetectorWorker::detectorIdle,
            this, &MainWindow::onDetectorIdle);

    detectorThread->start(QThread::LowPriority);
}

void MainWindow::startSensorWorker()
{
    sensorThread = new QThread(this);
    sensorWorker = new SensorWorker("/dev/ttymxc5", 115200);

    sensorWorker->moveToThread(sensorThread);

    connect(sensorThread, &QThread::started,
            sensorWorker, &SensorWorker::start);

    connect(sensorThread, &QThread::finished,
            sensorWorker, &QObject::deleteLater);

    connect(sensorWorker, &SensorWorker::sensorUpdated,
            this, &MainWindow::onSensorUpdated);

    connect(sensorWorker, &SensorWorker::serialStatus,
            this, &MainWindow::onSerialStatus);
    connect(sensorWorker, &SensorWorker::pirStateChanged,
            this, &MainWindow::onPirStateChanged);

    connect(sensorWorker, &SensorWorker::hostSleepReady,
            this, &MainWindow::onHostSleepReady);

    connect(sensorWorker, &SensorWorker::hostSleepDenied,
            this, &MainWindow::onHostSleepDenied);
    connect(sensorWorker, &SensorWorker::hostAwakeAck,
            this, &MainWindow::onHostAwakeAck);

    connect(servoCenterButton, &QPushButton::clicked, this, [this]() {
        qDebug() << "[BTN] Servo Center clicked";

        autoTrackEnabled = false;

        if (autoTrackButton)
            autoTrackButton->setText("Auto\nOFF");

        servoYawAngle = SERVO_CENTER_YAW;
        servoPitchAngle = SERVO_CENTER_PITCH;

        servoYawCmd = SERVO_CENTER_YAW;
        servoPitchCmd = SERVO_CENTER_PITCH;

        lastSentYaw = SERVO_CENTER_YAW;
        lastSentPitch = SERVO_CENTER_PITCH;

        haveTrackTarget = false;
        errorFilterInited = false;
        filteredErrorX = 0.0;
        filteredErrorY = 0.0;

        QMetaObject::invokeMethod(sensorWorker,
                              "sendServoAngle",
                              Qt::QueuedConnection,
                              Q_ARG(int, servoYawAngle),
                              Q_ARG(int, servoPitchAngle));
    });

    connect(stepperOpenButton, &QPushButton::clicked, this, [this]() {
    qDebug() << "[BTN] Stepper Open clicked";

    QMetaObject::invokeMethod(sensorWorker,
                              "sendStepperOpen",
                              Qt::QueuedConnection);
    });

    sensorThread->start();


    QTimer::singleShot(5000, this, [this]() {
    qDebug() << "[AUTO_TEST] send servo center";

    QMetaObject::invokeMethod(sensorWorker,
                              "sendServoCenter",
                              Qt::QueuedConnection);
    });

    QTimer::singleShot(8000, this, [this]() {
    qDebug() << "[AUTO_TEST] send stepper open";

    QMetaObject::invokeMethod(sensorWorker,
                              "sendStepperOpen",
                              Qt::QueuedConnection);
    });
}

void MainWindow::onSensorUpdated(double temperature,
                                 double humidity,
                                 int mq2Adc,
                                 double mq2Voltage,
                                 double mq2Rs,
                                 double mq2Ppm,
                                 bool gasAlarm)
{
    currentTemp = temperature;
    currentHumi = humidity;
    currentMq2Adc = mq2Adc;
    currentMq2Voltage = mq2Voltage;
    currentMq2Rs = mq2Rs;
    currentMq2Ppm = mq2Ppm;
    currentGasAlarm = gasAlarm;
    sensorValid = true;

    updateDisplay();
}

void MainWindow::onSerialStatus(const QString& text)
{
    qDebug() << "[SERIAL]" << text;

    if (serialStatusLabel)
    {
        serialStatusLabel->setText("Serial: " + text);
    }
}

void MainWindow::updateDisplay()
{
    if (sensorValid)
    {
        tempLabel->setText(QString("Temp: %1 C").arg(currentTemp, 0, 'f', 1));
        humiLabel->setText(QString("Humi: %1 %").arg(currentHumi, 0, 'f', 1));
        mq2AdcLabel->setText(QString("MQ2_ADC: %1").arg(currentMq2Adc));
        mq2PpmLabel->setText(QString("MQ2_PPM: %1").arg(currentMq2Ppm, 0, 'f', 1));
    }
    else
    {
        tempLabel->setText("Temp: WAIT");
        humiLabel->setText("Humi: WAIT");
        mq2AdcLabel->setText("MQ2_ADC: WAIT");
        mq2PpmLabel->setText("MQ2_PPM: WAIT");
    }

    if (currentGasAlarm)
    {
        gasAlarmLabel->setText("Gas Alarm: YES");
        gasAlarmLabel->setStyleSheet("color: red; font-weight: bold;");
    }
    else
    {
        gasAlarmLabel->setText("Gas Alarm: NO");
        gasAlarmLabel->setStyleSheet("color: lightgreen; font-weight: bold;");
    }

    if (currentPersonDetected)
    {
        personLabel->setText("Person: YES");
        personLabel->setStyleSheet("color: red; font-weight: bold;");
    }
    else
    {
        personLabel->setText("Person: NO");
        personLabel->setStyleSheet("color: lightgreen; font-weight: bold;");
    }

    if (currentGasAlarm)
    {
        systemStateLabel->setText("System State: DANGER");
        systemStateLabel->setStyleSheet("color: red; font-weight: bold;");
    }
    else
    {
        systemStateLabel->setText("System State: NORMAL");
        systemStateLabel->setStyleSheet("color: lightgreen; font-weight: bold;");
    }
}

void MainWindow::startVisionWorker()
{
     if (visionThread || visionWorker)
        return;
    
    visionThread = new QThread(this);
    visionWorker = new VisionWorker("/dev/video1", 320, 240);

    visionWorker->moveToThread(visionThread);

    connect(visionThread, &QThread::started,
            visionWorker, &VisionWorker::start);

    connect(visionThread, &QThread::finished,
            visionWorker, &QObject::deleteLater);

    connect(visionWorker, &VisionWorker::frameReady,
            this, &MainWindow::onVideoFrame);

    connect(visionWorker, &VisionWorker::visionStatus,
            this, &MainWindow::onVisionStatus);
    connect(visionWorker, &VisionWorker::detectorFrameReady,
            this, &MainWindow::onDetectorFrameCandidate);
    connect(visionWorker, &VisionWorker::trackingResult,
            this, &MainWindow::onTrackingResult);

    visionThread->start();
}

void MainWindow::onVideoFrame(const QImage& image)
{
    if (powerState != POWER_ACTIVE)
        return;
    
    if (!videoLabel || image.isNull())
        return;

    lastFrameW = image.width();
    lastFrameH = image.height();

    /*
     * displayImage 是本地显示画面。
     * 你原来已经在这里用 QPainter 画了人体框。
     */
    QImage displayImage = image.copy();

    if (currentPersonDetected && personBoxW > 0 && personBoxH > 0)
    {
        QPainter painter(&displayImage);
        painter.setPen(QPen(Qt::red, 3));

        QRect box(personBoxX,
                  personBoxY,
                  personBoxW,
                  personBoxH);

        box = box.intersected(QRect(0,
                                    0,
                                    displayImage.width(),
                                    displayImage.height()));

        if (box.width() > 0 && box.height() > 0)
        {
            painter.drawRect(box);

            QString text = QString("Person %1")
                           .arg(personScore, 0, 'f', 2);

            int textY = box.y() - 6;

            if (textY < 20)
                textY = box.y() + 20;

            painter.drawText(box.x(), textY, text);
        }
    }

    /*
     * 本地 LCD 显示。
     */
    QPixmap pixmap = QPixmap::fromImage(displayImage);

    pixmap = pixmap.scaled(videoLabel->size(),
                           Qt::KeepAspectRatio,
                           Qt::FastTransformation);

    videoLabel->setPixmap(pixmap);

    /*
     * RTMP 推流：
     * 只推 displayImage，不再调用 drawStreamOverlay()。
     *
     * 注意：
     * displayImage 已经包含 QPainter 画的人框；
     * 这里不再额外画温湿度、MQ2、Yaw、Pitch。
     */
    static int streamFrameCount = 0;
    streamFrameCount++;

    /*
     * 每 5 帧推 1 帧，降低 CPU 压力。
     * 如果还是卡，可以改成 8 或 10。
     * 如果不卡，可以改成 3。
     */
    if (streamFrameCount % 1 == 0)
    {
        cv::Mat streamFrame = qImageToBgrMat(displayImage);

        if (!streamFrame.empty())
        {
            emit streamFrameReady(streamFrame);
        }
    }
}

void MainWindow::onVisionStatus(const QString& text)
{
    qDebug() << "[VISION]" << text;

    if (visionStatusLabel)
    {
        visionStatusLabel->setText("Vision: " + text);
    }
}

void MainWindow::onDetectorFrameCandidate(const QImage& image)
{
    if (powerState != POWER_ACTIVE)
        return;
    
    if (!detectorWorker)
        return;

    /*
     * 关键：
     * 如果 YOLO 还没处理完上一帧，直接丢掉这帧。
     * 这样不会产生检测帧堆积。
     */
    if (detectorBusy)
        return;

    detectorBusy = true;

    QMetaObject::invokeMethod(detectorWorker,
                              "processFrame",
                              Qt::QueuedConnection,
                              Q_ARG(QImage, image.copy()));
}

void MainWindow::onDetectionResult(bool detected,
                                   int x,
                                   int y,
                                   int w,
                                   int h,
                                   float score)
{
    if (detected)
    {
        qDebug() << "[DETECT] YOLO found person, init MOSSE";

        currentPersonDetected = true;
        lastHumanActivityMs = powerClock.elapsed();

        pirWakeVerifyMode = false;
        pirMotion = false;

        personBoxX = x;
        personBoxY = y;
        personBoxW = w;
        personBoxH = h;
        personScore = score;

        updateDisplay();

        QMetaObject::invokeMethod(visionWorker,
                                  "initTrackerFromDetection",
                                  Qt::QueuedConnection,
                                  Q_ARG(bool, true),
                                  Q_ARG(int, x),
                                  Q_ARG(int, y),
                                  Q_ARG(int, w),
                                  Q_ARG(int, h),
                                  Q_ARG(float, score));
    }
    else
    {
        /*
         * 只有在当前没有跟踪目标时，才显示 Person:NO。
         */
        if (!currentPersonDetected)
        {
            personBoxX = 0;
            personBoxY = 0;
            personBoxW = 0;
            personBoxH = 0;
            personScore = 0.0f;
            updateDisplay();
        }
    }
}

void MainWindow::onDetectorStatus(const QString& text)
{
    qDebug() << "[DETECT]" << text;

    if (visionStatusLabel)
    {
        visionStatusLabel->setText("Detector: " + text);
    }
}

void MainWindow::onDetectorIdle()
{
    detectorBusy = false;
}

void MainWindow::onPersonDetected(bool detected)
{
    currentPersonDetected = detected;
    updateDisplay();
}

void MainWindow::onTrackingResult(bool tracked,
                                  int x,
                                  int y,
                                  int w,
                                  int h,
                                  float score)
{
    currentPersonDetected = tracked;

    if (tracked)
    {
    	   lastHumanActivityMs = powerClock.elapsed();
    	   
        personBoxX = x;
        personBoxY = y;
        personBoxW = w;
        personBoxH = h;
        personScore = score;

        int targetCx = personBoxX + personBoxW / 2;
        int targetCy = personBoxY + personBoxH / 2;

        int imageCx = lastFrameW / 2;
        int imageCy = lastFrameH / 2;

        double errorX = targetCx - imageCx;
        double errorY = targetCy - imageCy;

        if (!errorFilterInited)
        {
            filteredErrorX = errorX;
            filteredErrorY = errorY;
            errorFilterInited = true;
        }
        else
        {
            filteredErrorX = ERROR_FILTER_ALPHA * errorX
                           + (1.0 - ERROR_FILTER_ALPHA) * filteredErrorX;

            filteredErrorY = ERROR_FILTER_ALPHA * errorY
                           + (1.0 - ERROR_FILTER_ALPHA) * filteredErrorY;
        }

        haveTrackTarget = true;
    }
    else
    {
        personBoxX = 0;
        personBoxY = 0;
        personBoxW = 0;
        personBoxH = 0;
        personScore = 0.0f;

        haveTrackTarget = false;
        errorFilterInited = false;
        filteredErrorX = 0.0;
        filteredErrorY = 0.0;
    }

    updateDisplay();
}



void MainWindow::startServoControlTimer()
{
    servoControlTimer = new QTimer(this);

    connect(servoControlTimer, &QTimer::timeout,
            this, &MainWindow::servoControlTick);

    servoControlTimer->start(SERVO_SMOOTH_INTERVAL_MS);

    lastServoTickMs = controlClock.isValid() ? controlClock.elapsed() : 0;
}

double MainWindow::clampDouble(double value, double minValue, double maxValue)
{
    if (value < minValue)
        return minValue;

    if (value > maxValue)
        return maxValue;

    return value;
}

double MainWindow::applyDeadZone(double error, double deadZone)
{
    if (std::abs(error) <= deadZone)
        return 0.0;

    if (error > 0)
        return error - deadZone;
    else
        return error + deadZone;
}

int MainWindow::clampYawAngle(int angle)
{
    if (angle < SERVO_YAW_MIN_ANGLE)
        return SERVO_YAW_MIN_ANGLE;

    if (angle > SERVO_YAW_MAX_ANGLE)
        return SERVO_YAW_MAX_ANGLE;

    return angle;
}

int MainWindow::clampPitchAngle(int angle)
{
    if (angle < SERVO_PITCH_MIN_ANGLE)
        return SERVO_PITCH_MIN_ANGLE;

    if (angle > SERVO_PITCH_MAX_ANGLE)
        return SERVO_PITCH_MAX_ANGLE;

    return angle;
}

void MainWindow::servoControlTick()
{
    /*
     * 使用静态变量保存增量式 PID 的历史误差。
     * 这样只需要改 mainwindow.cpp，不需要修改 mainwindow.h。
     */
    static bool pidInited = false;

    static double yawErrLast1 = 0.0;
    static double yawErrLast2 = 0.0;

    static double pitchErrLast1 = 0.0;
    static double pitchErrLast2 = 0.0;

    if (!controlClock.isValid())
        controlClock.start();

    qint64 nowMs = controlClock.elapsed();
    lastServoTickMs = nowMs;

    if (!autoTrackEnabled)
    {
        pidInited = false;
        return;
    }

    if (!currentPersonDetected || !haveTrackTarget)
    {
        pidInited = false;
        return;
    }

    if (!sensorWorker)
    {
        pidInited = false;
        return;
    }

    /*
     * filteredErrorX / filteredErrorY 由 onTrackingResult() 更新。
     * 这里经过死区处理后，得到真正参与 PID 控制的误差。
     */
    double errX = applyDeadZone(filteredErrorX, TRACK_DEAD_ZONE_X);
    double errY = applyDeadZone(filteredErrorY, TRACK_DEAD_ZONE_Y);

    if (!pidInited)
    {
        yawErrLast1 = 0.0;
        yawErrLast2 = 0.0;

        pitchErrLast1 = 0.0;
        pitchErrLast2 = 0.0;

        servoYawCmd = servoYawAngle;
        servoPitchCmd = servoPitchAngle;

        lastSentYaw = servoYawAngle;
        lastSentPitch = servoPitchAngle;

        pidInited = true;
    }

    double deltaYaw = 0.0;
    double deltaPitch = 0.0;

    /*
     * 水平轴增量式 PID：
     * delta = Kp*(e[k]-e[k-1])
     *       + Ki*e[k]
     *       + Kd*(e[k]-2e[k-1]+e[k-2])
     */
    if (errX != 0.0)
    {
        double pidYaw = YAW_PID_KP * (errX - yawErrLast1)
                      + YAW_PID_KI * errX
                      + YAW_PID_KD * (errX - 2.0 * yawErrLast1 + yawErrLast2);

        deltaYaw = YAW_SIGN * pidYaw;

        
        deltaYaw = clampDouble(deltaYaw,
                               -YAW_DELTA_LIMIT,
                               YAW_DELTA_LIMIT);
    }
    else
    {
        /*
         * 进入死区后清空该轴历史误差。
         * 防止目标已经居中后，PID 历史项继续推动舵机漂移。
         */
        yawErrLast1 = 0.0;
        yawErrLast2 = 0.0;
    }

    /*
     * 俯仰轴增量式 PID。
     */
    if (errY != 0.0)
    {
        double pidPitch = PITCH_PID_KP * (errY - pitchErrLast1)
                        + PITCH_PID_KI * errY
                        + PITCH_PID_KD * (errY - 2.0 * pitchErrLast1 + pitchErrLast2);

        deltaPitch = PITCH_SIGN * pidPitch;

        

        deltaPitch = clampDouble(deltaPitch,
                                 -PITCH_DELTA_LIMIT,
                                 PITCH_DELTA_LIMIT);
    }
    else
    {
        pitchErrLast1 = 0.0;
        pitchErrLast2 = 0.0;
    }

    if (deltaYaw == 0.0 && deltaPitch == 0.0)
        return;

    /*
     * 保留 double 小数累积。
     * 不要写 servoYawCmd = yawInt;
     * 不要写 servoPitchCmd = pitchInt;
     */
    servoYawCmd += deltaYaw;
    servoPitchCmd += deltaPitch;

    if (servoYawCmd < SERVO_YAW_MIN_ANGLE)
        servoYawCmd = SERVO_YAW_MIN_ANGLE;
    if (servoYawCmd > SERVO_YAW_MAX_ANGLE)
        servoYawCmd = SERVO_YAW_MAX_ANGLE;

    if (servoPitchCmd < SERVO_PITCH_MIN_ANGLE)
        servoPitchCmd = SERVO_PITCH_MIN_ANGLE;
    if (servoPitchCmd > SERVO_PITCH_MAX_ANGLE)
        servoPitchCmd = SERVO_PITCH_MAX_ANGLE;

    int yawInt = clampYawAngle((int)(servoYawCmd + 0.5));
    int pitchInt = clampPitchAngle((int)(servoPitchCmd + 0.5));

    if (yawInt == lastSentYaw && pitchInt == lastSentPitch)
    {
        yawErrLast2 = yawErrLast1;
        yawErrLast1 = errX;

        pitchErrLast2 = pitchErrLast1;
        pitchErrLast1 = errY;

        return;
    }

    lastSentYaw = yawInt;
    lastSentPitch = pitchInt;

    servoYawAngle = yawInt;
    servoPitchAngle = pitchInt;

    qDebug() << "[PID_SERVO]"
             << "rawErrX=" << filteredErrorX
             << "rawErrY=" << filteredErrorY
             << "ctrlErrX=" << errX
             << "ctrlErrY=" << errY
             << "deltaYaw=" << deltaYaw
             << "deltaPitch=" << deltaPitch
             << "sendYaw=" << yawInt
             << "sendPitch=" << pitchInt;

    QMetaObject::invokeMethod(sensorWorker,
                              "sendServoAngle",
                              Qt::QueuedConnection,
                              Q_ARG(int, yawInt),
                              Q_ARG(int, pitchInt));

    yawErrLast2 = yawErrLast1;
    yawErrLast1 = errX;

    pitchErrLast2 = pitchErrLast1;
    pitchErrLast1 = errY;
}

void MainWindow::startMqttWorker()
{
    mqttThread = new QThread(this);
    mqttWorker = new MqttWorker();

    mqttWorker->moveToThread(mqttThread);

    connect(mqttThread, &QThread::started,
            mqttWorker, &MqttWorker::start);

    connect(mqttThread, &QThread::finished,
            mqttWorker, &QObject::deleteLater);

    connect(mqttWorker, &MqttWorker::mqttStatus,
            this, &MainWindow::onMqttStatus);

    connect(mqttWorker, &MqttWorker::controlMessage,
            this, &MainWindow::onMqttControl);

    mqttThread->start();

    /*
     * 每 2 秒发布一次设备状态。
     */
    mqttPublishTimer = new QTimer(this);

    connect(mqttPublishTimer, &QTimer::timeout,
            this, &MainWindow::publishingMqttStatus);

    mqttPublishTimer->start(2000);
}

void MainWindow::onMqttStatus(const QString& text)
{
    qDebug() << "[MQTT]" << text;
}

void MainWindow::onMqttControl(const QString& topic,
                               const QString& payload)
{
    qDebug() << "[MQTT_RX]"
             << "topic=" << topic
             << "payload=" << payload;

    QJsonParseError err;
    QJsonDocument doc = QJsonDocument::fromJson(payload.toUtf8(), &err);

    if (err.error != QJsonParseError::NoError || !doc.isObject())
    {
        qDebug() << "[ALIYUN_SET] bad json";
        return;
    }

    QJsonObject obj = doc.object();

    if (obj.value("method").toString() == "thing.service.property.set")
    {
        QJsonObject params = obj.value("params").toObject();

        bool needServo = false;

        int yaw = servoYawAngle;
        int pitch = servoPitchAngle;

        if (params.contains("AutoTrack"))
        {
            int autoValue = params.value("AutoTrack").toInt(0);
            autoTrackEnabled = (autoValue != 0);

            if (autoTrackButton)
            {
                autoTrackButton->setText(autoTrackEnabled ?
                                         "Auto\nON" :
                                         "Auto\nOFF");
            }
        }

        if (params.contains("Yaw"))
        {
            yaw = params.value("Yaw").toInt(servoYawAngle);
            needServo = true;
        }

        if (params.contains("Pitch"))
        {
            pitch = params.value("Pitch").toInt(servoPitchAngle);
            needServo = true;
        }

        if (needServo)
        {
            autoTrackEnabled = false;

            if (autoTrackButton)
                autoTrackButton->setText("Auto\nOFF");

            yaw = clampYawAngle(yaw);
            pitch = clampPitchAngle(pitch);

            servoYawAngle = yaw;
            servoPitchAngle = pitch;
            servoYawCmd = yaw;
            servoPitchCmd = pitch;
            lastSentYaw = yaw;
            lastSentPitch = pitch;

            if (sensorWorker)
            {
                QMetaObject::invokeMethod(sensorWorker,
                                          "sendServoAngle",
                                          Qt::QueuedConnection,
                                          Q_ARG(int, yaw),
                                          Q_ARG(int, pitch));
            }
        }

        publishPropertySetReply(obj.value("id").toString(), true);
        publishMqttStatus();

        updateDisplay();
        return;
    }

void MainWindow::publishPropertySetReply(const QString& id, bool ok)
{
    if (!mqttWorker)
        return;

    QJsonObject obj;
    obj["id"] = id;
    obj["version"] = "1.0";
    obj["code"] = ok ? 200 : 500;
    obj["message"] = ok ? "success" : "failed";
    obj["data"] = QJsonObject();

    QJsonDocument doc(obj);
    QString json = QString::fromUtf8(doc.toJson(QJsonDocument::Compact));

    qDebug() << "[ALIYUN_SET_REPLY]" << json;

    QMetaObject::invokeMethod(mqttWorker,
                              "publishPropertySetReplyJson",
                              Qt::QueuedConnection,
                              Q_ARG(QString, json));
}

void MainWindow::publishMqttStatus()
{
    if (!mqttWorker)
        return;

    static int msgId = 1;

    QJsonObject params;

    params["Temp"] = currentTemp;
    params["Humi"] = currentHumi;
    params["Mq2Adc"] = currentMq2Adc;
    params["Mq2Ppm"] = currentMq2Ppm;
    params["GasAlarm"] = currentGasAlarm ? 1 : 0;

    params["PersonDetected"] = currentPersonDetected ? 1 : 0;
    params["Tracked"] = haveTrackTarget ? 1 : 0;

    params["Yaw"] = servoYawAngle;
    params["Pitch"] = servoPitchAngle;
    params["AutoTrack"] = autoTrackEnabled ? 1 : 0;

    QJsonObject obj;
    obj["id"] = QString::number(msgId++);
    obj["version"] = "1.0";
    obj["params"] = params;
    obj["method"] = "thing.event.property.post";

    QJsonDocument doc(obj);
    QString json = QString::fromUtf8(doc.toJson(QJsonDocument::Compact));

    qDebug() << "[ALIYUN_PROPERTY_POST]" << json;

    QMetaObject::invokeMethod(mqttWorker,
                              "publishPropertyJson",
                              Qt::QueuedConnection,
                              Q_ARG(QString, json));
}

void MainWindow::startRtmpStreamer()
{
    if (streamThread || rtmpStreamer)
        return;

    streamThread = new QThread(this);
    rtmpStreamer = new RtmpStreamer();

    rtmpStreamer->moveToThread(streamThread);

    connect(streamThread, &QThread::started,
            rtmpStreamer, &RtmpStreamer::startStream);

    connect(this, &MainWindow::streamFrameReady,
            rtmpStreamer, &RtmpStreamer::updateFrame,
            Qt::QueuedConnection);

    connect(rtmpStreamer, &RtmpStreamer::streamStatus,
            this, [](const QString& text)
            {
                qDebug() << "[RTMP_STATUS]" << text;
            });

    connect(streamThread, &QThread::finished,
            rtmpStreamer, &QObject::deleteLater);

    streamThread->start();
}

void MainWindow::stopRtmpStreamer()
{
    if (rtmpStreamer)
    {
        QMetaObject::invokeMethod(rtmpStreamer,
                                  "stopStream",
                                  Qt::BlockingQueuedConnection);
    }

    if (streamThread)
    {
        streamThread->quit();
        streamThread->wait();

        streamThread = nullptr;
        rtmpStreamer = nullptr;
    }
}

void MainWindow::enterIdleMode()
{
    if (powerState != POWER_ACTIVE)
        return;

    qDebug() << "[POWER] enter IDLE";

    powerState = POWER_IDLE;

    suspendRequested = false;
    pirWakeVerifyMode = false;
    pirMotion = false;

    currentPersonDetected = false;
    haveTrackTarget = false;
    errorFilterInited = false;

    /*
     * 应用级 IDLE 不是系统级休眠准备。
     * 所以这里必须允许 SensorWorker 正常工作。
     */
    if (sensorWorker)
    {
        QMetaObject::invokeMethod(sensorWorker,
                                  "setSleepPrepareMode",
                                  Qt::QueuedConnection,
                                  Q_ARG(bool, false));
    }

    autoTrackEnabled = false;

    if (autoTrackButton)
        autoTrackButton->setText("Auto\nOFF");

    stopRtmpStreamer();
    stopDetectorWorker();
    stopVisionWorker();

    if (videoLabel)
    {
        videoLabel->setPixmap(QPixmap());
        videoLabel->setText("IDLE\nWaiting for PIR");
    }

    publishMqttStatus();
}

void MainWindow::leaveIdleMode()
{
    if (powerState == POWER_ACTIVE)
        return;

    qDebug() << "[POWER] leave IDLE";

    powerState = POWER_ACTIVE;
    suspendRequested = false;

    if (sensorWorker)
    {
        QMetaObject::invokeMethod(sensorWorker,
                                  "setSleepPrepareMode",
                                  Qt::QueuedConnection,
                                  Q_ARG(bool, false));
    }

    pirMotion = false;
    currentPersonDetected = false;
    haveTrackTarget = false;
    errorFilterInited = false;
    filteredErrorX = 0.0;
    filteredErrorY = 0.0;

    lastHumanActivityMs = powerClock.elapsed();

    startVisionWorker();
    startDetectorWorker();
    startRtmpStreamer();

    publishMqttStatus();
}

void MainWindow::stopVisionWorker()
{
    if (visionWorker && visionThread && visionThread->isRunning())
    {
        QMetaObject::invokeMethod(visionWorker,
                                  "stop",
                                  Qt::BlockingQueuedConnection);
    }

    if (visionThread)
    {
        visionThread->quit();
        visionThread->wait();

        visionThread = nullptr;
        visionWorker = nullptr;
    }
}

void MainWindow::stopDetectorWorker()
{
    if (detectorWorker && detectorThread && detectorThread->isRunning())
    {
        QMetaObject::invokeMethod(detectorWorker,
                                  "stop",
                                  Qt::BlockingQueuedConnection);
    }

    if (detectorThread)
    {
        detectorThread->quit();
        detectorThread->wait();

        detectorThread = nullptr;
        detectorWorker = nullptr;
        detectorBusy = false;
    }
}

void MainWindow::onHostSleepReady()
{
    qDebug() << "[POWER] HOST,SLEEP_READY received";

    if (powerState != POWER_SUSPEND_PENDING)
        return;

    /*
     * 再次确保高功耗模块已关闭。
     */
    stopRtmpStreamer();
    stopDetectorWorker();
    stopVisionWorker();

    qDebug() << "[POWER] enter Linux suspend";

    /*
     * 这里会阻塞。
     * 执行 echo mem 后系统休眠。
     * 被 STM32 WAKE_OUT 唤醒后，这个函数继续向下执行。
     */
    QProcess::execute("/root/project/bin/system_suspend.sh");

    qDebug() << "[POWER] Linux resumed";

    afterSystemResume();
}

void MainWindow::onHostSleepDenied()
{
    qDebug() << "[POWER] HOST,SLEEP_DENY received";

    suspendRequested = false;
    powerState = POWER_IDLE;

    if (sensorWorker)
    {
        QMetaObject::invokeMethod(sensorWorker,
                                  "setSleepPrepareMode",
                                  Qt::QueuedConnection,
                                  Q_ARG(bool, false));
    }
}

void MainWindow::afterSystemResume()
{
    qDebug() << "[POWER] after system resume";

    suspendRequested = false;
    powerState = POWER_ACTIVE;

    if (!powerClock.isValid())
        powerClock.start();

    lastHumanActivityMs = powerClock.elapsed();

    pirWakeVerifyMode = true;
    pirWakeStartMs = powerClock.elapsed();

    pirMotion = false;
    currentPersonDetected = false;
    haveTrackTarget = false;
    errorFilterInited = false;
    filteredErrorX = 0.0;
    filteredErrorY = 0.0;

    if (sensorWorker)
    {
        QMetaObject::invokeMethod(sensorWorker,
                                  "sendHostAwake",
                                  Qt::QueuedConnection);
    }

    /*
     * 先恢复摄像头
     */
    QTimer::singleShot(500, this, [this]() {
        qDebug() << "[POWER] restart vision after resume";
        startVisionWorker();
    });

    /*
     * 再恢复检测线程
     */
    QTimer::singleShot(2000, this, [this]() {
        qDebug() << "[POWER] restart detector after resume";
        startDetectorWorker();
    });

    /*
     * 最后恢复 RTMP，给网络几秒恢复时间
     */

    QTimer::singleShot(1000, this, [this]() {
       qDebug() << "[POWER] restore network after resume";
       QProcess::execute("/root/project/bin/resume_network.sh");
    });
     
    QTimer::singleShot(6000, this, [this]() {
        qDebug() << "[POWER] restart RTMP after resume";
        startRtmpStreamer();
    });

    publishMqttStatus();
}

void MainWindow::onHostAwakeAck()
{
    qDebug() << "[POWER] HOST,AWAKE_ACK received";
}
