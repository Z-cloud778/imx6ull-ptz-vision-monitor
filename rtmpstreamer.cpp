/*************************************************
 * @File: rtmpstreamer.cpp
 * @Author: Z-cloud778
 * @Date: 2026-07-14
*************************************************/
#include "rtmpstreamer.h"
#include <QDebug>

RtmpStreamer::RtmpStreamer(QObject *parent)
    : QObject(parent),
      ffmpeg(nullptr),
      sendTimer(nullptr),
      hasFrame(false),
      running(false),
      width(320),
      height(240),
      fps(10),
      frameIntervalMs(125),
      rtmpUrl("rtmp://192.168.5.11/live/home001")
{
}

RtmpStreamer::~RtmpStreamer()
{
    stopStream();
}

void RtmpStreamer::startStream()
{
    if (running)
        return;

    ffmpeg = new QProcess(this);
    sendTimer = new QTimer(this);

    connect(sendTimer, &QTimer::timeout,
            this, &RtmpStreamer::sendLatestFrame);

    connect(ffmpeg,
            SIGNAL(error(QProcess::ProcessError)),
            this,
            SLOT(onFfmpegErrorOccurred(QProcess::ProcessError)));

    connect(ffmpeg,
            SIGNAL(finished(int,QProcess::ExitStatus)),
            this,
            SLOT(onFfmpegFinished(int,QProcess::ExitStatus)));

    connect(ffmpeg, &QProcess::readyReadStandardError,
            this, &RtmpStreamer::readFfmpegStderr);

    QStringList args;

    args << "-f" << "rawvideo";
    args << "-pixel_format" << "bgr24";
    args << "-video_size" << QString("%1x%2").arg(width).arg(height);
    args << "-framerate" << QString::number(fps);
    args << "-i" << "pipe:0";

    args << "-vf" << "format=yuv420p";
    args << "-c:v" << "libopenh264";
    args << "-b:v" << "200k";
    args << "-maxrate" << "200k";
    args << "-bufsize" << "200k";

    args << "-g" << QString::number(fps);
    args << "-bf" << "0";
    args << "-an";
    args << "-f" << "flv";
    args << rtmpUrl;

    emit streamStatus("Starting FFmpeg RTMP stream...");
    qDebug() << "[RTMP] ffmpeg" << args;

    ffmpeg->start("ffmpeg", args);

    if (!ffmpeg->waitForStarted(3000))
    {
        emit streamStatus("FFmpeg start failed");
        qDebug() << "[RTMP] FFmpeg start failed";
        ffmpeg->deleteLater();
        ffmpeg = nullptr;

        sendTimer->deleteLater();
        sendTimer = nullptr;
        return;
    }

    running = true;
    sendTimer->start(frameIntervalMs);

    emit streamStatus("RTMP stream started");
}

void RtmpStreamer::stopStream()
{
    running = false;

    if (sendTimer)
    {
        sendTimer->stop();
        sendTimer->deleteLater();
        sendTimer = nullptr;
    }

    if (ffmpeg)
    {
        ffmpeg->closeWriteChannel();
        ffmpeg->terminate();

        if (!ffmpeg->waitForFinished(1500))
        {
            ffmpeg->kill();
            ffmpeg->waitForFinished(1000);
        }

        ffmpeg->deleteLater();
        ffmpeg = nullptr;
    }

    QMutexLocker locker(&frameMutex);
    latestFrame.release();
    hasFrame = false;

    emit streamStatus("RTMP stream stopped");
}

void RtmpStreamer::updateFrame(cv::Mat frame)
{
    if (frame.empty())
        return;

    cv::Mat resized;

    if (frame.cols != width || frame.rows != height)
    {
        cv::resize(frame, resized, cv::Size(width, height));
    }
    else
    {
        resized = frame;
    }

    if (resized.type() != CV_8UC3)
        return;

    QMutexLocker locker(&frameMutex);
    latestFrame = resized.clone();
    hasFrame = true;
}

void RtmpStreamer::sendLatestFrame()
{
    if (!running || !ffmpeg)
        return;

    if (ffmpeg->state() != QProcess::Running)
        return;

    const qint64 oneFrameBytes = width * height * 3;

    if (ffmpeg->bytesToWrite() > oneFrameBytes * 3)
    {
        qDebug() << "[RTMP] drop frame, ffmpeg pipe backlog="
                 << ffmpeg->bytesToWrite();
        return;
    }

    cv::Mat frame;

    {
        QMutexLocker locker(&frameMutex);
        if (!hasFrame || latestFrame.empty())
            return;
        frame = latestFrame.clone();
    }

    if (!frame.isContinuous())
        frame = frame.clone();

    qint64 written = ffmpeg->write(
                reinterpret_cast<const char*>(frame.data),
                oneFrameBytes);

    if (written != oneFrameBytes)
    {
        qDebug() << "[RTMP] write incomplete"
                 << written
                 << "expected"
                 << oneFrameBytes;
    }
}

void RtmpStreamer::onFfmpegErrorOccurred(QProcess::ProcessError error)
{
    qDebug() << "[RTMP] FFmpeg process error:" << error;
    emit streamStatus(QString("FFmpeg error: %1").arg(error));
}

void RtmpStreamer::onFfmpegFinished(int exitCode,
                                    QProcess::ExitStatus exitStatus)
{
    qDebug() << "[RTMP] FFmpeg finished"
             << "exitCode=" << exitCode
             << "exitStatus=" << exitStatus;

    running = false;

    if (sendTimer)
        sendTimer->stop();

    emit streamStatus("FFmpeg exited");
}

void RtmpStreamer::readFfmpegStderr()
{
    if (!ffmpeg)
        return;

    QByteArray data = ffmpeg->readAllStandardError();

    if (!data.isEmpty())
    {
        qDebug() << "[FFMPEG]" << QString::fromLocal8Bit(data);
    }
}