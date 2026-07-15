/*************************************************
 * @File: rtmpstreamer.h
 * @Author: Z-cloud778
 * @Date: 2026-07-14
*************************************************/
#ifndef RTMPSTREAMER_H
#define RTMPSTREAMER_H

#include <QObject>
#include <QProcess>
#include <QTimer>
#include <QMutex>
#include <QElapsedTimer>
#include <QString>
#include <opencv2/opencv.hpp>

Q_DECLARE_METATYPE(cv::Mat)

class RtmpStreamer : public QObject
{
    Q_OBJECT

public:
    explicit RtmpStreamer(QObject *parent = nullptr);
    ~RtmpStreamer();

public slots:
    void startStream();
    void stopStream();
    void updateFrame(cv::Mat frame);

signals:
    void streamStatus(const QString& text);

private slots:
    void sendLatestFrame();
    void onFfmpegErrorOccurred(QProcess::ProcessError error);
    void onFfmpegFinished(int exitCode, QProcess::ExitStatus exitStatus);
    void readFfmpegStderr();

private:
    QProcess *ffmpeg;
    QTimer *sendTimer;

    QMutex frameMutex;
    cv::Mat latestFrame;
    bool hasFrame;

    bool running;

    int width;
    int height;
    int fps;
    int frameIntervalMs;

    QString rtmpUrl;
};

#endif