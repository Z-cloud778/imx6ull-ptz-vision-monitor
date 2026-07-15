/*************************************************
 * @File: visionworker.h
 * @Author: Z-cloud778
 * @Date: 2026-07-14
*************************************************/

#ifndef VISIONWORKER_H
#define VISIONWORKER_H

#include <QObject>
#include <QString>
#include <QImage>
#include <QTimer>
#include <QVector>
#include <QElapsedTimer>

#include <stddef.h>

#include <opencv2/core.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/tracking/tracking_legacy.hpp>

class VisionWorker : public QObject
{
    Q_OBJECT

public:
    explicit VisionWorker(const QString& dev = "/dev/video1",
                          int w = 320,
                          int h = 240,
                          QObject *parent = nullptr);
    ~VisionWorker();

public slots:
    void start();
    void stop();

    /*
     * DetectorWorker 检测到人以后，
     * MainWindow 会把 YOLO 框传给 VisionWorker，
     * VisionWorker 用这个框初始化 MOSSE。
     */
    void initTrackerFromDetection(bool detected,
                                  int x,
                                  int y,
                                  int w,
                                  int h,
                                  float score);

signals:
    void frameReady(const QImage& image);
    void detectorFrameReady(const QImage& image);
    void visionStatus(const QString& text);

    /*
     * MOSSE 每帧跟踪后的结果。
     */
    void trackingResult(bool tracked,
                        int x,
                        int y,
                        int w,
                        int h,
                        float score);

private:
    struct Buffer
    {
        void *start;
        size_t length;
    };

private:
    bool openCamera();
    bool initCamera();
    bool startStream();
    void stopStream();
    void closeCamera();

    void captureOnce();
    int xioctl(int request, void *arg);

    QImage yuyvToImage(const unsigned char *yuyv, int width, int height);
    int clampColor(int v);

private:
    QString devName;
    int fd;

    int width;
    int height;

    QVector<Buffer> buffers;
    QTimer *captureTimer;
    bool streaming;

    QElapsedTimer visionClock;
    qint64 lastDetectorSubmitMs;

     /*
     * MOSSE 跟踪相关变量。
     */
    bool pendingTrackerInit;
    cv::Rect2d pendingBox;
    float pendingScore;

    bool trackingActive;
    cv::Ptr<cv::legacy::TrackerMOSSE> mosseTracker;
    cv::Rect2d trackBox;
    float trackScore;
    int trackLostCount;
};

#endif