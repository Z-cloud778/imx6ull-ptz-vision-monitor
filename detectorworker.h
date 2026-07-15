/*************************************************
 * @File: detectorworker.h
 * @Author: Z-cloud778
 * @Date: 2026-07-14
 * @Note: YOLOv8 人体检测推理模块头文件
*************************************************/
#ifndef DETECTORWORKER_H
#define DETECTORWORKER_H

#include <QObject>
#include <QString>
#include <QImage>

#include <vector>

namespace ncnn
{
    class Net;
}

struct DetectorGridAndStride
{
    int grid0;
    int grid1;
    int stride;
};

class DetectorWorker : public QObject
{
    Q_OBJECT

public:
    explicit DetectorWorker(QObject *parent = nullptr);
    ~DetectorWorker();

public slots:
    void start();
    void stop();
    void processFrame(const QImage& image);

signals:
    void detectorStatus(const QString& text);

    /*
     * detected：是否检测到人
     * x/y/w/h：人体框，坐标对应原始摄像头图像尺寸，例如 320x240
     * score：置信度
     */
    void detectionResult(bool detected,
                         int x,
                         int y,
                         int w,
                         int h,
                         float score);

    /*
     * 告诉 MainWindow：当前 YOLO 推理结束，可以接收下一帧了。
     */
    void detectorIdle();

private:
    bool loadYoloModel();

private:
    ncnn::Net *yoloNet;
    bool yoloLoaded;

    std::vector<DetectorGridAndStride> gridStrides;
};

#endif