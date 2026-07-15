/*************************************************
 * @File: visionworker.cpp
 * @Author: Z-cloud778
 * @Date: 2026-07-14
*************************************************/

#define _DEFAULT_SOURCE

#include "visionworker.h"

#include <QDebug>

#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <stdio.h>

#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/select.h>

#include <linux/videodev2.h>

static const int DETECTOR_SUBMIT_INTERVAL_TRACKING_MS = 8000;

/*
 * 没有目标时，每隔一段时间提交一帧给 YOLO。
 * 因为 MainWindow 有 detectorBusy 判断，所以不会堆积旧帧。
 */
static const int DETECTOR_SUBMIT_INTERVAL_MS = 300;

/*
 * MOSSE 连续失败几次后认为目标丢失。
 */
static const int TRACK_LOST_MAX_COUNT = 2;

static cv::Rect2d clampTrackBox(const cv::Rect2d& box, int imgW, int imgH)
{
    double x1 = std::max(0.0, box.x);
    double y1 = std::max(0.0, box.y);
    double x2 = std::min((double)imgW - 1.0, box.x + box.width);
    double y2 = std::min((double)imgH - 1.0, box.y + box.height);

    if (x2 <= x1 || y2 <= y1)
        return cv::Rect2d();

    return cv::Rect2d(x1, y1, x2 - x1, y2 - y1);
}

static bool isTrackBoxValid(const cv::Rect2d& box, int imgW, int imgH)
{
    if (box.width < 8 || box.height < 8)
        return false;

    if (box.x < 0 || box.y < 0)
        return false;

    if (box.x + box.width > imgW)
        return false;

    if (box.y + box.height > imgH)
        return false;

    return true;
}

/*
 * 从 YUYV 图像中直接提取 Y 分量作为灰度图。
 * MOSSE 用灰度图即可，不需要每帧转 BGR，速度更快。
 */
static cv::Mat yuyvToGrayMat(const unsigned char *yuyv, int w, int h)
{
    cv::Mat gray(h, w, CV_8UC1);

    for (int y = 0; y < h; y++)
    {
        unsigned char *dst = gray.ptr<unsigned char>(y);
        const unsigned char *src = yuyv + y * w * 2;

        for (int x = 0; x < w; x++)
        {
            dst[x] = src[x * 2];
        }
    }

    return gray;
}

VisionWorker::VisionWorker(const QString& dev, int w, int h, QObject *parent)
    : QObject(parent),
      devName(dev),
      fd(-1),
      width(w),
      height(h),
      captureTimer(nullptr),
      streaming(false),
      lastDetectorSubmitMs(-100000),
      pendingTrackerInit(false),
      pendingScore(0.0f),
      trackingActive(false),
      trackScore(0.0f),
      trackLostCount(0)
{
}

VisionWorker::~VisionWorker()
{
    stop();
}

int VisionWorker::xioctl(int request, void *arg)
{
    int r;

    do
    {
        r = ioctl(fd, request, arg);
    }
    while (r == -1 && errno == EINTR);

    return r;
}

bool VisionWorker::openCamera()
{
    fd = ::open(devName.toLocal8Bit().constData(), O_RDWR | O_NONBLOCK, 0);

    if (fd < 0)
    {
        emit visionStatus(QString("open camera failed: %1").arg(strerror(errno)));
        return false;
    }

    emit visionStatus(QString("camera open OK: %1").arg(devName));
    return true;
}

bool VisionWorker::initCamera()
{
    struct v4l2_capability cap;
    memset(&cap, 0, sizeof(cap));

    if (xioctl(VIDIOC_QUERYCAP, &cap) < 0)
    {
        emit visionStatus(QString("VIDIOC_QUERYCAP failed: %1").arg(strerror(errno)));
        return false;
    }

    if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE))
    {
        emit visionStatus("device is not video capture");
        return false;
    }

    if (!(cap.capabilities & V4L2_CAP_STREAMING))
    {
        emit visionStatus("device does not support streaming");
        return false;
    }

    struct v4l2_format fmt;
    memset(&fmt, 0, sizeof(fmt));

    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = width;
    fmt.fmt.pix.height = height;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
    fmt.fmt.pix.field = V4L2_FIELD_NONE;

    if (xioctl(VIDIOC_S_FMT, &fmt) < 0)
    {
        emit visionStatus(QString("VIDIOC_S_FMT failed: %1").arg(strerror(errno)));
        return false;
    }

    width = fmt.fmt.pix.width;
    height = fmt.fmt.pix.height;

    emit visionStatus(QString("camera format: %1x%2 YUYV").arg(width).arg(height));

    struct v4l2_requestbuffers req;
    memset(&req, 0, sizeof(req));

    req.count = 4;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;

    if (xioctl(VIDIOC_REQBUFS, &req) < 0)
    {
        emit visionStatus(QString("VIDIOC_REQBUFS failed: %1").arg(strerror(errno)));
        return false;
    }

    if (req.count < 2)
    {
        emit visionStatus("not enough V4L2 buffers");
        return false;
    }

    buffers.resize(req.count);

    for (unsigned int i = 0; i < req.count; i++)
    {
        struct v4l2_buffer buf;
        memset(&buf, 0, sizeof(buf));

        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;

        if (xioctl(VIDIOC_QUERYBUF, &buf) < 0)
        {
            emit visionStatus(QString("VIDIOC_QUERYBUF failed: %1").arg(strerror(errno)));
            return false;
        }

        buffers[(int)i].length = buf.length;
        buffers[(int)i].start = mmap(NULL,
                                     buf.length,
                                     PROT_READ | PROT_WRITE,
                                     MAP_SHARED,
                                     fd,
                                     buf.m.offset);

        if (buffers[(int)i].start == MAP_FAILED)
        {
            emit visionStatus(QString("mmap failed: %1").arg(strerror(errno)));
            return false;
        }
    }

    for (int i = 0; i < buffers.size(); i++)
    {
        struct v4l2_buffer buf;
        memset(&buf, 0, sizeof(buf));

        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;

        if (xioctl(VIDIOC_QBUF, &buf) < 0)
        {
            emit visionStatus(QString("VIDIOC_QBUF failed: %1").arg(strerror(errno)));
            return false;
        }
    }

    return true;
}

bool VisionWorker::startStream()
{
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    if (xioctl(VIDIOC_STREAMON, &type) < 0)
    {
        emit visionStatus(QString("VIDIOC_STREAMON failed: %1").arg(strerror(errno)));
        return false;
    }

    streaming = true;
    emit visionStatus("camera stream ON");
    return true;
}

void VisionWorker::stopStream()
{
    if (fd >= 0 && streaming)
    {
        enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        xioctl(VIDIOC_STREAMOFF, &type);
        streaming = false;
    }

    for (int i = 0; i < buffers.size(); i++)
    {
        if (buffers[i].start && buffers[i].start != MAP_FAILED)
        {
            munmap(buffers[i].start, buffers[i].length);
            buffers[i].start = nullptr;
            buffers[i].length = 0;
        }
    }

    buffers.clear();
}

void VisionWorker::closeCamera()
{
    if (fd >= 0)
    {
        ::close(fd);
        fd = -1;
    }
}

void VisionWorker::start()
{
    if (!openCamera())
        return;

    if (!initCamera())
    {
        closeCamera();
        return;
    }

    if (!startStream())
    {
        stopStream();
        closeCamera();
        return;
    }

    visionClock.start();
    lastDetectorSubmitMs = -100000;

    captureTimer = new QTimer(this);
    connect(captureTimer, &QTimer::timeout, this, &VisionWorker::captureOnce);

    /*
     * 50ms 采集一次，约 20fps。
     * 这里不跑 YOLO，所以显示线程不会被 YOLO 直接卡死。
     */
    captureTimer->start(50);

    emit visionStatus("vision worker started");
}

void VisionWorker::stop()
{
    if (captureTimer)
    {
        captureTimer->stop();
        captureTimer->deleteLater();
        captureTimer = nullptr;
    }

    stopStream();
    closeCamera();

    emit visionStatus("vision worker stopped");
}

void VisionWorker::captureOnce()
{
    if (fd < 0 || !streaming)
        return;

    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(fd, &fds);

    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 0;

    int r = select(fd + 1, &fds, NULL, NULL, &tv);

    if (r <= 0)
        return;

    struct v4l2_buffer buf;
    memset(&buf, 0, sizeof(buf));

    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;

    if (xioctl(VIDIOC_DQBUF, &buf) < 0)
    {
        if (errno != EAGAIN)
        {
            emit visionStatus(QString("VIDIOC_DQBUF failed: %1").arg(strerror(errno)));
        }

        return;
    }

    if ((int)buf.index < buffers.size())
    {
        unsigned char *data = static_cast<unsigned char *>(buffers[(int)buf.index].start);

        /*
         * 1. 转成 QImage，给 Qt 显示用。
         */
        QImage image = yuyvToImage(data, width, height);

        /*
         * 2. 如果需要初始化/更新 MOSSE，再提取灰度图。
         *    普通显示帧不跑 YOLO，不做 BGR 转换。
         */
        bool needGray = pendingTrackerInit || trackingActive;
        cv::Mat gray;

        if (needGray)
        {
            gray = yuyvToGrayMat(data, width, height);
        }

        /*
         * 3. YOLO 检测到人后，用下一帧初始化 MOSSE。
         */
        if (pendingTrackerInit && !gray.empty())
        {
            mosseTracker = cv::legacy::TrackerMOSSE::create();

            trackBox = pendingBox;
            trackScore = pendingScore;
            trackLostCount = 0;

            mosseTracker->init(gray, trackBox);

            trackingActive = true;
            pendingTrackerInit = false;

            emit visionStatus("MOSSE tracking started");

            emit trackingResult(true,
                                (int)trackBox.x,
                                (int)trackBox.y,
                                (int)trackBox.width,
                                (int)trackBox.height,
                                trackScore);
        }
        /*
         * 4. MOSSE 已经启动后，每帧快速更新目标框。
         */
        else if (trackingActive && mosseTracker && !gray.empty())
        {
            cv::Rect2d newBox = trackBox;

            bool ok = mosseTracker->update(gray, newBox);

            newBox = clampTrackBox(newBox, width, height);

            if (ok && isTrackBoxValid(newBox, width, height))
            {
                trackBox = newBox;
                trackLostCount = 0;

                emit trackingResult(true,
                                    (int)trackBox.x,
                                    (int)trackBox.y,
                                    (int)trackBox.width,
                                    (int)trackBox.height,
                                    trackScore);
            }
            else
            {
                trackLostCount++;

                if (trackLostCount >= TRACK_LOST_MAX_COUNT)
                {
                    trackingActive = false;
                    pendingTrackerInit = false;
                    mosseTracker.release();

                    trackBox = cv::Rect2d();
                    trackScore = 0.0f;

                    emit visionStatus("MOSSE tracking lost");

                    emit trackingResult(false, 0, 0, 0, 0, 0.0f);
                }
            }
        }

        /*
         * 5. 没有跟踪目标时，才提交图像给 DetectorWorker 跑 YOLO。
         *    有目标时不提交，避免 YOLO 一直占 CPU。
         */
        if (!pendingTrackerInit && !image.isNull())
	   {
    		  qint64 nowMs = visionClock.isValid() ? visionClock.elapsed() : 0;

    		  int interval = trackingActive ?
                   		  DETECTOR_SUBMIT_INTERVAL_TRACKING_MS :
                   		  DETECTOR_SUBMIT_INTERVAL_MS;

    		  if (nowMs - lastDetectorSubmitMs >= interval)
    		  {
        		 lastDetectorSubmitMs = nowMs;
        		 emit detectorFrameReady(image.copy());
      	  }
	   }

        /*
         * 6. 每一帧都给 MainWindow 显示。
         *    MainWindow 根据 trackingResult 画框。
         */
        if (!image.isNull())
        {
            emit frameReady(image);
        }
    }

    if (xioctl(VIDIOC_QBUF, &buf) < 0)
    {
        emit visionStatus(QString("VIDIOC_QBUF failed: %1").arg(strerror(errno)));
    }
}

int VisionWorker::clampColor(int v)
{
    if (v < 0)
        return 0;

    if (v > 255)
        return 255;

    return v;
}

QImage VisionWorker::yuyvToImage(const unsigned char *yuyv, int w, int h)
{
    if (!yuyv || w <= 0 || h <= 0)
        return QImage();

    QImage img(w, h, QImage::Format_RGB888);

    for (int y = 0; y < h; y++)
    {
        unsigned char *dst = img.scanLine(y);
        const unsigned char *src = yuyv + y * w * 2;

        for (int x = 0; x < w; x += 2)
        {
            int y0 = src[0];
            int u  = src[1];
            int y1 = src[2];
            int v  = src[3];

            int c0 = y0 - 16;
            int c1 = y1 - 16;
            int d = u - 128;
            int e = v - 128;

            int r0 = clampColor((298 * c0 + 409 * e + 128) >> 8);
            int g0 = clampColor((298 * c0 - 100 * d - 208 * e + 128) >> 8);
            int b0 = clampColor((298 * c0 + 516 * d + 128) >> 8);

            int r1 = clampColor((298 * c1 + 409 * e + 128) >> 8);
            int g1 = clampColor((298 * c1 - 100 * d - 208 * e + 128) >> 8);
            int b1 = clampColor((298 * c1 + 516 * d + 128) >> 8);

            dst[0] = (unsigned char)r0;
            dst[1] = (unsigned char)g0;
            dst[2] = (unsigned char)b0;

            dst[3] = (unsigned char)r1;
            dst[4] = (unsigned char)g1;
            dst[5] = (unsigned char)b1;

            src += 4;
            dst += 6;
        }
    }

    return img;
}

void VisionWorker::initTrackerFromDetection(bool detected,
                                            int x,
                                            int y,
                                            int w,
                                            int h,
                                            float score)
{
    if (!detected)
        return;

    cv::Rect2d box(x, y, w, h);

    /*
     * YOLO 框通常包含一些背景。
     * MOSSE 初始化时稍微缩小框，可以减少背景干扰。
     */
    double shrinkX = box.width * 0.10;
    double shrinkY = box.height * 0.08;
 
    box.x += shrinkX;
    box.y += shrinkY;
    box.width -= 2.0 * shrinkX;
    box.height -= 2.0 * shrinkY;

    box = clampTrackBox(box, width, height);

    if (!isTrackBoxValid(box, width, height))
    {
        emit visionStatus("MOSSE init failed: invalid YOLO box");
        return;
    }

    /*
     * 不在这里直接 init。
     * 因为这里不一定正好有当前帧图像。
     * 所以先保存 YOLO 框，等下一帧 captureOnce() 中用当前帧初始化 MOSSE。
     */
    pendingBox = box;
    pendingScore = score;
    pendingTrackerInit = true;

    emit visionStatus(QString("MOSSE pending init, score=%1").arg(score, 0, 'f', 2));
}