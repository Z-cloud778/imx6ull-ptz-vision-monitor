/*************************************************
 * @File: detectorworker.cpp
 * @Author: Z-cloud778
 * @Date: 2026-07-14
 * @Note: YOLOv8n人体目标检测推理模块，开源已脱敏本地绝对路径
*************************************************/
#include "detectorworker.h"

#include <QDebug>
#include <QElapsedTimer>

#include <opencv2/opencv.hpp>
#include "net.h"

#include <vector>
#include <cmath>
#include <algorithm>
#include <cstring>

// ===================== YOLO 参数 =====================

static const int YOLO_INPUT_W = 320;
static const int YOLO_INPUT_H = 320;
static const int YOLO_REG_MAX = 16;

static const float YOLO_PROB_THRESHOLD = 0.35f;
static const float YOLO_NMS_THRESHOLD = 0.45f;

// 使用者将模型放置项目根目录models文件夹即可
static const char* YOLO_PARAM_PATH = "./models/yolov8n.ncnn.param";
static const char* YOLO_BIN_PATH   = "./models/yolov8n.ncnn.bin";

struct YoloObject
{
    cv::Rect_<float> rect;
    int label;
    float prob;
};

static float sigmoid(float x)
{
    return 1.f / (1.f + expf(-x));
}

static cv::Rect2d clamp_rect_to_image(const cv::Rect2d& r, int img_w, int img_h)
{
    double x1 = std::max(0.0, r.x);
    double y1 = std::max(0.0, r.y);
    double x2 = std::min((double)img_w - 1.0, r.x + r.width);
    double y2 = std::min((double)img_h - 1.0, r.y + r.height);

    if (x2 <= x1 || y2 <= y1)
        return cv::Rect2d();

    return cv::Rect2d(x1, y1, x2 - x1, y2 - y1);
}

static bool is_rect_valid(const cv::Rect2d& r, int img_w, int img_h)
{
    if (r.width < 6 || r.height < 6)
        return false;

    if (r.x < -img_w * 0.2 || r.y < -img_h * 0.2)
        return false;

    if (r.x + r.width > img_w * 1.2 || r.y + r.height > img_h * 1.2)
        return false;

    return true;
}

static void generate_grids_and_stride(int input_w,
                                      int input_h,
                                      std::vector<DetectorGridAndStride>& grid_strides)
{
    grid_strides.clear();

    const int strides[] = {8, 16, 32};

    for (int s = 0; s < 3; s++)
    {
        int stride = strides[s];
        int num_grid_w = input_w / stride;
        int num_grid_h = input_h / stride;

        for (int g1 = 0; g1 < num_grid_h; g1++)
        {
            for (int g0 = 0; g0 < num_grid_w; g0++)
            {
                DetectorGridAndStride gs;
                gs.grid0 = g0;
                gs.grid1 = g1;
                gs.stride = stride;
                grid_strides.push_back(gs);
            }
        }
    }
}

static float dfl_decode(const float* ptr)
{
    float max_v = ptr[0];

    for (int i = 1; i < YOLO_REG_MAX; i++)
    {
        if (ptr[i] > max_v)
            max_v = ptr[i];
    }

    float exp_sum = 0.f;
    float dist = 0.f;

    for (int i = 0; i < YOLO_REG_MAX; i++)
    {
        float e = expf(ptr[i] - max_v);
        exp_sum += e;
        dist += e * i;
    }

    return dist / exp_sum;
}

static cv::Mat letterbox(const cv::Mat& src,
                         int target_w,
                         int target_h,
                         float& scale,
                         float& pad_x,
                         float& pad_y)
{
    scale = std::min(target_w / (float)src.cols,
                     target_h / (float)src.rows);

    int new_w = (int)std::round(src.cols * scale);
    int new_h = (int)std::round(src.rows * scale);

    cv::Mat resized;
    cv::resize(src, resized, cv::Size(new_w, new_h));

    pad_x = (target_w - new_w) * 0.5f;
    pad_y = (target_h - new_h) * 0.5f;

    cv::Mat dst(target_h, target_w, src.type(), cv::Scalar(114, 114, 114));
    resized.copyTo(dst(cv::Rect((int)pad_x, (int)pad_y, new_w, new_h)));

    return dst;
}

static void nms(std::vector<YoloObject>& objects, float iou_threshold)
{
    if (objects.empty())
        return;

    std::sort(objects.begin(), objects.end(),
              [](const YoloObject& a, const YoloObject& b)
              {
                  return a.prob > b.prob;
              });

    std::vector<bool> keep(objects.size(), true);

    for (size_t i = 0; i < objects.size(); i++)
    {
        if (!keep[i])
            continue;

        for (size_t j = i + 1; j < objects.size(); j++)
        {
            if (!keep[j])
                continue;

            float x1 = std::max(objects[i].rect.x, objects[j].rect.x);
            float y1 = std::max(objects[i].rect.y, objects[j].rect.y);
            float x2 = std::min(objects[i].rect.x + objects[i].rect.width,
                                objects[j].rect.x + objects[j].rect.width);
            float y2 = std::min(objects[i].rect.y + objects[i].rect.height,
                                objects[j].rect.y + objects[j].rect.height);

            float inter_w = std::max(0.0f, x2 - x1);
            float inter_h = std::max(0.0f, y2 - y1);
            float inter_area = inter_w * inter_h;

            float area_i = objects[i].rect.width * objects[i].rect.height;
            float area_j = objects[j].rect.width * objects[j].rect.height;

            float iou = inter_area / (area_i + area_j - inter_area + 1e-6f);

            if (iou > iou_threshold)
                keep[j] = false;
        }
    }

    std::vector<YoloObject> filtered;

    for (size_t i = 0; i < objects.size(); i++)
    {
        if (keep[i])
            filtered.push_back(objects[i]);
    }

    objects.swap(filtered);
}

static void decode_yolov8_person(const ncnn::Mat& out,
                                 std::vector<YoloObject>& objects,
                                 const std::vector<DetectorGridAndStride>& grid_strides,
                                 float prob_threshold,
                                 int input_w,
                                 int input_h,
                                 int orig_w,
                                 int orig_h,
                                 float pad_x,
                                 float pad_y,
                                 float scale)
{
    objects.clear();

    if (out.dims != 2 || out.w != 144)
    {
        printf("[YOLO] unexpected out shape: dims=%d w=%d h=%d c=%d\n",
               out.dims,
               out.w,
               out.h,
               out.c);
        return;
    }

    int num_boxes = out.h;

    if ((int)grid_strides.size() != num_boxes)
    {
        printf("[YOLO] grid_strides mismatch: grid=%zu, out=%d\n",
               grid_strides.size(),
               num_boxes);
        return;
    }

    float feat[144];

    for (int i = 0; i < num_boxes; i++)
    {
        const float* ptr = out.row(i);
        memcpy(feat, ptr, 144 * sizeof(float));

        const DetectorGridAndStride& gs = grid_strides[i];

        float cx = (gs.grid0 + 0.5f) * gs.stride;
        float cy = (gs.grid1 + 0.5f) * gs.stride;

        float l = dfl_decode(feat + 0)  * gs.stride;
        float t = dfl_decode(feat + 16) * gs.stride;
        float r = dfl_decode(feat + 32) * gs.stride;
        float b = dfl_decode(feat + 48) * gs.stride;

        float x0 = cx - l;
        float y0 = cy - t;
        float x1 = cx + r;
        float y1 = cy + b;

        /*
         * COCO person 类别为 0。
         * YOLOv8 输出中 feat[64] 开始是类别分数。
         */
        float person_prob = sigmoid(feat[64]);

        if (person_prob < prob_threshold)
            continue;

        x0 = std::max(0.0f, std::min(x0, (float)input_w - 1.0f));
        y0 = std::max(0.0f, std::min(y0, (float)input_h - 1.0f));
        x1 = std::max(0.0f, std::min(x1, (float)input_w - 1.0f));
        y1 = std::max(0.0f, std::min(y1, (float)input_h - 1.0f));

        if (x1 <= x0 || y1 <= y0)
            continue;

        float ox0 = (x0 - pad_x) / scale;
        float oy0 = (y0 - pad_y) / scale;
        float ox1 = (x1 - pad_x) / scale;
        float oy1 = (y1 - pad_y) / scale;

        ox0 = std::max(0.0f, std::min(ox0, (float)orig_w - 1.0f));
        oy0 = std::max(0.0f, std::min(oy0, (float)orig_h - 1.0f));
        ox1 = std::max(0.0f, std::min(ox1, (float)orig_w - 1.0f));
        oy1 = std::max(0.0f, std::min(oy1, (float)orig_h - 1.0f));

        if (ox1 <= ox0 || oy1 <= oy0)
            continue;

        YoloObject obj;
        obj.rect = cv::Rect_<float>(ox0, oy0, ox1 - ox0, oy1 - oy0);
        obj.label = 0;
        obj.prob = person_prob;

        objects.push_back(obj);
    }
}

static int pick_best_person(const std::vector<YoloObject>& objects)
{
    if (objects.empty())
        return -1;

    int best_idx = 0;
    float best_score = objects[0].prob;

    for (size_t i = 1; i < objects.size(); i++)
    {
        if (objects[i].prob > best_score)
        {
            best_score = objects[i].prob;
            best_idx = (int)i;
        }
    }

    return best_idx;
}

static bool yolo_detect_person(ncnn::Net& net,
                               const cv::Mat& frame_bgr,
                               const std::vector<DetectorGridAndStride>& grid_strides,
                               cv::Rect2d& person_box,
                               float& person_score)
{
    float scale = 1.f;
    float pad_x = 0.f;
    float pad_y = 0.f;

    cv::Mat input_img = letterbox(frame_bgr,
                                  YOLO_INPUT_W,
                                  YOLO_INPUT_H,
                                  scale,
                                  pad_x,
                                  pad_y);

    ncnn::Mat in = ncnn::Mat::from_pixels(input_img.data,
                                          ncnn::Mat::PIXEL_BGR,
                                          YOLO_INPUT_W,
                                          YOLO_INPUT_H);

    const float mean_vals[3] = {0.f, 0.f, 0.f};
    const float norm_vals[3] = {
        1.f / 255.f,
        1.f / 255.f,
        1.f / 255.f
    };

    in.substract_mean_normalize(mean_vals, norm_vals);

    ncnn::Extractor ex = net.create_extractor();

    if (ex.input(0, in) != 0)
    {
        printf("[YOLO] input failed\n");
        return false;
    }

    ncnn::Mat out;

    if (ex.extract("out0", out) != 0)
    {
        printf("[YOLO] extract out0 failed\n");
        return false;
    }

    std::vector<YoloObject> objects;

    decode_yolov8_person(out,
                         objects,
                         grid_strides,
                         YOLO_PROB_THRESHOLD,
                         YOLO_INPUT_W,
                         YOLO_INPUT_H,
                         frame_bgr.cols,
                         frame_bgr.rows,
                         pad_x,
                         pad_y,
                         scale);

    nms(objects, YOLO_NMS_THRESHOLD);

    int best_idx = pick_best_person(objects);

    if (best_idx < 0)
        return false;

    person_box = cv::Rect2d(objects[best_idx].rect.x,
                            objects[best_idx].rect.y,
                            objects[best_idx].rect.width,
                            objects[best_idx].rect.height);

    person_box = clamp_rect_to_image(person_box,
                                     frame_bgr.cols,
                                     frame_bgr.rows);

    person_score = objects[best_idx].prob;

    if (!is_rect_valid(person_box, frame_bgr.cols, frame_bgr.rows))
        return false;

    return true;
}

// ===================== DetectorWorker =====================

DetectorWorker::DetectorWorker(QObject *parent)
    : QObject(parent),
      yoloNet(nullptr),
      yoloLoaded(false)
{
}

DetectorWorker::~DetectorWorker()
{
    stop();
}

void DetectorWorker::start()
{
    if (loadYoloModel())
    {
        emit detectorStatus("detector started");
    }
    else
    {
        emit detectorStatus("detector start failed");
    }
}

void DetectorWorker::stop()
{
    if (yoloNet)
    {
        delete yoloNet;
        yoloNet = nullptr;
    }

    yoloLoaded = false;
    gridStrides.clear();

    emit detectorStatus("detector stopped");
}

bool DetectorWorker::loadYoloModel()
{
    if (yoloLoaded)
        return true;

    yoloNet = new ncnn::Net();

    yoloNet->opt.use_vulkan_compute = false;
    yoloNet->opt.num_threads = 1;
    yoloNet->opt.lightmode = true;
    yoloNet->opt.use_packing_layout = true;

    if (yoloNet->load_param(YOLO_PARAM_PATH) != 0)
    {
        emit detectorStatus(QString("load YOLO param failed: %1").arg(YOLO_PARAM_PATH));
        delete yoloNet;
        yoloNet = nullptr;
        return false;
    }

    if (yoloNet->load_model(YOLO_BIN_PATH) != 0)
    {
        emit detectorStatus(QString("load YOLO bin failed: %1").arg(YOLO_BIN_PATH));
        delete yoloNet;
        yoloNet = nullptr;
        return false;
    }

    generate_grids_and_stride(YOLO_INPUT_W,
                              YOLO_INPUT_H,
                              gridStrides);

    if (gridStrides.empty())
    {
        emit detectorStatus("YOLO grid generate failed");
        delete yoloNet;
        yoloNet = nullptr;
        return false;
    }

    yoloLoaded = true;

    emit detectorStatus(QString("YOLO loaded, grid=%1").arg((int)gridStrides.size()));

    return true;
}

void DetectorWorker::processFrame(const QImage& image)
{
    if (!yoloLoaded || !yoloNet)
    {
        emit detectorStatus("YOLO not loaded");
        emit detectionResult(false, 0, 0, 0, 0, 0.0f);
        emit detectorIdle();
        return;
    }

    if (image.isNull())
    {
        emit detectorIdle();
        return;
    }

    QElapsedTimer timer;
    timer.start();

    /*
     * VisionWorker 传来的是 RGB888 QImage。
     * YOLO 这里转成 OpenCV BGR。
     */
    QImage rgb = image.convertToFormat(QImage::Format_RGB888);

    cv::Mat frame_rgb(rgb.height(),
                      rgb.width(),
                      CV_8UC3,
                      const_cast<uchar*>(rgb.constBits()),
                      rgb.bytesPerLine());

    cv::Mat frame_bgr;
    cv::cvtColor(frame_rgb, frame_bgr, cv::COLOR_RGB2BGR);

    cv::Rect2d personBox;
    float personScore = 0.0f;

    bool found = yolo_detect_person(*yoloNet,
                                    frame_bgr,
                                    gridStrides,
                                    personBox,
                                    personScore);

    qint64 costMs = timer.elapsed();

    if (found)
    {
        emit detectionResult(true,
                             (int)personBox.x,
                             (int)personBox.y,
                             (int)personBox.width,
                             (int)personBox.height,
                             personScore);

        emit detectorStatus(QString("person score=%1 cost=%2ms")
                            .arg(personScore, 0, 'f', 2)
                            .arg(costMs));
    }
    else
    {
        emit detectionResult(false, 0, 0, 0, 0, 0.0f);

        emit detectorStatus(QString("person lost cost=%1ms")
                            .arg(costMs));
    }

    emit detectorIdle();
}