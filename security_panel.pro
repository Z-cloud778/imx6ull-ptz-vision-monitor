# ==============================================
# Project: Smart Security Monitor
# Author: Z-cloud778
# Date: 2026-07-14
# ==============================================
QT += core gui widgets network

CONFIG += c++11

TARGET = security_panel
TEMPLATE = app

SOURCES += \
    main.cpp \
    mainwindow.cpp \
    sensorworker.cpp \
    visionworker.cpp \
    detectorworker.cpp \
    mqttworker.cpp \
    rtmpstreamer.cpp

HEADERS += \
    mainwindow.h \
    sensorworker.h \
    visionworker.h \
    detectorworker.h \
    mqttworker.h \
    rtmpstreamer.h

INCLUDEPATH += /home/book/yolov8_arm_project/opencv_build/install/include/opencv4
INCLUDEPATH += /home/book/yolov8_arm_project/ncnn_source/src
INCLUDEPATH += /home/book/yolov8_arm_project/build_arm/src

QMAKE_CXXFLAGS += -fopenmp
QMAKE_LFLAGS += -fopenmp

LIBS += /home/book/yolov8_arm_project/build_arm/src/libncnn.a

LIBS += -L/home/book/yolov8_arm_project/opencv_build/install/lib
LIBS += -lopencv_imgproc
LIBS += -lopencv_imgcodecs
LIBS += -lopencv_videoio
LIBS += -lopencv_highgui
LIBS += -lopencv_core
LIBS += -lopencv_tracking
LIBS += -lopencv_video

LIBS += -lpthread -ldl -lm -lrt -latomic -fopenmp