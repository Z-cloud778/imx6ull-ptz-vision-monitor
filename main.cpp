/*************************************************
 * @File: main.cpp
 * @Author: Z-cloud778
 * @Date: 2026-07-14
*************************************************/
#include <QApplication>
#include "mainwindow.h"

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);

    MainWindow w;
    w.showFullScreen();

    return app.exec();
}