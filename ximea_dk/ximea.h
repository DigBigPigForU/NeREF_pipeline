#ifndef XIMEA_H
#define XIMEA_H

#include <stdio.h>
#include <fstream>
#include <iostream>
#include <vector>
#include <array>
#include <xiApiPlusOcv.hpp>
#include <QThread>
#include <QDebug>
#include <xiApi.h>
#include <memory.h>
#include <QTimer>
#include <QVector>
#include <QRgb>
#include <QProcess>

#define HandleResult(res,place) if (res!=XI_OK) {printf("Error after %s (%d)\n",place,res);}
#define VIDEO 0

class Ximea : public QThread
{
    Q_OBJECT
public:
    explicit Ximea(int exp,QObject *parent = 0);
    ~Ximea();
protected:
    void run();
signals:
    void ximeaWindowsignal();
//    void ximeaScore(double score);
public:
    int exposure;
    xiAPIplusCameraOcv cam;
    int count;
    XI_IMG_FORMAT format;
    cv::Mat cv_mat_image;
    double xi_score;
};

#endif // XIMEA_H
