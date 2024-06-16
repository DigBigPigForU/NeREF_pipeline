#include "ximea.h"
#include <QImage>
#include <QDateTime>
#include <QPainter>
#include <QFile>

using namespace cv;
using namespace std;

Ximea::Ximea(int exp, QObject *parent) : QThread(parent)
{
    exposure = exp;
    printf("Opening first camera...\n");
    cam.OpenFirst();

    // Set exposure
    cam.SetExposureTime(exposure); //10000 us = 10 ms
    // Note: The default parameters of each camera might be different in different API versions

    printf("Starting acquisition...\n");
    cam.StartAcquisition();

    printf("First pixel value \n");
    format = cam.GetImageDataFormat();
    
    xi_score = 0;
}

Ximea::~Ximea()
{
    cam.StopAcquisition();
    cam.Close();
}

void Ximea::run()  //获取视频并等待保存数据
{
    count = 0;
    //for (int images = 0; images < EXPECTED_IMAGES; images++)
    while(1)
        {
        cv_mat_image = cam.GetNextImageOcvMat();
        if (format == XI_RAW16 || format == XI_MONO16)
        normalize(cv_mat_image, cv_mat_image, 0, 65536, NORM_MINMAX, -1, Mat()); // 0 - 65536, 16 bit unsigned integer range
        emit ximeaWindowsignal();//请求显示在窗口
//        cv::imshow("Image from camera", cv_mat_image);
//        if (waitKey(20) == ' ')
//            {
////                emit Ximea::saveTifSignal();
//            }
    }
}
