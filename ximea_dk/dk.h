#ifndef DK_H
#define DK_H
#include <vtkAutoInit.h>
VTK_MODULE_INIT(vtkRenderingOpenGL)
VTK_MODULE_INIT(vtkInteractionStyle)

#include <memory.h>
#include <QTimer>
#include <QVector>
#include <QRgb>
#include <QDir>
#include <QProcess>
#include <QThread>
#include <QDebug>
#include <k4a/k4a.hpp>
#include <fstream>
#include <iostream>
#include <vector>
#include <array>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>
#include "k4a_grabber.h"

using namespace std;
using namespace cv;
using namespace boost;
using namespace pcl;

typedef pcl::PointXYZRGBA PointType;

class dk : public QThread
{
    Q_OBJECT
public:
    explicit dk(QObject *parent = 0);
    friend class dkSave;
    friend class pcSave;
    friend class pcProc;
    ~dk();
protected:
    void run();
signals:
    void dkWindowsignal();
//    void dkScore(double score);
public:
    uint32_t deviceCount;
    k4a_device_configuration_t config;
    k4a::device device;
    k4a::calibration calibration;
    k4a::transformation transformation;
    uint8_t *colorTextureBuffer;
    uint8_t *t_colorTextureBuffer;
    int16_t *ct_point_cloud_image_data;
    int16_t *dt_point_cloud_image_data;
    k4a::capture capture;

    k4a::image depthImage;
    k4a::image colorImage;
    k4a::image transformed_color_image;
    k4a::image transformed_depth_image;
    k4a::image ct_point_cloud_image;
    k4a::image dt_point_cloud_image;

    cv::Mat my_ct_xyzFrame;
    cv::Mat my_dt_xyzFrame;
    cv::Mat colorFrame;
    cv::Mat t_colorFrame;
    cv::Mat my_ct_FloatFrame;
    cv::Mat my_dt_FloatFrame;
    cv::Mat my_ct_zFrame;
    cv::Mat my_dt_zFrame;
    cv::Mat my_ct_zFloatFrame;
    cv::Mat my_dt_zFloatFrame;

    int i;
    int i_pc;
    string colorStr;
    string t_colorStr;
    string dt_xyzStr;
    string ct_xyzStr;
    string dt_fStr;
    string ct_fStr;
    string pcStr;

    vector<Mat>ct_channels;
    vector<Mat>dt_channels;
    vector<Mat>ct_Fchannels;
    vector<Mat>dt_Fchannels;
    
    double dk_score;

    //pcl module
//    // PCL Visualizer
//    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    // Point Cloud
    pcl::PointCloud<PointType> dk_cloud;
//    // Retrieved Point Cloud Callback Function
//    boost::mutex mutex;
//    boost::function<void(const pcl::PointCloud<PointType>::ConstPtr&)> function;
//    // KinectAzureDKGrabber
//    boost::shared_ptr<pcl::Grabber> grabber;
//    boost::shared_ptr<pcl::KinectAzureDKGrabber> grabber_;
//    // Register Callback Function
//    boost::signals2::connection connection;
//    k4a_calibration_intrinsic_parameters_t *intrinsics;
//    Eigen::Matrix3f intrinsics_eigen;
//    Eigen::Matrix4f extrinsics_eigen;
};

#endif // XIMEA_H
