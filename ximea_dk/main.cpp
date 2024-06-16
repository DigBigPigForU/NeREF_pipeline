#include "mywidget.h"
#include <QApplication>
//#define EXPECTED_IMAGES 50
using namespace cv;
using namespace std;
int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    myWidget w;
    //w.showFullScreen();
    w.show();

    return a.exec();
//    //original code
//    const uint32_t deviceCount = k4a::device::get_installed_count();
//    if (deviceCount == 0)
//    {
//        std::cout << "no azure kinect devices detected!" << std::endl;
//    }

//    // PCL Visualizer
//    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Point Cloud Viewer"));

//    // Point Cloud
//    pcl::PointCloud<PointType>::ConstPtr cloud;

//    // Retrieved Point Cloud Callback Function
//    boost::mutex mutex;
//    boost::function<void(const pcl::PointCloud<PointType>::ConstPtr&)> function =
//        [&cloud, &mutex](const pcl::PointCloud<PointType>::ConstPtr& ptr)
//    {
//        boost::mutex::scoped_lock lock(mutex);

//        /* Point Cloud Processing */

//        cloud = ptr->makeShared();
//    };

//    // KinectAzureDKGrabber
//    boost::shared_ptr<pcl::Grabber> grabber =
//        boost::make_shared<pcl::KinectAzureDKGrabber>(0, K4A_DEPTH_MODE_WFOV_2X2BINNED, K4A_IMAGE_FORMAT_COLOR_BGRA32, K4A_COLOR_RESOLUTION_720P);

//    boost::shared_ptr<pcl::KinectAzureDKGrabber> grabber_ = boost::dynamic_pointer_cast<pcl::KinectAzureDKGrabber>(grabber);


//    // Register Callback Function
//    boost::signals2::connection connection = grabber->registerCallback(function);

//    // Start Grabber
//    grabber->start();

//    k4a::calibration calibration = grabber_->getCalibration();
//    k4a_calibration_intrinsic_parameters_t *intrinsics = &calibration.color_camera_calibration.intrinsics.parameters;
//    Eigen::Matrix3f intrinsics_eigen;
//    intrinsics_eigen <<
//        intrinsics->param.fx, 0.0f, intrinsics->param.cx,
//        0.0f, intrinsics->param.fy, intrinsics->param.cy,
//        0.0f, 0.0f, 1.0f;
//    Eigen::Matrix4f extrinsics_eigen = Eigen::Matrix4f::Identity();
//    viewer->setCameraParameters(intrinsics_eigen, extrinsics_eigen);

//    while (!viewer->wasStopped())
//    {
//        // Update Viewer
//        viewer->spinOnce();
//        boost::mutex::scoped_try_lock lock(mutex);
//        if (lock.owns_lock() && cloud)
//        {
//            // Update Point Cloud
//            if (!viewer->updatePointCloud(cloud, "cloud"))
//            {
//                viewer->addPointCloud(cloud, "cloud");
//            }
//        }
//    }

//    // Stop Grabber
//    grabber->stop();

//    // Disconnect Callback Function
//    if (connection.connected())
//    {
//        connection.disconnect();
//    }
//    return 0;
}
