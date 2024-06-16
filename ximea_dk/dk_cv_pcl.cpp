#include "dk.h"

dk::dk(QObject *parent) : QThread(parent)
{
    i=0;
    i_pc=0;
    dk_score = 0;
}

dk::~dk()
{

}

void dk::run()  //图像获取在线程中，发送求显示信号
{   
    const uint32_t deviceCount = k4a::device::get_installed_count();
    if (deviceCount == 0)
    {
        std::cout << "no azure kinect devices detected!" << std::endl;
    }

    // PCL Visualizer
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Point Cloud Viewer"));

    // Point Cloud
    pcl::PointCloud<PointType>::ConstPtr cloud;

    // Retrieved Point Cloud Callback Function
    boost::mutex mutex;
    boost::function<void(const pcl::PointCloud<PointType>::ConstPtr&)> function =
        [&cloud, &mutex](const pcl::PointCloud<PointType>::ConstPtr& ptr)
    {
        boost::mutex::scoped_lock lock(mutex);

        /* Point Cloud Processing */

        cloud = ptr->makeShared();
    };

    // KinectAzureDKGrabber
    config.camera_fps = K4A_FRAMES_PER_SECOND_30;
    config.depth_mode = K4A_DEPTH_MODE_WFOV_2X2BINNED;
    config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
    config.color_resolution = K4A_COLOR_RESOLUTION_720P;
    boost::shared_ptr<pcl::Grabber> grabber =
        boost::make_shared<pcl::KinectAzureDKGrabber>(0, config.depth_mode, config.color_format, config.color_resolution);

    boost::shared_ptr<pcl::KinectAzureDKGrabber> grabber_ = boost::dynamic_pointer_cast<pcl::KinectAzureDKGrabber>(grabber);


    // Register Callback Function
    boost::signals2::connection connection = grabber->registerCallback(function);

    // Start Grabber
    grabber->start();

    k4a::calibration calibration = grabber_->getCalibration();
    k4a_calibration_intrinsic_parameters_t *intrinsics = &calibration.color_camera_calibration.intrinsics.parameters;
    Eigen::Matrix3f intrinsics_eigen;
    intrinsics_eigen <<
        intrinsics->param.fx, 0.0f, intrinsics->param.cx,
        0.0f, intrinsics->param.fy, intrinsics->param.cy,
        0.0f, 0.0f, 1.0f;
    Eigen::Matrix4f extrinsics_eigen = Eigen::Matrix4f::Identity();
    viewer->setCameraParameters(intrinsics_eigen, extrinsics_eigen);

    while (!viewer->wasStopped())
    {
        // Update Viewer
        viewer->spinOnce();
        boost::mutex::scoped_try_lock lock(mutex);
        if (lock.owns_lock() && cloud)
        {
            // Update Point Cloud
            if (!viewer->updatePointCloud(cloud, "cloud"))
            {
                viewer->addPointCloud(cloud, "cloud");
            }
        }
          dk_cloud = grabber_->getPointCloud();
//        dk_cloud = *cloud;
//        dk_cloud = *static_cast<pcl::PointCloud<PointType>::Ptr>(cloud.get());
//        dk_cloud = *(cloud.get());
//        dk_cloud = *cloud->makeShared();
        if (0) //image process? yes 1 no 0
        {
                depthImage = grabber_->getDepthImage();
                colorImage = grabber_->getColorImage();

                if (!depthImage.is_valid() || !colorImage.is_valid() )
                {
                    continue;
                }

                int depth_image_width_pixels = depthImage.get_width_pixels();
                int depth_image_height_pixels = depthImage.get_height_pixels();
                int color_image_width_pixels = colorImage.get_width_pixels();
                int color_image_height_pixels = colorImage.get_height_pixels();

                transformation = k4a::transformation(grabber_->getCalibration());

                transformed_color_image = NULL;
                transformed_color_image = k4a::image::create(K4A_IMAGE_FORMAT_COLOR_BGRA32,
                    depth_image_width_pixels,
                    depth_image_height_pixels,
                    depth_image_width_pixels * 4 * (int)sizeof(uint8_t));
                transformation.color_image_to_depth_camera(depthImage, colorImage, &transformed_color_image);

                transformed_depth_image = NULL;
                transformed_depth_image = k4a::image::create(K4A_IMAGE_FORMAT_DEPTH16,
                    color_image_width_pixels,
                    color_image_height_pixels,
                    color_image_width_pixels * (int)sizeof(uint16_t));
                transformation.depth_image_to_color_camera(depthImage,&transformed_depth_image);

                ct_point_cloud_image = NULL;
                ct_point_cloud_image = k4a::image::create(K4A_IMAGE_FORMAT_CUSTOM,
                    depth_image_width_pixels,
                    depth_image_height_pixels,
                    depth_image_width_pixels * 3 * (int)sizeof(int16_t));
                transformation.depth_image_to_point_cloud(depthImage, K4A_CALIBRATION_TYPE_DEPTH, &ct_point_cloud_image);

                dt_point_cloud_image = NULL;
                dt_point_cloud_image = k4a::image::create(K4A_IMAGE_FORMAT_CUSTOM,
                    color_image_width_pixels,
                    color_image_height_pixels,
                    color_image_width_pixels * 3 * (int)sizeof(int16_t));
                transformation.depth_image_to_point_cloud(transformed_depth_image, K4A_CALIBRATION_TYPE_COLOR, &dt_point_cloud_image);


                colorTextureBuffer = colorImage.get_buffer();
                t_colorTextureBuffer = transformed_color_image.get_buffer();

                ct_point_cloud_image_data = (int16_t *)(void *)ct_point_cloud_image.get_buffer();
                dt_point_cloud_image_data = (int16_t *)(void *)dt_point_cloud_image.get_buffer();

                int ct_width = ct_point_cloud_image.get_width_pixels();
                int ct_height = ct_point_cloud_image.get_height_pixels();
                int dt_width = dt_point_cloud_image.get_width_pixels();
                int dt_height = dt_point_cloud_image.get_height_pixels();
                //cout << ct_width <<"*"<< ct_height << " " << dt_width << "*" << dt_height << endl;
                //640*576  1280*720

                colorFrame = cv::Mat(colorImage.get_height_pixels(), colorImage.get_width_pixels(), CV_8UC4, colorTextureBuffer);
                t_colorFrame = cv::Mat(transformed_color_image.get_height_pixels(), transformed_color_image.get_width_pixels(), CV_8UC4, t_colorTextureBuffer);

                int ct = 0;
                cv::Mat ct_xyzFrame(ct_height, ct_width, CV_16SC3);

                for (int i = 0; i < ct_xyzFrame.rows; i++) {
                    for (int j = 0; j < ct_xyzFrame.cols; j++) {
                        for(int z = 0;z<3;z++)
                        {
                            ct_xyzFrame.at<Vec3s>(i, j)[z] = ct_point_cloud_image_data[3 * ct + z];
                        }
                        ++ct;
                    }
                }
                cv::Mat ct_FloatFrame(ct_height, ct_width, CV_32FC3);
                ct_xyzFrame.convertTo(ct_FloatFrame, CV_32FC3, 0.001, 0.000);

                int dt = 0;
                cv::Mat dt_xyzFrame(dt_height, dt_width, CV_16SC3);
                for (int i = 0; i < dt_xyzFrame.rows; i++) {
                    for (int j = 0; j < dt_xyzFrame.cols; j++) {
                        for (int z = 0; z < 3; z++)
                        {
                            dt_xyzFrame.at<Vec3s>(i, j)[z] = dt_point_cloud_image_data[3 * dt + z];
                        }
                        ++dt;
                    }
                }
                cv::Mat dt_FloatFrame(dt_height, dt_width, CV_32FC3);
                dt_xyzFrame.convertTo(dt_FloatFrame, CV_32FC3,0.001,0.000);

                //cv::Mat ct_xFrame(ct_height, ct_width, CV_16SC1);
                //cv::Mat ct_yFrame(ct_height, ct_width, CV_16SC1);
                cv::Mat ct_zFrame(ct_height, ct_width, CV_16SC1);
                //cv::Mat dt_xFrame(dt_height, dt_width, CV_16SC1);
                //cv::Mat dt_yFrame(dt_height, dt_width, CV_16SC1);
                cv::Mat dt_zFrame(dt_height, dt_width, CV_16SC1);
                cv::Mat ct_zFloatFrame(ct_height, ct_width, CV_32FC3);
                cv::Mat dt_zFloatFrame(dt_height, dt_width, CV_32FC3);

                //赋值回给类内的对象“my_~”


                //cv::imshow("kinect color_frame master", colorFrame);
                //cv::imshow("kinect t_color_frame master", t_colorFrame);
                //*****************int16  mm
                split(ct_xyzFrame, ct_channels);
                //ct_xFrame = channels.at(0);
                //cv::imshow("kinect ct_xframe master", ct_xFrame);
                //ct_yFrame = channels.at(1);
                //cv::imshow("kinect ct_yframe master", ct_yFrame);
                //ct_zFrame = ct_channels.at(2);
                //cv::imshow("kinect ct_zframe master", ct_zFrame);
                split(dt_xyzFrame, dt_channels);
                //dt_xFrame = channels.at(0);
                //cv::imshow("kinect dt_xframe master", dt_xFrame);
                //dt_yFrame = channels.at(1);
                //cv::imshow("kinect dt_yframe master", dt_yFrame);
                //dt_zFrame = dt_channels.at(2);
                //cv::imshow("kinect dt_zframe master", dt_zFrame);

                //*********************float32  m
                //cv::imshow("kinect ct_float_frame master", ct_FloatFrame);
                //cv::imshow("kinect dt_float_frame master", dt_FloatFrame);
                split(ct_FloatFrame, ct_Fchannels);
                split(dt_FloatFrame, dt_Fchannels);
                ct_zFloatFrame = ct_Fchannels.at(2);
                //cv::imshow("kinect ct_zfloat_frame master", ct_zFloatFrame);
                dt_zFloatFrame = dt_Fchannels.at(2);
                //cv::imshow("kinect dt_zfloat_frame master", dt_zFloatFrame);


                //赋值回给类内的对象“my_~”,方便调用,color frame 和 t color frame 不需要赋值回，其已经是类内成员
                my_ct_xyzFrame = ct_xyzFrame;
                my_dt_xyzFrame = dt_xyzFrame;
                my_ct_FloatFrame = ct_FloatFrame;
                my_dt_FloatFrame = dt_FloatFrame;
                my_ct_zFrame = ct_zFrame;
                my_dt_zFrame = dt_zFrame;
                my_ct_zFloatFrame = ct_zFloatFrame;  //待显示
                my_dt_zFloatFrame = dt_zFloatFrame; //待显示

                emit dkWindowsignal();
            if ( waitKey(30) == ' ')
            {

            }
        }
    }

    // Stop Grabber
    grabber->stop();

    // Disconnect Callback Function
    if (connection.connected())
    {
        connection.disconnect();
    }


//    deviceCount = k4a::device::get_installed_count();
//        if (deviceCount == 0)
//        {
//            cout << "no azure kinect devices detected!" << endl;
//        }
//    config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
//    config.camera_fps = K4A_FRAMES_PER_SECOND_30;
//    config.depth_mode = K4A_DEPTH_MODE_WFOV_2X2BINNED;
//    config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
//    config.color_resolution = K4A_COLOR_RESOLUTION_720P;
//    config.synchronized_images_only = true;
//    cout << "Started opening K4A device..." << endl;

//    device = k4a::device::open(K4A_DEVICE_DEFAULT);
//    device.start_cameras(&config);
//    calibration = device.get_calibration(config.depth_mode, config.color_resolution);
//    transformation = k4a::transformation(calibration);
//    cout << "Finished opening K4A device." << endl;

//    while (1)
//    {

//        if (!device.get_capture(&capture, std::chrono::milliseconds(0)))
//        {
//            continue;
//        }

//        depthImage = capture.get_depth_image();
//        if (depthImage == nullptr)
//        {
//            throw std::exception("Failed to get depth image from capture\n");
//        }

//        colorImage = capture.get_color_image();
//        if (colorImage == nullptr)
//        {
//            throw std::exception("Failed to get color image from capture\n");
//        }

//            if (!depthImage.is_valid() || !colorImage.is_valid() )
//            {
//                continue;
//            }

//            int depth_image_width_pixels = depthImage.get_width_pixels();
//            int depth_image_height_pixels = depthImage.get_height_pixels();
//            int color_image_width_pixels = colorImage.get_width_pixels();
//            int color_image_height_pixels = colorImage.get_height_pixels();


//            transformed_color_image = NULL;
//            transformed_color_image = k4a::image::create(K4A_IMAGE_FORMAT_COLOR_BGRA32,
//                depth_image_width_pixels,
//                depth_image_height_pixels,
//                depth_image_width_pixels * 4 * (int)sizeof(uint8_t));
//            transformation.color_image_to_depth_camera(depthImage, colorImage, &transformed_color_image);

//            transformed_depth_image = NULL;
//            transformed_depth_image = k4a::image::create(K4A_IMAGE_FORMAT_DEPTH16,
//                color_image_width_pixels,
//                color_image_height_pixels,
//                color_image_width_pixels * (int)sizeof(uint16_t));
//            transformation.depth_image_to_color_camera(depthImage,&transformed_depth_image);

//            ct_point_cloud_image = NULL;
//            ct_point_cloud_image = k4a::image::create(K4A_IMAGE_FORMAT_CUSTOM,
//                depth_image_width_pixels,
//                depth_image_height_pixels,
//                depth_image_width_pixels * 3 * (int)sizeof(int16_t));
//            transformation.depth_image_to_point_cloud(depthImage, K4A_CALIBRATION_TYPE_DEPTH, &ct_point_cloud_image);

//            dt_point_cloud_image = NULL;
//            dt_point_cloud_image = k4a::image::create(K4A_IMAGE_FORMAT_CUSTOM,
//                color_image_width_pixels,
//                color_image_height_pixels,
//                color_image_width_pixels * 3 * (int)sizeof(int16_t));
//            transformation.depth_image_to_point_cloud(transformed_depth_image, K4A_CALIBRATION_TYPE_COLOR, &dt_point_cloud_image);


//            colorTextureBuffer = colorImage.get_buffer();
//            t_colorTextureBuffer = transformed_color_image.get_buffer();

//            ct_point_cloud_image_data = (int16_t *)(void *)ct_point_cloud_image.get_buffer();
//            dt_point_cloud_image_data = (int16_t *)(void *)dt_point_cloud_image.get_buffer();

//            dk_cloud.width = color_image_width_pixels;
//            dk_cloud.height = color_image_height_pixels;
//            dk_cloud.is_dense = false;
//            dk_cloud.points.resize(dk_cloud.height * dk_cloud.width);

//            int16_t *point_cloud_image_data = dt_point_cloud_image_data;
//            uint8_t *color_image_data = colorTextureBuffer;

//            for (int i = 0; i < color_image_width_pixels * color_image_height_pixels; ++i)
//            {
//                PointXYZRGBA point;

//                point.x = point_cloud_image_data[3 * i + 0] / 1000.0f;
//                point.y = point_cloud_image_data[3 * i + 1] / 1000.0f;
//                point.z = point_cloud_image_data[3 * i + 2] / 1000.0f;

//                if (point.z == 0)
//                {
//                    continue;
//                }


//                point.b = color_image_data[4 * i + 0];
//                point.g = color_image_data[4 * i + 1];
//                point.r = color_image_data[4 * i + 2];
//                point.a = color_image_data[4 * i + 3];

//                if (point.b == 0 && point.g == 0 && point.r == 0 && point.a == 0)
//                {
//                    continue;
//                }

//                dk_cloud.points[i] = point;
//            }


//            int ct_width = ct_point_cloud_image.get_width_pixels();
//            int ct_height = ct_point_cloud_image.get_height_pixels();
//            int dt_width = dt_point_cloud_image.get_width_pixels();
//            int dt_height = dt_point_cloud_image.get_height_pixels();
//            //cout << ct_width <<"*"<< ct_height << " " << dt_width << "*" << dt_height << endl;
//            //640*576  1280*720

//            colorFrame = cv::Mat(colorImage.get_height_pixels(), colorImage.get_width_pixels(), CV_8UC4, colorTextureBuffer);
//            t_colorFrame = cv::Mat(transformed_color_image.get_height_pixels(), transformed_color_image.get_width_pixels(), CV_8UC4, t_colorTextureBuffer);

//            int ct = 0;
//            cv::Mat ct_xyzFrame(ct_height, ct_width, CV_16SC3);

//            for (int i = 0; i < ct_xyzFrame.rows; i++) {
//                for (int j = 0; j < ct_xyzFrame.cols; j++) {
//                    for(int z = 0;z<3;z++)
//                    {
//                        ct_xyzFrame.at<Vec3s>(i, j)[z] = ct_point_cloud_image_data[3 * ct + z];
//                    }
//                    ++ct;
//                }
//            }
//            cv::Mat ct_FloatFrame(ct_height, ct_width, CV_32FC3);
//            ct_xyzFrame.convertTo(ct_FloatFrame, CV_32FC3, 0.001, 0.000);

//            int dt = 0;
//            cv::Mat dt_xyzFrame(dt_height, dt_width, CV_16SC3);
//            for (int i = 0; i < dt_xyzFrame.rows; i++) {
//                for (int j = 0; j < dt_xyzFrame.cols; j++) {
//                    for (int z = 0; z < 3; z++)
//                    {
//                        dt_xyzFrame.at<Vec3s>(i, j)[z] = dt_point_cloud_image_data[3 * dt + z];
//                    }
//                    ++dt;
//                }
//            }
//            cv::Mat dt_FloatFrame(dt_height, dt_width, CV_32FC3);
//            dt_xyzFrame.convertTo(dt_FloatFrame, CV_32FC3,0.001,0.000);

//            //cv::Mat ct_xFrame(ct_height, ct_width, CV_16SC1);
//            //cv::Mat ct_yFrame(ct_height, ct_width, CV_16SC1);
//            cv::Mat ct_zFrame(ct_height, ct_width, CV_16SC1);
//            //cv::Mat dt_xFrame(dt_height, dt_width, CV_16SC1);
//            //cv::Mat dt_yFrame(dt_height, dt_width, CV_16SC1);
//            cv::Mat dt_zFrame(dt_height, dt_width, CV_16SC1);
//            cv::Mat ct_zFloatFrame(ct_height, ct_width, CV_32FC3);
//            cv::Mat dt_zFloatFrame(dt_height, dt_width, CV_32FC3);

//            //赋值回给类内的对象“my_~”


//            //cv::imshow("kinect color_frame master", colorFrame);
//            //cv::imshow("kinect t_color_frame master", t_colorFrame);
//            //*****************int16  mm
//            split(ct_xyzFrame, ct_channels);
//            //ct_xFrame = channels.at(0);
//            //cv::imshow("kinect ct_xframe master", ct_xFrame);
//            //ct_yFrame = channels.at(1);
//            //cv::imshow("kinect ct_yframe master", ct_yFrame);
//            //ct_zFrame = ct_channels.at(2);
//            //cv::imshow("kinect ct_zframe master", ct_zFrame);
//            split(dt_xyzFrame, dt_channels);
//            //dt_xFrame = channels.at(0);
//            //cv::imshow("kinect dt_xframe master", dt_xFrame);
//            //dt_yFrame = channels.at(1);
//            //cv::imshow("kinect dt_yframe master", dt_yFrame);
//            //dt_zFrame = dt_channels.at(2);
//            //cv::imshow("kinect dt_zframe master", dt_zFrame);

//            //*********************float32  m
//            //cv::imshow("kinect ct_float_frame master", ct_FloatFrame);
//            //cv::imshow("kinect dt_float_frame master", dt_FloatFrame);
//            split(ct_FloatFrame, ct_Fchannels);
//            split(dt_FloatFrame, dt_Fchannels);
//            ct_zFloatFrame = ct_Fchannels.at(2);
//            //cv::imshow("kinect ct_zfloat_frame master", ct_zFloatFrame);
//            dt_zFloatFrame = dt_Fchannels.at(2);
//            //cv::imshow("kinect dt_zfloat_frame master", dt_zFloatFrame);


//            //赋值回给类内的对象“my_~”,方便调用,color frame 和 t color frame 不需要赋值回，其已经是类内成员
//            my_ct_xyzFrame = ct_xyzFrame;
//            my_dt_xyzFrame = dt_xyzFrame;
//            my_ct_FloatFrame = ct_FloatFrame;
//            my_dt_FloatFrame = dt_FloatFrame;
//            my_ct_zFrame = ct_zFrame;
//            my_dt_zFrame = dt_zFrame;
//            my_ct_zFloatFrame = ct_zFloatFrame;  //待显示
//            my_dt_zFloatFrame = dt_zFloatFrame; //待显示

//            emit dkWindowsignal();

//        if ( waitKey(30) == ' ')
//        {

//        }
//    }
}
