#include "acthread.h"

acThread::acThread(QObject *parent) : QThread(parent)
{

}

acThread::~acThread()
{

}

void acThread::run()
{
//    dataVisualization(kpcProcess->vec_plant_memory_base[0]);
//    dataVisualization(kpcProcess->plant_memory);
//    AABBVisualization(kpcProcess->plant_memory, kpcProcess->bboxPoints);
    AABBAndPoseVisualization(kpcProcess->plant_memory, kpcProcess->bboxPoints,kpcProcess->posFromAABB,kpcProcess->orientation_AABB,kpcProcess->isReachable_AABB);
//    clusterVisualization(kpcProcess->colored_cluster_cloud);
//    OBBAndPoseVisualization(kpcProcess->colored_cluster_cloud,kpcProcess->obbPoints_vec,kpcProcess->rotational_matrix_OBB_vec,kpcProcess->posFromOBB,kpcProcess->orientation_OBB,kpcProcess->isReachable_OBB);
//    OBBAndPoseWithAndWithoutOcclusionVisualization(kpcProcess->colored_cluster_cloud, kpcProcess->obbPoints_vec_ok, kpcProcess->rotational_matrix_OBB_vec_ok, kpcProcess->posFromOBB_ok, kpcProcess->ur_pose_iter, kpcProcess->axis_ok, kpcProcess->axis_iter, kpcProcess->isOccluded);
//    boundingLineVisulization(kpcProcess->pc_clusters_single_frame, kpcProcess->pc_b, kpcProcess->j_best);
//    sphereProjectionVisulization(kpcProcess->pc_sp, kpcProcess->j_best);
//    adjacentLineVisulization(kpcProcess->pc_clusters_single_frame, kpcProcess->idx_spa, kpcProcess->j_best);
    //...
}

void acThread::dataVisualization(pcl::PointCloud<PointType> pc)
{
//    kpcProcess->adaptiveCapturing();//execute in thread
//    kpcProcess->dataVisualization((kpcProcess->vec_pc_memory_base[0]));
    //pc -> ptr
    pcl::PointCloud<PointType>::Ptr cloud1(new pcl::PointCloud<PointType>);
    cloud1 = pc.makeShared();
    //end

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer1(new pcl::visualization::PCLVisualizer("display"));
    viewer1->addPointCloud(cloud1, "cloud1");

    while (!viewer1->wasStopped())
    {
        viewer1->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }

//    while (!viewer1->wasStopped())
//    {
//        // Update Viewer
//        viewer1->spinOnce(100);
////        if (cloud1)
////        {
////            // Update Point Cloud
////            if (!viewer1->updatePointCloud(cloud1, "cloud1"))
////            {
////                viewer1->addPointCloud(cloud1, "cloud1");
////            }
////        }
//    }
}

void acThread::AABBVisualization(pcl::PointCloud<PointType> pc, vector<PointType> points)
{
    //pc -> ptr
    pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);
    cloud = pc.makeShared();
    //end
    PointType min_point_AABB = points[0];
    PointType max_point_AABB = points[1];
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("display"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addCoordinateSystem (0.2);
    viewer->initCameraParameters ();
    viewer->addPointCloud<PointType> (cloud,"sample cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,2,"sample cloud");
    viewer->addCube (min_point_AABB.x, max_point_AABB.x, min_point_AABB.y, max_point_AABB.y, min_point_AABB.z, max_point_AABB.z, 1.0, 0.0, 0.0, "AABB");
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY,0.5,"AABB");
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH,0.5,"AABB");
    //  Eigen::Vector3f position (position_OBB.x, position_OBB.y, position_OBB.z);

    //     cout<<"position_OBB: "<<position_OBB<<endl;
    //     cout<<"mass_center: "<<mass_center<<  endl;
    //  Eigen::Quaternionf quat (rotational_matrix_OBB);
    //  viewer->addCube (position, quat, max_point_OBB.x - min_point_OBB.x, max_point_OBB.y - min_point_OBB.y, max_point_OBB.z - min_point_OBB.z, "OBB");
    //  viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,0,0,1,"OBB");
    //  viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY,0.5,"OBB");
    //  viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH,3,"OBB");
    //  viewer->setRepresentationToWireframeForAllActors();
    //  pcl::PointXYZ center (mass_center (0), mass_center (1), mass_center (2));
    //  pcl::PointXYZ x_axis (major_vector (0) + mass_center (0), major_vector (1) + mass_center (1), major_vector (2) + mass_center (2));
    //  pcl::PointXYZ y_axis (middle_vector (0) + mass_center (0), middle_vector (1) + mass_center (1), middle_vector (2) + mass_center (2));
    //  pcl::PointXYZ z_axis (minor_vector (0) + mass_center (0), minor_vector (1) + mass_center (1), minor_vector (2) + mass_center (2));
    //  viewer->addLine (center, x_axis, 1.0f, 0.0f, 0.0f, "major eigen vector");
    //  viewer->addLine (center, y_axis, 0.0f, 1.0f, 0.0f, "middle eigen vector");
    //  viewer->addLine (center, z_axis, 0.0f, 0.0f, 1.0f, "minor eigen vector");
    //       cout<<"size of cloud :"<<cloud->points.size()<<endl;
    //       cout<<"moment_of_inertia :"<<moment_of_inertia.size()<<endl;
    //       cout<<"eccentricity :"<<eccentricity.size()<<endl;

    while(!viewer->wasStopped())
    {
        viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
}

void acThread::AABBAndPoseVisualization(pcl::PointCloud<PointType> pc, vector<PointType> points, vector<vector<double> > posFromAABB, vector<Eigen::Vector3d> orientation_AABB, vector<int> isReachable_AABB)
{
    //pc -> ptr
    pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);
    cloud = pc.makeShared();
    //end
    PointType min_point_AABB = points[0];
    PointType max_point_AABB = points[1];
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("display"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addCoordinateSystem (0.2);
    viewer->initCameraParameters ();
    viewer->addPointCloud<PointType> (cloud,"sample cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,2,"sample cloud");
    viewer->addCube (min_point_AABB.x, max_point_AABB.x, min_point_AABB.y, max_point_AABB.y, min_point_AABB.z, max_point_AABB.z, 1.0, 0.0, 0.0, "AABB");
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY,0.5,"AABB");
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH,0.5,"AABB");
    //  Eigen::Vector3f position (position_OBB.x, position_OBB.y, position_OBB.z);

    //     cout<<"position_OBB: "<<position_OBB<<endl;
    //     cout<<"mass_center: "<<mass_center<<  endl;
    //  Eigen::Quaternionf quat (rotational_matrix_OBB);
    //  viewer->addCube (position, quat, max_point_OBB.x - min_point_OBB.x, max_point_OBB.y - min_point_OBB.y, max_point_OBB.z - min_point_OBB.z, "OBB");
    //  viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,0,0,1,"OBB");
    //  viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY,0.5,"OBB");
    //  viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH,3,"OBB");
    //  viewer->setRepresentationToWireframeForAllActors();
    for(int i = 0;i<posFromAABB.size();i++)
    {
        Eigen::Vector3f major_vector = orientation_AABB[3*i+0].cast<float>();
        Eigen::Vector3f middle_vector = orientation_AABB[3*i+1].cast<float>();
        Eigen::Vector3f minor_vector = orientation_AABB[3*i+2].cast<float>();
        Eigen::Vector3f mass_center;
        mass_center(0) = float(posFromAABB[i][0]);
        mass_center(1) = float(posFromAABB[i][1]);
        mass_center(2) = float(posFromAABB[i][2]);
        std::string str1 = QString("major eigen vector %1").arg(i).toStdString();
        std::string str2 = QString("middle eigen vector %1").arg(i).toStdString();
        std::string str3 = QString("minor eigen vector %1").arg(i).toStdString();
        pcl::PointXYZ center (mass_center (0), mass_center (1), mass_center (2));
        pcl::PointXYZ x_axis (SCALE_AABB*major_vector (0) + mass_center (0), SCALE_AABB*major_vector (1) + mass_center (1), SCALE_AABB*major_vector (2) + mass_center (2));
        pcl::PointXYZ y_axis (SCALE_AABB*middle_vector (0) + mass_center (0), SCALE_AABB*middle_vector (1) + mass_center (1), SCALE_AABB*middle_vector (2) + mass_center (2));
        pcl::PointXYZ z_axis (SCALE_AABB*minor_vector (0) + mass_center (0), SCALE_AABB*minor_vector (1) + mass_center (1), SCALE_AABB*minor_vector (2) + mass_center (2));
//        if(isReachable_AABB[i]==1)
        if(1)
        {
            viewer->addLine (center, x_axis, 1.0f, 0.0f, 0.0f, str1);
            viewer->addLine (center, y_axis, 0.0f, 1.0f, 0.0f, str2);
            viewer->addLine (center, z_axis, 0.0f, 0.0f, 1.0f, str3);
        }
        else
        {
            viewer->addLine (center, x_axis, 1.0f, 0.0f, 0.0f, str1);
            viewer->addLine (center, y_axis, 1.0f, 0.0f, 0.0f, str2);
            viewer->addLine (center, z_axis, 1.0f, 0.0f, 0.0f, str3);
        }
    }
    while(!viewer->wasStopped())
    {
        viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
}

void acThread::OBBAndPoseVisualization(pcl::PointCloud<PointType> pc, vector<vector<PointType>> points_vec,vector<Eigen::Matrix3f> rotational_matrix_OBB, vector<vector<double> > posFromOBB, vector<Eigen::Vector3d> orientation_OBB, vector<int> isReachable_OBB)
{
    //pc -> ptr
    pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);
    cloud = pc.makeShared();
    //end
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("display"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addCoordinateSystem (0.2);
    viewer->initCameraParameters ();
    viewer->addPointCloud<PointType> (cloud,"sample cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,2,"sample cloud");
    viewer->setRepresentationToWireframeForAllActors();
    pcl::visualization::PointCloudColorHandlerRGBField<PointType> rgba(cloud);

    for(int i = 0;i<posFromOBB.size();i++)
    {
        Eigen::Vector3f position (points_vec[i][2].x, points_vec[i][2].y, points_vec[i][2].z);
        Eigen::Quaternionf quat (rotational_matrix_OBB[i]);

        std::string str0 = QString("OBB %1").arg(i).toStdString();
        viewer->addCube (position, quat, points_vec[i][1].x - points_vec[i][0].x, points_vec[i][1].y - points_vec[i][0].y, points_vec[i][1].z - points_vec[i][0].z, str0);
        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,0,0,1,str0);
        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY,0.5,str0);
        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH,0.5,str0);

        Eigen::Vector3f major_vector = orientation_OBB[3*i+0].cast<float>();
        Eigen::Vector3f middle_vector = orientation_OBB[3*i+1].cast<float>();
        Eigen::Vector3f minor_vector = orientation_OBB[3*i+2].cast<float>();
        Eigen::Vector3f mass_center;
        mass_center(0) = float(posFromOBB[i][0]);
        mass_center(1) = float(posFromOBB[i][1]);
        mass_center(2) = float(posFromOBB[i][2]);
        std::string str1 = QString("major eigen vector %1").arg(i).toStdString();
        std::string str2 = QString("middle eigen vector %1").arg(i).toStdString();
        std::string str3 = QString("minor eigen vector %1").arg(i).toStdString();
        pcl::PointXYZ center (mass_center (0), mass_center (1), mass_center (2));
        pcl::PointXYZ x_axis (SCALE_AABB*major_vector (0) + mass_center (0), SCALE_AABB*major_vector (1) + mass_center (1), SCALE_AABB*major_vector (2) + mass_center (2));
        pcl::PointXYZ y_axis (SCALE_AABB*middle_vector (0) + mass_center (0), SCALE_AABB*middle_vector (1) + mass_center (1), SCALE_AABB*middle_vector (2) + mass_center (2));
        pcl::PointXYZ z_axis (SCALE_AABB*minor_vector (0) + mass_center (0), SCALE_AABB*minor_vector (1) + mass_center (1), SCALE_AABB*minor_vector (2) + mass_center (2));
//        if(isReachable_OBB[i]==1)
        if(1)
        {
            viewer->addLine (center, x_axis, 1.0f, 0.0f, 0.0f, str1);
            viewer->addLine (center, y_axis, 0.0f, 1.0f, 0.0f, str2);
            viewer->addLine (center, z_axis, 0.0f, 0.0f, 1.0f, str3);
        }
        else
        {
            viewer->addLine (center, x_axis, 1.0f, 0.0f, 0.0f, str1);
            viewer->addLine (center, y_axis, 1.0f, 0.0f, 0.0f, str2);
            viewer->addLine (center, z_axis, 1.0f, 0.0f, 0.0f, str3);
        }
    }
    while(!viewer->wasStopped())
    {
        viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
}

void acThread::OBBAndPoseWithAndWithoutOcclusionVisualization(pcl::PointCloud<PointType> pc, vector<vector<PointType> > points_vec, vector<Eigen::Matrix3f> rotational_matrix_OBB, vector<vector<double> > posFromOBB, vector<vector<double> > posFromOBB_iter, vector<Eigen::Vector3d> orientation_OBB, vector<Eigen::Vector3d> orientation_OBB_iter, vector<int> isOccluded)
{
    //pc -> ptr
    pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);
    cloud = pc.makeShared();
    //end
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("display"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addCoordinateSystem (0.2);
    viewer->initCameraParameters ();
    viewer->addPointCloud<PointType> (cloud,"sample cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,2,"sample cloud");
    viewer->setRepresentationToWireframeForAllActors();
    pcl::visualization::PointCloudColorHandlerRGBField<PointType> rgba(cloud);

    for(int i = 0;i<posFromOBB_iter.size();i++)
    {
        Eigen::Vector3f position (points_vec[i][2].x, points_vec[i][2].y, points_vec[i][2].z);
        Eigen::Quaternionf quat (rotational_matrix_OBB[i]);

        std::string str0 = QString("OBB %1").arg(i).toStdString();
        viewer->addCube (position, quat, points_vec[i][1].x - points_vec[i][0].x, points_vec[i][1].y - points_vec[i][0].y, points_vec[i][1].z - points_vec[i][0].z, str0);
        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,0,0,1,str0);
        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY,0.5,str0);
        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH,0.5,str0);

        Eigen::Vector3f major_vector = orientation_OBB_iter[3*i+0].cast<float>();
        Eigen::Vector3f middle_vector = orientation_OBB_iter[3*i+1].cast<float>();
        Eigen::Vector3f minor_vector = orientation_OBB_iter[3*i+2].cast<float>();
        Eigen::Vector3f mass_center;
        mass_center(0) = float(posFromOBB_iter[i][0]);
        mass_center(1) = float(posFromOBB_iter[i][1]);
        mass_center(2) = float(posFromOBB_iter[i][2]);
        std::string str1 = QString("major eigen vector %1").arg(i).toStdString();
        std::string str2 = QString("middle eigen vector %1").arg(i).toStdString();
        std::string str3 = QString("minor eigen vector %1").arg(i).toStdString();
        pcl::PointXYZ center (mass_center (0), mass_center (1), mass_center (2));
        pcl::PointXYZ x_axis (SCALE_AABB*major_vector (0) + mass_center (0), SCALE_AABB*major_vector (1) + mass_center (1), SCALE_AABB*major_vector (2) + mass_center (2));
        pcl::PointXYZ y_axis (SCALE_AABB*middle_vector (0) + mass_center (0), SCALE_AABB*middle_vector (1) + mass_center (1), SCALE_AABB*middle_vector (2) + mass_center (2));
        pcl::PointXYZ z_axis (SCALE_AABB*minor_vector (0) + mass_center (0), SCALE_AABB*minor_vector (1) + mass_center (1), SCALE_AABB*minor_vector (2) + mass_center (2));

        viewer->addLine (center, x_axis, 1.0f, 0.0f, 0.0f, str1);
        viewer->addLine (center, y_axis, 0.0f, 1.0f, 0.0f, str2);
        viewer->addLine (center, z_axis, 0.0f, 0.0f, 1.0f, str3);
//        if(isReachable_OBB[i]==1)
//        if(1)
        if(isOccluded[i]==1)//set the original data other color
        {
            Eigen::Vector3f major_vector = orientation_OBB[3*i+0].cast<float>();
            Eigen::Vector3f middle_vector = orientation_OBB[3*i+1].cast<float>();
            Eigen::Vector3f minor_vector = orientation_OBB[3*i+2].cast<float>();
            Eigen::Vector3f mass_center;
            mass_center(0) = float(posFromOBB[i][0]);
            mass_center(1) = float(posFromOBB[i][1]);
            mass_center(2) = float(posFromOBB[i][2]);
            std::string str1 = QString("ori major eigen vector %1").arg(i).toStdString();
            std::string str2 = QString("ori middle eigen vector %1").arg(i).toStdString();
            std::string str3 = QString("ori minor eigen vector %1").arg(i).toStdString();
            pcl::PointXYZ center (mass_center (0), mass_center (1), mass_center (2));
            pcl::PointXYZ x_axis (SCALE_AABB*major_vector (0) + mass_center (0), SCALE_AABB*major_vector (1) + mass_center (1), SCALE_AABB*major_vector (2) + mass_center (2));
            pcl::PointXYZ y_axis (SCALE_AABB*middle_vector (0) + mass_center (0), SCALE_AABB*middle_vector (1) + mass_center (1), SCALE_AABB*middle_vector (2) + mass_center (2));
            pcl::PointXYZ z_axis (SCALE_AABB*minor_vector (0) + mass_center (0), SCALE_AABB*minor_vector (1) + mass_center (1), SCALE_AABB*minor_vector (2) + mass_center (2));

            viewer->addLine (center, x_axis, 1.0f, 0.0f, 0.0f, str1);
            viewer->addLine (center, y_axis, 1.0f, 0.0f, 0.0f, str2);
            viewer->addLine (center, z_axis, 1.0f, 0.0f, 0.0f, str3);
        }
    }
    while(!viewer->wasStopped())
    {
        viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
}

void acThread::boundingLineVisulization(vector<pcl::PointCloud<PointType> > pc_clusters_single_frame, vector<pcl::PointCloud<PointType> > pc_b, int j_best)
{
    //pc -> ptr
    pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);
    //end
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("display"));
    viewer->setBackgroundColor (0, 0, 0);

    for(int i=0;i<pc_clusters_single_frame.size();i++)
    {
        cloud = pc_clusters_single_frame[i].makeShared();
        std::string str1 = QString("cloud cluster %1").arg(i).toStdString();
        if(i==j_best)
        {
            pcl::visualization::PointCloudColorHandlerCustom<PointType> src_1(cloud, 0, 0, 255);//use blue color
            viewer->addPointCloud(cloud, src_1, str1);
        }
        else
        {
            viewer->addPointCloud<PointType> (cloud,str1);//use original color
        }
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,2,str1);

        cloud = pc_b[i].makeShared();
        std::string str2 = QString("cloud bounding %1").arg(i).toStdString();
        pcl::visualization::PointCloudColorHandlerCustom<PointType> src(cloud, 0, 255, 0);//use green color
        viewer->addPointCloud(cloud, src, str2);
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,2,str2);
    }

    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(10000));
    }
}

void acThread::sphereProjectionVisulization(vector<pcl::PointCloud<PointType> > pc_sp, int j_best)
{
    //pc -> ptr
    pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);
    //end
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("display"));
    viewer->setBackgroundColor (0, 0, 0);

    for(int i=0;i<pc_sp.size();i++)
    {
        cloud = pc_sp[i].makeShared();
        std::string str = QString("projected bounding %1").arg(i).toStdString();
        if(i==j_best)
        {
            pcl::visualization::PointCloudColorHandlerCustom<PointType> src(cloud, 0, 255, 0);//use green color
            viewer->addPointCloud(cloud, src, str);
        }
        else
        {
            pcl::visualization::PointCloudColorHandlerCustom<PointType> src(cloud, 0, 0, 255);//use blue color
            viewer->addPointCloud(cloud, src, str);
        }
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,2,str);
    }

    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(10000));
    }
}

void acThread::adjacentLineVisulization(vector<pcl::PointCloud<PointType> > pc_clusters_single_frame, vector<int> idx_spa, int j_best)
{
    //pc -> ptr
    pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);
    pcl::PointCloud<PointType>::Ptr cloud_adjacent(new pcl::PointCloud<PointType>);
    //end
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("display"));
    viewer->setBackgroundColor (0, 0, 0);

    for(int i=0;i<pc_clusters_single_frame.size();i++)
    {
        cloud = pc_clusters_single_frame[i].makeShared();
        std::string str1 = QString("cloud cluster %1").arg(i).toStdString();
        if(i==j_best)
        {
            pcl::visualization::PointCloudColorHandlerCustom<PointType> src_1(cloud, 0, 0, 255);//use blue color
            viewer->addPointCloud(cloud, src_1, str1);

            pcl::PointCloud<PointType> pc_idx;
            for(int j = 0;j<idx_spa.size();j++)
            {
                pc_idx.push_back(cloud->points[idx_spa[j]]);
            }
            cloud_adjacent = pc_idx.makeShared();
            std::string str2 = QString("adjacent bounding %1").arg(i).toStdString();
            pcl::visualization::PointCloudColorHandlerCustom<PointType> src_2(cloud_adjacent, 0, 255, 0);//use green color
            viewer->addPointCloud(cloud_adjacent, src_2, str2);
            viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,2,str2);
        }
        else
        {
            viewer->addPointCloud<PointType> (cloud,str1);//use original color
        }
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,2,str1);
    }

    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(10000));
    }
}

void acThread::clusterVisualization(pcl::PointCloud<PointType> colored_cluster_cloud)
{
    //pc -> ptr
    pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);
    cloud = colored_cluster_cloud.makeShared();
    //end
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    pcl::visualization::PointCloudColorHandlerRGBField<PointType> rgba(cloud);
    viewer->setWindowName("region growth seg");
    viewer->addPointCloud<PointType>(cloud, "sample cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");

    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
}

void acThread::multiWindowVisualization(pcl::PointCloud<PointType> pc1, pcl::PointCloud<PointType> pc2, pcl::PointCloud<PointType> pc3)
{
    //pc -> ptr
    pcl::PointCloud<PointType>::Ptr source(new pcl::PointCloud<PointType>);
    pcl::PointCloud<PointType>::Ptr target(new pcl::PointCloud<PointType>);
    pcl::PointCloud<PointType>::Ptr icp(new pcl::PointCloud<PointType>);
    source = pc1.makeShared();
    target = pc2.makeShared();
    icp = pc3.makeShared();
    //end
    
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("RegistrationCloud"));
    int v1 = 0;
    int v2 = 1;
    viewer->createViewPort(0, 0, 0.5, 1, v1);
    viewer->createViewPort(0.5, 0, 1, 1, v2);
    viewer->setBackgroundColor(0, 0, 0, v1);
    viewer->setBackgroundColor(0.05, 0, 0, v2);
    viewer->addText("Raw point clouds", 10, 10, "v1_text", v1);
    viewer->addText("Registed point clouds", 10, 10, "v2_text", v2);
    pcl::visualization::PointCloudColorHandlerCustom<PointType> src_h(source, 0, 255, 0);
    pcl::visualization::PointCloudColorHandlerCustom<PointType> tgt_h(target, 0, 0, 255);
    pcl::visualization::PointCloudColorHandlerCustom<PointType> transe(icp, 255, 0, 0);
    //viewer->setBackgroundColor(255, 255, 255);
    viewer->addPointCloud(source, src_h, "source cloud", v1);
    viewer->addPointCloud(target, tgt_h, "target cloud", v1);

    viewer->addPointCloud(target, tgt_h, "target cloud1", v2);
    viewer->addPointCloud(icp, transe, "pcs cloud", v2);

    //viewer->addCoordinateSystem(0.1);
    //viewer->initCameraParameters();
    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(10000));
    }
}
