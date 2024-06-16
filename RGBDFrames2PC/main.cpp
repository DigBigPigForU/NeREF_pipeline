#pragma once
#pragma comment(lib, "User32.lib")
#pragma comment(lib, "gdi32.lib")
#define SCALE_VEC 0.05
#define SCALE_AABB 0.05
#define OFFSET_FOR_CAPTURING 0.3
#include <vtkAutoInit.h>
VTK_MODULE_INIT(vtkRenderingOpenGL)
VTK_MODULE_INIT(vtkInteractionStyle)

#include <QCoreApplication>

#include <conio.h>
#include <string>
#include <QFile>
#include <iostream>
#include <vector>
#include <math.h>
#include <algorithm>
#include <k4a/k4a.hpp>
#include "ur_kin.h"

#include <iostream>
#include <vector>
#include <array>
#include <chrono>
#include <ratio>
#include <thread>

#include <iostream>
#include <filesystem>
#include <fstream>
#include <sstream>

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pcl/io/boost.h>
#include <pcl/io/grabber.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <Eigen/Core>

//radius outlier removal
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/time.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/visualization/pcl_visualizer.h>

//pass through
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>

//bbox
#include<iostream>
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/visualization/cloud_viewer.h>
#include <boost/thread/thread.hpp>

//convex hull
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/surface/concave_hull.h>
#include <boost/thread/thread.hpp>
#include <pcl/visualization/pcl_visualizer.h>

//plane projection
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>

//plane hull
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/time.h>

//hull sorting
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/geometry/planar_polygon.h>

//S calculation
#include <iostream>
#include <pcl/common/common.h>
#include <pcl/geometry/planar_polygon.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>

//3d circle
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_circle3D.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>

//centroid
#include <iostream>
#include <Eigen/Core>
#include <pcl\io\pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>    //#include <pcl/common/transforms.h>
#include <boost/thread/thread.hpp>
#include <pcl/visualization/pcl_visualizer.h>

//normal estimation
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
//#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>


using namespace std;
using namespace cv;
using namespace boost;
using namespace pcl;
using namespace std::chrono;
using namespace ur_kinematics;

typedef pcl::PointXYZRGBA PointType;
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr convertRGBADepthToPointXYZRGBA(k4a::image&,k4a::image&,k4a::transformation & );
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr convertRGBADepthToPointXYZRGBA_transformed(k4a::image&,k4a::image&,k4a::transformation &);
inline bool endsWith(const string& , const string& );
void frames2pc(const string& , const string&,const string&, const bool&,const bool&,k4a::transformation &);//rgbd pcd
void frames2pc_v2(const string& , const string&,const string&,const string&,k4a::transformation &);//vi+rgbd txt
void frames2pc_v3(const string& , const string&,const string&,const string&,k4a::transformation &,Eigen::Isometry3d &);//vi+rgbd+transformation txt
void frames2pc_v4(const string& , const string&,const vector<string>&,const string&,k4a::transformation &,Eigen::Isometry3d &);//ms+rgbd+transformation txt
pcl::PointCloud<PointType>::Ptr radiusFilteringPc( pcl::PointCloud<PointType>::Ptr);//radius filtering Pc to remove the noise , return result
pcl::PointCloud<PointType>::Ptr passThroughPc(pcl::PointCloud<PointType>::Ptr);
void bboxCalculationForPc(pcl::PointCloud<PointType>::Ptr,
                            vector <float> &,//vector <float> moment_of_inertia;
                            vector <float> &,//vector <float> eccentricity
                            vector <PointType> &,//min_point_AABB, max_point_AABB, min_point_OBB, max_point_OBB, position_OBB
                          Eigen::Matrix3f &,// rotational_matrix_OBB
                          vector <float> &,//major_value, middle_value, minor_value
                          vector<Eigen::Vector3f> &//  Eigen::Vector3f major_vector, middle_vector, minor_vector, mass_center;
                          );//very slow,20s
void pcaOBB(pcl::PointCloud<PointType>::Ptr,//pc in
            Eigen::Vector3f &, //w h d
            vector <PointType> & ,//min_point_AABB, max_point_AABB, min_point_OBB, max_point_OBB, position_OBB
          Eigen::Matrix3f & ,// rotational_matrix_OBB
          vector <float> & ,//major_value, middle_value, minor_value
          vector<Eigen::Vector3f> &//  Eigen::Vector3f major_vector, middle_vector, minor_vector, mass_center;
            );//can replace the original code for bbox calculation,aabb: 5ms,obb:45ms
void posCalculationFromAABB(vector <PointType> ,//min_point_AABB, max_point_AABB
                          vector<vector<double>>&,//list of 6d pos vectors UR
                            vector<Eigen::Isometry3d>&,//T matrix vector
                            vector<Eigen::Vector3d>&//restore the orientation vec for vtk visualization
                            );//AABB is on the left side
void posCalculationFromOBB(vector <PointType>,//min_point_OBB, max_point_OBB, position_OBB
                           vector<Eigen::Vector3f>,
                           vector<double>&,// OBB just need one pos to move UR
                           Eigen::Isometry3d&,//T matrix
                           vector<Eigen::Vector3d>&//restore the orientation vec for vtk visualization
                           );
void planeProjection(float,//a
                     float,//b
                     float,//c
                     float,//d ax+by+cz+d=0
                     pcl::PointCloud<PointType>::Ptr,//pc in
                     pcl::PointCloud<PointType>::Ptr//pc out
                     );
void hull_extraction(pcl::PointCloud<PointType>::Ptr,//pc in
                     pcl::PointCloud<PointType>::Ptr,//pc out
                     float alpha//alpha
                     );
void CSEstimationForExistingPointCloud(pcl::PointCloud<PointType>::Ptr);
void CircleEstimation(pcl::PointCloud<PointType>::Ptr);
void CounterClockWiseSortPoints(pcl::PointCloud<PointType>&);
void polygonGeneration(pcl::PointCloud<PointType>::Ptr ,pcl::PointCloud<PointType>&);
float S(pcl::PointCloud<PointType>);
float C(pcl::PointCloud<PointType>);
float pointDistance(PointType,PointType);
float str2int(string);
void vec3fNorm(Eigen::Vector3f &);
void vec3dNorm(Eigen::Vector3d &);
Eigen::Vector3f vec3fCross(Eigen::Vector3f,Eigen::Vector3f);
double vectorLength(double rx,double ry,double rz);
Eigen::Vector3d vec3dCross(Eigen::Vector3d,Eigen::Vector3d);

//global var for bbox estimating
vector<Eigen::Vector3f> whds;
vector<float> Ss;
vector<float> Cs;
vector<float> Ss_ellipse;
vector<float> Cs_ellipse;
vector<float> R2s;
vector<float> RMSEs;
vector<float> Rs;

int k=0;

//int main(int argc, char *argv[])//test code for parts
//{
//    pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>());

//    pcl::io::loadPCDFile("D:\\rgbd_pc_20220807\\data\\Ball only top ball\\1.pcd", *cloud);
//    CSEstimationForExistingPointCloud(cloud);
//    pcl::io::loadPCDFile("D:\\rgbd_pc_20220807\\data\\Ball only top ball\\2.pcd", *cloud);
//    CSEstimationForExistingPointCloud(cloud);
//    pcl::io::loadPCDFile("D:\\rgbd_pc_20220807\\data\\Ball only top ball\\3.pcd", *cloud);
//    CSEstimationForExistingPointCloud(cloud);
//    pcl::io::loadPCDFile("D:\\rgbd_pc_20220807\\data\\Ball only top ball\\4.pcd", *cloud);
//    CSEstimationForExistingPointCloud(cloud);

//    //csv saving for obb and aabb of xilanhua
//    ofstream outFile;
//    outFile.open("bounding_box_data_of_xilanhua.csv", ios::out); // 打开模式可省略
//    outFile << "No." << ',' <<"w_obb"<<','<<"h_obb"<<','<<"d_obb"<<','<<"S_ellipse"<<','<<"C_ellipse"<<','<<"S_projection"<<','<<"C_projection"<<','<<"R"<<','<<"R_square"<<','<<"RMSE"<<endl;
////        outFile << "No." << ',' << "w_aabb" << ',' << "l_aabb" << ','<<"h_aabb"<<','<<"w_obb"<<','<<"l_obb"<<','<<"h_obb"<<endl;

//    for(int i = 0;i<4;i++)
//    {
//        outFile << to_string(i+1) << ','<<to_string(double(whds[i](0))) << ',' << to_string(double(whds[i](1))) << ',' << to_string(double(whds[i](2))) <<',' << to_string(double(Ss_ellipse[i])) <<',' << to_string(double(Cs_ellipse[i])) <<',' << to_string(double(Ss[i])) <<',' << to_string(double(Cs[i]))<<',' << to_string(double(Rs[i]))<<',' << to_string(double(R2s[i]))<<',' << to_string(double(RMSEs[i])) <<endl;
//    }

//    std::cout<<"csv saving done!"<<std::endl;
//    outFile.close();

//    return 0;
//}

//int main(int argc, char *argv[]) //transform from cam to base for each folder, just rgbd
//{
//    ifstream inFile("D:\\rgbd_pc_20220807\\data\\data.csv", ios::in);
//    string lineStr;
//    vector< vector<string> > strArray; //vector 类型文string
//    vector<vector<float>> data;
//    vector<Eigen::Isometry3d> T_b2c;

//    while (getline(inFile, lineStr)) // 从 inFile 中读取一行， 放到 lineStr 中
//    {
////        cout << lineStr << endl;

//        stringstream ss(lineStr); //读取内容放置在 ss流 中, 括号相当于初始化
//        string str;
//        vector<string> lineArray;
//        vector<float> _data;
//        // 按照逗号分隔
//        while (getline(ss, str, ',')) // ss 中， 按照 “，” 逗号分割将ss 分割成一个个str
//        {
//            lineArray.push_back(str);   // 将字符串放置到 line Array
////            cout << str << endl;
//            _data.push_back(str2int(str));
////            cout << str2int(str) << endl;
//        }
//        strArray.push_back(lineArray);
//        data.push_back(_data);

//        Eigen::Isometry3d _T_b2c;
//        _T_b2c.matrix() << _data[0], _data[1], _data[2], _data[3],
//                            _data[4], _data[5], _data[6], _data[7],
//                            _data[8], _data[9], _data[10], _data[11],
//                            _data[12], _data[13], _data[14], _data[15];
//        T_b2c.push_back(_T_b2c);
//    }

//    int start_row_num = 1;
//    int start_num = 1;
//    int end_num = 4;
//    string folder_str = "D:\\rgbd_pc_20220807\\data\\adaptive_0720\\zisu1_fix\\";

//    for(int i=start_num;i<=end_num;i++)
//    {
//        pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>());
//        pcl::io::loadPCDFile(folder_str+to_string(i)+"_pc.pcd", *cloud);
//        pcl::PointCloud<PointType> transformed_cloud;
//        pcl::transformPointCloud (*cloud, transformed_cloud, T_b2c[start_row_num+i-start_num-1].matrix());
//        string pc_path = folder_str + to_string(i) + "_base.pcd";
//        pcl::io::savePCDFileASCII (pc_path,transformed_cloud);
//        cout << i <<" is done!" << endl;
//    }

//     start_row_num = 5;
//     start_num = 5;
//     end_num = 8;
//     folder_str = "D:\\rgbd_pc_20220807\\data\\adaptive_0720\\zisu1_vertical\\";

//    for(int i=start_num;i<=end_num;i++)
//    {
//        pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>());
//        pcl::io::loadPCDFile(folder_str+to_string(i)+"_pc.pcd", *cloud);
//        pcl::PointCloud<PointType> transformed_cloud;
//        pcl::transformPointCloud (*cloud, transformed_cloud, T_b2c[start_row_num+i-start_num-1].matrix());
//        string pc_path = folder_str + to_string(i) + "_base.pcd";
//        pcl::io::savePCDFileASCII (pc_path,transformed_cloud);
//        cout << i <<" is done!" << endl;
//    }

//     start_row_num = 9;
//     start_num = 9;
//     end_num = 12;
//     folder_str = "D:\\rgbd_pc_20220807\\data\\adaptive_0720\\zisu1_adaptive\\";

//    for(int i=start_num;i<=end_num;i++)
//    {
//        pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>());
//        pcl::io::loadPCDFile(folder_str+to_string(i)+"_pc.pcd", *cloud);
//        pcl::PointCloud<PointType> transformed_cloud;
//        pcl::transformPointCloud (*cloud, transformed_cloud, T_b2c[start_row_num+i-start_num-1].matrix());
//        string pc_path = folder_str + to_string(i) + "_base.pcd";
//        pcl::io::savePCDFileASCII (pc_path,transformed_cloud);
//        cout << i <<" is done!" << endl;
//    }

//     start_row_num = 13;
//     start_num = 13;
//     end_num = 13;
//     folder_str = "D:\\rgbd_pc_20220807\\data\\adaptive_0720\\zisu1_adaptive_v2\\";

//    for(int i=start_num;i<=end_num;i++)
//    {
//        pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>());
//        pcl::io::loadPCDFile(folder_str+to_string(i)+"_pc.pcd", *cloud);
//        pcl::PointCloud<PointType> transformed_cloud;
//        pcl::transformPointCloud (*cloud, transformed_cloud, T_b2c[start_row_num+i-start_num-1].matrix());
//        string pc_path = folder_str + to_string(i) + "_base.pcd";
//        pcl::io::savePCDFileASCII (pc_path,transformed_cloud);
//        cout << i <<" is done!" << endl;
//    }

//     start_row_num = 14;
//     start_num = 1;
//     end_num = 4;
//     folder_str = "D:\\rgbd_pc_20220807\\data\\adaptive_0721\\zisu2_fix\\";

//    for(int i=start_num;i<=end_num;i++)
//    {
//        pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>());
//        pcl::io::loadPCDFile(folder_str+to_string(i)+"_pc.pcd", *cloud);
//        pcl::PointCloud<PointType> transformed_cloud;
//        pcl::transformPointCloud (*cloud, transformed_cloud, T_b2c[start_row_num+i-start_num-1].matrix());
//        string pc_path = folder_str + to_string(i) + "_base.pcd";
//        pcl::io::savePCDFileASCII (pc_path,transformed_cloud);
//        cout << i <<" is done!" << endl;
//    }

//     start_row_num = 18;
//     start_num = 5;
//     end_num = 10;
//     folder_str = "D:\\rgbd_pc_20220807\\data\\adaptive_0721\\zisu2_vertical\\";

//    for(int i=start_num;i<=end_num;i++)
//    {
//        pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>());
//        pcl::io::loadPCDFile(folder_str+to_string(i)+"_pc.pcd", *cloud);
//        pcl::PointCloud<PointType> transformed_cloud;
//        pcl::transformPointCloud (*cloud, transformed_cloud, T_b2c[start_row_num+i-start_num-1].matrix());
//        string pc_path = folder_str + to_string(i) + "_base.pcd";
//        pcl::io::savePCDFileASCII (pc_path,transformed_cloud);
//        cout << i <<" is done!" << endl;
//    }

//     start_row_num = 24;
//     start_num = 11;
//     end_num = 16;
//     folder_str = "D:\\rgbd_pc_20220807\\data\\adaptive_0721\\zisu2_adaptive\\";

//    for(int i=start_num;i<=end_num;i++)
//    {
//        pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>());
//        pcl::io::loadPCDFile(folder_str+to_string(i)+"_pc.pcd", *cloud);
//        pcl::PointCloud<PointType> transformed_cloud;
//        pcl::transformPointCloud (*cloud, transformed_cloud, T_b2c[start_row_num+i-start_num-1].matrix());
//        string pc_path = folder_str + to_string(i) + "_base.pcd";
//        pcl::io::savePCDFileASCII (pc_path,transformed_cloud);
//        cout << i <<" is done!" << endl;
//    }

//     start_row_num = 30;
//     start_num = 1;
//     end_num = 5;
//     folder_str = "D:\\rgbd_pc_20220807\\data\\adaptive_0722\\fanqie1_fix\\";

//    for(int i=start_num;i<=end_num;i++)
//    {
//        pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>());
//        pcl::io::loadPCDFile(folder_str+to_string(i)+"_pc.pcd", *cloud);
//        pcl::PointCloud<PointType> transformed_cloud;
//        pcl::transformPointCloud (*cloud, transformed_cloud, T_b2c[start_row_num+i-start_num-1].matrix());
//        string pc_path = folder_str + to_string(i) + "_base.pcd";
//        pcl::io::savePCDFileASCII (pc_path,transformed_cloud);
//        cout << i <<" is done!" << endl;
//    }

//     start_row_num = 35;
//     start_num = 6;
//     end_num = 10;
//     folder_str = "D:\\rgbd_pc_20220807\\data\\adaptive_0722\\fanqie1_vertical\\";

//    for(int i=start_num;i<=end_num;i++)
//    {
//        pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>());
//        pcl::io::loadPCDFile(folder_str+to_string(i)+"_pc.pcd", *cloud);
//        pcl::PointCloud<PointType> transformed_cloud;
//        pcl::transformPointCloud (*cloud, transformed_cloud, T_b2c[start_row_num+i-start_num-1].matrix());
//        string pc_path = folder_str + to_string(i) + "_base.pcd";
//        pcl::io::savePCDFileASCII (pc_path,transformed_cloud);
//        cout << i <<" is done!" << endl;
//    }

//     start_row_num = 40;
//     start_num = 11;
//     end_num = 15;
//     folder_str = "D:\\rgbd_pc_20220807\\data\\adaptive_0722\\fanqie1_adaptive\\";

//    for(int i=start_num;i<=end_num;i++)
//    {
//        pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>());
//        pcl::io::loadPCDFile(folder_str+to_string(i)+"_pc.pcd", *cloud);
//        pcl::PointCloud<PointType> transformed_cloud;
//        pcl::transformPointCloud (*cloud, transformed_cloud, T_b2c[start_row_num+i-start_num-1].matrix());
//        string pc_path = folder_str + to_string(i) + "_base.pcd";
//        pcl::io::savePCDFileASCII (pc_path,transformed_cloud);
//        cout << i <<" is done!" << endl;
//    }

//     start_row_num = 45;
//     start_num = 1;
//     end_num = 5;
//     folder_str = "D:\\rgbd_pc_20220807\\data\\adaptive_0722\\fanqie2_fix\\";

//    for(int i=start_num;i<=end_num;i++)
//    {
//        pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>());
//        pcl::io::loadPCDFile(folder_str+to_string(i)+"_pc.pcd", *cloud);
//        pcl::PointCloud<PointType> transformed_cloud;
//        pcl::transformPointCloud (*cloud, transformed_cloud, T_b2c[start_row_num+i-start_num-1].matrix());
//        string pc_path = folder_str + to_string(i) + "_base.pcd";
//        pcl::io::savePCDFileASCII (pc_path,transformed_cloud);
//        cout << i <<" is done!" << endl;
//    }

//     start_row_num = 50;
//     start_num = 6;
//     end_num = 10;
//     folder_str = "D:\\rgbd_pc_20220807\\data\\adaptive_0722\\fanqie2_vertical\\";

//    for(int i=start_num;i<=end_num;i++)
//    {
//        pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>());
//        pcl::io::loadPCDFile(folder_str+to_string(i)+"_pc.pcd", *cloud);
//        pcl::PointCloud<PointType> transformed_cloud;
//        pcl::transformPointCloud (*cloud, transformed_cloud, T_b2c[start_row_num+i-start_num-1].matrix());
//        string pc_path = folder_str + to_string(i) + "_base.pcd";
//        pcl::io::savePCDFileASCII (pc_path,transformed_cloud);
//        cout << i <<" is done!" << endl;
//    }

//     start_row_num = 55;
//     start_num = 11;
//     end_num = 15;
//     folder_str = "D:\\rgbd_pc_20220807\\data\\adaptive_0722\\fanqie2_adaptive\\";

//    for(int i=start_num;i<=end_num;i++)
//    {
//        pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>());
//        pcl::io::loadPCDFile(folder_str+to_string(i)+"_pc.pcd", *cloud);
//        pcl::PointCloud<PointType> transformed_cloud;
//        pcl::transformPointCloud (*cloud, transformed_cloud, T_b2c[start_row_num+i-start_num-1].matrix());
//        string pc_path = folder_str + to_string(i) + "_base.pcd";
//        pcl::io::savePCDFileASCII (pc_path,transformed_cloud);
//        cout << i <<" is done!" << endl;
//    }

//    cout <<"All is done!" << endl;

//    return 0;
//}

int main(int argc, char *argv[]) //transform from cam to base for each folder, ms+rgbd
{
   ifstream inFile("D:\\rgbd_pc_20220807\\data\\data_hemi.csv", ios::in);
   string lineStr;
   vector< vector<string> > strArray; //vector 类型文string
   vector<vector<float>> data;
   vector<Eigen::Isometry3d> T_b2c;

   while (getline(inFile, lineStr)) // 从 inFile 中读取一行， 放到 lineStr 中
   {
//        cout << lineStr << endl;

       stringstream ss(lineStr); //读取内容放置在 ss流 中, 括号相当于初始化
       string str;
       vector<string> lineArray;
       vector<float> _data;
       // 按照逗号分隔
       while (getline(ss, str, ',')) // ss 中， 按照 “，” 逗号分割将ss 分割成一个个str
       {
           lineArray.push_back(str);   // 将字符串放置到 line Array
//            cout << str << endl;
           _data.push_back(str2int(str));
//            cout << str2int(str) << endl;
       }
       strArray.push_back(lineArray);
       data.push_back(_data);

       Eigen::Isometry3d _T_b2c;
       _T_b2c.matrix() << _data[0], _data[1], _data[2], _data[3],
                           _data[4], _data[5], _data[6], _data[7],
                           _data[8], _data[9], _data[10], _data[11],
                           _data[12], _data[13], _data[14], _data[15];
       T_b2c.push_back(_T_b2c);
   }

   //folders
   bool is_transformed = true;//this is of no use
   bool is_mkv_frames = false;
   string folder_str_rgbd = "";
   string folder_str_ms = "";

   k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
   config.camera_fps = K4A_FRAMES_PER_SECOND_30;
   config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
   config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
   config.color_resolution = K4A_COLOR_RESOLUTION_720P;
   config.synchronized_images_only = true;

   const uint32_t deviceCount = k4a::device::get_installed_count();
   if (deviceCount == 0)
   {
       cout << "no azure kinect devices detected!" << endl;
   }

   cout << "Started opening K4A device..." << endl;
   k4a::device device = k4a::device::open(K4A_DEVICE_DEFAULT);
   device.start_cameras(&config);
   k4a::calibration calibration;
   k4a::transformation transformation;
   calibration = device.get_calibration(config.depth_mode, config.color_resolution);
   transformation = k4a::transformation(calibration);

   int start_row_num = 1;
   int start_num = 1;
   int end_num = 28;
   folder_str_rgbd = "D:\\rgbd_pc_20220807\\data\\20221208_hemi\\";
   folder_str_ms = "D:\\rgbd_pc_20220807\\data\\register_MS_hemi\\";

   if(!is_mkv_frames)
   {
       for(int i = start_num;i<=end_num;i++)
       {
           //if folders   (in order)
           string color_file_path = folder_str_rgbd+std::to_string(i)+"\\"+std::to_string(i)+"_color_uint8.png";
           string depth_file_path = folder_str_rgbd+std::to_string(i)+"\\"+std::to_string(i)+"_depth_uint16.png";
           vector<string> folder_str_ms_vec;
           for (int b = 1; b<=25; b++)
           {
               string  ms_file_path = folder_str_ms+"register_"+std::to_string(i)+"\\"+"b_"+std::to_string(b)+".png";
               folder_str_ms_vec.push_back(ms_file_path);
           }
           string save_file_path = folder_str_rgbd+std::to_string(i)+"_base_pc_with_zero.txt";
           cout << "Start processing "<< i << " ..."<< endl;
           frames2pc_v4(color_file_path, depth_file_path, folder_str_ms_vec, save_file_path, transformation, T_b2c[start_row_num+i-start_num-1]);
           cout << "--------"<< i << "--------"<<"Saving file done!" << endl;
       }
   }

   cout <<"All is done!" << endl;

   return 0;
}

//int main(int argc, char *argv[]) //transform from cam to base for each folder, vi+rgbd
//{
//    ifstream inFile("D:\\rgbd_pc_20220807\\data\\data.csv", ios::in);
//    string lineStr;
//    vector< vector<string> > strArray; //vector 类型文string
//    vector<vector<float>> data;
//    vector<Eigen::Isometry3d> T_b2c;

//    while (getline(inFile, lineStr)) // 从 inFile 中读取一行， 放到 lineStr 中
//    {
////        cout << lineStr << endl;

//        stringstream ss(lineStr); //读取内容放置在 ss流 中, 括号相当于初始化
//        string str;
//        vector<string> lineArray;
//        vector<float> _data;
//        // 按照逗号分隔
//        while (getline(ss, str, ',')) // ss 中， 按照 “，” 逗号分割将ss 分割成一个个str
//        {
//            lineArray.push_back(str);   // 将字符串放置到 line Array
////            cout << str << endl;
//            _data.push_back(str2int(str));
////            cout << str2int(str) << endl;
//        }
//        strArray.push_back(lineArray);
//        data.push_back(_data);

//        Eigen::Isometry3d _T_b2c;
//        _T_b2c.matrix() << _data[0], _data[1], _data[2], _data[3],
//                            _data[4], _data[5], _data[6], _data[7],
//                            _data[8], _data[9], _data[10], _data[11],
//                            _data[12], _data[13], _data[14], _data[15];
//        T_b2c.push_back(_T_b2c);
//    }

//    //folders
//    bool is_transformed = true;//this is of no use
//    bool is_mkv_frames = false;
//    string folder_str = "";

//    k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
//    config.camera_fps = K4A_FRAMES_PER_SECOND_30;
//    config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
//    config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
//    config.color_resolution = K4A_COLOR_RESOLUTION_720P;
//    config.synchronized_images_only = true;

//    const uint32_t deviceCount = k4a::device::get_installed_count();
//    if (deviceCount == 0)
//    {
//        cout << "no azure kinect devices detected!" << endl;
//    }

//    cout << "Started opening K4A device..." << endl;
//    k4a::device device = k4a::device::open(K4A_DEVICE_DEFAULT);
//    device.start_cameras(&config);
//    k4a::calibration calibration;
//    k4a::transformation transformation;
//    calibration = device.get_calibration(config.depth_mode, config.color_resolution);
//    transformation = k4a::transformation(calibration);

//    int start_row_num = 1;
//    int start_num = 1;
//    int end_num = 4;
//    folder_str = "D:\\rgbd_pc_20220807\\data\\adaptive_0720\\zisu1_fix\\";

//    if(!is_mkv_frames)
//    {
//        for(int i = start_num;i<=end_num;i++)
//        {
//            //if folders   (in order)
//            string color_file_path = folder_str+std::to_string(i)+"\\"+std::to_string(i)+"_color_uint8.png";
//            string depth_file_path = folder_str+std::to_string(i)+"\\"+std::to_string(i)+"_depth_uint16.png";
//            string rvi_file_path = folder_str+std::to_string(i)+"\\"+"registered_rvi.png";
////            string save_file_path = folder_str+std::to_string(i)+"_base_pc.txt";
//            string save_file_path = folder_str+std::to_string(i)+"_base_pc_no_zero.txt";
//            frames2pc_v3(color_file_path, depth_file_path, rvi_file_path, save_file_path, transformation, T_b2c[start_row_num+i-start_num-1]);
//            cout << "--------"<< i << "--------"<<"Saving file done!" << endl;
//        }
//    }

//     start_row_num = 5;
//     start_num = 5;
//     end_num = 8;
//     folder_str = "D:\\rgbd_pc_20220807\\data\\adaptive_0720\\zisu1_vertical\\";

//     if(!is_mkv_frames)
//     {
//         for(int i = start_num;i<=end_num;i++)
//         {
//             //if folders   (in order)
//             string color_file_path = folder_str+std::to_string(i)+"\\"+std::to_string(i)+"_color_uint8.png";
//             string depth_file_path = folder_str+std::to_string(i)+"\\"+std::to_string(i)+"_depth_uint16.png";
//             string rvi_file_path = folder_str+std::to_string(i)+"\\"+"registered_rvi.png";
////             string save_file_path = folder_str+std::to_string(i)+"_base_pc.txt";
//             string save_file_path = folder_str+std::to_string(i)+"_base_pc_no_zero.txt";
//             frames2pc_v3(color_file_path, depth_file_path, rvi_file_path, save_file_path, transformation, T_b2c[start_row_num+i-start_num-1]);
//             cout << "--------"<< i << "--------"<<"Saving file done!" << endl;
//         }
//     }

//     start_row_num = 9;
//     start_num = 9;
//     end_num = 12;
//     folder_str = "D:\\rgbd_pc_20220807\\data\\adaptive_0720\\zisu1_adaptive\\";

//     if(!is_mkv_frames)
//     {
//         for(int i = start_num;i<=end_num;i++)
//         {
//             //if folders   (in order)
//             string color_file_path = folder_str+std::to_string(i)+"\\"+std::to_string(i)+"_color_uint8.png";
//             string depth_file_path = folder_str+std::to_string(i)+"\\"+std::to_string(i)+"_depth_uint16.png";
//             string rvi_file_path = folder_str+std::to_string(i)+"\\"+"registered_rvi.png";
////             string save_file_path = folder_str+std::to_string(i)+"_base_pc.txt";
//             string save_file_path = folder_str+std::to_string(i)+"_base_pc_no_zero.txt";
//             frames2pc_v3(color_file_path, depth_file_path, rvi_file_path, save_file_path, transformation, T_b2c[start_row_num+i-start_num-1]);
//             cout << "--------"<< i << "--------"<<"Saving file done!" << endl;
//         }
//     }

//     start_row_num = 13;
//     start_num = 13;
//     end_num = 13;
//     folder_str = "D:\\rgbd_pc_20220807\\data\\adaptive_0720\\zisu1_adaptive_v2\\";

//     if(!is_mkv_frames)
//     {
//         for(int i = start_num;i<=end_num;i++)
//         {
//             //if folders   (in order)
//             string color_file_path = folder_str+std::to_string(i)+"\\"+std::to_string(i)+"_color_uint8.png";
//             string depth_file_path = folder_str+std::to_string(i)+"\\"+std::to_string(i)+"_depth_uint16.png";
//             string rvi_file_path = folder_str+std::to_string(i)+"\\"+"registered_rvi.png";
////             string save_file_path = folder_str+std::to_string(i)+"_base_pc.txt";
//             string save_file_path = folder_str+std::to_string(i)+"_base_pc_no_zero.txt";
//             frames2pc_v3(color_file_path, depth_file_path, rvi_file_path, save_file_path, transformation, T_b2c[start_row_num+i-start_num-1]);
//             cout << "--------"<< i << "--------"<<"Saving file done!" << endl;
//         }
//     }

//     start_row_num = 14;
//     start_num = 1;
//     end_num = 4;
//     folder_str = "D:\\rgbd_pc_20220807\\data\\adaptive_0721\\zisu2_fix\\";

//     if(!is_mkv_frames)
//     {
//         for(int i = start_num;i<=end_num;i++)
//         {
//             //if folders   (in order)
//             string color_file_path = folder_str+std::to_string(i)+"\\"+std::to_string(i)+"_color_uint8.png";
//             string depth_file_path = folder_str+std::to_string(i)+"\\"+std::to_string(i)+"_depth_uint16.png";
//             string rvi_file_path = folder_str+std::to_string(i)+"\\"+"registered_rvi.png";
////             string save_file_path = folder_str+std::to_string(i)+"_base_pc.txt";
//             string save_file_path = folder_str+std::to_string(i)+"_base_pc_no_zero.txt";
//             frames2pc_v3(color_file_path, depth_file_path, rvi_file_path, save_file_path, transformation, T_b2c[start_row_num+i-start_num-1]);
//             cout << "--------"<< i << "--------"<<"Saving file done!" << endl;
//         }
//     }

//     start_row_num = 18;
//     start_num = 5;
//     end_num = 10;
//     folder_str = "D:\\rgbd_pc_20220807\\data\\adaptive_0721\\zisu2_vertical\\";

//     if(!is_mkv_frames)
//     {
//         for(int i = start_num;i<=end_num;i++)
//         {
//             //if folders   (in order)
//             string color_file_path = folder_str+std::to_string(i)+"\\"+std::to_string(i)+"_color_uint8.png";
//             string depth_file_path = folder_str+std::to_string(i)+"\\"+std::to_string(i)+"_depth_uint16.png";
//             string rvi_file_path = folder_str+std::to_string(i)+"\\"+"registered_rvi.png";
////             string save_file_path = folder_str+std::to_string(i)+"_base_pc.txt";
//             string save_file_path = folder_str+std::to_string(i)+"_base_pc_no_zero.txt";
//             frames2pc_v3(color_file_path, depth_file_path, rvi_file_path, save_file_path, transformation, T_b2c[start_row_num+i-start_num-1]);
//             cout << "--------"<< i << "--------"<<"Saving file done!" << endl;
//         }
//     }

//     start_row_num = 24;
//     start_num = 11;
//     end_num = 16;
//     folder_str = "D:\\rgbd_pc_20220807\\data\\adaptive_0721\\zisu2_adaptive\\";

//     if(!is_mkv_frames)
//     {
//         for(int i = start_num;i<=end_num;i++)
//         {
//             //if folders   (in order)
//             string color_file_path = folder_str+std::to_string(i)+"\\"+std::to_string(i)+"_color_uint8.png";
//             string depth_file_path = folder_str+std::to_string(i)+"\\"+std::to_string(i)+"_depth_uint16.png";
//             string rvi_file_path = folder_str+std::to_string(i)+"\\"+"registered_rvi.png";
////             string save_file_path = folder_str+std::to_string(i)+"_base_pc.txt";
//             string save_file_path = folder_str+std::to_string(i)+"_base_pc_no_zero.txt";
//             frames2pc_v3(color_file_path, depth_file_path, rvi_file_path, save_file_path, transformation, T_b2c[start_row_num+i-start_num-1]);
//             cout << "--------"<< i << "--------"<<"Saving file done!" << endl;
//         }
//     }

//     start_row_num = 30;
//     start_num = 1;
//     end_num = 5;
//     folder_str = "D:\\rgbd_pc_20220807\\data\\adaptive_0722\\fanqie1_fix\\";

//     if(!is_mkv_frames)
//     {
//         for(int i = start_num;i<=end_num;i++)
//         {
//             //if folders   (in order)
//             string color_file_path = folder_str+std::to_string(i)+"\\"+std::to_string(i)+"_color_uint8.png";
//             string depth_file_path = folder_str+std::to_string(i)+"\\"+std::to_string(i)+"_depth_uint16.png";
//             string rvi_file_path = folder_str+std::to_string(i)+"\\"+"registered_rvi.png";
////             string save_file_path = folder_str+std::to_string(i)+"_base_pc.txt";
//             string save_file_path = folder_str+std::to_string(i)+"_base_pc_no_zero.txt";
//             frames2pc_v3(color_file_path, depth_file_path, rvi_file_path, save_file_path, transformation, T_b2c[start_row_num+i-start_num-1]);
//             cout << "--------"<< i << "--------"<<"Saving file done!" << endl;
//         }
//     }

//     start_row_num = 35;
//     start_num = 6;
//     end_num = 10;
//     folder_str = "D:\\rgbd_pc_20220807\\data\\adaptive_0722\\fanqie1_vertical\\";

//     if(!is_mkv_frames)
//     {
//         for(int i = start_num;i<=end_num;i++)
//         {
//             //if folders   (in order)
//             string color_file_path = folder_str+std::to_string(i)+"\\"+std::to_string(i)+"_color_uint8.png";
//             string depth_file_path = folder_str+std::to_string(i)+"\\"+std::to_string(i)+"_depth_uint16.png";
//             string rvi_file_path = folder_str+std::to_string(i)+"\\"+"registered_rvi.png";
////             string save_file_path = folder_str+std::to_string(i)+"_base_pc.txt";
//             string save_file_path = folder_str+std::to_string(i)+"_base_pc_no_zero.txt";
//             frames2pc_v3(color_file_path, depth_file_path, rvi_file_path, save_file_path, transformation, T_b2c[start_row_num+i-start_num-1]);
//             cout << "--------"<< i << "--------"<<"Saving file done!" << endl;
//         }
//     }

//     start_row_num = 40;
//     start_num = 11;
//     end_num = 15;
//     folder_str = "D:\\rgbd_pc_20220807\\data\\adaptive_0722\\fanqie1_adaptive\\";

//     if(!is_mkv_frames)
//     {
//         for(int i = start_num;i<=end_num;i++)
//         {
//             //if folders   (in order)
//             string color_file_path = folder_str+std::to_string(i)+"\\"+std::to_string(i)+"_color_uint8.png";
//             string depth_file_path = folder_str+std::to_string(i)+"\\"+std::to_string(i)+"_depth_uint16.png";
//             string rvi_file_path = folder_str+std::to_string(i)+"\\"+"registered_rvi.png";
////             string save_file_path = folder_str+std::to_string(i)+"_base_pc.txt";
//             string save_file_path = folder_str+std::to_string(i)+"_base_pc_no_zero.txt";
//             frames2pc_v3(color_file_path, depth_file_path, rvi_file_path, save_file_path, transformation, T_b2c[start_row_num+i-start_num-1]);
//             cout << "--------"<< i << "--------"<<"Saving file done!" << endl;
//         }
//     }

//     start_row_num = 45;
//     start_num = 1;
//     end_num = 5;
//     folder_str = "D:\\rgbd_pc_20220807\\data\\adaptive_0722\\fanqie2_fix\\";

//     if(!is_mkv_frames)
//     {
//         for(int i = start_num;i<=end_num;i++)
//         {
//             //if folders   (in order)
//             string color_file_path = folder_str+std::to_string(i)+"\\"+std::to_string(i)+"_color_uint8.png";
//             string depth_file_path = folder_str+std::to_string(i)+"\\"+std::to_string(i)+"_depth_uint16.png";
//             string rvi_file_path = folder_str+std::to_string(i)+"\\"+"registered_rvi.png";
////             string save_file_path = folder_str+std::to_string(i)+"_base_pc.txt";
//             string save_file_path = folder_str+std::to_string(i)+"_base_pc_no_zero.txt";
//             frames2pc_v3(color_file_path, depth_file_path, rvi_file_path, save_file_path, transformation, T_b2c[start_row_num+i-start_num-1]);
//             cout << "--------"<< i << "--------"<<"Saving file done!" << endl;
//         }
//     }

//     start_row_num = 50;
//     start_num = 6;
//     end_num = 10;
//     folder_str = "D:\\rgbd_pc_20220807\\data\\adaptive_0722\\fanqie2_vertical\\";

//     if(!is_mkv_frames)
//     {
//         for(int i = start_num;i<=end_num;i++)
//         {
//             //if folders   (in order)
//             string color_file_path = folder_str+std::to_string(i)+"\\"+std::to_string(i)+"_color_uint8.png";
//             string depth_file_path = folder_str+std::to_string(i)+"\\"+std::to_string(i)+"_depth_uint16.png";
//             string rvi_file_path = folder_str+std::to_string(i)+"\\"+"registered_rvi.png";
////             string save_file_path = folder_str+std::to_string(i)+"_base_pc.txt";
//             string save_file_path = folder_str+std::to_string(i)+"_base_pc_no_zero.txt";
//             frames2pc_v3(color_file_path, depth_file_path, rvi_file_path, save_file_path, transformation, T_b2c[start_row_num+i-start_num-1]);
//             cout << "--------"<< i << "--------"<<"Saving file done!" << endl;
//         }
//     }

//     start_row_num = 55;
//     start_num = 11;
//     end_num = 15;
//     folder_str = "D:\\rgbd_pc_20220807\\data\\adaptive_0722\\fanqie2_adaptive\\";

//     if(!is_mkv_frames)
//     {
//         for(int i = start_num;i<=end_num;i++)
//         {
//             //if folders   (in order)
//             string color_file_path = folder_str+std::to_string(i)+"\\"+std::to_string(i)+"_color_uint8.png";
//             string depth_file_path = folder_str+std::to_string(i)+"\\"+std::to_string(i)+"_depth_uint16.png";
//             string rvi_file_path = folder_str+std::to_string(i)+"\\"+"registered_rvi.png";
////             string save_file_path = folder_str+std::to_string(i)+"_base_pc.txt";
//             string save_file_path = folder_str+std::to_string(i)+"_base_pc_no_zero.txt";
//             frames2pc_v3(color_file_path, depth_file_path, rvi_file_path, save_file_path, transformation, T_b2c[start_row_num+i-start_num-1]);
//             cout << "--------"<< i << "--------"<<"Saving file done!" << endl;
//         }
//     }

//    cout <<"All is done!" << endl;

//    return 0;
//}

//int main(int argc, char *argv[])// for converting msi+rgbd into ms_point cloud
//{
//    QCoreApplication a(argc, argv);
//    int start_num = 2;
//    int end_num = 2;
//    //folders
//    bool is_transformed = true;//this is of no use
//    bool is_mkv_frames = false;
//    string tempFolder = "D:\\rgbd_pc_20220807\\data";

//    k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
//    config.camera_fps = K4A_FRAMES_PER_SECOND_30;
//    config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
//    config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
//    config.color_resolution = K4A_COLOR_RESOLUTION_720P;
//    config.synchronized_images_only = true;

//    const uint32_t deviceCount = k4a::device::get_installed_count();
//    if (deviceCount == 0)
//    {
//        cout << "no azure kinect devices detected!" << endl;
//    }

//    cout << "Started opening K4A device..." << endl;
//    k4a::device device = k4a::device::open(K4A_DEVICE_DEFAULT);
//    device.start_cameras(&config);
//    k4a::calibration calibration;
//    k4a::transformation transformation;
//    calibration = device.get_calibration(config.depth_mode, config.color_resolution);
//    transformation = k4a::transformation(calibration);

//    if(!is_mkv_frames)
//    {
//        for(int i = start_num;i<=end_num;i++)
//        {
//            //if folders   (in order)
//            string color_file_path = tempFolder+"\\"+std::to_string(i)+"\\"+std::to_string(i)+"_color_uint8.png";
//            string depth_file_path = tempFolder+"\\"+std::to_string(i)+"\\"+std::to_string(i)+"_depth_uint16.png";
//            string rvi_file_path = tempFolder+"\\"+std::to_string(i)+"\\"+"registered_rvi.png";
//            string save_file_path = tempFolder+"\\"+std::to_string(i)+"_pc.txt";
////            frames2pc(color_file_path,depth_file_path,save_file_path,is_transformed,is_mkv_frames,transformation);
//            frames2pc_v2(color_file_path, depth_file_path, rvi_file_path, save_file_path, transformation);
//            cout << "--------"<< i << "--------"<<"Saving file done!" << endl;
//        }
//    }

//    cout << "--------"<< "All" << "--------"<<"Saving file done!" << endl;
//    return a.exec();
////    return 0;
//}

//int main(int argc, char *argv[])// for converting frames to point clouds
//{
//    QCoreApplication a(argc, argv);
//    int start_num = 1;
//    int end_num = 284;
//    //mkv_frames
//    bool is_transformed = false;
//    bool is_mkv_frames = true;
////    string tempFolder = "D:\\rgbd_pc_20220807\\data\\hq2_fine";
////    string tempFolder = "D:\\rgbd_pc_20220807\\data\\zz_date2_2_fine";
////    string tempFolder = "D:\\rgbd_pc_20220807\\data";
//    string tempFolder = "D:\\rgbd_pc_20220807\\data\\20231116\\13-180";

////    //folders
////    bool is_transformed = true;
////    bool is_mkv_frames = false;
////    string tempFolder = "D:\\rgbd_pc_20220807\\data\\PSed_frames_1234\\4_";

//    k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
//    config.camera_fps = K4A_FRAMES_PER_SECOND_30;
//    config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
//    config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
//    config.color_resolution = K4A_COLOR_RESOLUTION_720P;
//    config.synchronized_images_only = true;

//    const uint32_t deviceCount = k4a::device::get_installed_count();
//    if (deviceCount == 0)
//    {
//        cout << "no azure kinect devices detected!" << endl;
//    }

//    cout << "Started opening K4A device..." << endl;
//    k4a::device device = k4a::device::open(K4A_DEVICE_DEFAULT);
//    device.start_cameras(&config);
//    k4a::calibration calibration;
//    k4a::transformation transformation;
//    calibration = device.get_calibration(config.depth_mode, config.color_resolution);
//    transformation = k4a::transformation(calibration);

//    //// I have not find any issues in codes below, they can not be used now!
////    std::basic_ifstream<uint8_t> fin(calibration_file_path);
////    auto eos = std::istreambuf_iterator<uint8_t>();
////    auto buffer = std::vector<uint8_t>(std::istreambuf_iterator<uint8_t>(fin),eos);

////    k4a::calibration calibration = k4a::calibration::get_from_raw(buffer.data(),
////                                                 buffer.size(),
////                                                 K4A_DEPTH_MODE_NFOV_UNBINNED,
////                                                 K4A_COLOR_RESOLUTION_720P);

////    k4a::transformation transformation = k4a::transformation(calibration);

//    if(!is_mkv_frames)
//    {
//        for(int i = start_num;i<=end_num;i++)
//        {
//            //if folders   (in order)
//            string color_file_path = tempFolder+"\\"+std::to_string(i)+"\\"+std::to_string(i)+"_color_uint8.png";
//            string depth_file_path = tempFolder+"\\"+std::to_string(i)+"\\"+std::to_string(i)+"_depth_uint16.png";
//        //    string calibration_file_path = "D:\\rgbd_pc_20220807\\data\\1_calibration.json";
//            string save_file_path = tempFolder+"\\"+std::to_string(i)+"_pc.pcd";
//            frames2pc(color_file_path,depth_file_path,save_file_path,is_transformed,is_mkv_frames,transformation);
//            cout << "--------"<< i << "--------"<<"Saving file done!" << endl;
//        }
//    }
//    else
//    {
//        //if mkv frames (no order)
//        vector<string> inputFiles;
//        filesystem::path p1(tempFolder);

//        if (filesystem::exists(p1))
//        {
//            for (const auto& entry : filesystem::directory_iterator(tempFolder))
//            {
//                string filename = entry.path().filename().string();
//                if (endsWith(filename, "png"))
//                {
//                    inputFiles.push_back(filename);
//                }
//            }
//        }

//        for(int i = 0;i<inputFiles.size()/2;i++)
//        {
//            string color_file_path = tempFolder+"\\"+inputFiles[i];
//            cout<<color_file_path<<endl;
//            string depth_file_path = tempFolder+"\\"+inputFiles[inputFiles.size()/2+i];
//            cout<<depth_file_path<<endl;
//            string save_file_path = tempFolder+"\\"+std::to_string(i+1)+"_pc.pcd";
//            cout<<save_file_path<<endl;
//            cout<<"------------------------"<<endl;
//            frames2pc(color_file_path,depth_file_path,save_file_path,is_transformed,is_mkv_frames,transformation);
//            cout << "--------"<< i+1 << "--------"<<"Saving file done!" << endl;
//        }

////        //csv saving for obb and aabb of xilanhua
////        ofstream outFile;
////        outFile.open("bounding_box_data_of_xilanhua.csv", ios::out); // 打开模式可省略
////        outFile << "No." << ',' <<"w_obb"<<','<<"h_obb"<<','<<"d_obb"<<','<<"S_ellipse"<<','<<"C_ellipse"<<','<<"S_projection"<<','<<"C_projection"<<endl;
//////        outFile << "No." << ',' << "w_aabb" << ',' << "l_aabb" << ','<<"h_aabb"<<','<<"w_obb"<<','<<"l_obb"<<','<<"h_obb"<<endl;

////        for(int i = 0;i<inputFiles.size()/2;i++)
////        {
////            outFile << to_string(i+1) << ','<<to_string(double(whds[i](0))) << ',' << to_string(double(whds[i](1))) << ',' << to_string(double(whds[i](2))) <<',' << to_string(double(Ss_ellipse[i])) <<',' << to_string(double(Cs_ellipse[i])) <<',' << to_string(double(Ss[i])) <<',' << to_string(double(Cs[i])) <<endl;
////        }

////        std::cout<<"csv saving done!"<<std::endl;
////        outFile.close();
//    }

//    cout << "--------"<< "All" << "--------"<<"Saving file done!" << endl;
//    return a.exec();
////    return 0;
//}

//int main(int argc, char *argv[])//for convex reconstruction
//{
//    QCoreApplication a(argc, argv);

//    pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);
//    pcl::PCDReader reader;
//    reader.read("D:\\rgbd_pc_20220807\\data\\26_pc.pcd", *cloud);
//    //time count in ms us
//    auto t1 = system_clock::now();


//    pcl::ConvexHull<PointType> hull;
//    hull.setInputCloud(cloud);
//    hull.setDimension(3);
//    vector<pcl::Vertices> polygons;

//    pcl::PointCloud<PointType>::Ptr surface_hull(new pcl::PointCloud<PointType>);

//    hull.setComputeAreaVolume(true);
//    hull.reconstruct(*surface_hull, polygons);
//    float Area = hull.getTotalArea();
//    float Volume = hull.getTotalVolume();
//    cout << "area:" << Area << endl;
//    cout << "volume:" << Volume << endl;
//    auto t2 = system_clock::now();

//    // floating-point duration: no duration_cast needed
//    duration<double, std::milli> fp_ms = t2 - t1;

//    // integral duration: requires duration_cast
//    auto int_ms = duration_cast<milliseconds>(fp_ms);

//    // converting integral duration to integral duration of shorter divisible time unit: no duration_cast needed
//    duration<long, std::micro> int_usec = int_ms;

//    std::cout << "took " << fp_ms.count() << " ms, "
//              << "or " << int_ms.count() << " whole milliseconds "
//              << "(which is " << int_usec.count() << " whole microseconds)" << std::endl;

//    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("crophull display"));
//    //for_visualizer_v->setBackgroundColor(255,255,255);

//    viewer->setWindowName("visualization");
//    viewer->setBackgroundColor(0, 0, 0 );
//    pcl::visualization::PointCloudColorHandlerGenericField<PointType> fildColor(cloud, "z");
//    viewer->addPointCloud<PointType>(cloud, fildColor, "cloud");
//    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");


//    viewer->addPolygonMesh<PointType>(surface_hull, polygons,"backview_hull_polyline");
//    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "backview_hull_polyline");
//    viewer->setRepresentationToWireframeForAllActors();

//    while (!viewer->wasStopped())
//    {

//        viewer->spinOnce(1000);
//        boost::this_thread::sleep(boost::posix_time::microseconds(1000));

//    }
//    return a.exec();
////    return 0;
//}

//int main(int argc, char *argv[]) //hemisphere point cloud & poses
//{
//    float x0 = 0.6;
//    float y0 = 0;
//    float z0 = 0.3;
//    float r0 = 0.05;
//    float r1 = 0.35;

//    pcl::PointCloud<PointType>::Ptr hemi_cloud(new pcl::PointCloud<PointType>);
//    pcl::PointCloud<PointType>::Ptr pose_cloud(new pcl::PointCloud<PointType>);
//    vector<Eigen::Vector3f> X_norm;
//    vector<Eigen::Vector3f> Y_norm;
//    vector<Eigen::Vector3f> Z_norm;
////    vector<Eigen::Isometry3d> T_base2cam;
////    vector<Eigen::Isometry3d> T_base2tcp;
//    vector<int> isAbleToArrive;

//    ofstream outFile;
//    outFile.open("res.csv", ios::out);

//    for(int theta_ = 0; theta_ < 360; theta_ = theta_ + 1)
//        for(int phai_ = 0; phai_ <= 90; phai_ = phai_ + 1)
//        {
//            float theta = float(theta_)*M_PI/180.0;
//            float phai = float(phai_)*M_PI/180.0;
//            PointType hc;
//            hc.x = r0*cos(theta)*sin(phai)+x0; hc.y = r0*sin(theta)*sin(phai)+y0; hc.z = r0*cos(phai)+z0;
//            hc.r = 1; hc.g = 1; hc.b = 1;
//            hemi_cloud->points.push_back(hc);
//        }

//    for(int theta_ = 0; theta_ < 360; theta_ = theta_ + 15)
//        for(int phai_ = 0; phai_ < 90; phai_ = phai_ + 15)
//        {
//            float theta = float(theta_)*M_PI/180.0;
//            float phai = float(phai_)*M_PI/180.0;
//            PointType pc; pc.x = r1*cos(theta)*sin(phai)+x0; pc.y = r1*sin(theta)*sin(phai)+y0; pc.z = r1*cos(phai)+z0;
//            Eigen::Vector3f X(r1*cos(theta)/sin(phai)-r1*cos(theta)*sin(phai),r1*sin(theta)/sin(phai)-r1*sin(theta)*sin(phai),-r1*cos(phai));
//            Eigen::Vector3f Z(-r1*cos(theta)*sin(phai),-r1*sin(theta)*sin(phai),-r1*cos(phai));
//            vec3fNorm(X);
//            vec3fNorm(Z);
//            Eigen::Vector3f Y = vec3fCross(Z, X);
//            Eigen::Isometry3d T_b2c;
//            T_b2c.matrix() << cos(theta)*cos(phai), sin(theta), -cos(theta)*sin(phai), pc.x,
//                                sin(theta)*cos(phai), -cos(theta), -sin(theta)*sin(phai), pc.y,
//                                -sin(phai), 0, -cos(phai), pc.z,
//                                0,0,0,1;
//            Eigen::Isometry3d T_t2c;
//            T_t2c.matrix() << 0, 0.99955, 0.029996, -0.03459,
//                                  0, -0.03, 0.99955, 0.065924,
//                                  1, 0, 0, 0.12,
//                                  0, 0, 0, 1;
//            Eigen::Isometry3d T_b2t;
//            T_b2t.matrix() = T_b2c.matrix() * T_t2c.inverse().matrix();
//            double* T = new double[16];
//            T[0] = T_b2t.matrix()(0,0);  T[1] =T_b2t.matrix()(0,1);    T[2] = T_b2t.matrix()(0,2);   T[3] = T_b2t.matrix()(0,3);
//            T[4] = T_b2t.matrix()(1,0);  T[5] = T_b2t.matrix()(1,1);   T[6] = T_b2t.matrix()(1,2);   T[7] = T_b2t.matrix()(1,3);
//            T[8] = T_b2t.matrix()(2,0);  T[9] =T_b2t.matrix()(2,1) ;   T[10] = T_b2t.matrix()(2,2) ; T[11] = T_b2t.matrix()(2,3);
//            T[12] = T_b2t.matrix()(3,0); T[13] = T_b2t.matrix()(3,1) ; T[14] = T_b2t.matrix()(3,2) ; T[15] = T_b2t.matrix()(3,3);
//            double q_sols[8 * 6];
//            int num_sols;
//            num_sols = ur_kinematics::inverse(T, q_sols);
//            int isAbleToArrive_;
//            if (num_sols == 0)
//            {
//                isAbleToArrive_ = 0;
//            }
//            else
//            {
//                isAbleToArrive_ = 1;
//                outFile << to_string(q_sols[0]) << ',' << to_string(q_sols[1]) << ',' << to_string(q_sols[2]) << ',' << to_string(q_sols[3]) << ',' << to_string(q_sols[4]) << ',' << to_string(q_sols[5]) << endl;
//            }

//            pose_cloud->points.push_back(pc);
//            X_norm.push_back(X);
//            Y_norm.push_back(Y);
//            Z_norm.push_back(Z);
//            isAbleToArrive.push_back(isAbleToArrive_);
//        }

//    outFile.close();

//    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("display"));
//    viewer->setBackgroundColor (0, 0, 0);
//    viewer->addCoordinateSystem (0.2);
//    viewer->initCameraParameters ();
//    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,2,"hemi_cloud");
//    pcl::visualization::PointCloudColorHandlerCustom<PointType> rgb(hemi_cloud, 255, 255, 255);
//    viewer->addPointCloud<PointType> (hemi_cloud,rgb,"hemi_cloud");
//    for(int i = 0;i<isAbleToArrive.size();i++)
//    {
//        std::string str1 = QString("major eigen vector %1").arg(i).toStdString();
//        std::string str2 = QString("middle eigen vector %1").arg(i).toStdString();
//        std::string str3 = QString("minor eigen vector %1").arg(i).toStdString();
//        pcl::PointXYZ center (pose_cloud->points[i].x,pose_cloud->points[i].y, pose_cloud->points[i].z);
//        pcl::PointXYZ x_axis (SCALE_VEC*X_norm[i](0) + center.x, SCALE_VEC*X_norm[i](1) + center.y, SCALE_VEC*X_norm[i](2) + center.z);
//        pcl::PointXYZ y_axis (SCALE_VEC*Y_norm[i](0) + center.x, SCALE_VEC*Y_norm[i](1) + center.y, SCALE_VEC*Y_norm[i](2) + center.z);
//        pcl::PointXYZ z_axis (SCALE_VEC*Z_norm[i](0) + center.x, SCALE_VEC*Z_norm[i](1) + center.y, SCALE_VEC*Z_norm[i](2) + center.z);
//        if(isAbleToArrive[i]==1)
//        {
//            viewer->addLine (center, x_axis, 1.0f, 0.0f, 0.0f, str1);
//            viewer->addLine (center, y_axis, 0.0f, 1.0f, 0.0f, str2);
//            viewer->addLine (center, z_axis, 0.0f, 0.0f, 1.0f, str3);
//        }
//        else
//        {
//            viewer->addLine (center, x_axis, 1.0f, 1.0f, 1.0f, str1);
//            viewer->addLine (center, y_axis, 1.0f, 1.0f, 1.0f, str2);
//            viewer->addLine (center, z_axis, 1.0f, 1.0f, 1.0f, str3);
//        }
//    }
//    while(!viewer->wasStopped())
//    {
//        viewer->spinOnce (100);
//        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
//    }
//}

// int main(int argc, char *argv[]) //plant convex & aabb & obb & poses
// {
//     //variables initialization
//     pcl::PointCloud<PointType>::Ptr cloud_convex(new pcl::PointCloud<PointType>);
//     pcl::PointCloud<PointType>::Ptr cloud_aabb(new pcl::PointCloud<PointType>);
//     pcl::PointCloud<PointType>::Ptr cloud_obb(new pcl::PointCloud<PointType>);
//     vector<pcl::PointCloud<PointType>> pc_clusters_obb;
//     vector <float> feature_values;//no use
//     Eigen::Vector3f whd;//no use
//     vector <PointType> bboxPoints;
//     Eigen::Matrix3f rotational_matrix_OBB;
//     vector<Eigen::Vector3f> feature_vectors;
//     vector<vector<double>> posFromAABB;
//     vector<vector<double>> posFromOBB;
//     vector<Eigen::Vector3d> orientation_AABB;//x1 y1 z1 x2 y2 z2 ...
//     vector<Eigen::Vector3d> orientation_OBB;
//     vector<vector <PointType>> obbPoints_vec;
//     vector<Eigen::Matrix3f> rotational_matrix_OBB_vec;
//     vector<vector<Eigen::Vector3f>> feature_vectors_vec;

//     //reader
//     pcl::PCDReader reader;
// //    string tempFolder = "D:\\rgbd_pc_20220807\\data\\bb_parts\\fanqie1";
//     string tempFolder = "D:\\rgbd_pc_20220807\\data\\rapeseed_parts_v2\\10";
//     string file_path = "";
//     file_path = tempFolder+"_adapt.pcd";
//     reader.read(file_path, *cloud_convex);
//     file_path = tempFolder+"_aabb.pcd";
//     reader.read(file_path, *cloud_aabb);
//     file_path = tempFolder+"_obb.pcd";
//     reader.read(file_path, *cloud_obb);
//     for(int i = 1;i<=8;i++)
//     {
//         file_path = tempFolder+"_obb_"+to_string(i)+".pcd";
//         pcl::PointCloud<PointType>::Ptr cloud_temp(new pcl::PointCloud<PointType>);
//         reader.read(file_path, *cloud_temp);
//         pc_clusters_obb.push_back(*cloud_temp);
//     }

//     //time count in ms us
//     auto t1 = system_clock::now();

//     //aabb estimation
//     pcaOBB(cloud_aabb,//pc in
//             whd, //w h d
//             bboxPoints ,//min_point_AABB, max_point_AABB, min_point_OBB, max_point_OBB, position_OBB
//             rotational_matrix_OBB ,// rotational_matrix_OBB
//             feature_values ,//major_value, middle_value, minor_value
//             feature_vectors//  Eigen::Vector3f major_vector, middle_vector, minor_vector, mass_center;
//             );

//     //6D pos estimation according to estimated aabbs
//     vector<Eigen::Isometry3d> T_base2tcp_AABB;
//     vector<PointType> AABBPoints(bboxPoints.begin(), bboxPoints.begin() + 2);//first two elements of bboxPoints vec
//     posCalculationFromAABB(AABBPoints, posFromAABB,T_base2tcp_AABB,orientation_AABB);

//     //obb calculation for each part/cluster of the plant
//      for(int i=0;i<pc_clusters_obb.size();i++)
//      {
//          vector <PointType> bboxPoints_;
//          Eigen::Matrix3f rotational_matrix_OBB_;
//          vector<Eigen::Vector3f> feature_vectors_;
//          pcaOBB(pc_clusters_obb[i].makeShared(),//pc in
//                 whd, //w h d
//                 bboxPoints_,//min_point_AABB, max_point_AABB, min_point_OBB, max_point_OBB, position_OBB
//                 rotational_matrix_OBB_,// rotational_matrix_OBB
//                 feature_values,//major_value, middle_value, minor_value
//                 feature_vectors_//  Eigen::Vector3f major_vector, middle_vector, minor_vector, mass_center;
//                 );
//          vector<PointType> OBBPoints(bboxPoints_.end() - 3, bboxPoints_.end());//last three elements of bboxPoints vec
//          obbPoints_vec.push_back(OBBPoints);
//          rotational_matrix_OBB_vec.push_back(rotational_matrix_OBB_);
//          feature_vectors_vec.push_back(feature_vectors_);
//      }

//      //6D pos estimation according to estimated obbs
//      vector<Eigen::Isometry3d> T_base2tcp_OBB;
//      for(int i=0;i<obbPoints_vec.size();i++)
//      {
//          vector<double>  posFromOBB_;
//          Eigen::Isometry3d T_base2tcp_OBB_;
//          posCalculationFromOBB(obbPoints_vec[i], feature_vectors_vec[i], posFromOBB_,T_base2tcp_OBB_,orientation_OBB);
//          posFromOBB.push_back(posFromOBB_);
//          T_base2tcp_OBB.push_back(T_base2tcp_OBB_);
//      }

//     //convex
//     pcl::ConvexHull<PointType> hull;
//     hull.setInputCloud(cloud_convex);
//     hull.setDimension(3);
//     vector<pcl::Vertices> polygons;

//     pcl::PointCloud<PointType>::Ptr surface_hull(new pcl::PointCloud<PointType>);

//     hull.setComputeAreaVolume(true);
//     hull.reconstruct(*surface_hull, polygons);
//     float Area = hull.getTotalArea();
//     float Volume = hull.getTotalVolume();
//     cout << "area:" << Area << endl;
//     cout << "volume:" << Volume << endl;
//     auto t2 = system_clock::now();

//     // floating-point duration: no duration_cast needed
//     duration<double, std::milli> fp_ms = t2 - t1;

//     // integral duration: requires duration_cast
//     auto int_ms = duration_cast<milliseconds>(fp_ms);

//     // converting integral duration to integral duration of shorter divisible time unit: no duration_cast needed
//     duration<long, std::micro> int_usec = int_ms;

//     std::cout << "took " << fp_ms.count() << " ms, "
//               << "or " << int_ms.count() << " whole milliseconds "
//               << "(which is " << int_usec.count() << " whole microseconds)" << std::endl;

//     boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("display"));
//     viewer->setWindowName("visualization");
// //    viewer->setBackgroundColor(0, 0, 0 );
//     viewer->setBackgroundColor(255, 255, 255 );
//     viewer->addCoordinateSystem (0.2);
//     viewer->initCameraParameters ();
//     viewer->setCameraPosition(-0.478138,1.037314,1.057774,0.380845,-0.289120,0.878275);

// //    //convex visulization
// //    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud_convex");

// //    viewer->addPolygonMesh<PointType>(surface_hull, polygons,"backview_hull_polyline");
// //    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "backview_hull_polyline");
// //    viewer->setRepresentationToWireframeForAllActors();

// //    //aabb visualization
// //    PointType min_point_AABB = bboxPoints[0];
// //    PointType max_point_AABB = bboxPoints[1];

// //    viewer->addPointCloud<PointType> (cloud_aabb,"aabb cloud");
// //    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,2,"aabb cloud");
// //    viewer->addCube (min_point_AABB.x, max_point_AABB.x, min_point_AABB.y, max_point_AABB.y, min_point_AABB.z, max_point_AABB.z, 1.0, 0.0, 0.0, "AABB");
// //    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY,0.5,"AABB");
// //    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH,0.5,"AABB");
// //    for(int i = 0;i<posFromAABB.size();i++)
// //    {
// //        Eigen::Vector3f major_vector = orientation_AABB[3*i+0].cast<float>();
// //        Eigen::Vector3f middle_vector = orientation_AABB[3*i+1].cast<float>();
// //        Eigen::Vector3f minor_vector = orientation_AABB[3*i+2].cast<float>();
// //        Eigen::Vector3f mass_center;
// //        mass_center(0) = float(posFromAABB[i][0]);
// //        mass_center(1) = float(posFromAABB[i][1]);
// //        mass_center(2) = float(posFromAABB[i][2]);
// //        std::string str1 = QString("major eigen vector aabb %1").arg(i).toStdString();
// //        std::string str2 = QString("middle eigen vector aabb %1").arg(i).toStdString();
// //        std::string str3 = QString("minor eigen vector aabb %1").arg(i).toStdString();
// //        pcl::PointXYZ center (mass_center (0), mass_center (1), mass_center (2));
// //        pcl::PointXYZ x_axis (SCALE_AABB*major_vector (0) + mass_center (0), SCALE_AABB*major_vector (1) + mass_center (1), SCALE_AABB*major_vector (2) + mass_center (2));
// //        pcl::PointXYZ y_axis (SCALE_AABB*middle_vector (0) + mass_center (0), SCALE_AABB*middle_vector (1) + mass_center (1), SCALE_AABB*middle_vector (2) + mass_center (2));
// //        pcl::PointXYZ z_axis (SCALE_AABB*minor_vector (0) + mass_center (0), SCALE_AABB*minor_vector (1) + mass_center (1), SCALE_AABB*minor_vector (2) + mass_center (2));
// ////        viewer->addLine (center, x_axis, 1.0f, 0.0f, 0.0f, str1);
// ////        viewer->addLine (center, y_axis, 0.0f, 1.0f, 0.0f, str2);
// ////        viewer->addLine (center, z_axis, 0.0f, 0.0f, 1.0f, str3);
// //        viewer->addLine (center, x_axis, 244.0f/255.0f, 14.0f/255.0f, 12.0f/255.0f, str1);
// //        viewer->addLine (center, y_axis, 57.0f/255.0f, 222.0f/255.0f, 58.0f/255.0f, str2);
// //        viewer->addLine (center, z_axis, 40.0f/255.0f, 171.0f/255.0f, 229.0f/255.0f, str3);
// //        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH,4,str1);
// //        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH,4,str2);
// //        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH,4,str3);
// //    }

//     //obb visulization
//     viewer->addPointCloud<PointType> (cloud_obb,"obb cloud");
//     viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,2,"obb cloud");
//     pcl::visualization::PointCloudColorHandlerRGBField<PointType> rgba(cloud_obb);

//     for(int i = 0;i<posFromOBB.size();i++)
//     {
//         Eigen::Vector3f position (obbPoints_vec[i][2].x, obbPoints_vec[i][2].y, obbPoints_vec[i][2].z);
//         Eigen::Quaternionf quat (rotational_matrix_OBB_vec[i]);

//         std::string str0 = QString("OBB %1").arg(i).toStdString();
//         viewer->addCube (position, quat, obbPoints_vec[i][1].x - obbPoints_vec[i][0].x, obbPoints_vec[i][1].y - obbPoints_vec[i][0].y, obbPoints_vec[i][1].z - obbPoints_vec[i][0].z, str0);
//         viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,0,0,1,str0);
//         viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY,0.5,str0);
//         viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH,0.5,str0);

//         Eigen::Vector3f major_vector = orientation_OBB[3*i+0].cast<float>();
//         Eigen::Vector3f middle_vector = orientation_OBB[3*i+1].cast<float>();
//         Eigen::Vector3f minor_vector = orientation_OBB[3*i+2].cast<float>();
//         Eigen::Vector3f mass_center;
//         mass_center(0) = float(posFromOBB[i][0]);
//         mass_center(1) = float(posFromOBB[i][1]);
//         mass_center(2) = float(posFromOBB[i][2]);
//         std::string str1 = QString("major eigen vector obb %1").arg(i).toStdString();
//         std::string str2 = QString("middle eigen vector obb %1").arg(i).toStdString();
//         std::string str3 = QString("minor eigen vector obb %1").arg(i).toStdString();
//         pcl::PointXYZ center (mass_center (0), mass_center (1), mass_center (2));
//         pcl::PointXYZ x_axis (SCALE_AABB*major_vector (0) + mass_center (0), SCALE_AABB*major_vector (1) + mass_center (1), SCALE_AABB*major_vector (2) + mass_center (2));
//         pcl::PointXYZ y_axis (SCALE_AABB*middle_vector (0) + mass_center (0), SCALE_AABB*middle_vector (1) + mass_center (1), SCALE_AABB*middle_vector (2) + mass_center (2));
//         pcl::PointXYZ z_axis (SCALE_AABB*minor_vector (0) + mass_center (0), SCALE_AABB*minor_vector (1) + mass_center (1), SCALE_AABB*minor_vector (2) + mass_center (2));
// ////        viewer->addLine (center, x_axis, 1.0f, 0.0f, 0.0f, str1);
// ////        viewer->addLine (center, y_axis, 0.0f, 1.0f, 0.0f, str2);
// ////        viewer->addLine (center, z_axis, 0.0f, 0.0f, 1.0f, str3);
//         viewer->addLine (center, x_axis, 244.0f/255.0f, 14.0f/255.0f, 12.0f/255.0f, str1);
//         viewer->addLine (center, y_axis, 57.0f/255.0f, 222.0f/255.0f, 58.0f/255.0f, str2);
//         viewer->addLine (center, z_axis, 40.0f/255.0f, 171.0f/255.0f, 229.0f/255.0f, str3);
//         viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH,4,str1);
//         viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH,4,str2);
//         viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH,4,str3);
//     }

//     while (!viewer->wasStopped())
//     {
//         viewer->spinOnce(1000);
//         if (_kbhit())
//         {
//             pcl::visualization::Camera camera;
//             viewer->getCameraParameters(camera);
//             printf("%lf,%lf,%lf,", camera.pos[0], camera.pos[1], camera.pos[2]);
//             printf("%lf,%lf,%lf\n", camera.view[0], camera.view[1], camera.view[2]);
//         }
//         boost::this_thread::sleep(boost::posix_time::microseconds(1000));
//     }
// }

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr convertRGBADepthToPointXYZRGBA(k4a::image & colorImage,k4a::image & depthImage, k4a::transformation & transformation)
{
    PointCloud<PointXYZRGBA>::Ptr cloud(new PointCloud<PointXYZRGBA>());
    int color_image_width_pixels = colorImage.get_width_pixels();
    int color_image_height_pixels = colorImage.get_height_pixels();

    k4a::image transformed_depth_image = NULL;
    transformed_depth_image = k4a::image::create(K4A_IMAGE_FORMAT_DEPTH16,
        color_image_width_pixels,
        color_image_height_pixels,
        color_image_width_pixels * (int)sizeof(uint16_t));

    k4a::image point_cloud_image = NULL;
    point_cloud_image = k4a::image::create(K4A_IMAGE_FORMAT_CUSTOM,
        color_image_width_pixels,
        color_image_height_pixels,
        color_image_width_pixels * 3 * (int)sizeof(int16_t));

    transformation.depth_image_to_color_camera(depthImage, &transformed_depth_image);
    transformation.depth_image_to_point_cloud(transformed_depth_image, K4A_CALIBRATION_TYPE_COLOR, &point_cloud_image);

    int width = colorImage.get_width_pixels();
    int height = colorImage.get_height_pixels();

    cloud->width = width;
    cloud->height = height;
    cloud->is_dense = false;
    cloud->points.resize(cloud->height * cloud->width);

    int16_t *point_cloud_image_data = (int16_t *)(void *)point_cloud_image.get_buffer();
    uint8_t *color_image_data = colorImage.get_buffer();

    for (int i = 0; i < width * height; ++i)
    {
        PointXYZRGBA point;

        point.x = point_cloud_image_data[3 * i + 0] / 1000.0f;
        point.y = point_cloud_image_data[3 * i + 1] / 1000.0f;
        point.z = point_cloud_image_data[3 * i + 2] / 1000.0f;

        if (point.z == 0)
        {
            continue;
        }

        point.b = color_image_data[4 * i + 0];
        point.g = color_image_data[4 * i + 1];
        point.r = color_image_data[4 * i + 2];
        point.a = color_image_data[4 * i + 3];

//        if (point.b == 0 && point.g == 0 && point.r == 0 && point.a == 0)
//        {
//            continue;
//        }

        if (point.b == 0 && point.g == 0 && point.r == 0)
        {
            continue;
        }

        cloud->points[i] = point;
    }
    PointCloud<PointXYZRGBA>::Ptr cloud_filtered(new PointCloud<PointXYZRGBA>());
//    cloud_filtered = radiusFilteringPc( cloud);
//    cloud_filtered = cloud;
    cloud_filtered = passThroughPc(cloud);
    return cloud_filtered;
}

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr convertRGBADepthToPointXYZRGBA_transformed(k4a::image & colorImage,k4a::image & depthImage, k4a::transformation & transformation)
{
    PointCloud<PointXYZRGBA>::Ptr cloud(new PointCloud<PointXYZRGBA>());
    int color_image_width_pixels = colorImage.get_width_pixels();
    int color_image_height_pixels = colorImage.get_height_pixels();

    k4a::image transformed_depth_image = NULL;
    transformed_depth_image = k4a::image::create(K4A_IMAGE_FORMAT_DEPTH16,
        color_image_width_pixels,
        color_image_height_pixels,
        color_image_width_pixels * (int)sizeof(uint16_t));

    k4a::image point_cloud_image = NULL;
    point_cloud_image = k4a::image::create(K4A_IMAGE_FORMAT_CUSTOM,
        color_image_width_pixels,
        color_image_height_pixels,
        color_image_width_pixels * 3 * (int)sizeof(int16_t));

//    transformation.depth_image_to_color_camera(depthImage, &transformed_depth_image);
    transformed_depth_image = depthImage;
    transformation.depth_image_to_point_cloud(transformed_depth_image, K4A_CALIBRATION_TYPE_COLOR, &point_cloud_image);

    int width = colorImage.get_width_pixels();
    int height = colorImage.get_height_pixels();

    cloud->width = width;
    cloud->height = height;
    cloud->is_dense = false;
    cloud->points.resize(cloud->height * cloud->width);

    int16_t *point_cloud_image_data = (int16_t *)(void *)point_cloud_image.get_buffer();
    uint8_t *color_image_data = colorImage.get_buffer();

    for (int i = 0; i < width * height; ++i)
    {
        PointXYZRGBA point;

        point.x = point_cloud_image_data[3 * i + 0] / 1000.0f;
        point.y = point_cloud_image_data[3 * i + 1] / 1000.0f;
        point.z = point_cloud_image_data[3 * i + 2] / 1000.0f;

        if (point.z == 0)
        {
            continue;
        }

        point.b = color_image_data[4 * i + 0];
        point.g = color_image_data[4 * i + 1];
        point.r = color_image_data[4 * i + 2];
        point.a = color_image_data[4 * i + 3];

//        if (point.b == 0 && point.g == 0 && point.r == 0 && point.a == 0)
//        {
//            continue;
//        }

        if (point.b == 0 && point.g == 0 && point.r == 0)
        {
            continue;
        }

        cloud->points[i] = point;
    }
    PointCloud<PointXYZRGBA>::Ptr cloud_filtered(new PointCloud<PointXYZRGBA>());
//    cloud_filtered = radiusFilteringPc( cloud);
//    cloud_filtered = cloud;
    cloud_filtered = passThroughPc(cloud);
    return cloud_filtered;
}

inline bool endsWith(const string& str, const string& suffix)
{

    if (str.size() < suffix.size()) {
        return false;
    }

    auto tstr = str.substr(str.size() - suffix.size());

    return tstr.compare(suffix) == 0;
}

void frames2pc_v4(const string& color_file_path, const string& depth_file_path,const vector<string>& ms_file_path_vec,const string& save_file_path,k4a::transformation & transformation, Eigen::Isometry3d & T_b2c)
{
    cv::Mat color_img = cv::imread(color_file_path,cv::IMREAD_UNCHANGED);
    if(color_img.empty()||color_img.depth()!=CV_8U)
    {
        std::cerr
                <<"WARNING: cannot read color image. No such a file, or the image format is not CV_8U"
               <<std::endl;
    }

    //if the num of channnels is 3, then  set it to 4
    if(color_img.channels() == 3)
    {
        cv::cvtColor(color_img, color_img, COLOR_BGR2BGRA);
    }

    cv::Mat depth_img = cv::imread(depth_file_path,cv::IMREAD_ANYDEPTH);
    if(depth_img.empty()||depth_img.depth()!=CV_16U)
    {
        std::cerr
                <<"WARNING: cannot read depth image. No such a file, or the image format is not CV_16U"
               <<std::endl;
    }

    vector<cv::Mat> ms_img_vec;
    for (int b = 0; b<25; b++)
    {
        cv::Mat ms_img = cv::imread(ms_file_path_vec[b],cv::IMREAD_UNCHANGED);
        if(ms_img.empty()||ms_img.depth()!=CV_8U)
        {
            std::cerr
                    <<"WARNING: cannot read ms image. No such a file, or the image format is not CV_8U"
                   <<std::endl;
        }
        ms_img_vec.push_back(ms_img);
    }

    k4a::image colorImage = NULL;
    k4a::image depthImage = NULL;

    colorImage = k4a::image::create(K4A_IMAGE_FORMAT_COLOR_BGRA32,
                                    color_img.size().width,
                                    color_img.size().height,
                                    color_img.size().width*4*(int)sizeof(uint8_t));

    depthImage = k4a::image::create(K4A_IMAGE_FORMAT_DEPTH16,
                                    depth_img.size().width,
                                    depth_img.size().height,
                                    depth_img.size().width * (int)sizeof(uint16_t));

    memcpy(colorImage.get_buffer(),
           color_img.data,
           color_img.size().height*color_img.size().width*4*(int)sizeof(uint8_t));

    memcpy(depthImage.get_buffer(),
           depth_img.data,
           depth_img.size().height*depth_img.size().width*(int)sizeof(uint16_t));

    int color_image_width_pixels = colorImage.get_width_pixels();
    int color_image_height_pixels = colorImage.get_height_pixels();

    k4a::image transformed_depth_image = NULL;
    transformed_depth_image = k4a::image::create(K4A_IMAGE_FORMAT_DEPTH16,
        color_image_width_pixels,
        color_image_height_pixels,
        color_image_width_pixels * (int)sizeof(uint16_t));

    k4a::image point_cloud_image = NULL;
    point_cloud_image = k4a::image::create(K4A_IMAGE_FORMAT_CUSTOM,
        color_image_width_pixels,
        color_image_height_pixels,
        color_image_width_pixels * 3 * (int)sizeof(int16_t));

    transformed_depth_image = depthImage;
    transformation.depth_image_to_point_cloud(transformed_depth_image, K4A_CALIBRATION_TYPE_COLOR, &point_cloud_image);

    int16_t *point_cloud_image_data = (int16_t *)(void *)point_cloud_image.get_buffer();
    uint8_t *color_image_data = colorImage.get_buffer();
    vector<uint8_t *> ms_image_data_vec;
    for (int b = 0;b<25;b++)
    {
        uint8_t * ms_image_data = ms_img_vec[b].data;
        ms_image_data_vec.push_back(ms_image_data);
    }

    int Num =  color_image_width_pixels * color_image_height_pixels;
    float *VX = new float[Num] {0};
    float *VY = new float[Num] {0};
    float *VZ = new float[Num] {0};
    float *NX = new float[Num] {0};
    float *NY = new float[Num] {0};
    float *NZ = new float[Num] {0};
    float *X = new float[Num] {0};
    float *Y = new float[Num] {0};
    float *Z = new float[Num] {0};
    uint8_t *R = new uint8_t[Num] {0};
    uint8_t *G = new uint8_t[Num] {0};
    uint8_t *B = new uint8_t[Num] {0};
    vector<uint8_t *> MS_vec;
    pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>());

    for (int i = 0; i < Num; ++i)
    {
        X[i] = point_cloud_image_data[3 * i + 0] / 1000.0f;
        Y[i] = point_cloud_image_data[3 * i + 1] / 1000.0f;
        Z[i] = point_cloud_image_data[3 * i + 2] / 1000.0f;

        VX[i] = 0;
        VY[i] = 0;
        VZ[i] = 0;

        NX[i] = 0;
        NY[i] = 0;
        NZ[i] = 0;

        PointType p;
        p.x = X[i];
        p.y = Y[i];
        p.z = Z[i];
        cloud->points.push_back(p);

        B[i] = color_image_data[4 * i + 0];
        G[i] = color_image_data[4 * i + 1];
        R[i] = color_image_data[4 * i + 2];
    }

    for(int b=0;b<25;b++)
    {
        uint8_t *MS = new uint8_t[Num] {0};
        for (int i = 0; i < Num; ++i)
        {
            MS[i] = (ms_image_data_vec[b][i]);
        }
        MS_vec.push_back(MS);
    }

    pcl::PointCloud<PointType> transformed_cloud;
    pcl::transformPointCloud (*cloud, transformed_cloud, T_b2c.matrix());

//    //normal estimation and reorientation
//    pcl::NormalEstimationOMP<PointType, pcl::Normal> n;
//    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
//    pcl::search::KdTree<PointType>::Ptr tree(new pcl::search::KdTree<PointType>());
//    n.setNumberOfThreads(6);
//    n.setViewPoint(0.6,0,1);
//    n.setInputCloud(transformed_cloud.makeShared());
//    n.setSearchMethod(tree);
//    n.setKSearch(10);
//    n.compute(*normals);

    for (int i = 0; i < Num; ++i)
    {
        if(Z[i] == 0)
        {
            continue;
        }
        else
        {
            X[i] = transformed_cloud.points[i].x;
            Y[i] = transformed_cloud.points[i].y;
            Z[i] = transformed_cloud.points[i].z;

            VX[i] = transformed_cloud.points[i].x - T_b2c.matrix()(0,3);
            VY[i] = transformed_cloud.points[i].y - T_b2c.matrix()(1,3);
            VZ[i] = transformed_cloud.points[i].z - T_b2c.matrix()(2,3);
            
//            NX[i] = normals->points[i].normal[0];
//            NY[i] = normals->points[i].normal[1];
//            NZ[i] = normals->points[i].normal[2];
        }
    }

    //txt point cloud and dataset vx vy vz nx ny nz ms_1-25
    ofstream zos(save_file_path);
    for (int i = 0; i<Num; i++)
    {
        if (Z[i] == 0 || (unsigned(R[i])==0 && unsigned(G[i]) == 0 && unsigned(B[i]) == 0)) //Z ==0 or rgb ==0
        {
            continue;
        }
        else
        {
            zos << X[i] << " " << Y[i] << " " << Z[i] << " " << unsigned(R[i]) << " " << unsigned(G[i])<< " " << unsigned(B[i]);
            for (int b = 0; b<25; b++)
            {
               zos  << " " <<  unsigned(MS_vec[b][i]);
            }
            zos  << " " << VX[i] << " " << VY[i] << " " << VZ[i];
//            zos  << " " << VX[i] << " " << VY[i] << " " << VZ[i] << " " << NX[i] << " " << NY[i]<< " " << NZ[i];
        }
        zos << endl;
    }

    //TXT point cloud
//    ofstream zos(save_file_path);
//    for (int i = 0; i<Num; i++)
//    {
//        if (Z[i] == 0 || (unsigned(R[i])==0 && unsigned(G[i]) == 0 && unsigned(B[i]) == 0)) //Z ==0 or rgb ==0
//        {
//            continue;
//        }
//        else
//        {
//            zos << X[i] << " " << Y[i] << " " << Z[i] << " " << unsigned(R[i]) << " " << unsigned(G[i])<< " " << unsigned(B[i]);
//            for (int b = 0; b<25; b++)
//            {
//               zos  << " " <<  unsigned(MS_vec[b][i]);
//            }
//        }
//        zos << endl;
//    }
    cout << "generation has done!!!" << endl;
}

void frames2pc_v3(const string& color_file_path, const string& depth_file_path,const string& rvi_file_path,const string& save_file_path,k4a::transformation & transformation, Eigen::Isometry3d & T_b2c)
{
    cv::Mat color_img = cv::imread(color_file_path,cv::IMREAD_UNCHANGED);
    if(color_img.empty()||color_img.depth()!=CV_8U)
    {
        std::cerr
                <<"WARNING: cannot read color image. No such a file, or the image format is not CV_8U"
               <<std::endl;
    }

    //if the num of channnels is 3, then  set it to 4
    if(color_img.channels() == 3)
    {
        cv::cvtColor(color_img, color_img, COLOR_BGR2BGRA);
    }

    cv::Mat depth_img = cv::imread(depth_file_path,cv::IMREAD_ANYDEPTH);
    if(depth_img.empty()||depth_img.depth()!=CV_16U)
    {
        std::cerr
                <<"WARNING: cannot read depth image. No such a file, or the image format is not CV_16U"
               <<std::endl;
    }

    cv::Mat rvi_img = cv::imread(rvi_file_path,cv::IMREAD_ANYDEPTH);
    if(rvi_img.empty()||rvi_img.depth()!=CV_16U)
    {
        std::cerr
                <<"WARNING: cannot read rvi image. No such a file, or the image format is not CV_8U"
               <<std::endl;
    }

    k4a::image colorImage = NULL;
    k4a::image depthImage = NULL;

    colorImage = k4a::image::create(K4A_IMAGE_FORMAT_COLOR_BGRA32,
                                    color_img.size().width,
                                    color_img.size().height,
                                    color_img.size().width*4*(int)sizeof(uint8_t));

    depthImage = k4a::image::create(K4A_IMAGE_FORMAT_DEPTH16,
                                    depth_img.size().width,
                                    depth_img.size().height,
                                    depth_img.size().width * (int)sizeof(uint16_t));

    memcpy(colorImage.get_buffer(),
           color_img.data,
           color_img.size().height*color_img.size().width*4*(int)sizeof(uint8_t));

    memcpy(depthImage.get_buffer(),
           depth_img.data,
           depth_img.size().height*depth_img.size().width*(int)sizeof(uint16_t));

    int color_image_width_pixels = colorImage.get_width_pixels();
    int color_image_height_pixels = colorImage.get_height_pixels();

    k4a::image transformed_depth_image = NULL;
    transformed_depth_image = k4a::image::create(K4A_IMAGE_FORMAT_DEPTH16,
        color_image_width_pixels,
        color_image_height_pixels,
        color_image_width_pixels * (int)sizeof(uint16_t));

    k4a::image point_cloud_image = NULL;
    point_cloud_image = k4a::image::create(K4A_IMAGE_FORMAT_CUSTOM,
        color_image_width_pixels,
        color_image_height_pixels,
        color_image_width_pixels * 3 * (int)sizeof(int16_t));

//    transformation.depth_image_to_color_camera(depthImage, &transformed_depth_image);
    transformed_depth_image = depthImage;
    transformation.depth_image_to_point_cloud(transformed_depth_image, K4A_CALIBRATION_TYPE_COLOR, &point_cloud_image);

    int16_t *point_cloud_image_data = (int16_t *)(void *)point_cloud_image.get_buffer();
    uint8_t *color_image_data = colorImage.get_buffer();
    uint16_t * rvi_image_data = (uint16_t *)rvi_img.data;

    int Num =  color_image_width_pixels * color_image_height_pixels;
    float *X = new float[Num] {0};
    float *Y = new float[Num] {0};
    float *Z = new float[Num] {0};
    uint8_t *R = new uint8_t[Num] {0};
    uint8_t *G = new uint8_t[Num] {0};
    uint8_t *B = new uint8_t[Num] {0};
    float *RVI = new float[Num] {0};
    pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>());

    for (int i = 0; i < Num; ++i)
    {
        X[i] = point_cloud_image_data[3 * i + 0] / 1000.0f;
        Y[i] = point_cloud_image_data[3 * i + 1] / 1000.0f;
        Z[i] = point_cloud_image_data[3 * i + 2] / 1000.0f;

        PointType p;
        p.x = X[i];
        p.y = Y[i];
        p.z = Z[i];
        cloud->points.push_back(p);

        B[i] = color_image_data[4 * i + 0];
        G[i] = color_image_data[4 * i + 1];
        R[i] = color_image_data[4 * i + 2];

        RVI[i] = (rvi_image_data[i]) / 10000.0f;
    }

    pcl::PointCloud<PointType> transformed_cloud;
    pcl::transformPointCloud (*cloud, transformed_cloud, T_b2c.matrix());

    for (int i = 0; i < Num; ++i)
    {
        if(Z[i] == 0)
        {
            continue;
        }
        else
        {
            X[i] = transformed_cloud.points[i].x;
            Y[i] = transformed_cloud.points[i].y;
            Z[i] = transformed_cloud.points[i].z;
        }
    }

    ofstream zos(save_file_path);
    for (int i = 0; i<Num; i++)
    {
//        if (Z[i] == 0  ||  RVI[i] == 0) //Z ==0 or RVI ==0
//        {
//            continue;
//        }
//        if (Z[i] == 0 || (unsigned(R[i])==0 && unsigned(G[i]) == 0 && unsigned(B[i]) == 0)) //Z ==0 or rgb ==0
//        {
//            continue;
//        }
//        if (Z[i] == 0 || (unsigned(R[i])==0 && unsigned(G[i]) == 0 && unsigned(B[i]) == 0) || RVI[i] == 0) //Z ==0 or rgb ==0 or RVI ==0
//        {
//            continue;
//        }
        if (Z[i] == 0 || (unsigned(R[i])==0 && unsigned(G[i]) == 0 && unsigned(B[i]) == 0) || (RVI[i] <1 || RVI[i] >4)) //Z ==0 or rgb ==0 or 1<RVI<4
        {
            continue;
        }
        else
        {
            zos << X[i] << " " << Y[i] << " " << Z[i] << " " << unsigned(R[i]) << " " << unsigned(G[i])<< " " << unsigned(B[i]) << " " << RVI[i]<< endl;
        }
    }

    cout << "generation has done!!!" << endl;
}

void frames2pc_v2(const string& color_file_path, const string& depth_file_path,const string& rvi_file_path,const string& save_file_path,k4a::transformation & transformation)
{
    cv::Mat color_img = cv::imread(color_file_path,cv::IMREAD_UNCHANGED);
    if(color_img.empty()||color_img.depth()!=CV_8U)
    {
        std::cerr
                <<"WARNING: cannot read color image. No such a file, or the image format is not CV_8U"
               <<std::endl;
    }

    //if the num of channnels is 3, then  set it to 4
    if(color_img.channels() == 3)
    {
        cv::cvtColor(color_img, color_img, COLOR_BGR2BGRA);
    }

    cv::Mat depth_img = cv::imread(depth_file_path,cv::IMREAD_ANYDEPTH);
    if(depth_img.empty()||depth_img.depth()!=CV_16U)
    {
        std::cerr
                <<"WARNING: cannot read depth image. No such a file, or the image format is not CV_16U"
               <<std::endl;
    }

    cv::Mat rvi_img = cv::imread(rvi_file_path,cv::IMREAD_ANYDEPTH);
    if(rvi_img.empty()||rvi_img.depth()!=CV_16U)
    {
        std::cerr
                <<"WARNING: cannot read rvi image. No such a file, or the image format is not CV_8U"
               <<std::endl;
    }

    k4a::image colorImage = NULL;
    k4a::image depthImage = NULL;

    colorImage = k4a::image::create(K4A_IMAGE_FORMAT_COLOR_BGRA32,
                                    color_img.size().width,
                                    color_img.size().height,
                                    color_img.size().width*4*(int)sizeof(uint8_t));

    depthImage = k4a::image::create(K4A_IMAGE_FORMAT_DEPTH16,
                                    depth_img.size().width,
                                    depth_img.size().height,
                                    depth_img.size().width * (int)sizeof(uint16_t));

    memcpy(colorImage.get_buffer(),
           color_img.data,
           color_img.size().height*color_img.size().width*4*(int)sizeof(uint8_t));

    memcpy(depthImage.get_buffer(),
           depth_img.data,
           depth_img.size().height*depth_img.size().width*(int)sizeof(uint16_t));

    int color_image_width_pixels = colorImage.get_width_pixels();
    int color_image_height_pixels = colorImage.get_height_pixels();

    k4a::image transformed_depth_image = NULL;
    transformed_depth_image = k4a::image::create(K4A_IMAGE_FORMAT_DEPTH16,
        color_image_width_pixels,
        color_image_height_pixels,
        color_image_width_pixels * (int)sizeof(uint16_t));

    k4a::image point_cloud_image = NULL;
    point_cloud_image = k4a::image::create(K4A_IMAGE_FORMAT_CUSTOM,
        color_image_width_pixels,
        color_image_height_pixels,
        color_image_width_pixels * 3 * (int)sizeof(int16_t));

//    transformation.depth_image_to_color_camera(depthImage, &transformed_depth_image);
    transformed_depth_image = depthImage;
    transformation.depth_image_to_point_cloud(transformed_depth_image, K4A_CALIBRATION_TYPE_COLOR, &point_cloud_image);

    int16_t *point_cloud_image_data = (int16_t *)(void *)point_cloud_image.get_buffer();
    uint8_t *color_image_data = colorImage.get_buffer();
    uint16_t * rvi_image_data = (uint16_t *)rvi_img.data;

    int Num =  color_image_width_pixels * color_image_height_pixels;
    float *X = new float[Num] {0};
    float *Y = new float[Num] {0};
    float *Z = new float[Num] {0};
    uint8_t *R = new uint8_t[Num] {0};
    uint8_t *G = new uint8_t[Num] {0};
    uint8_t *B = new uint8_t[Num] {0};
    float *RVI = new float[Num] {0};

    for (int i = 0; i < Num; ++i)
    {
        X[i] = point_cloud_image_data[3 * i + 0] / 1000.0f;
        Y[i] = point_cloud_image_data[3 * i + 1] / 1000.0f;
        Z[i] = point_cloud_image_data[3 * i + 2] / 1000.0f;

        B[i] = color_image_data[4 * i + 0];
        G[i] = color_image_data[4 * i + 1];
        R[i] = color_image_data[4 * i + 2];

        RVI[i] = (rvi_image_data[i]) / 10000.0f;
    }

    ofstream zos(save_file_path);
    for (int i = 0; i<Num; i++)
    {
//        if (Z[i] == 0  ||  RVI[i] == 0) //Z ==0 or RVI ==0
//        {
//            continue;
//        }
        if (Z[i] == 0 || (unsigned(R[i])==0 && unsigned(G[i]) == 0 && unsigned(B[i]) == 0)) //Z ==0 or rgb ==0
        {
            continue;
        }
        else
        {
            zos << X[i] << " " << Y[i] << " " << Z[i] << " " << unsigned(R[i]) << " " << unsigned(G[i])<< " " << unsigned(B[i]) << " " << RVI[i]<< endl;
        }
    }

    cout << "generation has done!!!" << endl;
}

void frames2pc(const string& color_file_path, const string& depth_file_path,const string& save_file_path,const bool& is_transformed,const bool& is_mkv_frames,k4a::transformation & transformation)
{
    cv::Mat color_img = cv::imread(color_file_path,cv::IMREAD_UNCHANGED);
    if(color_img.empty()||color_img.depth()!=CV_8U)
    {
        std::cerr
                <<"WARNING: cannot read color image. No such a file, or the image format is not CV_8U"
               <<std::endl;
    }

//    //if mkv frames, the channels is 3 rather than 4
//    if(is_mkv_frames)
//    {
//        cv::cvtColor(color_img, color_img, COLOR_BGR2BGRA);
//    }
//    //

    //if the num of channnels is 3, then  set it to 4
    if(color_img.channels() == 3)
    {
        cv::cvtColor(color_img, color_img, COLOR_BGR2BGRA);
    }
    //

    cv::Mat depth_img = cv::imread(depth_file_path,cv::IMREAD_ANYDEPTH);
    if(depth_img.empty()||depth_img.depth()!=CV_16U)
    {
        std::cerr
                <<"WARNING: cannot read depth image. No such a file, or the image format is not CV_16U"
               <<std::endl;
    }

    k4a::image colorImage = NULL;
    k4a::image depthImage = NULL;

    colorImage = k4a::image::create(K4A_IMAGE_FORMAT_COLOR_BGRA32,
                                    color_img.size().width,
                                    color_img.size().height,
                                    color_img.size().width*4*(int)sizeof(uint8_t));

    depthImage = k4a::image::create(K4A_IMAGE_FORMAT_DEPTH16,
                                    depth_img.size().width,
                                    depth_img.size().height,
                                    depth_img.size().width * (int)sizeof(uint16_t));

    memcpy(colorImage.get_buffer(),
           color_img.data,
           color_img.size().height*color_img.size().width*4*(int)sizeof(uint8_t));

    memcpy(depthImage.get_buffer(),
           depth_img.data,
           depth_img.size().height*depth_img.size().width*(int)sizeof(uint16_t));

    PointCloud<PointXYZRGBA>::Ptr cloud(new PointCloud<PointXYZRGBA>());
    if(is_transformed)
    {
        cloud = convertRGBADepthToPointXYZRGBA_transformed(colorImage,depthImage,transformation);
    }
    else
    {
        cloud = convertRGBADepthToPointXYZRGBA(colorImage,depthImage,transformation);
    }

//    Eigen::Vector3f whd; //w h d
//    vector <PointType> bboxPoints ;//min_point_AABB, max_point_AABB, min_point_OBB, max_point_OBB, position_OBB
//    Eigen::Matrix3f  rotational_matrix_OBB;
//    vector <float> feature_values ;//major_value, middle_value, minor_value
//    vector<Eigen::Vector3f> feature_vectors;//  Eigen::Vector3f major_vector, middle_vector, minor_vector, mass_center;//time count in ms us

//    auto t1 = system_clock::now();

//    pcaOBB(cloud,//pc in
//         whd,//w h d
//         bboxPoints,//min_point_AABB, max_point_AABB, min_point_OBB, max_point_OBB, position_OBB
//         rotational_matrix_OBB,// rotational_matrix_OBB
//         feature_values,//major_value, middle_value, minor_value
//         feature_vectors//  Eigen::Vector3f major_vector, middle_vector, minor_vector, mass_center;
//              );

//    auto t2 = system_clock::now();

//    // floating-point duration: no duration_cast needed
//    duration<double, std::milli> fp_ms = t2 - t1;

//    // integral duration: requires duration_cast
//    auto int_ms = duration_cast<milliseconds>(fp_ms);

//    // converting integral duration to integral duration of shorter divisible time unit: no duration_cast needed
//    duration<long, std::micro> int_usec = int_ms;

//    std::cout << "took " << fp_ms.count() << " ms, "
//              << "or " << int_ms.count() << " whole milliseconds "
//              << "(which is " << int_usec.count() << " whole microseconds)" << std::endl;

//    whds.push_back(whd);


//    for(int i =0;i<bboxPoints.size();i++)
//    {
//    cout<<bboxPoints[i].getVector3fMap()<<endl;
//    }
//    cout<<rotational_matrix_OBB<<endl;
//    for(int i =0;i<feature_values.size();i++)
//    {
//    cout<<feature_values[i]<<endl;
//    }
//    for(int i =0;i<feature_vectors.size();i++)
//    {
//    cout<<feature_vectors[i]<<endl;
//    }

//    //plane projection
//    PointCloud<PointXYZRGBA>::Ptr cloud_projected(new PointCloud<PointXYZRGBA>());
//    planeProjection(feature_vectors[2](0),feature_vectors[2](1),feature_vectors[2](2),0,cloud,cloud_projected);

//    //hull extraction
//    PointCloud<PointXYZRGBA>::Ptr cloud_hull(new PointCloud<PointXYZRGBA>());
//    hull_extraction(cloud_projected,//pc in
//                   cloud_hull,//pc out
//                   0.05//alpha
//                   );

//    pcl::PointCloud<PointType> contour;
//    polygonGeneration(cloud_hull,contour);

//    //S & C
//    cout<<"S:"<<S(contour)<<endl;
//    cout<<"C:"<<C(contour)<<endl;

//    Ss.push_back(S(contour));
//    Cs.push_back(C(contour));

//    Ss_ellipse.push_back(0.25*3.1416*whd(1)*whd(2));
//    Cs_ellipse.push_back(3.1416*min(whd(1),whd(2))+2*abs(whd(1)-whd(2)));

    pcl::io::savePCDFileASCII (save_file_path, *cloud);
}

void CSEstimationForExistingPointCloud(pcl::PointCloud<PointType>::Ptr cloud)
{
    Eigen::Vector3f whd; //w h d
    vector <PointType> bboxPoints ;//min_point_AABB, max_point_AABB, min_point_OBB, max_point_OBB, position_OBB
    Eigen::Matrix3f  rotational_matrix_OBB;
    vector <float> feature_values ;//major_value, middle_value, minor_value
    vector<Eigen::Vector3f> feature_vectors;//  Eigen::Vector3f major_vector, middle_vector, minor_vector, mass_center;//time count in ms us

    auto t1 = system_clock::now();

    pcaOBB(cloud,//pc in
         whd,//w h d
         bboxPoints,//min_point_AABB, max_point_AABB, min_point_OBB, max_point_OBB, position_OBB
         rotational_matrix_OBB,// rotational_matrix_OBB
         feature_values,//major_value, middle_value, minor_value
         feature_vectors//  Eigen::Vector3f major_vector, middle_vector, minor_vector, mass_center;
              );

    auto t2 = system_clock::now();

    // floating-point duration: no duration_cast needed
    duration<double, std::milli> fp_ms = t2 - t1;

    // integral duration: requires duration_cast
    auto int_ms = duration_cast<milliseconds>(fp_ms);

    // converting integral duration to integral duration of shorter divisible time unit: no duration_cast needed
    duration<long, std::micro> int_usec = int_ms;

    std::cout << "took " << fp_ms.count() << " ms, "
              << "or " << int_ms.count() << " whole milliseconds "
              << "(which is " << int_usec.count() << " whole microseconds)" << std::endl;

    whds.push_back(whd);


    for(int i =0;i<bboxPoints.size();i++)
    {
    cout<<bboxPoints[i].getVector3fMap()<<endl;
    }
    cout<<rotational_matrix_OBB<<endl;
    for(int i =0;i<feature_values.size();i++)
    {
    cout<<feature_values[i]<<endl;
    }
    for(int i =0;i<feature_vectors.size();i++)
    {
    cout<<feature_vectors[i]<<endl;
    }

    //plane projection
    PointCloud<PointXYZRGBA>::Ptr cloud_projected(new PointCloud<PointXYZRGBA>());
    planeProjection(feature_vectors[2](0),feature_vectors[2](1),feature_vectors[2](2),0,cloud,cloud_projected);

    //hull extraction
    PointCloud<PointXYZRGBA>::Ptr cloud_hull(new PointCloud<PointXYZRGBA>());
    hull_extraction(cloud_projected,//pc in
                   cloud_hull,//pc out
                   0.5//alpha 0.05
                   );

//    pcl::io::savePCDFileASCII ("2_.pcd", *cloud_hull);
//    pcl::io::savePCDFileASCII ("3_.pcd", *cloud_hull);
//    pcl::io::savePCDFileASCII ("4.pcd", *cloud_hull);
    k++;
    pcl::io::savePCDFileASCII(to_string(k)+"_.pcd", *cloud_hull);

    CircleEstimation(cloud_hull);

    pcl::PointCloud<PointType> contour;
    polygonGeneration(cloud_hull,contour);

    //S & C
    cout<<"S:"<<S(contour)<<endl;
    cout<<"C:"<<C(contour)<<endl;

    Ss.push_back(S(contour));
    Cs.push_back(C(contour));

    Ss_ellipse.push_back(0.25*3.1416*whd(1)*whd(2));
    Cs_ellipse.push_back(3.1416*min(whd(1),whd(2))+2*abs(whd(1)-whd(2)));
}

pcl::PointCloud<PointType>::Ptr radiusFilteringPc( pcl::PointCloud<PointType>::Ptr cloud_in)
{
    pcl::PointCloud<PointType>::Ptr cloud_radius(new pcl::PointCloud<PointType>);
    pcl::RadiusOutlierRemoval<PointType> ror;
    cout <<"start filtering"<<endl;
    ror.setInputCloud(cloud_in);
    ror.setRadiusSearch(0.05);
    ror.setMinNeighborsInRadius(10);
    ror.filter(*cloud_radius);
    cout <<"end filtering"<<endl;
    return(cloud_radius);
}

pcl::PointCloud<PointType>::Ptr passThroughPc(pcl::PointCloud<PointType>::Ptr cloud)
{
    pcl::PointCloud<PointType>::Ptr cloud_filtered (new pcl::PointCloud<PointType>);
    std::cout << "loading point cloud: " << cloud->points.size() << std::endl;
    pcl::PassThrough<PointType> pass;

    //z  0-60
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (-0.01, 0.01);
    // pass.setKeepOrganized(true);
    pass.setNegative (true);
    pass.filter (*cloud_filtered);

    std::cout << "Cloud after filtering: " << cloud_filtered ->points.size()<< std::endl;
    return(cloud_filtered);
}

void bboxCalculationForPc(pcl::PointCloud<PointType>::Ptr cloud,
                            vector <float> & moment_of_inertia,//vector <float> moment_of_inertia;
                            vector <float> & eccentricity,//vector <float> eccentricity
                            vector <PointType> & bboxPoints,//min_point_AABB, max_point_AABB, min_point_OBB, max_point_OBB, position_OBB
                          Eigen::Matrix3f & rotational_matrix_OBB,// rotational_matrix_OBB
                          vector <float> & feature_values,//major_value, middle_value, minor_value
                          vector<Eigen::Vector3f> & feature_vectors//  Eigen::Vector3f major_vector, middle_vector, minor_vector, mass_center;
                          )
{
    pcl::MomentOfInertiaEstimation <PointType> feature_extractor;
    feature_extractor.setInputCloud (cloud);
    feature_extractor.compute();
//    vector <float> moment_of_inertia;
//    vector <float> eccentricity;
    PointType min_point_AABB;
    PointType max_point_AABB;
    PointType min_point_OBB;
    PointType max_point_OBB;
    PointType position_OBB;
//    Eigen::Matrix3f rotational_matrix_OBB;
    float major_value, middle_value, minor_value;
    Eigen::Vector3f major_vector, middle_vector, minor_vector;
    Eigen::Vector3f mass_center;
    feature_extractor.getMomentOfInertia (moment_of_inertia);
    feature_extractor.getEccentricity (eccentricity);
    feature_extractor.getAABB (min_point_AABB, max_point_AABB);
    feature_extractor.getOBB (min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
    feature_extractor.getEigenValues (major_value, middle_value, minor_value);
    feature_extractor.getEigenVectors (major_vector, middle_vector, minor_vector);
    feature_extractor.getMassCenter (mass_center);

    bboxPoints.push_back(min_point_AABB);
    bboxPoints.push_back(max_point_AABB);
    bboxPoints.push_back(min_point_OBB);
    bboxPoints.push_back(max_point_OBB);
    bboxPoints.push_back(position_OBB);
    feature_values.push_back(major_value);
    feature_values.push_back(middle_value);
    feature_values.push_back(minor_value);
    feature_vectors.push_back(major_vector);
    feature_vectors.push_back(middle_vector);
    feature_vectors.push_back(minor_vector);
    feature_vectors.push_back(mass_center);
}

void pcaOBB(pcl::PointCloud<PointType>::Ptr cloud,//pc in
            Eigen::Vector3f& whd,//w h d
            vector <PointType> & bboxPoints,//min_point_AABB, max_point_AABB, min_point_OBB, max_point_OBB, position_OBB
          Eigen::Matrix3f & rotational_matrix_OBB,// rotational_matrix_OBB
          vector <float> & feature_values,//major_value, middle_value, minor_value
          vector<Eigen::Vector3f> & feature_vectors//  Eigen::Vector3f major_vector, middle_vector, minor_vector, mass_center;
            )
{
    //time count in ms us
    auto t1 = system_clock::now();

    // 计算点云质心和协方差矩阵
    Eigen::Vector4f pcaCentroid;
    pcl::compute3DCentroid(*cloud, pcaCentroid);
    Eigen::Matrix3f covariance;
    pcl::computeCovarianceMatrixNormalized(*cloud, pcaCentroid, covariance);
    // 协方差矩阵分解求特征值特征向量
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
    Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
    Eigen::Vector3f eigenValuesPCA = eigen_solver.eigenvalues();
    // 校正主方向间垂直
    eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));
    eigenVectorsPCA.col(0) = eigenVectorsPCA.col(1).cross(eigenVectorsPCA.col(2));
    eigenVectorsPCA.col(1) = eigenVectorsPCA.col(2).cross(eigenVectorsPCA.col(0));

    cout << "va(3x1):\n" << eigenValuesPCA << endl; // Eigen计算出来的特征值默认是从小到大排列
    cout << "ve(3x3):\n" << eigenVectorsPCA << endl;
    cout << "(4x1):\n" << pcaCentroid << endl;

    feature_values.push_back(eigenValuesPCA(2));
    feature_values.push_back(eigenValuesPCA(1));
    feature_values.push_back(eigenValuesPCA(0));
    feature_vectors.push_back((-1)*eigenVectorsPCA.col(2));
    feature_vectors.push_back((-1)*eigenVectorsPCA.col(1));
    feature_vectors.push_back((-1)*eigenVectorsPCA.col(0));
    feature_vectors.push_back(pcaCentroid.head<3>());
    /*
    // 另一种计算点云协方差矩阵特征值和特征向量的方式:通过pcl中的pca接口，如下，这种情况得到的特征向量相似特征向量
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPCAprojection (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PCA<pcl::PointXYZ> pca;
    pca.setInputCloud(cloudSegmented);
    pca.project(*cloudSegmented, *cloudPCAprojection);
    std::cerr << std::endl << "EigenVectors: " << pca.getEigenVectors() << std::endl;//计算特征向量
    std::cerr << std::endl << "EigenValues: " << pca.getEigenValues() << std::endl;//计算特征值
    */
    // 将输入点云转换至原点
    Eigen::Matrix4f tm = Eigen::Matrix4f::Identity();     // 定义变换矩阵
    Eigen::Matrix4f tm_inv = Eigen::Matrix4f::Identity(); // 定义变换矩阵的逆
    tm.block<3, 3>(0, 0) = eigenVectorsPCA.transpose();   // 旋转矩阵R.
    tm.block<3, 1>(0, 3) = -1.0f * (eigenVectorsPCA.transpose()) * (pcaCentroid.head<3>());// 平移向量 -R*t
    tm_inv = tm.inverse();
    rotational_matrix_OBB.block(0,0,3,1) = (-1)*eigenVectorsPCA.col(2);
    rotational_matrix_OBB.block(0,1,3,1) = (-1)*eigenVectorsPCA.col(1);
    rotational_matrix_OBB.block(0,2,3,1) = (-1)*eigenVectorsPCA.col(0);

    std::cout << "tm(4x4):\n" << tm << std::endl;
    std::cout << "tm'(4x4):\n" << tm_inv << std::endl;

    pcl::PointCloud<PointType>::Ptr transformedCloud(new pcl::PointCloud<PointType>);
    pcl::transformPointCloud(*cloud, *transformedCloud, tm);

    PointType min_p1, max_p1;
    Eigen::Vector3f c1, c;

    pcl::getMinMax3D(*transformedCloud, min_p1, max_p1);

    PointType min_aabb, max_aabb;
    pcl::getMinMax3D(*cloud,min_aabb,max_aabb);
    bboxPoints.push_back(min_aabb);
    bboxPoints.push_back(max_aabb);

    c1 = 0.5f * (min_p1.getVector3fMap() + max_p1.getVector3fMap());

    cout << "c1(3x1):\n" << c1 << endl;

    Eigen::Affine3f tm_inv_aff(tm_inv);
    pcl::transformPoint(c1, c, tm_inv_aff);

    Eigen::Vector3f _min, _max;
    pcl::transformPoint(min_p1.getVector3fMap(), _min, tm_inv_aff);
    pcl::transformPoint(max_p1.getVector3fMap(), _max, tm_inv_aff);

    PointType p1_min,p1_max,p_c;
    p1_min.x = min_p1.z;
    p1_min.y = min_p1.y;
    p1_min.z = min_p1.x;
    bboxPoints.push_back(p1_min);
//    bboxPoints.push_back(min_p1);

    p1_max.x = max_p1.z;
    p1_max.y = max_p1.y;
    p1_max.z = max_p1.x;
    bboxPoints.push_back(p1_max);
//    bboxPoints.push_back(max_p1);

    p_c.x = c(0);
    p_c.y = c(1);
    p_c.z = c(2);
    bboxPoints.push_back(p_c);

    Eigen::Vector3f whd1;
    whd1 = max_p1.getVector3fMap() - min_p1.getVector3fMap();
    whd = whd1;
    float sc1 = (whd1(0) + whd1(1) + whd1(2)) / 3;  //点云平均尺度，用于设置主方向箭头大小

    cout << "width1 or delta_z=" << whd1(0) << endl;
    cout << "heght1 or delta_y=" << whd1(1) << endl;
    cout << "depth1 or delta_x=" << whd1(2) << endl;
    cout << "scale1=" << sc1 << endl;

    const Eigen::Quaternionf bboxQ1(Eigen::Quaternionf::Identity());
    const Eigen::Vector3f    bboxT1(c1);

    const Eigen::Quaternionf bboxQ(tm_inv.block<3, 3>(0, 0));
    const Eigen::Vector3f    bboxT(c);


    // 变换到原点的点云主方向
    PointType op;
    op.x = 0.0;
    op.y = 0.0;
    op.z = 0.0;
    Eigen::Vector3f px, py, pz;
    Eigen::Affine3f tm_aff(tm);
    pcl::transformVector(eigenVectorsPCA.col(0), px, tm_aff);
    pcl::transformVector(eigenVectorsPCA.col(1), py, tm_aff);
    pcl::transformVector(eigenVectorsPCA.col(2), pz, tm_aff);
    PointType pcaX;
    pcaX.x = sc1 * px(0);
    pcaX.y = sc1 * px(1);
    pcaX.z = sc1 * px(2);
    PointType pcaY;
    pcaY.x = sc1 * py(0);
    pcaY.y = sc1 * py(1);
    pcaY.z = sc1 * py(2);
    PointType pcaZ;
    pcaZ.x = sc1 * pz(0);
    pcaZ.y = sc1 * pz(1);
    pcaZ.z = sc1 * pz(2);

    // 初始点云的主方向
    PointType cp;
    cp.x = pcaCentroid(0);
    cp.y = pcaCentroid(1);
    cp.z = pcaCentroid(2);
    PointType pcX;
    pcX.x = sc1 * eigenVectorsPCA(0, 0) + cp.x;
    pcX.y = sc1 * eigenVectorsPCA(1, 0) + cp.y;
    pcX.z = sc1 * eigenVectorsPCA(2, 0) + cp.z;
    PointType pcY;
    pcY.x = sc1 * eigenVectorsPCA(0, 1) + cp.x;
    pcY.y = sc1 * eigenVectorsPCA(1, 1) + cp.y;
    pcY.z = sc1 * eigenVectorsPCA(2, 1) + cp.z;
    PointType pcZ;
    pcZ.x = sc1 * eigenVectorsPCA(0, 2) + cp.x;
    pcZ.y = sc1 * eigenVectorsPCA(1, 2) + cp.y;
    pcZ.z = sc1 * eigenVectorsPCA(2, 2) + cp.z;

    auto t2 = system_clock::now();

    // floating-point duration: no duration_cast needed
    duration<double, std::milli> fp_ms = t2 - t1;

    // integral duration: requires duration_cast
    auto int_ms = duration_cast<milliseconds>(fp_ms);

    // converting integral duration to integral duration of shorter divisible time unit: no duration_cast needed
    duration<long, std::micro> int_usec = int_ms;

    std::cout << "took " << fp_ms.count() << " ms, "
              << "or " << int_ms.count() << " whole milliseconds "
              << "(which is " << int_usec.count() << " whole microseconds)" << std::endl;
}

void planeProjection(float a,//a
                     float b,//b
                     float c,//c
                     float d,//d ax+by+cz+d=0
                     pcl::PointCloud<PointType>::Ptr cloud,//pc in
                     pcl::PointCloud<PointType>::Ptr cloud_projected//pc out
                     )
{
    //本例使用ax+by+cz+d=0的平面模型 创建一个系数为a=b==d=0,c=1的平面,也就是X-Y平面。Z轴相关的点全部投影在X-Y面上
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    coefficients->values.resize(4);
    coefficients->values[0] = a;
    coefficients->values[1] = b;
    coefficients->values[2] = c;
    coefficients->values[3] = d;
    // --------------创建滤波器对象-------------------
    pcl::ProjectInliers<PointType> proj;
    proj.setModelType(pcl::SACMODEL_PLANE);
    proj.setInputCloud(cloud);
    proj.setModelCoefficients(coefficients);
    proj.filter(*cloud_projected);
}

void hull_extraction(pcl::PointCloud<PointType>::Ptr cloud,//pc in
                     pcl::PointCloud<PointType>::Ptr cloud_hull,//pc out
                     float alpha = 0.1//alpha
                     )
{
    pcl::console::TicToc time;
    time.tic();

    pcl::ConcaveHull<PointType> chull;
    chull.setInputCloud(cloud); // 输入点云为投影后的点云
    chull.setAlpha(alpha);        // 设置alpha值为0.1
    chull.reconstruct(*cloud_hull);

    cout << "bounding point num: " << cloud_hull->points.size() << endl;
    cout << "time cost: " << time.toc() / 1000 << " sec " << endl;
}

void CounterClockWiseSortPoints(pcl::PointCloud<PointType>& vPoints)
{
    int cnt = static_cast<int>(vPoints.size());
    if (cnt < 3)
        return;
    //计算中心
    Eigen::Vector4f centroid;					// 质心
    pcl::compute3DCentroid(vPoints, centroid);	// 齐次坐标，（c0,c1,c2,1）
    PointType center;
    center.x = centroid[0];
    center.y = centroid[1];
    center.z = centroid[2];
    //若点a小于点b,即点a在点b逆时针方向,返回true,否则返回false
    auto PointCmp = [](const PointType& a, const PointType& b, const PointType& center)
    {
        if (a.x <= 0 && b.x > 0)
            return true;
        if (a.x == 0 && b.x == 0)
            return a.y < b.y;
        //向量OA和向量OB的叉积，向量OA和OB的叉积大于0，则向量OA在向量OB的逆时针方向，即点A小于点B。
        float det = (a.x - center.x) * (b.y - center.y) - (b.x - center.x) * (a.y - center.y);
        if (det < 0)
            return false;
        if (det > 0)
            return true;
        //向量OA和向量OB共线，以距离判断大小
        float d1 = (a.x - center.x) * (a.x - center.x) + (a.y - center.y) * (a.y - center.y);
        float d2 = (b.x - center.x) * (b.x - center.y) + (b.y - center.y) * (b.y - center.y);
        return d1 > d2;
    };

    //冒泡排序
    for (int i = 0; i < cnt - 1; i++)
    {
        for (int j = 0; j < cnt - i - 1; j++)
        {
            if (PointCmp(vPoints[j], vPoints[j + 1], center))
                std::swap(vPoints[j], vPoints[j + 1]);
        }
    }
}

void polygonGeneration(pcl::PointCloud<PointType>::Ptr cloud,pcl::PointCloud<PointType>& contour)
{
    CounterClockWiseSortPoints(*cloud);
//    contour.width = cloud->points.size();
    contour.width = cloud->width;
    contour.height = 1;
    contour.is_dense = false;
    contour.resize(contour.height * contour.width);

    for (size_t i = 0; i < cloud->points.size(); ++i)
    {
        contour.points[i] = cloud->points[i];
    }
}

float S(pcl::PointCloud<PointType> contour)
{
    return pcl::calculatePolygonArea(contour);
}

float C(pcl::PointCloud<PointType> contour)
{
    float distance = 0.0;
    for (size_t i = 0; i < contour.points.size(); ++i)
    {
        if(i==contour.points.size()-1)
        {
            distance = distance + pointDistance(contour.points[i], contour.points[0]);
        }
        else
        {
            distance = distance + pointDistance(contour.points[i], contour.points[i+1]);
        }
    }

    return distance;
}

float pointDistance(PointType p1, PointType p2)
{
    float d;
    d = float(sqrt(double((p1.x-p2.x)*(p1.x-p2.x)+(p1.y-p2.y)*(p1.y-p2.y)+(p1.z-p2.z)*(p1.z-p2.z))));
    return d;
}

void CircleEstimation(pcl::PointCloud<PointType>::Ptr cloud)
{
    pcl::SampleConsensusModelCircle3D<PointType>::Ptr model_circle3D(new pcl::SampleConsensusModelCircle3D<PointType>(cloud));
    pcl::RandomSampleConsensus<PointType> ransac(model_circle3D);
    ransac.setDistanceThreshold(0.05);	        // 距离阈值，与模型距离小于0.01的点作为内点
    ransac.setMaxIterations(100);		        // 最大迭代次数
    ransac.computeModel();				        // 拟合3D圆
    pcl::IndicesPtr inliers(new vector <int>());// 存储内点索引的向量
    ransac.getInliers(*inliers);			    // 提取内点对应的索引

    //----------------根据索引提取圆上的点-------------------------
    pcl::PointCloud<PointType>::Ptr circle_3D(new pcl::PointCloud<PointType>);
    pcl::copyPointCloud<PointType>(*cloud, *inliers, *circle_3D);

    Eigen::VectorXf coeff;
    ransac.getModelCoefficients(coeff);

    cout << "cx:" << coeff[0] << "\n"
        << "cy:" << coeff[1] << "\n"
        << "cz:" << coeff[2] << "\n"
        << "cr:" << coeff[3] << "\n"
        << "cv:" << coeff[4] << ","
        << coeff[5] << ","
        << coeff[6] << endl;

    Rs.push_back(coeff[3]);

    PointType center;
    center.x =  coeff[0] ;
    center.y = coeff[1];
    center.z = coeff[2];

    PointType p_c; // 列表初始化，C++11
    p_c.x += 0;
    p_c.y += 0;
    p_c.z += 0;

    for (auto p : cloud->points)
    {
        p_c.x += p.x;
        p_c.y += p.y;
        p_c.z += p.z;
    }

    p_c.x /= cloud->points.size();
    p_c.y /= cloud->points.size();
    p_c.z /= cloud->points.size();


    float delta_d = 0;
    float ssreg = 0;
    float sstot = 0;
    float ssres = 0;
    float d_bar = 0;
    for (int j =0;j<cloud->points.size();j++)
    {
        d_bar += pointDistance(p_c,cloud->points[j]);
    }
    d_bar = d_bar/cloud->points.size();

    for (int j =0;j<cloud->points.size();j++)
    {
        delta_d += (pointDistance(center,cloud->points[j])-coeff[3])*(pointDistance(center,cloud->points[j])-coeff[3]);
//        sstot += (pointDistance(p_c,cloud->points[j])-d_bar)*(pointDistance(p_c,cloud->points[j])-d_bar);
//        ssreg += (d_bar-coeff[3])*(d_bar-coeff[3]);
        sstot += pointDistance(p_c,cloud->points[j])*pointDistance(p_c,cloud->points[j]);
        ssres += (pointDistance(p_c,cloud->points[j])-coeff[3])*(pointDistance(p_c,cloud->points[j])-coeff[3]);
    }
    delta_d = float(sqrt(double(delta_d/cloud->points.size())));
    float r_square = 1-ssres/sstot;
    cout << "R2:" << r_square << "\n"<<endl;
    cout << "RMSE:" << delta_d << "\n"<<endl;

    R2s.push_back(r_square);
    RMSEs.push_back(delta_d);
}

float str2int(string num) {
    float res;
    stringstream stream(num);
    stream >> res;
    return res;
}

void vec3fNorm(Eigen::Vector3f & vec)
{
    float d = sqrt(vec(0)*vec(0)+vec(1)*vec(1)+vec(2)*vec(2));
    vec(0) = vec(0)/d;
    vec(1) = vec(1)/d;
    vec(2) = vec(2)/d;
}

Eigen::Vector3f vec3fCross(Eigen::Vector3f m, Eigen::Vector3f n)
{
    Eigen::Vector3f res_vec;
    res_vec(0) = m(1)*n(2) - m(2)*n(1);
    res_vec(1) = (-1)*(m(0)*n(2) - m(2)*n(0));
    res_vec(2) = m(0)*n(1) - m(1)*n(0);
    return res_vec;
}

void posCalculationFromOBB(vector <PointType> bboxPoints, vector<Eigen::Vector3f> feature_vectors, vector<double> & posFromOBB,Eigen::Isometry3d& T_base2tcp_tar,vector<Eigen::Vector3d>& orientation_OBB)
{
    vector<Eigen::Vector3d> fvs_d;
    PointType min_point_OBB;
    PointType max_point_OBB;
    PointType position_OBB;
    min_point_OBB = bboxPoints[0];
    max_point_OBB = bboxPoints[1];
    position_OBB = bboxPoints[2];
    for(int i = 0;i<feature_vectors.size();i++)
    {
        Eigen::Vector3d fv_d(double(feature_vectors[i](0)),double(feature_vectors[i](1)),double(feature_vectors[i](2)));
        vec3dNorm(fv_d);
        fvs_d.push_back(fv_d);
    }

    if(fvs_d[2](2)>0)//redirect the vector
    {
        for(int i=1;i<fvs_d.size();i++)
            for(int j=0;j<3;j++)
            {
                fvs_d[i](j) = -fvs_d[i](j);//only need to change y & z ,x is not necessary, otherwise the coordinate sys. direction will be wrong
            }
    }

    PointType transformedPosition;//UR position(x y z)
    transformedPosition.x = position_OBB.x - (OFFSET_FOR_CAPTURING + max_point_OBB.z - min_point_OBB.z)*fvs_d[2](0);
    transformedPosition.y = position_OBB.y - (OFFSET_FOR_CAPTURING + max_point_OBB.z - min_point_OBB.z)*fvs_d[2](1);
    transformedPosition.z = position_OBB.z - (OFFSET_FOR_CAPTURING + max_point_OBB.z - min_point_OBB.z)*fvs_d[2](2);
    Eigen::Isometry3d T_base2cam;
    //obb orientation without clear the data , just put the addition in the end
    orientation_OBB.push_back(fvs_d[0]);//x
    orientation_OBB.push_back(fvs_d[1]);//y
    orientation_OBB.push_back(fvs_d[2]);//z
    T_base2cam.matrix()<<    fvs_d[0](0), fvs_d[1](0), fvs_d[2](0), double(transformedPosition.x),
                             fvs_d[0](1), fvs_d[1](1), fvs_d[2](1), double(transformedPosition.y),
                             fvs_d[0](2), fvs_d[1](2), fvs_d[2](2), double(transformedPosition.z),
                                       0,           0,           0,                             1;
//    Eigen::Isometry3d T_base2tcp_tar;
    Eigen::Isometry3d T_tcp2cam;
    T_tcp2cam.matrix() << 0, 0.99955, 0.029996, -0.03459,
                          0, -0.03, 0.99955, 0.065924,
                          1, 0, 0, 0.12,
                          0, 0, 0, 1;
    T_base2tcp_tar.matrix() = T_base2cam.matrix() * T_tcp2cam.inverse().matrix();

    Eigen::AngleAxisd rv(T_base2tcp_tar.rotation());
    posFromOBB.push_back(double(T_base2tcp_tar.translation()(0)));//x
    posFromOBB.push_back(double(T_base2tcp_tar.translation()(1)));//y
    posFromOBB.push_back(double(T_base2tcp_tar.translation()(2)));//z
    posFromOBB.push_back(double(rv.angle()*rv.axis()(0)));//rx
    posFromOBB.push_back(double(rv.angle()*rv.axis()(1)));//ry
    posFromOBB.push_back(double(rv.angle()*rv.axis()(2)));//rz
}

void posCalculationFromAABB(vector<PointType> bboxPoints, vector<vector<double>> & posFromAABB,vector<Eigen::Isometry3d>& T_base2tcp_tar,vector<Eigen::Vector3d>& orientation_AABB)
{
    PointType min_point_AABB,max_point_AABB;
    min_point_AABB = bboxPoints[0];
    max_point_AABB = bboxPoints[1];
    std::cout<<"min_aabb:"<<min_point_AABB.x<<","<<min_point_AABB.y<<","<<min_point_AABB.z<<","<<std::endl;
    std::cout<<"max_aabb:"<<max_point_AABB.x<<","<<max_point_AABB.y<<","<<max_point_AABB.z<<","<<std::endl;

    vector<float> delta;//delta_x,_y,_z
    delta.push_back(max_point_AABB.x - min_point_AABB.x);//x
    delta.push_back(max_point_AABB.y - min_point_AABB.y);//y
    delta.push_back(max_point_AABB.z - min_point_AABB.z);//z

    PointType p;

    vector<PointType> ABCDEFGH;//abcdefgh
    p.x = min_point_AABB.x; p.y = min_point_AABB.y + delta[1]; p.z = min_point_AABB.z;
    ABCDEFGH.push_back(p);
    p.x = min_point_AABB.x; p.y = min_point_AABB.y; p.z = min_point_AABB.z;
    ABCDEFGH.push_back(p);
    p.x = min_point_AABB.x + delta[0]; p.y = min_point_AABB.y; p.z = min_point_AABB.z;
    ABCDEFGH.push_back(p);
    p.x = min_point_AABB.x + delta[0]; p.y = min_point_AABB.y + delta[1]; p.z = min_point_AABB.z;
    ABCDEFGH.push_back(p);

    p.x = min_point_AABB.x; p.y = min_point_AABB.y + delta[1]; p.z = min_point_AABB.z + delta[2];
    ABCDEFGH.push_back(p);
    p.x = min_point_AABB.x; p.y = min_point_AABB.y; p.z = min_point_AABB.z + delta[2];
    ABCDEFGH.push_back(p);
    p.x = min_point_AABB.x + delta[0]; p.y = min_point_AABB.y; p.z = min_point_AABB.z + delta[2];
    ABCDEFGH.push_back(p);
    p.x = min_point_AABB.x + delta[0]; p.y = min_point_AABB.y + delta[1]; p.z = min_point_AABB.z + delta[2];
    ABCDEFGH.push_back(p);

    vector<PointType> recCenterPoints;
    p.x = 0.5 * (ABCDEFGH[0].x + ABCDEFGH[6].x);p.y = 0.5 * (ABCDEFGH[0].y + ABCDEFGH[6].y);p.z = 0.5 * (ABCDEFGH[0].z + ABCDEFGH[6].z);
    recCenterPoints.push_back(p);//A-p-G middle point
    p.x = 0.5 * (ABCDEFGH[4].x + ABCDEFGH[6].x);p.y = 0.5 * (ABCDEFGH[4].y + ABCDEFGH[6].y);p.z = 0.5 * (ABCDEFGH[4].z + ABCDEFGH[6].z);
    recCenterPoints.push_back(p);//E-p-G middle point

    vector<PointType> lineCenterPoints;
    //middle point of up bounding line  , LOWER PLANT
//    p.x = 0.5 * (ABCDEFGH[4].x + ABCDEFGH[7].x);p.y = 0.5 * (ABCDEFGH[4].y + ABCDEFGH[7].y);p.z = 0.5 * (ABCDEFGH[4].z + ABCDEFGH[7].z);
//    lineCenterPoints.push_back(p);//E-p-H middle point
//    p.x = 0.5 * (ABCDEFGH[4].x + ABCDEFGH[5].x);p.y = 0.5 * (ABCDEFGH[4].y + ABCDEFGH[5].y);p.z = 0.5 * (ABCDEFGH[4].z + ABCDEFGH[5].z);
//    lineCenterPoints.push_back(p);//E-p-F middle point
//    p.x = 0.5 * (ABCDEFGH[5].x + ABCDEFGH[6].x);p.y = 0.5 * (ABCDEFGH[5].y + ABCDEFGH[6].y);p.z = 0.5 * (ABCDEFGH[5].z + ABCDEFGH[6].z);
//    lineCenterPoints.push_back(p);//F-p-G middle point
//    p.x = 0.5 * (ABCDEFGH[7].x + ABCDEFGH[6].x);p.y = 0.5 * (ABCDEFGH[7].y + ABCDEFGH[6].y);p.z = 0.5 * (ABCDEFGH[7].z + ABCDEFGH[6].z);
//    lineCenterPoints.push_back(p);//G-p-H middle point
    //middle point of side rec center point, HIGHER PLANT
    p.x = 0.5 * (ABCDEFGH[0].x + ABCDEFGH[7].x);p.y = 0.5 * (ABCDEFGH[0].y + ABCDEFGH[7].y);p.z = 0.5 * (ABCDEFGH[0].z + ABCDEFGH[7].z);
    lineCenterPoints.push_back(p);//A-p-H middle point
    p.x = 0.5 * (ABCDEFGH[0].x + ABCDEFGH[5].x);p.y = 0.5 * (ABCDEFGH[0].y + ABCDEFGH[5].y);p.z = 0.5 * (ABCDEFGH[0].z + ABCDEFGH[5].z);
    lineCenterPoints.push_back(p);//A-p-F middle point
    p.x = 0.5 * (ABCDEFGH[5].x + ABCDEFGH[2].x);p.y = 0.5 * (ABCDEFGH[5].y + ABCDEFGH[2].y);p.z = 0.5 * (ABCDEFGH[5].z + ABCDEFGH[2].z);
    lineCenterPoints.push_back(p);//F-p-C middle point
    p.x = 0.5 * (ABCDEFGH[7].x + ABCDEFGH[2].x);p.y = 0.5 * (ABCDEFGH[7].y + ABCDEFGH[2].y);p.z = 0.5 * (ABCDEFGH[7].z + ABCDEFGH[2].z);
    lineCenterPoints.push_back(p);//C-p-H middle point

    vector<PointType> transformedPositions;
    double d;
    int m,n;
    for (int i = 0;i<4;i++)   //attention  i = 3 may not be necessary for the pos choosing
    {
        m = i;n = 0;
        d = vectorLength(double(lineCenterPoints[m].x-recCenterPoints[n].x),double(lineCenterPoints[m].y-recCenterPoints[n].y),double(lineCenterPoints[m].z-recCenterPoints[n].z));
        p.x = lineCenterPoints[m].x - OFFSET_FOR_CAPTURING*(recCenterPoints[n].x - lineCenterPoints[m].x)/float(d);
        p.y = lineCenterPoints[m].y - OFFSET_FOR_CAPTURING*(recCenterPoints[n].y - lineCenterPoints[m].y)/float(d);
        p.z = lineCenterPoints[m].z - OFFSET_FOR_CAPTURING*(recCenterPoints[n].z - lineCenterPoints[m].z)/float(d);
        transformedPositions.push_back(p);
    }
    n = 1;
    p.x = recCenterPoints[n].x;
    p.y = recCenterPoints[n].y;
    p.z = recCenterPoints[n].z + OFFSET_FOR_CAPTURING;
    transformedPositions.push_back(p);

    vector<Eigen::Isometry3d> T_base2cam;
    Eigen::Isometry3d T;
    Eigen::Vector3d x_,y_,z_;//transformed x y z axis
    orientation_AABB.clear();

    //pos 1
    z_<< double(recCenterPoints[0].x - lineCenterPoints[0].x ),double( recCenterPoints[0].y - lineCenterPoints[0].y ),double(recCenterPoints[0].z - lineCenterPoints[0].z);
    y_<< double(ABCDEFGH[7].x-ABCDEFGH[4].x),double(ABCDEFGH[7].y-ABCDEFGH[4].y),double(ABCDEFGH[7].z-ABCDEFGH[4].z);
    vec3dNorm(z_);
    vec3dNorm(y_);
    x_ = vec3dCross(y_,z_);
    orientation_AABB.push_back(x_);
    orientation_AABB.push_back(y_);
    orientation_AABB.push_back(z_);
    T.matrix()<< x_(0), y_(0), z_(0), double(transformedPositions[0].x),
                 x_(1), y_(1), z_(1), double(transformedPositions[0].y),
                 x_(2), y_(2), z_(2), double(transformedPositions[0].z),
                     0,     0,     0,        1;
    T_base2cam.push_back(T);

    //pos 2
    z_<< double(recCenterPoints[0].x - lineCenterPoints[1].x ),double( recCenterPoints[0].y - lineCenterPoints[1].y ),double(recCenterPoints[0].z - lineCenterPoints[1].z);
    y_<< double(ABCDEFGH[4].x-ABCDEFGH[5].x),double(ABCDEFGH[4].y-ABCDEFGH[5].y),double(ABCDEFGH[4].z-ABCDEFGH[5].z);
    vec3dNorm(z_);
    vec3dNorm(y_);
    x_ = vec3dCross(y_,z_);
    orientation_AABB.push_back(x_);
    orientation_AABB.push_back(y_);
    orientation_AABB.push_back(z_);
    T.matrix()<< x_(0), y_(0), z_(0), double(transformedPositions[1].x),
                 x_(1), y_(1), z_(1), double(transformedPositions[1].y),
                 x_(2), y_(2), z_(2), double(transformedPositions[1].z),
                     0,     0,     0,        1;
    T_base2cam.push_back(T);

    //pos 3
    z_<< double(recCenterPoints[0].x - lineCenterPoints[2].x ),double( recCenterPoints[0].y - lineCenterPoints[2].y ),double(recCenterPoints[0].z - lineCenterPoints[2].z);
    y_<< double(ABCDEFGH[5].x-ABCDEFGH[6].x),double(ABCDEFGH[5].y-ABCDEFGH[6].y),double(ABCDEFGH[5].z-ABCDEFGH[6].z);
    vec3dNorm(z_);
    vec3dNorm(y_);
    x_ = vec3dCross(y_,z_);
    orientation_AABB.push_back(x_);
    orientation_AABB.push_back(y_);
    orientation_AABB.push_back(z_);
    T.matrix()<< x_(0), y_(0), z_(0), double(transformedPositions[2].x),
                 x_(1), y_(1), z_(1), double(transformedPositions[2].y),
                 x_(2), y_(2), z_(2), double(transformedPositions[2].z),
                     0,     0,     0,        1;
    T_base2cam.push_back(T);

//    //pos 4
//    z_<< double(recCenterPoints[0].x - lineCenterPoints[3].x ),double( recCenterPoints[0].y - lineCenterPoints[3].y ),double(recCenterPoints[0].z - lineCenterPoints[3].z);
//    y_<< double(ABCDEFGH[6].x-ABCDEFGH[7].x),double(ABCDEFGH[6].y-ABCDEFGH[7].y),double(ABCDEFGH[6].z-ABCDEFGH[7].z);
//    vec3dNorm(z_);
//    vec3dNorm(y_);
//    x_ = vec3dCross(y_,z_);
//    orientation_AABB.push_back(x_);
//    orientation_AABB.push_back(y_);
//    orientation_AABB.push_back(z_);
//    T.matrix()<< x_(0), y_(0), z_(0), double(transformedPositions[3].x),
//                 x_(1), y_(1), z_(1), double(transformedPositions[3].y),
//                 x_(2), y_(2), z_(2), double(transformedPositions[3].z),
//                     0,     0,     0,        1;
//    T_base2cam.push_back(T);

    //pos 5
    z_<< double(recCenterPoints[0].x - recCenterPoints[1].x ),double(recCenterPoints[0].y - recCenterPoints[1].y ),double(recCenterPoints[0].z - recCenterPoints[1].z);
    y_<< double(ABCDEFGH[6].x-ABCDEFGH[7].x),double(ABCDEFGH[6].y-ABCDEFGH[7].y),double(ABCDEFGH[6].z-ABCDEFGH[7].z);
    vec3dNorm(z_);
    vec3dNorm(y_);
    x_ = vec3dCross(y_,z_);
    orientation_AABB.push_back(x_);
    orientation_AABB.push_back(y_);
    orientation_AABB.push_back(z_);
    T.matrix()<< x_(0), y_(0), z_(0), double(transformedPositions[4].x),
                 x_(1), y_(1), z_(1), double(transformedPositions[4].y),
                 x_(2), y_(2), z_(2), double(transformedPositions[4].z),
                     0,     0,     0,        1;
    T_base2cam.push_back(T);

//    vector<Eigen::Isometry3d> T_base2tcp_tar;//for UR matrix form
    Eigen::Isometry3d T_tcp2cam;
    T_tcp2cam.matrix() << 0, 0.99955, 0.029996, -0.03459,
                          0, -0.03, 0.99955, 0.065924,
                          1, 0, 0, 0.12,
                          0, 0, 0, 1;
    for(int i=0;i<T_base2cam.size();++i)
    {
        Eigen::Isometry3d T_tar;
        T_tar.matrix() = T_base2cam[i].matrix() * T_tcp2cam.inverse().matrix();
        T_base2tcp_tar.push_back(T_tar);
    }

    for(int i = 0;i<T_base2tcp_tar.size();i++)
    {
        Eigen::AngleAxisd rv(T_base2tcp_tar[i].rotation());
        vector<double> pos_6d;
        pos_6d.push_back(double(T_base2tcp_tar[i].translation()(0)));//x
        pos_6d.push_back(double(T_base2tcp_tar[i].translation()(1)));//y
        pos_6d.push_back(double(T_base2tcp_tar[i].translation()(2)));//z
        pos_6d.push_back(double(rv.angle()*rv.axis()(0)));//rx
        pos_6d.push_back(double(rv.angle()*rv.axis()(1)));//ry
        pos_6d.push_back(double(rv.angle()*rv.axis()(2)));//rz

        posFromAABB.push_back(pos_6d);
        std::cout<<pos_6d[0]<<","<<pos_6d[1]<<","<<pos_6d[2]<<","<<pos_6d[3]<<","<<pos_6d[4]<<","<<pos_6d[5]<<","<<std::endl;
    }
}

void vec3dNorm(Eigen::Vector3d & vec)
{
    double d = sqrt(vec(0)*vec(0)+vec(1)*vec(1)+vec(2)*vec(2));
    vec(0) = vec(0)/d;
    vec(1) = vec(1)/d;
    vec(2) = vec(2)/d;
}

double vectorLength(double rx, double ry, double rz)
{
    return (double)sqrt(rx*rx+ry*ry+rz*rz);
}

Eigen::Vector3d vec3dCross(Eigen::Vector3d m, Eigen::Vector3d n)
{
    Eigen::Vector3d res_vec;
    res_vec(0) = m(1)*n(2) - m(2)*n(1);
    res_vec(1) = (-1)*(m(0)*n(2) - m(2)*n(0));
    res_vec(2) = m(0)*n(1) - m(1)*n(0);
    return res_vec;
}
