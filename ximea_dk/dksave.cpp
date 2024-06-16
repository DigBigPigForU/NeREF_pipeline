#include "dksave.h"
#include <stdlib.h>
#include <stdio.h>

dkSave::dkSave(QObject *parent) : QThread(parent)
{

}

dkSave::~dkSave()
{

}

void dkSave::run()  //保存RGBD图像
{
                    std::cout << "Button click!" << std::endl;
                    try
                    {
                    kinect->i++;
                    // 判断文件夹是否存在，不存在则创建
//                    QString fullPath = QString("D:\\ximea_dk_20210910\\%1").arg(kinect->i);
                    QString fullPath = QString("%1").arg(kinect->i);
                    QDir dir(fullPath);
                    if(!dir.exists()){
                        bool ismkdir = QDir().mkdir(fullPath);
                        if(!ismkdir)
                            qDebug() << "Create path fail" << endl;
                        else
                            qDebug() << "Create fullpath success" << endl;
                    }
                    else{
                        qDebug() << "fullpath exist" << endl;
                    }
//                    string command;
//                    command = "mkdir " + to_string(kinect->i);
//                    int sys = system(command.c_str());
//                    kinect->colorStr = to_string(kinect->i) + "\\" + to_string(kinect->i)+"_color.png";
//                    kinect->t_colorStr = to_string(kinect->i) + "\\" + to_string(kinect->i) + "_t_color.png";
//                    kinect->dt_xyzStr = to_string(kinect->i) + "\\" + to_string(kinect->i) + "_dt_xyz.tif";
//                    kinect->ct_xyzStr = to_string(kinect->i) + "\\" + to_string(kinect->i) + "_ct_xyz.tif";

                    kinect->colorStr = fullPath.toStdString() + "\\" + to_string(kinect->i)+"_color.png";
                    kinect->t_colorStr = fullPath.toStdString() + "\\" + to_string(kinect->i) + "_t_color.png";
                    kinect->dt_xyzStr = fullPath.toStdString() + "\\" + to_string(kinect->i) + "_dt_xyz.tif";
                    kinect->ct_xyzStr = fullPath.toStdString() + "\\" + to_string(kinect->i) + "_ct_xyz.tif";

                    kinect->pcStr = fullPath.toStdString() + "\\" + to_string(kinect->i) + "_pc.pcd";
                    //dt_fStr = to_string(i) + "_dt_float.tif";
                    //ct_fStr = to_string(i) + "_ct_float.tif";
                    //*********point cloud saving**************
                    pcl::io::savePCDFileASCII (kinect->pcStr,  kinect->dk_cloud);
                    //******************data collection programming here img uint8
                    cv::imwrite(kinect->colorStr, kinect->colorFrame);//ok
                    cv::imwrite(kinect->t_colorStr, kinect->t_colorFrame);//ok
                    cv::imwrite(kinect->dt_xyzStr, kinect->my_dt_xyzFrame);
                    cv::imwrite(kinect->ct_xyzStr, kinect->my_ct_xyzFrame);
                    //cv::imwrite(dt_fStr, dt_FloatFrame);
                    //cv::imwrite(ct_fStr, ct_FloatFrame);
                    //*********************data collection programming here Mat  original data int16 & float32/1000
                    ofstream fout;
//                    fout.open(to_string(kinect->i)+"\\"+to_string(kinect->i) + "_ct_x_int16.txt");
                    fout.open(fullPath.toStdString()+"\\"+to_string(kinect->i) + "_ct_x_int16.txt");
                    fout << kinect->ct_channels.at(0).rows << std::endl;
                    fout << kinect->ct_channels.at(0).cols << std::endl;
                    for (int i = 0; i < kinect->ct_channels.at(0).rows; i++) {
                        for (int j = 0; j < kinect->ct_channels.at(0).cols; j++) {
                            fout << kinect->ct_channels.at(0).at<int16_t>(i, j) << std::endl;
                        }
                    }
                    fout << flush;
                    fout.close();

//                    fout.open(to_string(kinect->i) + "\\" + to_string(kinect->i) + "_ct_y_int16.txt");
                    fout.open(fullPath.toStdString() + "\\" + to_string(kinect->i) + "_ct_y_int16.txt");
                    fout << kinect->ct_channels.at(1).rows << std::endl;
                    fout << kinect->ct_channels.at(1).cols << std::endl;
                    for (int i = 0; i < kinect->ct_channels.at(1).rows; i++) {
                        for (int j = 0; j < kinect->ct_channels.at(1).cols; j++) {
                            fout << kinect->ct_channels.at(1).at<int16_t>(i, j) << std::endl;
                        }
                    }
                    fout << flush;
                    fout.close();

//                    fout.open(to_string(kinect->i) + "\\" + to_string(kinect->i)+"_ct_z_int16.txt");
                    fout.open(fullPath.toStdString() + "\\" + to_string(kinect->i)+"_ct_z_int16.txt");
                    fout << kinect->ct_channels.at(2).rows << std::endl;
                    fout << kinect->ct_channels.at(2).cols << std::endl;
                    for (int i = 0; i < kinect->ct_channels.at(2).rows; i++) {
                        for (int j = 0; j < kinect->ct_channels.at(2).cols; j++) {
                            fout << kinect->ct_channels.at(2).at<int16_t>(i, j) << std::endl;
                        }
                    }
                    fout << flush;
                    fout.close();

//                    fout.open(to_string(kinect->i) + "\\" + to_string(kinect->i) + "_dt_x_int16.txt");
                    fout.open(fullPath.toStdString() + "\\" + to_string(kinect->i) + "_dt_x_int16.txt");
                    fout << kinect->dt_channels.at(0).rows << std::endl;
                    fout << kinect->dt_channels.at(0).cols << std::endl;
                    for (int i = 0; i < kinect->dt_channels.at(0).rows; i++) {
                        for (int j = 0; j < kinect->dt_channels.at(0).cols; j++) {
                            fout << kinect->dt_channels.at(0).at<int16_t>(i, j) << std::endl;
                        }
                    }
                    fout << flush;
                    fout.close();

//                    fout.open(to_string(kinect->i) + "\\" + to_string(kinect->i) + "_dt_y_int16.txt");
                    fout.open(fullPath.toStdString() + "\\" + to_string(kinect->i) + "_dt_y_int16.txt");
                    fout << kinect->dt_channels.at(1).rows << std::endl;
                    fout << kinect->dt_channels.at(1).cols << std::endl;
                    for (int i = 0; i < kinect->dt_channels.at(1).rows; i++) {
                        for (int j = 0; j < kinect->dt_channels.at(1).cols; j++) {
                            fout << kinect->dt_channels.at(1).at<int16_t>(i, j) << std::endl;
                        }
                    }
                    fout << flush;
                    fout.close();

//                    fout.open(to_string(kinect->i) + "\\" + to_string(kinect->i) + "_dt_z_int16.txt");
                    fout.open(fullPath.toStdString() + "\\" + to_string(kinect->i) + "_dt_z_int16.txt");
                    fout << kinect->dt_channels.at(2).rows << std::endl;
                    fout << kinect->dt_channels.at(2).cols << std::endl;
                    for (int i = 0; i < kinect->dt_channels.at(2).rows; i++) {
                        for (int j = 0; j < kinect->dt_channels.at(2).cols; j++) {
                            fout << kinect->dt_channels.at(2).at<int16_t>(i, j) << std::endl;
                        }
                    }
                    fout << flush;
                    fout.close();
                    emit isOK();
                    cout << "************************   " << "No."<< kinect->i <<" dk data    ************************" <<std::endl;
                    cout << "Successfully saved all data to path!" << std::endl;
                    }
                    catch (...)
                    {
                        cout <<"Unsuccessfully saved!" << std::endl;
                    }
}
