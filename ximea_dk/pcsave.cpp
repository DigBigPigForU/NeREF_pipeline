#include "pcsave.h"
#include <stdlib.h>
#include <stdio.h>

pcSave::pcSave(QObject *parent) : QThread(parent)
{

}

pcSave::~pcSave()
{

}

void pcSave::run()  //保存pc
{
                    std::cout << "Button click!" << std::endl;
                    try
                    {
                    kinect->i_pc++;
                    kinect->pcStr = to_string(kinect->i_pc) + "_pc.pcd";
                    //*********point cloud saving**************
                    pcl::io::savePCDFileASCII (kinect->pcStr, kinect->dk_cloud);
                    emit pcSaveIsOK();
                    cout << "************************   " << "No."<< kinect->i_pc <<" dk point cloud data    ************************" <<std::endl;
                    cout << "Successfully saved all pc data to path!" << std::endl;
                    }
                    catch (...)
                    {
                        cout <<"Unsuccessfully saved!" << std::endl;
                    }
}
