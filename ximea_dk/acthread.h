#ifndef ACTHREAD_H
#define ACTHREAD_H
#define SCALE_AABB 0.1
#include "pcproc.h"

class acThread : public QThread
{
    Q_OBJECT
public:
    explicit acThread(QObject *parent = 0);
    ~acThread();
protected:
    void run();
public:
    void dataVisualization(pcl::PointCloud<PointType>); //visualization in this thread

    void AABBVisualization(pcl::PointCloud<PointType>,vector <PointType>);

    //not necessary all pose can be reached
    void AABBAndPoseVisualization(pcl::PointCloud<PointType>,vector <PointType>,vector<vector<double>>,vector<Eigen::Vector3d>,vector<int>);

    //not necessary all pose can be reached
    void OBBAndPoseVisualization(pcl::PointCloud<PointType>,vector<vector<PointType>>,vector<Eigen::Matrix3f>,vector<vector<double>>,vector<Eigen::Vector3d>,vector<int>);

    //please be sure all pose can be reached, not necessary all pose is occluded
    void OBBAndPoseWithAndWithoutOcclusionVisualization(pcl::PointCloud<PointType>,vector<vector<PointType>>,vector<Eigen::Matrix3f>,vector<vector<double>>,vector<vector<double>>,vector<Eigen::Vector3d>,vector<Eigen::Vector3d>,vector<int>);

    //this func is for the visualization of the bounding pc of the single frame pc clusters
    void boundingLineVisulization(vector<pcl::PointCloud<PointType>>,vector<pcl::PointCloud<PointType>>,int);

    //this func is for the vis of the sphere projected bounding line pc, different color for distinguishing different clusters, especially for the j_best one
    void sphereProjectionVisulization(vector<pcl::PointCloud<PointType>>,int);
    
    //this func is for the vis of the adjacent pc line of the j_best pc cluster
    void adjacentLineVisulization(vector<pcl::PointCloud<PointType>>,vector<int>,int);

    void clusterVisualization(pcl::PointCloud<PointType>);

    void multiWindowVisualization(pcl::PointCloud<PointType>,pcl::PointCloud<PointType>,pcl::PointCloud<PointType>);

    pcProc *kpcProcess;// data acquiring
signals:

public slots:

};

#endif // ACTHREAD_H
