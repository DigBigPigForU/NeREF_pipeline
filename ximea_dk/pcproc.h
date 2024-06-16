#ifndef PCPROC_H
#define PCPROC_H
#define NOMINMAX

#define OFFSET_FOR_CAPTURING 0.3
#define X_MIN_  0.35
#define X_MAX_  1.05
#define Y_MIN_  -0.35
#define Y_MAX_  0.25
#define Z_MIN_  0.0
#define Z_MAX_  0.6

#include <QObject>
#include "ur.h"
#include "dk.h"
#include "ur_kin.h"
#include "pso.h"
//new addidition of include file

//hsv seg
#include <iostream>
#include <boost/thread/thread.hpp>
#include <boost/shared_ptr.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/io.h>
#include <pcl/io/ascii_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/point_types.h>
#include <pcl/point_types_conversion.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/pcl_plotter.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/poisson.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/features/principal_curvatures.h>
#include <boost/thread/thread.hpp>
#include <math.h>

//bbox
#include<iostream>
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/visualization/cloud_viewer.h>
#include <boost/thread/thread.hpp>

//region growth
#include <iostream>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <windows.h>
#include <stdio.h>
#include <psapi.h>

//pass through
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>

//radius outlier removal
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/time.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/visualization/pcl_visualizer.h>

//voxel filtering
#include <iostream>
#include <sstream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

//icp registration
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <boost/thread/thread.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/time.h>

//rmse calculation
#include <iostream>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/search/kdtree.h>

//boundary extraction
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/boundary.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>

//sphere point cloud formation
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>

//RKNN
#include <iostream>
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>

//octree using for the space detection
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/octree/octree.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>

//convex hull
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/surface/concave_hull.h>
#include <boost/thread/thread.hpp>
#include <pcl/visualization/pcl_visualizer.h>

//plane clipper 3d
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/plane_clipper3D.h>
#include <pcl/filters/extract_indices.h>
#include <boost/thread/thread.hpp>
#include <pcl/visualization/pcl_visualizer.h>

using namespace std;
using namespace cv;
using namespace boost;
using namespace pcl;
using namespace ur_kinematics;



struct sphere
{
    float centerX;
    float centerY;
    float centerZ;
    float radius;
};

typedef pcl::PointXYZRGBA PointType;

class pcProc : public QObject
{
    Q_OBJECT
public:
    explicit pcProc(QObject *parent = nullptr);
    ~pcProc();
    //pc data list
    vector<pcl::PointCloud<PointType>> vec_pc_memory_cam;
    vector<pcl::PointCloud<PointType>> vec_pc_memory_base;
    vector<pcl::PointCloud<PointType>> vec_plant_memory_base;
    vector<pcl::PointCloud<PointType>> vec_pc_bbox_cam;
    vector<pcl::PointCloud<PointType>> vec_pc_bbox_base;
    vector<pcl::PointCloud<PointType>> vec_plant_bbox_base;
    vector<pcl::PointCloud<PointType>> vec_pc_ok_cam;//best pos for data acquiring
    vector<pcl::PointCloud<PointType>> vec_pc_ok_base;
    pcl::PointCloud<PointType> pc_iter_cam;//iteration
    pcl::PointCloud<PointType> pc_iter_base;//iteration
    vector<pcl::PointCloud<PointType>> vec_plant_ok_base;
    pcl::PointCloud<PointType> plant_memory;
    pcl::PointCloud<PointType> plant_bbox;
    pcl::PointCloud<PointType> plant_ok;
    pcl::PointCloud<PointType> colored_cluster_cloud;
    vector<pcl::PointCloud<PointType>> pc_clusters;
    vector<int> isReachable_AABB;
    vector<int> isReachable_OBB;
    vector<vector<double>> posFromAABB;
    vector<vector<double>> posFromOBB;
    vector<Eigen::Vector3d> orientation_AABB;//x1 y1 z1 x2 y2 z2 ...
    vector<Eigen::Vector3d> orientation_OBB;
    //----occlusion related-------
    vector<Eigen::Vector3d> axis_ok;
    vector<Eigen::Isometry3d> T_base2tcp_ok;
    vector<pcl::PointCloud<PointType>> pc_clusters_ok;
    vector<vector<double>> posFromOBB_ok;
    vector<vector <PointType>> obbPoints_vec_ok;
    vector<Eigen::Matrix3f> rotational_matrix_OBB_vec_ok;
    //define a vec to figure out whether or not the pose need to be changed for occlusion
    vector<int> isOccluded;
    //define all the variables for iteration, the original variables will be not kept
    vector<vector<double>> ur_pose_iter;//after changed
    vector<Eigen::Isometry3d> T_base2tcp_iter;//after changed
    vector<Eigen::Vector3d> axis_iter;//after changed
    vector<pcl::PointCloud<PointType>> pc_clusters_iter;//after changed, fullfill the occluded cluster, the others kept
    //just for visualization, change them into global vars
    int j_best;
    vector<pcl::PointCloud<PointType>> pc_b;//bounding line pc data
    vector<pcl::PointCloud<PointType>> pc_clusters_single_frame;//pc data
    vector<pcl::PointCloud<PointType>> pc_sp;//sphere projected pc data
    vector<int> idx_spa;//sphere projected adjacent pc index
    //----------------new method based on Monte Carlo method-------------
    pcl::PointCloud<PointType> plant_convex_hull;
    vector<pcl::Vertices> polygons;
    vector<Eigen::Vector3d> orientation_MC;
    vector<vector<double>> posFromMC;
    vector<Eigen::Isometry3d> T_base2tcp_MC;
    vector<int> isReachable_MC;
    //-------------------------------

    //plant & bbox
    //aabb for the whole plant
    vector <float> moment_of_inertia,
                    eccentricity,
                    feature_values;
    vector <PointType> bboxPoints;
    Eigen::Matrix3f rotational_matrix_OBB;
    vector<Eigen::Vector3f> feature_vectors;
    //obb for plant parts (vec)
    vector<vector <float>> moment_of_inertia_vec,
                    eccentricity_vec,
                    feature_values_vec;
    vector<vector <PointType>> obbPoints_vec;
    vector<Eigen::Matrix3f> rotational_matrix_OBB_vec;
    vector<vector<Eigen::Vector3f>> feature_vectors_vec;
    //plant part & bbox

    Eigen::Isometry3d T_tcp2cam;
    vector<Eigen::Isometry3d> T_memory_base2tcp;
    vector<Eigen::Isometry3d> T_bbox_base2tcp;
    vector<Eigen::Isometry3d> T_ok_base2tcp;

    int flag;//select which kind of pc to be obtained
    bool isPcAcquiringFinished;
    //function for pc processing
    void dataClear();
    void adaptiveCapturing();//consider using multi-thread
    void autoSavingMemPc();
    void memoryPcAcquiring();
    void memoryTransformationAcquiring();
    void bboxPcAcquiring();
    void bboxTransformationAcquiring();
    void okPcAcquiring();
    void okTransformationAcquiring();
    void iterPcAcquiring();
    void vec3dNorm(Eigen::Vector3d &);
    void vec3fNorm(Eigen::Vector3f &);
    float norm2f(Eigen::Vector3f);
    double norm2d(Eigen::Vector3d);
    Eigen::Vector3d vec3dCross(Eigen::Vector3d,Eigen::Vector3d);
    float vec3fDot(Eigen::Vector3f,Eigen::Vector3f);
    //saving all point cloud data to the local path
    void autoPcDataSaving();
    void pcDataSaving();
    //firstly build a directory of the files
    //then save the pc data in the vector and the integrated data
    void waitForStablePc();//wait for stable pc data
    void waitForPcAcquiringFinished();
    void transformMemoryPcCamToBase();
    void transformBboxPcCamToBase();
    void transformOKPcCamToBase();
    //AABB OR OBB
    void bboxCalculationForPc(pcl::PointCloud<PointType>,
                                vector <float> &,//vector <float> moment_of_inertia;
                                vector <float> &,//vector <float> eccentricity
                                vector <PointType> &,//min_point_AABB, max_point_AABB, min_point_OBB, max_point_OBB, position_OBB
                              Eigen::Matrix3f &,// rotational_matrix_OBB
                              vector <float> &,//major_value, middle_value, minor_value
                              vector<Eigen::Vector3f> &//  Eigen::Vector3f major_vector, middle_vector, minor_vector, mass_center;
                              );
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
    void jointPoseFromMatrixAndCriteria(Eigen::Isometry3d,//T matrix
                                        vector<double>&,// joint pose
                                        vector<double>//joint pose criteria, current joint pose should be ok, will be normalized in this func
                                        );//for single
    void poseSorting_OBB(vector<vector<double>>&,//list of 6d pos vector, will be sorted after execute the function
                     vector<Eigen::Isometry3d>&,//list of T matrix, without sorting
                     vector<double>,//joint pose criteria, current joint pose should be ok, will be normalized in this func
                     vector<int>&,//is Reachable or not  {0,0,0,0,1,1,..}
                     vector<Eigen::Vector3d>&,//orientation vec for visualization
                     vector<pcl::PointCloud<PointType>>& pc_clusters//adjust the order of the pc clusters
                     );//for vector obb
    void poseSorting_AABB(vector<vector<double>>&,//list of 6d pos vector, will be sorted after execute the function
                     vector<Eigen::Isometry3d>&,//list of T matrix, without sorting
                     vector<double>,//joint pose criteria, current joint pose should be ok, will be normalized in this func
                     vector<int>&,//is Reachable or not  {0,0,0,0,1,1,..}
                     vector<Eigen::Vector3d>&//orientation vec for visualization
                     );//for vector aabb
    //cluster
    void pcClustering(pcl::PointCloud<PointType>,vector<PointIndices>&,pcl::PointCloud<PointType>&);//cluster a single pc to parts (plant -> leaves)
    void extractPcClustersFromIndices(pcl::PointCloud<PointType>,vector<PointIndices>,vector<pcl::PointCloud<PointType>>&);
    pcl::PointCloud<PointType> IcpRegistrationForPair(pcl::PointCloud<PointType>,pcl::PointCloud<PointType>);//registration for pc pair, return the output pc
    //down sampling & segmentation methods
    pcl::PointCloud<PointType> mergePcVec(vector<pcl::PointCloud<PointType>>);//merge pc vec to form a single pc (using downsampling)
    pcl::PointCloud<PointType> voxelFilteringPc(pcl::PointCloud<PointType>);//using voxel filtering to downsampl a single pc, return the result pc
    pcl::PointCloud<PointType> basicVoxelFilteringPc(pcl::PointCloud<PointType>);
    vector<pcl::PointCloud<PointType>> passThroughForVec(vector<pcl::PointCloud<PointType>>);
    pcl::PointCloud<PointType> passThroughPc(pcl::PointCloud<PointType>);//pass through  x y z  according limit. return result
    pcl::PointCloud<PointType> radiusFilteringPc(pcl::PointCloud<PointType>);//radius filtering Pc to remove the noise , return result
    pcl::PointCloud<PointType> HSVSegForPc(pcl::PointCloud<PointType>);//hsv space  segmentation  for a single point cloud , return the result
    vector<pcl::PointCloud<PointType>> pcVectorTransform(vector<pcl::PointCloud<PointType>>,vector<Eigen::Isometry3d>,Eigen::Isometry3d);//transformation for a pc vector
    vector<pcl::PointCloud<PointType>> IcpRegistrationForVec(vector<pcl::PointCloud<PointType>>);//registration for the whole vec, base on the 1st pc, return the registered vec(1st coordinate)
    double computeCloudRMSE(pcl::PointCloud<PointType>::ConstPtr,pcl::PointCloud<PointType>::ConstPtr,double);//rmse cal for pc pair
    void boundaryExtraction(pcl::PointCloud<PointType>,pcl::PointCloud<PointType>&);//point cloud boundary extraction
    void sphereFormation(pcl::PointCloud<PointType>,pcl::PointCloud<PointType>&,float,float,float,float);//pc projection on one specific sphere in out x y z r
    void RKNNForPoint(pcl::PointCloud<PointType>,PointType,vector<int>&,int,float);//pc_in search_point idx num_for_search r
    float pointDistance(PointType,PointType);
    int occlusionJudgeForPoint(PointType,PointType,PointType);//viewpoint p1 p2  if p1 nearer, return 1 ,else return -1,equal for 0
    int isOcclusionOrNotByRatio(pcl::PointCloud<PointType>,vector<int>,float);//use ratio to judge whether or not the occlusion exists,point cloud, idx, ratio to judge , point num of the bounding line
    int onceMoreIsNeededOrNotByOctree(pcl::PointCloud<PointType>,pcl::PointCloud<PointType>,float);//use ratio to judge whether or not to do the iteration once more, to judge the gaining of the point cloud octree in the space
    void RKNNForPointCloud(pcl::PointCloud<PointType>,pcl::PointCloud<PointType>,pcl::PointCloud<PointType>,pcl::PointCloud<PointType>,vector<int>&,PointType,int,float);//pc_in pc_search pc_in_bp pc_search_bp idx_all num_for_search r
    void RKNNForPointCloudVec(pcl::PointCloud<PointType>,vector<pcl::PointCloud<PointType>>,pcl::PointCloud<PointType>,vector<pcl::PointCloud<PointType>>,vector<int>&,PointType,int,float);//pc_in pc_search_vec pc_in_bp pc_search_bp_vec idx_all num_for_search r
    void occlusionPoseCalculation(pcl::PointCloud<PointType>,//leaf pc
                                  vector<int>,//bounding line pc index
                                  PointType&,//mass point (x,y,z) of the leaf pc ,just single frame (currently)
                                  Eigen::Vector3d&//orientation of the pose (x_,y_,z_)
                                  );//calculate the direction and orientation base on the pc of the leaves and the occlusion point cloud
    void poseTransformFromOcclusion(Eigen::Isometry3d,//original obb pose T_matrix
                                    vector<Eigen::Vector3d>,//original orientation three vectors represent the original three axis
                                    Eigen::Vector3d,//orientation of the original z, it will not change in the iteration
                                    PointType,//mass point of bounding line
                                    Eigen::Vector3d,//orientation of the occlusion pose (x_,y_,z_)
                                    int,//times0 1 2 3
                                    vector<double>&,// just need one pos to move UR (new)
                                    Eigen::Isometry3d&,//T matrix (new)
                                    vector<Eigen::Vector3d>&//restore the orientation vec for vtk visualization (new)
                                    );//45 deg anti-clockwise rotate & 5 cm offset
    //Monte Carlo method, some related functions
    void convexHullCalculation(pcl::PointCloud<PointType>,//cloud input
                               pcl::PointCloud<PointType>&,//cloud output
                               vector<pcl::Vertices>&//indices of polygons
                               );
    void bottomPlaneForOBB(vector <PointType>,//min_point_OBB, max_point_OBB, position_OBB
                           vector<Eigen::Vector3f>,//the feature vectors, in order : major , middle ,minor
                           Eigen::Vector4f &,//the 4 param of a plane
                           PointType &,//the center of the sphere
                           vector<Eigen::Vector3f> &//the axis of the sphere coordination, namely the x axis & z axis (position direction)
                           );//calculate some params for the bottom plane of the obb, e.g. the sphere & the plane & the center point
    pcl::PointCloud<PointType> pcFromHemisphere(PointType,//the center of the sphere
                                                vector<Eigen::Vector3f>,//the axis of the sphere coordination, namely the x axis & z axis (position direction)
                                                float,//the radius of the hemisphere
                                                int//the interval degree of generating point cloud, degree, 1/2/3/5...
                                                );//generate the pc according to the params that can form a hemisphere, theta & phi
    pcl::PointCloud<PointType> pcFromTriangle(PointType,//the center of the sphere, A
                                              PointType,//the center of the sphere, B
                                              PointType,//the center of the sphere, C
                                              float//the interval of the generating point cloud
                                              );//generate the pc according to the params that can form a triangle, p1,p2,p3
    void planeClipper( pcl::PointCloud<PointType>,//cloud in
                       pcl::PointCloud<PointType>&,//cloud out
                       pcl::IndicesPtr,//indices vec ptr
                       Eigen::Vector4f,//plane params
                       bool//+ or -
                       );
    void changeOrNotDetection(pcl::PointCloud<PointType>,//cloud in A
                              pcl::PointCloud<PointType>,//cloud in B
                              pcl::PointIndices::Ptr,//indices of changed
                              pcl::PointCloud<PointType> &,//cloud out, using bool to keep whether the changed or the unchanged
                              bool//keep which part? changed or unchanged?
                              );//use octree to detect the change or unchanged part between two point cloud
    float solidAngleForTriangle(PointType,//the center of the sphere, O, view point
                                PointType,//the center of the sphere, A
                                PointType,//the center of the sphere, B
                                PointType//the center of the sphere, C
                                );
    void selectPointInPc(pcl::PointCloud<PointType>,//cloud in
                         PointType &,//point out
                         float,//the radius of the hemisphere
                         int,//the interval of the generating point cloud, in degree
                         PointType,//the center of the sphere
                         Eigen::Vector3f//z axis
                         );//select a point that has most point nearby, using radius search
    PointType selectPointWihtMinAngle(pcl::PointCloud<PointType>::Ptr,//cloud in
                                      PointType,//the center of the sphere
                                      Eigen::Vector3f//z axis
                                      );//select a point that has min vector angle to z_axis
    void posCalculationFromMonteCarlo(PointType,//the view point, on the hemisphere, not considering the offset
                                    PointType,//the center of the sphere
                                    float,//radius of the sphere
                                    vector<Eigen::Vector3f>,//x  z  axis
                                    vector<double>&,// OBB just need one pos to move UR
                                    Eigen::Isometry3d&,//T matrix
                                    vector<Eigen::Vector3d>&//restore the orientation vec for vtk visualization
                                    );
    template<typename T>
    std::vector<int> findItems(std::vector<T> const &, int);

    template <typename T>
    vector<size_t> sort_indexes_e(vector<T> &);
    //signals for reminding ur or sensor to perform operations
    void pcDataSavingDown();

public slots:
    //slots
    void dealURArrival();

public:
    dk *kinect; //corrent point cloud obtaining source
    UR *ur; //corrent tcp pos obtaining source
};

#endif // PCPROC_H
