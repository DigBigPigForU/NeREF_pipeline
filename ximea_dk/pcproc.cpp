#include "pcproc.h"

pcProc::pcProc(QObject *parent) : QObject(parent)
{

}

pcProc::~pcProc()
{

}

void pcProc::autoSavingMemPc()
{
    //clear all the remaining data
    dataClear();

    //set flag 1 to acquire the pc data of the memory pos
    flag = 1;

    //move to all memory pos
    for(int i = 0;i < ur->memoryNum;i++)
    {
        //set the flag to judge whether or not the pc acquiring is finished
        isPcAcquiringFinished = 0;//reset to 0
        ur->tcpMoveDirectly(ur->vec_memory_tcp_pos[i][0],
                            ur->vec_memory_tcp_pos[i][1],
                            ur->vec_memory_tcp_pos[i][2],
                            ur->vec_memory_tcp_pos[i][3],
                            ur->vec_memory_tcp_pos[i][4],
                            ur->vec_memory_tcp_pos[i][5]);
        waitForPcAcquiringFinished();//0 is not ok until changed to 1
    }

    transformMemoryPcCamToBase();

    autoPcDataSaving();   //it may cause a block by adding this saving step, no reason yet...
}

void pcProc::adaptiveCapturing()
{
    //clear all the remaining data
    dataClear();

    //set flag 1 to acquire the pc data of the memory pos
    flag = 1;

    //move to all memory pos
    for(int i = 0;i < ur->memoryNum;i++)
    {
        //set the flag to judge whether or not the pc acquiring is finished
        isPcAcquiringFinished = 0;
        ur->tcpMoveDirectly(ur->vec_memory_tcp_pos[i][0],
                            ur->vec_memory_tcp_pos[i][1],
                            ur->vec_memory_tcp_pos[i][2],
                            ur->vec_memory_tcp_pos[i][3],
                            ur->vec_memory_tcp_pos[i][4],
                            ur->vec_memory_tcp_pos[i][5]);
        waitForPcAcquiringFinished();
    }

    //coordination tranformation  cam2base  memory pos
    transformMemoryPcCamToBase();

    //plant point cloud segmentation
    //need to optimize the best params for the hsv segmentation using the offline pc data
    vec_plant_memory_base = passThroughForVec(vec_pc_memory_base);

    //ICP fine registration
//    vector<pcl::PointCloud<PointType>> plant_memory_icp_base = IcpRegistrationForVec(vec_plant_memory_base);
    vector<pcl::PointCloud<PointType>> plant_memory_icp_base = vec_plant_memory_base;//replace the icp step for long time consuming

    //data fusion of point clouds base memory pos
    plant_memory = mergePcVec(plant_memory_icp_base);//replace the icp step for long time consuming
//    plant_memory = vec_plant_memory_base[0];
//    plant_memory = radiusFilteringPc(plant_memory);//delete the step for long time consuming

    //bbox estimation
    bboxCalculationForPc(plant_memory,
                        moment_of_inertia,//vector <float> moment_of_inertia;
                        eccentricity,//vector <float> eccentricity
                        bboxPoints,//min_point_AABB, max_point_AABB, min_point_OBB, max_point_OBB, position_OBB
                      rotational_matrix_OBB,// rotational_matrix_OBB
                      feature_values,//major_value, middle_value, minor_value
                      feature_vectors//  Eigen::Vector3f major_vector, middle_vector, minor_vector, mass_center;
                      );

    //6D pos estimation according to estimated bbox
    vector<Eigen::Isometry3d> T_base2tcp_AABB;
    vector<PointType> AABBPoints(bboxPoints.begin(), bboxPoints.begin() + 2);//first two elements of bboxPoints vec
    posCalculationFromAABB(AABBPoints, posFromAABB,T_base2tcp_AABB,orientation_AABB);

    //move to the target 6d UR pos , the second stage: bbox(AABB)
    //order the poses of all the AABB box to minimize the cost of time
    poseSorting_AABB(posFromAABB, T_base2tcp_AABB, ur->joint_pos,isReachable_AABB,orientation_AABB);

    //set flag 2 to acquire the pc data of the bbox pos
    flag = 2;

    //move to all bbox pos
    for(int i = 0;i < posFromAABB.size();i++)
        if(isReachable_AABB[i])
            {
                //set the flag to judge whether or not the pc acquiring is finished
                isPcAcquiringFinished = 0;
                ur->tcpMoveDirectly(posFromAABB[i][0],
                                    posFromAABB[i][1],
                                    posFromAABB[i][2],
                                    posFromAABB[i][3],
                                    posFromAABB[i][4],
                                    posFromAABB[i][5]);
                waitForPcAcquiringFinished();
            }

    //coordination tranformation  cam2base  bbox pos
    transformBboxPcCamToBase();

    //segmentation hsv or passthrough
    vec_plant_bbox_base = passThroughForVec(vec_pc_bbox_base);

//    //icp registration
//    vector<pcl::PointCloud<PointType>> plant_bbox_icp_base = IcpRegistrationForVec(vec_plant_bbox_base);
    vector<pcl::PointCloud<PointType>> plant_bbox_icp_base = vec_plant_bbox_base;

//    //data fusion
    plant_bbox = mergePcVec(plant_bbox_icp_base);
//    plant_bbox = radiusFilteringPc(plant_bbox);
//    plant_bbox = plant_memory;//just for not consuming time, using this step instead

    //clustering
    vector<PointIndices> clusters;//indicies of the clusters
//    vector<pcl::PointCloud<PointType>> pc_clusters;//plant parts after clustering
    pcClustering(plant_bbox,clusters,colored_cluster_cloud);
    extractPcClustersFromIndices(plant_bbox,clusters,pc_clusters);

    //bbox calculation for each part/cluster of the plant
    for(int i=0;i<pc_clusters.size();i++)
    {
        vector <float> moment_of_inertia_,
                        eccentricity_,
                        feature_values_;
        vector <PointType> bboxPoints_;
        Eigen::Matrix3f rotational_matrix_OBB_;
        vector<Eigen::Vector3f> feature_vectors_;
        bboxCalculationForPc(pc_clusters[i],
                            moment_of_inertia_,//vector <float> moment_of_inertia;
                            eccentricity_,//vector <float> eccentricity
                            bboxPoints_,//min_point_AABB, max_point_AABB, min_point_OBB, max_point_OBB, position_OBB
                            rotational_matrix_OBB_,// rotational_matrix_OBB
                            feature_values_,//major_value, middle_value, minor_value
                            feature_vectors_//  Eigen::Vector3f major_vector, middle_vector, minor_vector, mass_center;
                            );
        moment_of_inertia_vec.push_back(moment_of_inertia_);
        eccentricity_vec.push_back(eccentricity_);
        feature_values_vec.push_back(feature_values_);
        vector<PointType> OBBPoints(bboxPoints_.end() - 3, bboxPoints_.end());//last three elements of bboxPoints vec
        obbPoints_vec.push_back(OBBPoints);
        rotational_matrix_OBB_vec.push_back(rotational_matrix_OBB_);
        feature_vectors_vec.push_back(feature_vectors_);
    }

    //6D pos estimation according to estimated bbox
    vector<Eigen::Isometry3d> T_base2tcp_OBB;
    for(int i=0;i<obbPoints_vec.size();i++)
    {
        vector<double>  posFromOBB_;
        Eigen::Isometry3d T_base2tcp_OBB_;
        posCalculationFromOBB(obbPoints_vec[i], feature_vectors_vec[i], posFromOBB_,T_base2tcp_OBB_,orientation_OBB);
        posFromOBB.push_back(posFromOBB_);
        T_base2tcp_OBB.push_back(T_base2tcp_OBB_);
    }

    //Monte Carlo method: convex hull and Unobstructed solid angles
    convexHullCalculation(plant_bbox,//cloud input
                           plant_convex_hull,//cloud output
                           polygons//indices of polygons
                           );

    for(int i = 0; i<obbPoints_vec.size();i++)
    {
        Eigen::Vector4f plane;
        PointType sphere_center;
        vector<Eigen::Vector3f> sphere_axis;
        bottomPlaneForOBB(obbPoints_vec[i],//min_point_OBB, max_point_OBB, position_OBB
                           feature_vectors_vec[i],//the feature vectors, in order : major , middle ,minor
                           plane,//the 4 param of a plane
                           sphere_center,//the center of the sphere
                           sphere_axis//the axis of the sphere coordination, namely the x axis & z axis (position direction)
                           );//calculate some params for the bottom plane of the obb, e.g. the sphere & the plane & the center point
        pcl::PointCloud<PointType> sphere_pc;
        sphere_pc =  pcFromHemisphere(sphere_center,//the center of the sphere
                                        sphere_axis,//the axis of the sphere coordination, namely the x axis & z axis (position direction)
                                        0.05,//the radius of the hemisphere
                                        1//the interval degree of generating point cloud, degree, 1/2/3/5...
                                        );//generate the pc according to the params that can form a hemisphere, theta & phi

        vector<float> polygon_solid_angle;
        vector<pcl::PointCloud<PointType>> polygons_above;

        for(int j = 0; j<polygons.size();j++)
        {
            pcl::PointCloud<PointType> polygon;

            for (int k = 0; k < polygons[j].vertices.size(); k++)
            {
                polygon.points.push_back(plant_convex_hull.points[polygons[j].vertices[k]]);
            }

            pcl::IndicesPtr indices(new vector<int>());
            pcl::PointCloud<PointType> polygon_above;
            planeClipper(polygon,//cloud in
                            polygon_above,//cloud out
                            indices,//indices vec ptr
                            plane,//plane params
                            false//+ or -
                            );

            if(polygon_above.points.size()==polygon.points.size())
            {
             polygons_above.push_back(polygon);
             float solid_angle = solidAngleForTriangle(sphere_center,//the center of the sphere, O, view point
                                                         polygon.points[0],//the center of the sphere, A
                                                         polygon.points[1],//the center of the sphere, B
                                                         polygon.points[2]//the center of the sphere, C
                                                         );
             polygon_solid_angle.push_back(solid_angle);
            }
        }

        vector<pcl::PointCloud<PointType>> triangle_pc_sp_vec;
        //if the polygon above is void
        if(polygons_above.empty())
        {
            triangle_pc_sp_vec.push_back(sphere_pc);
        }
        else
        {
            vector<size_t> polygon_idx_after_sorting;
            polygon_idx_after_sorting = sort_indexes_e(polygon_solid_angle);

            vector<pcl::PointCloud<PointType>> triangle_pc_vec;
            for(int j = 0;j<polygon_idx_after_sorting.size();j++)
            {
                pcl::PointCloud<PointType> triangle_pc;
                triangle_pc = pcFromTriangle(polygons_above[polygon_idx_after_sorting[j]].points[0],//the center of the sphere, A
                                          polygons_above[polygon_idx_after_sorting[j]].points[1],//the center of the sphere, B
                                          polygons_above[polygon_idx_after_sorting[j]].points[2],//the center of the sphere, C
                                          0.01//the interval of the generating point cloud
                                          );//generate the pc according to the params that can form a triangle, p1,p2,p3
                triangle_pc_vec.push_back(triangle_pc);
                if(j==5)//only need the first 5, this value can be change
                {
                    break;
                }
            }

    //        vector<pcl::PointCloud<PointType>> triangle_pc_sp_vec;
            for(int j=0;j<triangle_pc_vec.size();j++)
            {
                pcl::PointCloud<PointType> triangle_pc_sp;
                sphereFormation(triangle_pc_vec[j], triangle_pc_sp, sphere_center.x, sphere_center.y, sphere_center.z, 0.05);
                triangle_pc_sp_vec.push_back(triangle_pc_sp);
            }
        }

        vector<pcl::PointCloud<PointType>> pc_clusters_sp_vec;
        for(int j=0;j<pc_clusters.size();j++)
        {
            if(j==i)
            {
                continue;//itself should not be kept and be projected
            }
            else
            {
                pcl::PointCloud<PointType> pc_clusters_sp;
                sphereFormation(pc_clusters[j], pc_clusters_sp, sphere_center.x, sphere_center.y, sphere_center.z, 0.05);
                pcl::IndicesPtr indices(new vector<int>());
                pcl::PointCloud<PointType> pc_clusters_sp_above;
                planeClipper(pc_clusters_sp,//cloud in
                                pc_clusters_sp_above,//cloud out
                                indices,//indices vec ptr
                                plane,//plane params
                                false//+ or -
                                );
                pc_clusters_sp_vec.push_back(pc_clusters_sp_above);
            }
        }

        vector<pcl::PointCloud<PointType>> triangle_pc_sp_remained_vec;
        //if the pc clusters above is void
        if(pc_clusters_sp_vec.empty())
        {
            for(int j = 0;j<triangle_pc_sp_vec.size();j++)
            {
                PointIndices::Ptr change1;
                pcl::PointCloud<PointType> sphere_pc_unchanged_with_triangle;
                changeOrNotDetection(triangle_pc_sp_vec[j], sphere_pc, change1, sphere_pc_unchanged_with_triangle, true);
                triangle_pc_sp_remained_vec.push_back(sphere_pc_unchanged_with_triangle);
            }
        }
        else
        {
            pcl::PointCloud<PointType> pc_clusters_sp_all = mergePcVec(pc_clusters_sp_vec);

    //        vector<pcl::PointCloud<PointType>> triangle_pc_sp_remained_vec;
            for(int j = 0;j<triangle_pc_sp_vec.size();j++)
            {
                PointIndices::Ptr change1;
                pcl::PointCloud<PointType> sphere_pc_unchanged_with_triangle;
                changeOrNotDetection(triangle_pc_sp_vec[j], sphere_pc, change1, sphere_pc_unchanged_with_triangle, true);
                PointIndices::Ptr change2;
                pcl::PointCloud<PointType> triangle_pc_sp_remained;
                changeOrNotDetection(pc_clusters_sp_all, sphere_pc_unchanged_with_triangle, change2, triangle_pc_sp_remained, false);
                triangle_pc_sp_remained_vec.push_back(triangle_pc_sp_remained);
            }
        }

        vector<int> size_of_vec;
        for(int j = 0;j<triangle_pc_sp_remained_vec.size();j++)
        {
            size_of_vec.push_back(triangle_pc_sp_remained_vec[j].points.size());
        }

        vector<size_t> idx_remained;
        idx_remained = sort_indexes_e(size_of_vec);

        pcl::PointCloud<PointType> ok_region_pc;
        ok_region_pc = triangle_pc_sp_remained_vec[idx_remained[0]];

        PointType ok_point;
        selectPointInPc(ok_region_pc,//cloud in
                         ok_point,//point out
                         0.05,//the radius of the hemisphere
                         1,//the interval of the generating point cloud, in degree
                         sphere_center,//the center of the sphere
                         sphere_axis[1]//z axis
                         );//select a point that has most point nearby, using radius search

        vector<double> pose_for_UR;
        Eigen::Isometry3d T_base2tcp_tar;
        vector<Eigen::Vector3d> orientation;
        posCalculationFromMonteCarlo(ok_point,//the view point, on the hemisphere, not considering the offset
                                sphere_center,//the center of the sphere
                                0.05,//radius of the sphere
                                sphere_axis,//x  z  axis
                                pose_for_UR,// hemisphere just need one pos to move UR
                                T_base2tcp_tar,//T matrix
                                orientation//restore the orientation vec for vtk visualization
                                );

        posFromMC.push_back(pose_for_UR);
        orientation_MC.push_back(orientation[0]);
        orientation_MC.push_back(orientation[1]);
        orientation_MC.push_back(orientation[2]);
        T_base2tcp_MC.push_back(T_base2tcp_tar);
    }

    //move to the target 6d UR pos , the third stage: bbox(OBB)
    //set flag 3 to acquire the pc data of the OBB bbox pos
    flag = 3;

    //order the poses of all the OBB box to minimize the cost of time
//    poseSorting_OBB(posFromOBB, T_base2tcp_OBB, ur->joint_pos, isReachable_OBB, orientation_OBB,pc_clusters);

    //order the poses of all the OBB box to minimize the cost of time
//    poseSorting_OBB(posFromMC, T_base2tcp_MC, ur->joint_pos, isReachable_MC, orientation_MC,pc_clusters);

//    //---------ABOVE HAS BEEN TESTED, NOTHING WRONG------------

    //move to all OBB bbox pos
    for(int i = 0;i < posFromOBB.size();i++)
        if(isReachable_OBB[i])
            {
                //set the flag to judge whether or not the pc acquiring is finished
                isPcAcquiringFinished = 0;
                ur->tcpMoveDirectly(posFromOBB[i][0],
                                    posFromOBB[i][1],
                                    posFromOBB[i][2],
                                    posFromOBB[i][3],
                                    posFromOBB[i][4],
                                    posFromOBB[i][5]);
                waitForPcAcquiringFinished();
            }
//    //coordination tranformation  cam2base  OBB bbox pos
//    transformOKPcCamToBase();
//    //segmentation hsv or passthrough
//    vec_plant_ok_base = passThroughForVec(vec_pc_ok_base);
//    //icp registration with the whole plant in the step of bbox
//    //insert the whole plant point cloud into the first place of the ok pc vec
//    vec_plant_ok_base.insert(vec_plant_ok_base.begin(),plant_bbox);
//    vector<pcl::PointCloud<PointType>> plant_ok_icp_base = IcpRegistrationForVec(vec_plant_ok_base);
//    //data fusion of the whole plant, not just fusing the single leaves together
//    plant_ok = mergePcVec(plant_ok_icp_base);
//    plant_ok = radiusFilteringPc(plant_ok);

    /*
    //-------------if above is ok. then start dealing with the occlusion problem--------------------
    //this code is for research only, the logic is not true for the real implement
    //----------------------------------------------------------------------------------------------
    //variables:
    //              plant_ok
    //              pc_clusters
    //              isReachable_OBB
    //              orientation_OBB
    //              T_base2tcp_OBB
    //              vec_pc_ok_base  (without preprocessing, just single frame, vec)(could be iterated)
    //----------------------------------------------------------------------------------------------
//    vector<Eigen::Vector3d> axis_ok;
//    vector<Eigen::Isometry3d> T_base2tcp_ok;
//    vector<pcl::PointCloud<PointType>> pc_clusters_ok;
//    vector<vector<double>> posFromOBB_ok;

    //prepare for the variables that is ok for all pose, all can be reached
    for(int i = 0; i<isReachable_OBB ; i++)
        if(isReachable_OBB[i])
            {
                axis_ok.push_back(orientation_OBB[3*i+0]);
                axis_ok.push_back(orientation_OBB[3*i+1]);
                axis_ok.push_back(orientation_OBB[3*i+2]);
                T_base2tcp_ok.push_back(T_base2tcp_OBB[i]);
                pc_clusters_ok.push_back(pc_clusters[i]);
                posFromOBB_ok.push_back(posFromOBB[i]);
                obbPoints_vec_ok.push_back(obbPoints_vec[i]);//just for visualization
                rotational_matrix_OBB_vec_ok.push_back(rotational_matrix_OBB_vec[i]);//just for visualization
            }

//    //or use io::load to load file from the existed files to replace the above step, just for test
//    pcl::io::loadPCDFile<PointType>("file_fold//*.pcd",plant_ok);//this also may not be put here, may be before the cluster step
//    pcl::io::loadPCDFile<PointType>("file_fold//*.pcd",colored_cluster_cloud);//this also may not be put here, may be before the cluster step
//    int clusters_num = 1;
//    for(int i = 0 ;i<clusters_num;i++)//temp num == 1
//    {
//        pcl::PointCloud<PointType> pc_cluster;
//        pcl::io::loadPCDFile<PointType>("file_fold//*.pcd",pc_cluster);
//        pc_clusters_ok.push_back(pc_cluster);

//        pcl::PointCloud<PointType> pc_frame;
//        pcl::io::loadPCDFile<PointType>("file_fold//*.pcd",pc_frame);
//        vec_pc_ok_base.push_back(pc_frame);

//        Eigen::Isometry3d T;
//        T.matrix()<<  1,  0,   0,   0,
//                      0,  1,   0,   0,
//                      0,  0,   1,   0,
//                      0,  0,   0,   1;//just fill the blank
//        T_base2tcp_ok.push_back(T);

//        vector<double> pose;
//        pose.push_back(0);
//        pose.push_back(0);
//        pose.push_back(0);
//        pose.push_back(1);
//        pose.push_back(0);
//        pose.push_back(0);//just fill the blank
//        posFromOBB_ok.push_back(pose);

//        Eigen::Vector3d axis_x;
//        axis_x(0) = 1;  axis_x(1) = 0;  axis_x(2) = 0;
//        Eigen::Vector3d axis_y;
//        axis_y(0) = 0;  axis_y(1) = 1;  axis_y(2) = 0;
//        Eigen::Vector3d axis_z;
//        axis_z(0) = 0;  axis_z(1) = 0;  axis_z(2) = 0;//just fill the blank
//        axis_ok.push_back(axis_x);
//        axis_ok.push_back(axis_y);
//        axis_ok.push_back(axis_z);
//    }

    //define a vec to figure out whether or not the pose need to be changed for occlusion
//    vector<int> isOccluded;
    //define all the variables for iteration, the original variables will be not kept
//    vector<vector<double>> ur_pose_iter = posFromOBB_ok;//after changed
//    vector<Eigen::Isometry3d> T_base2tcp_iter = T_base2tcp_ok;//after changed
//    vector<Eigen::Vector3d> axis_iter = axis_ok;//after changed
//    vector<pcl::PointCloud<PointType>> pc_clusters_iter = pc_clusters_ok;//after changed, fullfill the occluded cluster, the others kept
    ur_pose_iter = posFromOBB_ok;//after changed
    T_base2tcp_iter = T_base2tcp_ok;//after changed
    axis_iter = axis_ok;//after changed
    pc_clusters_iter = pc_clusters_ok;//after changed, fullfill the occluded cluster, the others kept

    //for each single frame, do the following step
    for(int i = 0;i<vec_pc_ok_base;i++)
    {
        int pose_change_times = 0;//iteration times
        while(1)//the major iteration of a single cluster, controlled values(pose change times , occlusion or not, the gainning is enough or not)
        {
            //cluster them using the same params
            vector<PointIndices> clusters_single_frame;//indicies of the clusters
            pcl::PointCloud<PointType> colored_cluster_cloud_single_frame;//colored cluster pc
//            vector<pcl::PointCloud<PointType>> pc_clusters_single_frame;//pc data
            //----some mid items-----
            pc_clusters_single_frame.clear();//pc data
            //-------------------------
            pcClustering(vec_pc_ok_base[i],clusters_single_frame,colored_cluster_cloud_single_frame);
            extractPcClustersFromIndices(vec_pc_ok_base[i],clusters_single_frame,pc_clusters_single_frame);

            //rmse calculation with the original clustered pc, choose the min rmse and the index of the pc
            double min_rmse = 100000000000.0;
//            int j_best;
            for(int j=0;j<pc_clusters_single_frame.size();j++)
            {
                double rmse = computeCloudRMSE(pc_clusters_iter[i].makeShared(),pc_clusters_single_frame[j].makeShared(),0.01);
                if (rmse < min_rmse)
                {
                    min_rmse = rmse;
                    j_best = j;
                }
            }

            if(pose_change_times)//time ==0 ,no judgement about the ratio of space gaining
            {
                //calculate the space gaining ratio
                if(onceMoreIsNeededOrNotByOctree(pc_clusters_iter[i], pc_clusters_iter[i] + pc_clusters_single_frame[j_best], 0.1))//judge the gainning space, if large than the ratio, then continue the loop
                {
                    //deal with pc_clusters_iter, merge pc_clusters_single_frame[j_best] & pc_clusters_iter[i] etc.
                    pc_clusters_iter[i] = pc_clusters_iter[i] + pc_clusters_single_frame[j_best];
                    pc_clusters_iter[i] = voxelFilteringPc(pc_clusters_iter[i]);
                }
                else//if less than the ratio, then stop the iteration
                {
                    //deal with pc_clusters_iter, merge pc_clusters_single_frame[j_best] & pc_clusters_iter[i] etc.
                    pc_clusters_iter[i] = pc_clusters_iter[i] + pc_clusters_single_frame[j_best];
                    pc_clusters_iter[i] = voxelFilteringPc(pc_clusters_iter[i]);
                    break;
                }

                if(pose_change_times == 2)//iteration times control, max time == 2
                {
                    break;
                }
            }

            //bounding line pc extration
//            vector<pcl::PointCloud<PointType>> pc_b;//bounding line pc data
            //----some mid items-----
            pc_b.clear();
            //-------------------------
            for(int j=0;j<pc_clusters_single_frame.size();j++)
            {
                pcl::PointCloud<PointType> pc_b_;
                boundaryExtraction(pc_clusters_single_frame[j],pc_b_);
                pc_b.push_back(pc_b_);
            }

            //sphere projection
            Eigen::Isometry3d T_base2cam_ok_;
            T_base2cam_ok_.matrix() = T_base2tcp_iter[i].matrix() * T_tcp2cam.matrix();//get the viewpoint (cam coordination)
            Eigen::Vector4f centroid_pc_ok_;
            pcl::compute3DCentroid(vec_pc_ok_base[i],centroid_pc_ok_);//get the distance for projection, just the average distance of the current frame pc data
            PointType vp,c_pc_ok;
            vp.x = float(T_base2cam_ok_.translation()(0));
            vp.y = float(T_base2cam_ok_.translation()(1));
            vp.z = float(T_base2cam_ok_.translation()(2));
            c_pc_ok.x = centroid_pc_ok_[0];
            c_pc_ok.y = centroid_pc_ok_[1];
            c_pc_ok.z = centroid_pc_ok_[2];
//            vector<pcl::PointCloud<PointType>> pc_sp;//sphere projected pc data
            //----some mid items-----
            pc_sp.clear();//sphere projected pc data
            //-------------------------
            for(int j=0;j<pc_b.size();j++)
            {
                pcl::PointCloud<PointType> pc_sp_;
                sphereFormation(pc_b[j],pc_sp_,vp.x,vp.y,vp.z,pointDistance(vp,c_pc_ok));
                pc_sp.push_back(pc_sp_);
            }

            //RKNN to find the adjacent bounding line pc for each projected cluster
//            vector<int> idx_spa;//sphere projected adjacent pc index
            //----some mid items-----
            idx_spa.clear();//sphere projected adjacent pc index
            //-------------------------
            RKNNForPointCloudVec(pc_sp[j_best],pc_sp ,pc_b[j_best], pc_b, idx_spa, vp,1, 0.01);

            //judge this occlusion by the length of the adjacent bounding line, ratio can be set as 0.1, whether or not the pose needs to be changed, is it really an occlusion?
            if(isOcclusionOrNotByRatio(pc_b[j_best], idx_spa, 0.1))//if yes
            {
                if(!pose_change_times)//only record for the first time, namely == 0
                {
                    isOccluded.push_back(1);
                }

                //calculate the occlusion orientation for the next move
                PointType mass_point_bounding_line;
                Eigen::Vector3d occlusion_orientation;
                occlusionPoseCalculation(pc_b[j_best], idx_spa, mass_point_bounding_line, occlusion_orientation);
                vector<Eigen::Vector3d> axis_ori(axis_iter.begin() + 3*i, axis_iter.begin() + 3*i + 3);//first three elements of axis vec
                vector<double> ur_pose_tar;//after changed
                Eigen::Isometry3d T_base2tcp_tar;//after changed
                vector<Eigen::Vector3d> axis_tar;//after changed
                poseTransformFromOcclusion(T_base2tcp_iter[i], axis_ori, axis_ok[3*i+2],mass_point_bounding_line, occlusion_orientation, pose_change_times,ur_pose_tar, T_base2tcp_tar, axis_tar);
                pose_change_times++;//pose has been changed, add 1 time

                //move UR for capturing, optional step!
                flag = 4;
                isPcAcquiringFinished = 0;
                ur->tcpMoveDirectly(ur_pose_tar[0],
                                    ur_pose_tar[1],
                                    ur_pose_tar[2],
                                    ur_pose_tar[3],
                                    ur_pose_tar[4],
                                    ur_pose_tar[5]);
                waitForPcAcquiringFinished();
                Eigen::Isometry3d T_b2c;
                T_b2c.matrix() = T_base2tcp_tar[i].matrix() * T_tcp2cam.matrix();
                pcl::transformPointCloud (pc_iter_cam, pc_iter_base, T_b2c.matrix());

                //update the variables
                vec_pc_ok_base[i] = pc_iter_base;
                ur_pose_iter[i] = ur_pose_tar;
                T_base2tcp_iter[i] = T_base2tcp_tar;
                axis_iter[3*i+0] = axis_tar[0];
                axis_iter[3*i+1] = axis_tar[1];
                axis_iter[3*i+2] = axis_tar[2];
            }
            else//it not
            {
                if(!pose_change_times)//only record for the first time, namely ==0
                {
                    isOccluded.push_back(0);
                }
                break;
            }
        }
    }
    */
    //----------------------------------------------------------------------------------------------
//    //end , further processing is not needed in the final step, if the obtained ok data is fine
//    //visualization in another thread
//    //data saving if necessary, please be sure which data is due for saving, change the code before saving
////    pcDataSaving();
}

void pcProc::posCalculationFromMonteCarlo(PointType view_point_on_hemisphere, PointType sphere_center, float radius, vector<Eigen::Vector3f> axis, vector<double> & pos_for_UR, Eigen::Isometry3d & T_base2tcp_tar, vector<Eigen::Vector3d> & orientation)
{
    Eigen::Vector3d view_z(double(sphere_center.x - view_point_on_hemisphere.x),double(sphere_center.y - view_point_on_hemisphere.y),double(sphere_center.z - view_point_on_hemisphere.z));
    double angle = M_PI - acos(vec3fDot(view_z,axis[1])/(norm2f(view_z)*norm2f(axis[1])));
    Eigen::Vector3d view_x;
    if(angle < 0.001)//angle == 0
    {
         view_x = axis[0];
    }
    else
    {
        double d =  radius/cos(angle);
        PointType D;
        D.x = sphere_center.x + d * axis[1](0);
        D.y = sphere_center.y + d * axis[1](1);
        D.z = sphere_center.z + d * axis[1](2);
        view_x(0) = double(view_point_on_hemisphere.x - D.x);
        view_x(1) = double(view_point_on_hemisphere.y - D.y);
        view_x(2) = double(view_point_on_hemisphere.z - D.z);
        vec3dNorm(view_x);
    }

    Eigen::Vector3d view_y = vec3dCross(view_z,view_x);

    PointType transformedPosition;//UR position(x y z)
    transformedPosition.x = sphere_center.x - (OFFSET_FOR_CAPTURING)*view_z(0);
    transformedPosition.y = sphere_center.y - (OFFSET_FOR_CAPTURING)*view_z(1);
    transformedPosition.z = sphere_center.z - (OFFSET_FOR_CAPTURING)*view_z(2);
    Eigen::Isometry3d T_base2cam;
    orientation.push_back(view_x);//x
    orientation.push_back(view_y);//y
    orientation.push_back(view_z);//z
    T_base2cam.matrix()<<    view_x(0), view_y(0), view_z(0), double(transformedPosition.x),
                             view_x(1), view_y(1), view_z(1), double(transformedPosition.y),
                             view_x(2), view_y(2), view_z(2), double(transformedPosition.z),
                                     0,         0,         0,                             1;
////    Eigen::Isometry3d T_base2tcp_tar;
    T_base2tcp_tar.matrix() = T_base2cam.matrix() * T_tcp2cam.inverse().matrix();

    Eigen::AngleAxisd rv(T_base2tcp_tar.rotation());
    pos_for_UR.push_back(double(T_base2tcp_tar.translation()(0)));//x
    pos_for_UR.push_back(double(T_base2tcp_tar.translation()(1)));//y
    pos_for_UR.push_back(double(T_base2tcp_tar.translation()(2)));//z
    pos_for_UR.push_back(double(rv.angle()*rv.axis()(0)));//rx
    pos_for_UR.push_back(double(rv.angle()*rv.axis()(1)));//ry
    pos_for_UR.push_back(double(rv.angle()*rv.axis()(2)));//rz
}

PointType pcProc::selectPointWihtMinAngle(pcl::PointCloud<PointType>::Ptr cloud, PointType sphere_center, Eigen::Vector3f z_axis)
{
    vector<double> angle;
    for(int i = 0;i<cloud->points.size();i++)
    {
        Eigen::Vector3f SP(cloud->points[i].x - sphere_center.x,cloud->points[i].y - sphere_center.y,cloud->points[i].z - sphere_center.z);
        double alpha = acos(double(vec3fDot(SP,z_axis)/(norm2f(SP)*norm2f(z_axis))));
        angle.push_back(alpha);
    }
    int min_value = *min_element(angle.begin(), angle.end());
    std::vector<int> indices = findItems(angle, min_value);
    return cloud->points[indices[0]];
}

void pcProc::selectPointInPc(pcl::PointCloud<PointType> pc, PointType & point, float r, int interval, PointType sphere_center, Eigen::Vector3f z_axis)
{
    pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);
    pcl::PointCloud<PointType>::Ptr cloud_iter(new pcl::PointCloud<PointType>);
    cloud = pc.makeShared();
    cloud_iter = pc.makeShared();

    pcl::KdTreeFLANN<PointType> kdtree;
    kdtree.setInputCloud(cloud);

    int j = 1;
    while(1)
    {
        j++;
        float radius = j * r * interval * M_PI / 180.0;
        vector<int> nearbyPointNum;

        for(int i = 0;i<cloud_iter->points.size();i++)
        {
            PointType searchPoint = cloud_iter->points[i];
            vector<int> pointIdxRadiusSearch;
            vector<float> pointRadiusSquaredDistance;

            if (kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
            {
                nearbyPointNum.push_back(pointIdxRadiusSearch.size());
            }
            else
            {
                nearbyPointNum.push_back(0);
            }
        }

        int max_value = *max_element(nearbyPointNum.begin(), nearbyPointNum.end());
        std::vector<int> indices = findItems(nearbyPointNum, max_value);

        PointIndices::Ptr change;
        for (const auto &it : indices)
        {
            change->indices.push_back(it);
        }
        pcl::ExtractIndices<PointType> extract;
        extract.setInputCloud(cloud_iter);
        extract.setIndices(change);
        extract.setNegative(false);
        extract.filter(*cloud_iter);

        if(max_value > cloud->points.size()-2 || indices.size() == 1)
        {
            break;
        }
    }

//    point = cloud_iter->points[0];
    point = selectPointWihtMinAngle(cloud_iter, sphere_center, z_axis);//select the only point that is nearest to the z_axis
}

void pcProc::changeOrNotDetection(pcl::PointCloud<PointType> pc_small, pcl::PointCloud<PointType> pc_large, PointIndices::Ptr change, pcl::PointCloud<PointType> & output, bool UnchangedNeeded)
{
    pcl::PointCloud<PointType>::Ptr cloudA(new pcl::PointCloud<PointType>);
    pcl::PointCloud<PointType>::Ptr cloudB(new pcl::PointCloud<PointType>);
    pc_small = basicVoxelFilteringPc(pc_small);
    pc_large = basicVoxelFilteringPc(pc_large);
    cloudA = pc_small.makeShared();
    cloudB = pc_large.makeShared();
    double resolution = 0.005f;

    pcl::octree::OctreePointCloudChangeDetector<PointType>octree(resolution);
    octree.setInputCloud(cloudA);
    octree.addPointsFromInputCloud();
    octree.switchBuffers();
    octree.setInputCloud(cloudB);
    octree.addPointsFromInputCloud();
    vector<int> newPointIdxVector;
    octree.getPointIndicesFromNewVoxels(newPointIdxVector);
    pcl::PointCloud<PointType>::Ptr cloud_change(new pcl::PointCloud<PointType>);
    pcl::copyPointCloud(*cloudB, newPointIdxVector, *cloud_change);

    for (const auto &it : newPointIdxVector)
    {
        change->indices.push_back(it);
    }
    pcl::ExtractIndices<PointType> extract;
    extract.setInputCloud(cloudB);//change or unchanged is according to the cloud B !!!!!!!
    extract.setIndices(change);

    if(UnchangedNeeded)
    {
        extract.setNegative(true);
        extract.filter(output);
    }
    else
    {
        extract.setNegative(false);
        extract.filter(output);
    }
}

void pcProc::planeClipper(pcl::PointCloud<PointType> pc, pcl::PointCloud<PointType> & filter,IndicesPtr indices, Eigen::Vector4f plane, bool flag)
{
    pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);
    cloud = pc.makeShared();

    pcl::PlaneClipper3D<PointType> clipper(plane);

    clipper.setPlaneParameters(plane);
    clipper.clipPointCloud3D(*cloud, *indices);

    pcl::ExtractIndices<PointType> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(indices);
    extract.setNegative(flag);//+
    extract.filter(filter);
}

pcl::PointCloud<PointType> pcProc::pcFromTriangle(PointType A, PointType B, PointType C, float interval)
{
    pcl::PointCloud<PointType> cloud;

    for (int i = 0;i < int(1.0/interval)+1;i++)
        for(int j = 0;j < int((1.0 - i * interval)/interval)+1;j++)
        {
            PointType P;
            float p1,p2,p3;
            p1 = i * interval;
            p2 = j * interval;
            p3 = 1 - p1 -p2;
            P.x = p1*A.x+p2*B.x+p3*C.x;
            P.y = p1*A.y+p2*B.y+p3*C.y;
            P.z = p1*A.z+p2*B.z+p3*C.z;

            cloud.push_back(P);
        }

    return cloud;
}

pcl::PointCloud<PointType> pcProc::pcFromHemisphere(PointType sphere_center, vector<Eigen::Vector3f> axis, float radius, int interval_degree)
{
    pcl::PointCloud<PointType> cloud;
    cloud.width = 360/interval_degree;//[,)
    cloud.height = 90/interval_degree + 1;//[,]
    cloud.is_dense = false;
    cloud.points.resize(cloud.width*cloud.height);

    for (size_t i = 0; i < cloud.width; ++i)
        for (size_t j = 0; j < cloud.height; ++j)
            {
                float theta,phi;
                theta = i * M_PI * interval_degree / 180;
                phi = j * M_PI * interval_degree / 180;
                PointType v_pc;

//                Eigen::AngleAxisf r1(theta, axis[1]);
//                Eigen::AngleAxisf r2((theta - M_PI / 2), axis[1]);
//                Eigen::AngleAxisf r3((M_PI / 2 - phi), r2 * axis[0]);
//                Eigen::Vector3f v = r3 * (r1 * axis[0]);

                Eigen::AngleAxisf r1((theta - M_PI / 2), axis[1]);
                Eigen::AngleAxisf r2((- phi), r1 * axis[0]);
                Eigen::Vector3f v = r2 * axis[1];

                v_pc.x = radius * v(0) + sphere_center.x;
                v_pc.y = radius * v(1) + sphere_center.y;
                v_pc.z = radius * v(2) + sphere_center.z;

                cloud.points[i*cloud.height+j] = v_pc;
            }

    return cloud;
}

void pcProc::bottomPlaneForOBB(vector<PointType> bboxPoints, vector<Eigen::Vector3f>feature_vectors, Eigen::Vector4f & plane, PointType & sphere_center, vector<Eigen::Vector3f> & axis)
{
    PointType min_point_OBB;
    PointType max_point_OBB;
    PointType position_OBB;
    min_point_OBB = bboxPoints[0];
    max_point_OBB = bboxPoints[1];
    position_OBB = bboxPoints[2];

    for(int i = 0;i<feature_vectors.size();i++)
    {
        if(i!=1)
        {
        Eigen::Vector3f fv_d((feature_vectors[i](0)),(feature_vectors[i](1)),(feature_vectors[i](2)));
        vec3fNorm(fv_d);（
        axis.push_back(fv_d);     //only x & z
        }
    }

    if(axis[1](2)<0)//redirect the vector, make sure the direction is positive +
    {
        for(int j=0;j<3;j++)
        {
            axis[1](j) = -axis[1](j);
        }
    }

    sphere_center.x = position_OBB.x - 0.5*(max_point_OBB.z - min_point_OBB.z)*axis[1](0);
    sphere_center.y = position_OBB.y - 0.5*(max_point_OBB.z - min_point_OBB.z)*axis[1](1);
    sphere_center.z = position_OBB.z - 0.5*(max_point_OBB.z - min_point_OBB.z)*axis[1](2);

    plane << axis[1](0),axis[1](1),axis[1](2),-(axis[1](0)*sphere_center.x+axis[1](1)*sphere_center.y+axis[1](2)*sphere_center.z);
}

void pcProc::convexHullCalculation(pcl::PointCloud<PointType> pc, pcl::PointCloud<PointType> & surface_hull, vector<Vertices> & polygons)
{
    pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);
    cloud = pc.makeShared();

    pcl::ConvexHull<PointType> hull;
    hull.setInputCloud(cloud);
    hull.setDimension(3);

    hull.setComputeAreaVolume(true);
    hull.reconstruct(surface_hull, polygons);
}

void pcProc::poseTransformFromOcclusion(Eigen::Isometry3d T_base2tcp_ori, vector<Eigen::Vector3d> axis_ori,Eigen::Vector3d leaf_normal_ok_, PointType mass_center, Eigen::Vector3d occlusion_orientation, int pose_change_times, vector<double> & ur_pose_tar, Eigen::Isometry3d & T_base2tcp_tar, vector<Eigen::Vector3d> & axis_tar)
{
//    Eigen::Vector3d rotation_vec = vec3dCross(occlusion_orientation,axis_ori[2]);//occ * z = r_v
    Eigen::Vector3d rotation_vec = vec3dCross(occlusion_orientation,leaf_normal_ok_);//occ * z = r_v
    vec3dNorm(rotation_vec);
    Eigen::AngleAxisd r_v((M_PI / 4)/float(pose_change_times+1), rotation_vec);//for each time the degree may cut for a half from 45 du

    for(int i = 0 ; i < axis_ori.size();i++)
    {
        axis_tar.push_back(r_v * axis_ori[i]);
    }

    Eigen::Isometry3d T_base2cam_ori;
    T_base2cam_ori.matrix() = T_base2tcp_ori.matrix() * T_tcp2cam.matrix();
    Eigen::Vector3d relative_vector_m_po(T_base2cam_ori.translation()(0) - double(mass_center.x),T_base2cam_ori.translation()(1) - double(mass_center.y),T_base2cam_ori.translation()(2) - double(mass_center.z));
    Eigen::Vector3d relative_vector_m_pt = r_v * relative_vector_m_po;
    double d = norm2d(relative_vector_m_pt);
    //for each time the distance offset may cut for a half from 5 cm
    Eigen::Vector3d relative_vector_m_pt_new((d+0.05/float(pose_change_times+1))/d*relative_vector_m_pt[0],(d+0.05/float(pose_change_times+1))/d*relative_vector_m_pt[1],(d+0.05/float(pose_change_times+1))/d*relative_vector_m_pt[2]);

    Eigen::Isometry3d T_base2cam_tar;
    T_base2cam_tar.matrix()<<axis_tar[0](0),axis_tar[1](0),axis_tar[2](0),relative_vector_m_pt_new[0]+mass_center.x,
                             axis_tar[0](1),axis_tar[1](1),axis_tar[2](1),relative_vector_m_pt_new[1]+mass_center.y,
                             axis_tar[0](2),axis_tar[1](2),axis_tar[2](2),relative_vector_m_pt_new[2]+mass_center.z,
                             0,0,0,1;

    T_base2tcp_tar.matrix() = T_base2cam_tar.matrix() * T_tcp2cam.inverse().matrix();

    Eigen::AngleAxisd rv(T_base2tcp_tar.rotation());
    ur_pose_tar.push_back(double(T_base2tcp_tar.translation()(0)));//x
    ur_pose_tar.push_back(double(T_base2tcp_tar.translation()(1)));//y
    ur_pose_tar.push_back(double(T_base2tcp_tar.translation()(2)));//z
    ur_pose_tar.push_back(double(rv.angle()*rv.axis()(0)));//rx
    ur_pose_tar.push_back(double(rv.angle()*rv.axis()(1)));//ry
    ur_pose_tar.push_back(double(rv.angle()*rv.axis()(2)));//rz
}

void pcProc::occlusionPoseCalculation(pcl::PointCloud<PointType> pc, vector<int> idx, PointType & mass_point_bounding_line, Eigen::Vector3d & occlusion_orientation)
{
    Eigen::Vector4f centroid,centroid_idx;
    pcl::compute3DCentroid(pc,centroid);

    occlusion_orientation[0] = 0.0;
    occlusion_orientation[1] = 0.0;
    occlusion_orientation[2] = 0.0;

    pcl::PointCloud<PointType> pc_idx;
    for(int i = 0;i<idx.size();i++)
    {
        pc_idx.push_back(pc.points[idx[i]]);
        occlusion_orientation[0] += centroid[0] - pc.points[idx[i]].x;
        occlusion_orientation[1] += centroid[1] - pc.points[idx[i]].y;
        occlusion_orientation[2] += centroid[2] - pc.points[idx[i]].z;
    }
    
    vec3dNorm(occlusion_orientation);
    pcl::compute3DCentroid(pc_idx,centroid_idx);

    mass_point_bounding_line.x = centroid_idx[0];
    mass_point_bounding_line.y = centroid_idx[1];
    mass_point_bounding_line.z = centroid_idx[2];
}

int pcProc::onceMoreIsNeededOrNotByOctree(pcl::PointCloud<PointType> pc_small, pcl::PointCloud<PointType> pc_large, float ratio)
{
    pcl::PointCloud<PointType>::Ptr cloudA(new pcl::PointCloud<PointType>);
    pcl::PointCloud<PointType>::Ptr cloudB(new pcl::PointCloud<PointType>);
    pc_small = basicVoxelFilteringPc(pc_small);
    pc_large = basicVoxelFilteringPc(pc_large);
    cloudA = pc_small.makeShared();
    cloudB = pc_large.makeShared();
    double resolution = 0.005f;

    pcl::octree::OctreePointCloudChangeDetector<PointType>octree(resolution);
    octree.setInputCloud(cloudA);
    octree.addPointsFromInputCloud();
    octree.switchBuffers();
    octree.setInputCloud(cloudB);
    octree.addPointsFromInputCloud();
    vector<int>newPointIdxVector;
    octree.getPointIndicesFromNewVoxels(newPointIdxVector);
    pcl::PointCloud<PointType>::Ptr cloud_change(new pcl::PointCloud<PointType>);
    pcl::copyPointCloud(*cloudB, newPointIdxVector, *cloud_change);
    if(float(newPointIdxVector.size())>(float(pc_small.points.size())*ratio))
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

int pcProc::isOcclusionOrNotByRatio(pcl::PointCloud<PointType> pc, vector<int> idx, float ratio)
{
    pcl::PointCloud<PointType> pc_idx;
    for(int i = 0;i<idx.size();i++)
    {
        pc_idx.push_back(pc.points[idx[i]]);
    }

    pcl::PointCloud<PointType> pc_voxel = basicVoxelFilteringPc(pc);
    pcl::PointCloud<PointType> pc_idx_voxel = basicVoxelFilteringPc(pc_idx);

    if(float(pc_idx_voxel.points.size())>(float(pc_voxel.points.size()) * ratio))//having not been tested this method and the ratio
    {
        return 1;//occlusion exists
    }
    else
    {
        return 0;//no occlusion
    }
}

void pcProc::RKNNForPointCloudVec(pcl::PointCloud<PointType> pc, vector<pcl::PointCloud<PointType> > pc_search_vec,pcl::PointCloud<PointType> pc_bp, vector<pcl::PointCloud<PointType> > pc_search_bp_vec, vector<int> & pc_vec_idx, PointType viewpoint,int max_nn, float radius)
{
    for (int i = 0;i<pc_search_vec.size();i++)
    {
        vector<int> pc_idx;
        RKNNForPointCloud(pc,pc_search_vec[i],pc_bp,pc_search_bp_vec[i],pc_idx,viewpoint,max_nn,radius);
        if(!pc_idx.empty())
        {
            for(int j = 0;j<pc_idx.size();j++)
            {
                pc_vec_idx.push_back(pc_idx[j]);//when max_nn == 1 , the repeat will not occur
            }
        }
    }

    sort(pc_vec_idx.begin(), pc_vec_idx.end());
    pc_vec_idx.erase(unique(pc_vec_idx.begin(), pc_vec_idx.end()), pc_vec_idx.end());//delete the repeated elements
}

void pcProc::RKNNForPointCloud(pcl::PointCloud<PointType> pc, pcl::PointCloud<PointType> pc_search, pcl::PointCloud<PointType> pc_bp, pcl::PointCloud<PointType> pc_search_bp,vector<int> & pc_idx, PointType viewpoint,int max_nn, float radius)
{
//    PointType viewpoint;
//    viewpoint.x = 0.0;
//    viewpoint.y = 0.0;
//    viewpoint.z = 0.0;

    for (size_t i = 0; i < pc_search.points.size (); ++i)
    {
      vector<int> pointIdxRN;
      RKNNForPoint(pc,pc_search.points[i],pointIdxRN,max_nn,radius);
      if(!pointIdxRN.empty())
      {
          for(int j = 0;j<pointIdxRN.size();j++)
          {
              if(occlusionJudgeForPoint(viewpoint,pc_search_bp.points[i],pc_bp.points[pointIdxRN[j]]))
              {
                 pc_idx.push_back(pointIdxRN[j]);//when max_nn == 1 , the repeat will not occur
              }
          }
      }
    }

    sort(pc_idx.begin(), pc_idx.end());
    pc_idx.erase(unique(pc_idx.begin(), pc_idx.end()), pc_idx.end());//delete the repeated elements
}

void pcProc::RKNNForPoint(pcl::PointCloud<PointType> pc,PointType searchPoint,vector<int>& pointIdxRN,int max_nn,float radius)
{
    pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);
    cloud = pc.makeShared();

    pcl::KdTreeFLANN<PointType> kdtree;
    kdtree.setInputCloud(cloud);
//    float radius= 0.03;
//    int max_nn= 150;
    vector<float> pointRNSqDistance;
    if (kdtree.radiusSearch(searchPoint, radius, pointIdxRN, pointRNSqDistance, max_nn) > 0)
    {
        //do nothing
    }
}

void pcProc::sphereFormation(pcl::PointCloud<PointType> pc, pcl::PointCloud<PointType> & pc_p, float x, float y, float z, float r)
{
    pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);
    cloud = pc.makeShared();

    sphere SP;
    SP.centerX = x;
    SP.centerY = y;
    SP.centerZ = z;
    SP.radius = r;

    pcl::PointCloud<PointType>::Ptr cloud_projected(new pcl::PointCloud<PointType>);

    for (size_t i = 0; i < cloud->points.size(); ++i)
    {
        PointType points;
//        float d = cloud->points[i].getVector3fMap().norm();//maybe sth wrong?   .norm cannot be used
        float d = norm2f(cloud->points[i].getVector3fMap());
        points.x = (cloud->points[i].x) * SP.radius / d + SP.centerX;
        points.y = (cloud->points[i].y) * SP.radius / d + SP.centerY;
        points.z = (cloud->points[i].z) * SP.radius / d + SP.centerZ;
        points.rgba = cloud->points[i].rgba;
        cloud_projected->push_back(points);
    }

    pc_p = *cloud_projected;
}

void pcProc::boundaryExtraction(pcl::PointCloud<PointType> pc, pcl::PointCloud<PointType> & pc_b)
{
    pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);
    cloud = pc.makeShared();

    pcl::NormalEstimation<PointType, pcl::Normal> n;
    pcl::search::KdTree<PointType>::Ptr tree(new pcl::search::KdTree<PointType>);
    n.setInputCloud(cloud);
    n.setSearchMethod(tree);
    n.setRadiusSearch(0.01);
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    n.compute(*normals);

    pcl::BoundaryEstimation<PointType, pcl::Normal, pcl::Boundary> boundEst;
    boundEst.setInputCloud(cloud);
    boundEst.setInputNormals(normals);
    boundEst.setRadiusSearch(0.02);
    boundEst.setAngleThreshold(M_PI / 2);

    boundEst.setSearchMethod(tree);
    pcl::PointCloud<pcl::Boundary> boundaries;
    boundEst.compute(boundaries);
    pcl::PointCloud<PointType>::Ptr cloud_boundary(new pcl::PointCloud<PointType>);
    for (int i = 0; i < cloud->points.size(); i++)
    {

        if (boundaries[i].boundary_point > 0)
        {
            cloud_boundary->push_back(cloud->points[i]);
        }
    }

    pc_b = *cloud_boundary;
}

double pcProc::computeCloudRMSE(pcl::PointCloud<PointType>::ConstPtr target,pcl::PointCloud<PointType>::ConstPtr source,double max_range)
{
    pcl::search::KdTree<PointType>::Ptr tree(new pcl::search::KdTree<PointType>);
    tree->setInputCloud(target);
    double fitness_score = 0.0;
    std::vector<int> nn_indices(1);
    std::vector<float> nn_dists(1);
    int nr = 0;

    for (size_t i = 0; i < source->points.size(); ++i) {
        if (!pcl_isfinite((*source)[i].x))
            continue;

        tree->nearestKSearch(source->points[i], 1, nn_indices, nn_dists);

        if (nn_dists[0] <= max_range * max_range) {
            fitness_score += nn_dists[0];
            nr++;
        }
    }

    if (nr > 0)
        return sqrt(fitness_score / nr);
    else
        return (std::numeric_limits<double>::max());
}


void pcProc::jointPoseFromMatrixAndCriteria(Eigen::Isometry3d T_matrix, vector<double> & q_res, vector<double> q_cri)
{
    double* T = new double[16];
    for (int i=0;i<4;i++)
        for (int j = 0; j < 4; j++)
    {
//        T[i] = T_matrix.matrix()(i);
        T[i*4+j] = T_matrix.matrix()(i,j);
    }
    for (int i = 0; i < 4; i++)
    {
        for (int j = i * 4; j < (i + 1) * 4; j++)
            printf("%1.3f ", T[j]);
        printf("\n");
    }
    double q_c[6] = { q_cri[0], q_cri[1], q_cri[2], q_cri[3], q_cri[4], q_cri[5] };
    double q_r[6];
    qNorm(q_c);
    std::cout<<q_c[0]<<","<<q_c[1]<<","<<q_c[2]<<","<<q_c[3]<<","<<q_c[4]<<","<<q_c[5]<<std::endl;
    double q_sols[8 * 6];
    int num_sols;
    num_sols = inverse(T, q_sols);
    std::cout<<num_sols<<std::endl;
    if(num_sols>0)//this pose has solution in joint space
    {
        qSelect(q_sols, q_c, q_r);
        for(int i =0;i<6;i++)
        {
            q_res.push_back(q_r[i]);
        }
    }
    else//no solution in joint space, for UR5 , this pose is unreachable
    {
        std::cout<<"No reachable UR5 joint pose for this position or orientation!"<<std::endl;
        q_res = q_cri;
    }
}

void pcProc::poseSorting_AABB(vector<vector<double> > & pos_vec, vector<Eigen::Isometry3d>& T_vec, vector<double> q_cri, vector<int>& isReachable,vector<Eigen::Vector3d>& orientation)
{
    vector<vector<double>> q_vec;
    isReachable.clear();
    for(int i=0;i<T_vec.size();i++)
    {
        vector<double> q_res;
        jointPoseFromMatrixAndCriteria(T_vec[i], q_res, q_cri);
        std::cout<<q_res[0]<<","<<q_res[1]<<","<<q_res[2]<<","<<q_res[3]<<","<<q_res[4]<<","<<q_res[5]<<std::endl;
        if(q_res[0] == q_cri[0] && q_res[1] == q_cri[1] &&q_res[2] == q_cri[2] &&q_res[3] == q_cri[3] &&q_res[4] == q_cri[4] &&q_res[5] == q_cri[5])
        {
            isReachable.push_back(0);
        }
        else
        {
            isReachable.push_back(1);
        }
        q_vec.push_back(q_res);
        std::cout<<"isReachable:"<<isReachable[i]<<std::endl;
    }

    srand((unsigned)time(NULL));
    Particle_Swarm_Optimization PSO1;
    PSO1.Init_Values(q_vec);
    PSO1.PSO();

    vector<vector<double>>  pos_vec_temp;
    vector<int> isReachableTemp;
    vector<Eigen::Vector3d> orientation_temp;
    vector<Eigen::Isometry3d> T_vec_temp;
    for(int i=0;i<pos_vec.size();i++)
    {
        T_vec_temp.push_back(T_vec[PSO1.Gbest_individual_[i]-1]);
        pos_vec_temp.push_back(pos_vec[PSO1.Gbest_individual_[i]-1]);
        isReachableTemp.push_back(isReachable[PSO1.Gbest_individual_[i]-1]);
        orientation_temp.push_back(orientation[(PSO1.Gbest_individual_[i]-1)*3+0]);
        orientation_temp.push_back(orientation[(PSO1.Gbest_individual_[i]-1)*3+1]);
        orientation_temp.push_back(orientation[(PSO1.Gbest_individual_[i]-1)*3+2]);
        std::cout<<"isReachableTemp:"<<isReachableTemp[i]<<std::endl;
    }

    pos_vec.clear();
    isReachable.clear();
    orientation.clear();
    T_vec.clear();
    isReachable = isReachableTemp;
    pos_vec = pos_vec_temp;
    orientation = orientation_temp;
    T_vec = T_vec_temp;
}

void pcProc::poseSorting_OBB(vector<vector<double> > & pos_vec, vector<Eigen::Isometry3d>& T_vec, vector<double> q_cri, vector<int>& isReachable,vector<Eigen::Vector3d>& orientation, vector<pcl::PointCloud<PointType>>& pc_clusters)
{
    vector<vector<double>> q_vec;
    isReachable.clear();
    for(int i=0;i<T_vec.size();i++)
    {
        vector<double> q_res;
        jointPoseFromMatrixAndCriteria(T_vec[i], q_res, q_cri);
        std::cout<<q_res[0]<<","<<q_res[1]<<","<<q_res[2]<<","<<q_res[3]<<","<<q_res[4]<<","<<q_res[5]<<std::endl;
        if(q_res[0] == q_cri[0] && q_res[1] == q_cri[1] &&q_res[2] == q_cri[2] &&q_res[3] == q_cri[3] &&q_res[4] == q_cri[4] &&q_res[5] == q_cri[5])
        {
            isReachable.push_back(0);
        }
        else
        {
            isReachable.push_back(1);
        }
        q_vec.push_back(q_res);
        std::cout<<"isReachable:"<<isReachable[i]<<std::endl;
    }

    srand((unsigned)time(NULL));
    Particle_Swarm_Optimization PSO1;
    PSO1.Init_Values(q_vec);
    PSO1.PSO();

    vector<vector<double>>  pos_vec_temp;
    vector<int> isReachableTemp;
    vector<Eigen::Vector3d> orientation_temp;
    vector<pcl::PointCloud<PointType>> pc_clusters_temp;
    vector<Eigen::Isometry3d> T_vec_temp;

    for(int i=0;i<pos_vec.size();i++)
    {
        std::cout<<"pose_index:\n"<<i<<std::endl;
        T_vec_temp.push_back(T_vec[PSO1.Gbest_individual_[i]-1]);
        std::cout<<"T_base2tcp_OBB_temp:\n"<<T_vec[PSO1.Gbest_individual_[i]-1].matrix()<<std::endl;
        pos_vec_temp.push_back(pos_vec[PSO1.Gbest_individual_[i]-1]);
//        std::cout<<"pose_OBB_temp:\n"<<pos_vec[PSO1.Gbest_individual_[i]-1]<<std::endl;
        isReachableTemp.push_back(isReachable[PSO1.Gbest_individual_[i]-1]);
        std::cout<<"isReachableTemp:\n"<<isReachableTemp[i]<<std::endl;
        pc_clusters_temp.push_back(pc_clusters[PSO1.Gbest_individual_[i]-1]);
        orientation_temp.push_back(orientation[(PSO1.Gbest_individual_[i]-1)*3+0]);
        std::cout<<"Axis_x_temp:\n"<<orientation[(PSO1.Gbest_individual_[i]-1)*3+0]<<std::endl;
        orientation_temp.push_back(orientation[(PSO1.Gbest_individual_[i]-1)*3+1]);
        std::cout<<"Axis_y_temp:\n"<<orientation[(PSO1.Gbest_individual_[i]-1)*3+1]<<std::endl;
        orientation_temp.push_back(orientation[(PSO1.Gbest_individual_[i]-1)*3+2]);
        std::cout<<"Axis_z_temp:\n"<<orientation[(PSO1.Gbest_individual_[i]-1)*3+2]<<std::endl;
    }

    pos_vec.clear();
    isReachable.clear();
    orientation.clear();
    pc_clusters.clear();
    T_vec.clear();
    isReachable = isReachableTemp;
    pos_vec = pos_vec_temp;
    orientation = orientation_temp;
    pc_clusters = pc_clusters_temp;
    T_vec = T_vec_temp;

}

void pcProc::posCalculationFromOBB(vector <PointType> bboxPoints, vector<Eigen::Vector3f> feature_vectors, vector<double> & posFromOBB,Eigen::Isometry3d& T_base2tcp_tar,vector<Eigen::Vector3d>& orientation_OBB)
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
    T_base2tcp_tar.matrix() = T_base2cam.matrix() * T_tcp2cam.inverse().matrix();

    Eigen::AngleAxisd rv(T_base2tcp_tar.rotation());
    posFromOBB.push_back(double(T_base2tcp_tar.translation()(0)));//x
    posFromOBB.push_back(double(T_base2tcp_tar.translation()(1)));//y
    posFromOBB.push_back(double(T_base2tcp_tar.translation()(2)));//z
    posFromOBB.push_back(double(rv.angle()*rv.axis()(0)));//rx
    posFromOBB.push_back(double(rv.angle()*rv.axis()(1)));//ry
    posFromOBB.push_back(double(rv.angle()*rv.axis()(2)));//rz
}

void pcProc::posCalculationFromAABB(vector<PointType> bboxPoints, vector<vector<double>> & posFromAABB,vector<Eigen::Isometry3d>& T_base2tcp_tar,vector<Eigen::Vector3d>& orientation_AABB)
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
        d = ur->vectorLength(double(lineCenterPoints[m].x-recCenterPoints[n].x),double(lineCenterPoints[m].y-recCenterPoints[n].y),double(lineCenterPoints[m].z-recCenterPoints[n].z));
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

    //pos 4
    z_<< double(recCenterPoints[0].x - lineCenterPoints[3].x ),double( recCenterPoints[0].y - lineCenterPoints[3].y ),double(recCenterPoints[0].z - lineCenterPoints[3].z);
    y_<< double(ABCDEFGH[6].x-ABCDEFGH[7].x),double(ABCDEFGH[6].y-ABCDEFGH[7].y),double(ABCDEFGH[6].z-ABCDEFGH[7].z);
    vec3dNorm(z_);
    vec3dNorm(y_);
    x_ = vec3dCross(y_,z_);
    orientation_AABB.push_back(x_);
    orientation_AABB.push_back(y_);
    orientation_AABB.push_back(z_);
    T.matrix()<< x_(0), y_(0), z_(0), double(transformedPositions[3].x),
                 x_(1), y_(1), z_(1), double(transformedPositions[3].y),
                 x_(2), y_(2), z_(2), double(transformedPositions[3].z),
                     0,     0,     0,        1;
    T_base2cam.push_back(T);

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

vector<pcl::PointCloud<PointType>> pcProc::IcpRegistrationForVec(vector<pcl::PointCloud<PointType>> vec)
{
    vector<pcl::PointCloud<PointType>> vec_res;
    for (int i = 0;i < vec.size();++i)
    {
        pcl::PointCloud<PointType> pc_res;
        pc_res = IcpRegistrationForPair( vec[i], vec[0]);
        vec_res.push_back(pc_res);
    }
    return(vec_res);
}

vector<pcl::PointCloud<PointType>> pcProc::passThroughForVec(vector<pcl::PointCloud<PointType>> vec)
{
    vector<pcl::PointCloud<PointType>> vec_res;
    for (int i = 0;i < vec.size();++i)
    {
        pcl::PointCloud<PointType> pc_res;
        pc_res = passThroughPc(vec[i]);
        vec_res.push_back(pc_res);
    }
    return(vec_res);
}

pcl::PointCloud<PointType> pcProc::IcpRegistrationForPair(pcl::PointCloud<PointType> pc_src,pcl::PointCloud<PointType> pc_tgt)
{
    pcl::console::TicToc time;
    //pc -> ptr
    pcl::PointCloud<PointType>::Ptr source(new pcl::PointCloud<PointType>);
    source = pc_src.makeShared();
    //end
    //pc -> ptr
    pcl::PointCloud<PointType>::Ptr target(new pcl::PointCloud<PointType>);
    target = pc_tgt.makeShared();
    //end
    time.tic();
    pcl::IterativeClosestPoint<PointType, PointType> icp;
    icp.setInputSource(source);
    icp.setInputTarget(target);
    icp.setTransformationEpsilon(1e-10);
    icp.setMaxCorrespondenceDistance(1);
    icp.setEuclideanFitnessEpsilon(0.001);
    icp.setMaximumIterations(35);
    icp.setUseReciprocalCorrespondences(true);
    pcl::PointCloud<PointType>::Ptr icp_cloud(new pcl::PointCloud<PointType>);
    icp.align(*icp_cloud);
    std::cout << "Applied " << 35 << " ICP iterations in " << time.toc() << " ms" << std::endl;
    std::cout << "\nICP has converged, score is " << icp.getFitnessScore() << std::endl;
    std::cout <<"The transformation matrix is: \n"<< icp.getFinalTransformation() << std::endl;
    pcl::transformPointCloud(*source, *icp_cloud, icp.getFinalTransformation());
    return (*icp_cloud);
}

pcl::PointCloud<PointType> pcProc::mergePcVec(vector<pcl::PointCloud<PointType>> vec)
{
    pcl::PointCloud<PointType> pc_res;
    for (int i = 0;i<vec.size();++i)
    {
        pc_res = pc_res + vec[i];
    }
    pc_res = voxelFilteringPc(pc_res);
    return(pc_res);
}

pcl::PointCloud<PointType> pcProc::voxelFilteringPc(pcl::PointCloud<PointType> pc)
{
    //new version of voxel grid filtering  -v1
    //pc -> ptr
    pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);
    cloud = pc.makeShared();
    //end

    std::cout << "PointCloud before filtering: " << cloud->width * cloud->height
        << " data points (" << pcl::getFieldsList(*cloud) << ")." << std::endl;

    pcl::VoxelGrid<PointType> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(0.005f, 0.005f, 0.005f);
    pcl::PointCloud<PointType> ::Ptr voxel_filtered(new pcl::PointCloud<PointType>);
    sor.filter(*voxel_filtered);

    pcl::KdTreeFLANN<PointType> kdtree;
    kdtree.setInputCloud(cloud);
    pcl::PointIndicesPtr inds = boost::shared_ptr<pcl::PointIndices>(new pcl::PointIndices());
    for (size_t i = 0; i < voxel_filtered->points.size(); i++) {
        PointType searchPoint;
        searchPoint.x = voxel_filtered->points[i].x;
        searchPoint.y = voxel_filtered->points[i].y;
        searchPoint.z = voxel_filtered->points[i].z;

        int K = 1;
        vector<int> pointIdxNKNSearch(K);
        vector<float> pointNKNSquaredDistance(K);
        if (kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {

            inds->indices.push_back(pointIdxNKNSearch[0]);

        }

    }
    pcl::PointCloud<PointType>::Ptr final_filtered(new pcl::PointCloud<PointType>);
    pcl::copyPointCloud(*cloud, inds->indices, *final_filtered);
    std::cout << "the number of final downsampling cloud " << final_filtered->points.size() << std::endl;

//    pcl::PCDWriter writer;
//    stringstream ss;
//    ss << "downsample_" << inputFile;
//    pcl::io::savePCDFile(ss.str(), *final_filtered);

    return (*final_filtered);


//    //original voxel grid filtering  -v0
//    //pc -> ptr
//    pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);
//    cloud = pc.makeShared();
//    //end
//	cout << "PointCloud before filtering: " << *cloud << endl;
//	pcl::PointCloud<PointType>::Ptr cloud_filtered(new pcl::PointCloud<PointType>);
//	pcl::VoxelGrid<PointType> vg;
//	vg.setInputCloud(cloud);
//	vg.setLeafSize(0.01f, 0.01f, 0.01f);
//	vg.filter(*cloud_filtered);
//	cout << "PointCloud after filtering: " << *cloud_filtered << endl;
//    return(*cloud_filtered);
}

pcl::PointCloud<PointType> pcProc::basicVoxelFilteringPc(pcl::PointCloud<PointType> pc)
{
    //original voxel grid filtering  -v0
    //pc -> ptr
    pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);
    cloud = pc.makeShared();
    //end
    std::cout << "PointCloud before filtering: " << *cloud << std::endl;
    pcl::PointCloud<PointType>::Ptr cloud_filtered(new pcl::PointCloud<PointType>);
    pcl::VoxelGrid<PointType> vg;
    vg.setInputCloud(cloud);
    vg.setLeafSize(0.005f, 0.005f, 0.005f);
    vg.filter(*cloud_filtered);
    std::cout << "PointCloud after filtering: " << *cloud_filtered << std::endl;
    return(*cloud_filtered);
}

pcl::PointCloud<PointType> pcProc::radiusFilteringPc(pcl::PointCloud<PointType> pc)
{
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
    //pc -> ptr
    pcl::PointCloud<PointType>::Ptr cloud_in(new pcl::PointCloud<PointType>);
    cloud_in = pc.makeShared();
    //end
    pcl::PointCloud<PointType>::Ptr cloud_radius(new pcl::PointCloud<PointType>);
    pcl::RadiusOutlierRemoval<PointType> ror;
    ror.setInputCloud(cloud_in);
    ror.setRadiusSearch(0.1);
    ror.setMinNeighborsInRadius(10);
    ror.filter(*cloud_radius);
    return(*cloud_radius);
}

pcl::PointCloud<PointType> pcProc::passThroughPc(pcl::PointCloud<PointType> pc)
{
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<PointType>::Ptr cloud_filtered (new pcl::PointCloud<PointType>);
    //pc -> ptr
    pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);
    cloud = pc.makeShared();
    //end
    std::cout << "loading point cloud: " << cloud->points.size() << std::endl;
    pcl::PassThrough<PointType> pass;

    //x  50-120
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("x");
    pass.setFilterLimits (X_MIN_, X_MAX_);
    // pass.setKeepOrganized(true);
    pass.setNegative (false);
    pass.filter (*cloud_filtered);

    //y  -70-0
    pass.setInputCloud (cloud_filtered);
    pass.setFilterFieldName ("y");
    pass.setFilterLimits (Y_MIN_, Y_MAX_);
    // pass.setKeepOrganized(true);
    pass.setNegative (false);
    pass.filter (*cloud_filtered);

    //z  0-60
    pass.setInputCloud (cloud_filtered);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (Z_MIN_, Z_MAX_);
    // pass.setKeepOrganized(true);
    pass.setNegative (false);
    pass.filter (*cloud_filtered);

    std::cout << "Cloud after filtering: " << cloud_filtered ->points.size()<< std::endl;
    return(*cloud_filtered);
}

void pcProc::pcClustering(pcl::PointCloud<PointType> pc,vector<PointIndices>& clusters,pcl::PointCloud<PointType>& colored_cluster_cloud)
{
    clusters.clear();
    //pc -> ptr
    pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);
    cloud = pc.makeShared();
    //end
    pcl::search::Search<PointType>::Ptr tree(new pcl::search::KdTree<PointType>);
    pcl::PointCloud <pcl::Normal>::Ptr normals(new pcl::PointCloud <pcl::Normal>);
    pcl::NormalEstimationOMP<PointType, pcl::Normal> n;
    n.setSearchMethod(tree);
    n.setInputCloud(cloud);
    n.setNumberOfThreads(6);
    n.setKSearch(10);
    n.compute(*normals);
    pcl::RegionGrowing<PointType, pcl::Normal> reg;
    reg.setMinClusterSize(250);
    reg.setMaxClusterSize(1000000);
    reg.setSearchMethod(tree);
    reg.setNumberOfNeighbours(10);
    reg.setInputCloud(cloud);
    reg.setInputNormals(normals);
    reg.setSmoothnessThreshold(pcl::deg2rad(15.0));
    reg.setCurvatureThreshold(1.0);
//    std::vector <pcl::PointIndices> clusters;
    reg.extract(clusters);
    pcl::PointCloud<PointType>::Ptr colored_cluster_cloud_ptr(new pcl::PointCloud<PointType>);
    colored_cluster_cloud_ptr = reg.getColoredCloudRGBA();
    colored_cluster_cloud = *colored_cluster_cloud_ptr;
    std::cout << "Number of clusters is equal to " << clusters.size() << std::endl;
    std::cout << "First cluster has " << clusters[0].indices.size() << " points." << std::endl;
    //save the clustered pc
    /*int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = clusters.begin(); it != clusters.end(); ++it)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);

        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++)
            cloud_cluster->points.push_back(cloud->points[*pit]);
        cloud_cluster->width = cloud_cluster->points.size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size() << " data points."
            << std::endl;
        std::stringstream ss;
        ss << "cloud_cluster_" << j << ".pcd";
        pcl::io::savePCDFileASCII(ss.str(), *cloud_cluster);
        cout << ss.str() << "Saved" << endl;
        j++;
    }

    */
}

void pcProc::extractPcClustersFromIndices(pcl::PointCloud<PointType> pc,vector<PointIndices> clusters,vector<pcl::PointCloud<PointType>>& pc_clusters)
{
    pc_clusters.clear();
    //pc -> ptr
    pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);
    cloud = pc.makeShared();
    //end
    for (std::vector<pcl::PointIndices>::const_iterator it = clusters.begin(); it != clusters.end(); ++it)
    {
        pcl::PointCloud<PointType>::Ptr cloud_cluster(new pcl::PointCloud<PointType>);

        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++)
            cloud_cluster->points.push_back(cloud->points[*pit]);

        cloud_cluster->width = cloud_cluster->points.size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        pc_clusters.push_back(*cloud_cluster);
    }
}

void pcProc::bboxCalculationForPc(pcl::PointCloud<PointType> pc,
                            vector <float> & moment_of_inertia,//vector <float> moment_of_inertia;
                            vector <float> & eccentricity,//vector <float> eccentricity
                            vector <PointType> & bboxPoints,//min_point_AABB, max_point_AABB, min_point_OBB, max_point_OBB, position_OBB
                          Eigen::Matrix3f & rotational_matrix_OBB,// rotational_matrix_OBB
                          vector <float> & feature_values,//major_value, middle_value, minor_value
                          vector<Eigen::Vector3f> & feature_vectors//  Eigen::Vector3f major_vector, middle_vector, minor_vector, mass_center;
                          )
{
    //pc -> ptr
    pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);
    cloud = pc.makeShared();
    //end
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

pcl::PointCloud<PointType> pcProc::HSVSegForPc(pcl::PointCloud<PointType> pc)
{
    pcl::PointCloud<PointType>::Ptr body(new pcl::PointCloud<PointType>);
    pcl::PointCloud<pcl::PointXYZHSV>::Ptr cloud_HSV(new pcl::PointCloud<pcl::PointXYZHSV>);
    //pc -> ptr
    pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);
    cloud = pc.makeShared();
    //end
    pcl::PointCloudXYZRGBAtoXYZHSV(*cloud, *cloud_HSV);

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

    for (int i = 0; i < cloud->points.size(); i++)
    {
        if ((cloud_HSV->points[i].h < 50 && cloud_HSV->points[i].h > 0) || cloud_HSV->points[i].h > 320)  //hsv threshold adjust
        {
            inliers->indices.push_back(i);
        }
    }
    pcl::ExtractIndices<PointType> eifilter(true);
    eifilter.setInputCloud(cloud);
    eifilter.setIndices(inliers);
    eifilter.filter(*body);

    return *body;//return: ptr->pc
}

void pcProc::autoPcDataSaving()
{
    // 判断文件夹是否存在，不存在则创建
    QDateTime current_date_time =QDateTime::currentDateTime();
    QString current_date =current_date_time.toString("yyyy_MM_dd_hh_mm_ss");
    QString fullPath = current_date + "_pcData";
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

    std::string pc_path;
//    //iteration among pc datas or vectors
//    if(!vec_pc_memory_cam.empty())
//    {
//        for(int i = 0;i<vec_pc_memory_cam.size();++i)
//        {
//            pc_path = fullPath.toStdString() + "\\" + "m" + to_string(i) + "_cam.pcd";
//            pcl::io::savePCDFileASCII (pc_path, vec_pc_memory_cam[i]);
//        }
//    }

    if(!vec_pc_memory_base.empty())
    {
        for(int i = 0;i<vec_pc_memory_base.size();++i)
        {
            pc_path = fullPath.toStdString() + "\\" + "m" + to_string(i) + "_base.pcd";
            pcl::io::savePCDFileASCII (pc_path, vec_pc_memory_base[i]);
        }
    }

//    if(!vec_plant_memory_base.empty())
//    {
//        for(int i = 0;i<vec_plant_memory_base.size();++i)
//        {
//            pc_path = fullPath.toStdString() + "\\" + "m" + to_string(i) + "_base_plant.pcd";
//            pcl::io::savePCDFileASCII (pc_path, vec_plant_memory_base[i]);
//        }
//    }

//    if(!vec_pc_bbox_cam.empty())
//    {
//        for(int i = 0;i<vec_pc_bbox_cam.size();++i)
//        {
//            pc_path = fullPath.toStdString() + "\\" + "b" + to_string(i) + "_cam.pcd";
//            pcl::io::savePCDFileASCII (pc_path, vec_pc_bbox_cam[i]);
//        }
//    }

//    if(!vec_pc_bbox_base.empty())
//    {
//        for(int i = 0;i<vec_pc_bbox_base.size();++i)
//        {
//            pc_path = fullPath.toStdString() + "\\" + "b" + to_string(i) + "_base.pcd";
//            pcl::io::savePCDFileASCII (pc_path, vec_pc_bbox_base[i]);
//        }
//    }

//    if(!vec_plant_bbox_base.empty())
//    {
//        for(int i = 0;i<vec_plant_bbox_base.size();++i)
//        {
//            pc_path = fullPath.toStdString() + "\\" + "b" + to_string(i) + "_base_plant.pcd";
//            pcl::io::savePCDFileASCII (pc_path, vec_plant_bbox_base[i]);
//        }
//    }

//    if(!vec_pc_ok_cam.empty())
//    {
//        for(int i = 0;i<vec_pc_ok_cam.size();++i)
//        {
//            pc_path = fullPath.toStdString() + "\\" + "ok" + to_string(i) + "_cam.pcd";
//            pcl::io::savePCDFileASCII (pc_path, vec_pc_ok_cam[i]);
//        }
//    }

//    if(!vec_pc_ok_base.empty())
//    {
//        for(int i = 0;i<vec_pc_ok_base.size();++i)
//        {
//            pc_path = fullPath.toStdString() + "\\" + "ok" + to_string(i) + "_base.pcd";
//            pcl::io::savePCDFileASCII (pc_path, vec_pc_ok_base[i]);
//        }
//    }

//    if(!vec_plant_ok_base.empty())
//    {
//        for(int i = 0;i<vec_plant_ok_base.size();++i)
//        {
//            pc_path = fullPath.toStdString() + "\\" + "ok" + to_string(i) + "_base_plant.pcd";
//            pcl::io::savePCDFileASCII (pc_path, vec_plant_ok_base[i]);
//        }
//    }

//    pc_path = fullPath.toStdString() + "\\" + "memory" + "_plant.pcd";
//    pcl::io::savePCDFileASCII (pc_path, plant_memory);

//    pc_path = fullPath.toStdString() + "\\" + "bbox" + "_plant.pcd";
//    pcl::io::savePCDFileASCII (pc_path, plant_bbox);

//    pc_path = fullPath.toStdString() + "\\" + "ok" + "_plant.pcd";
//    pcl::io::savePCDFileASCII (pc_path, plant_ok);

//    if(!pc_clusters.empty())
//    {
//        for(int i = 0;i<pc_clusters.size();++i)
//        {
//            pc_path = fullPath.toStdString() + "\\" + "cluster_" + to_string(i) + ".pcd";
//            pcl::io::savePCDFileASCII (pc_path, pc_clusters[i]);
//        }
//    }

    emit pcDataSavingDown();
}

void pcProc::pcDataSaving()
{
    // 判断文件夹是否存在，不存在则创建
    QDateTime current_date_time =QDateTime::currentDateTime();
    QString current_date =current_date_time.toString("yyyy_MM_dd_hh_mm_ss");
    QString fullPath = current_date + "_pcData";
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

    std::string pc_path;
//    //iteration among pc datas or vectors
//    if(!vec_pc_memory_cam.empty())
//    {
//        for(int i = 0;i<vec_pc_memory_cam.size();++i)
//        {
//            pc_path = fullPath.toStdString() + "\\" + "m" + to_string(i) + "_cam.pcd";
//            pcl::io::savePCDFileASCII (pc_path, vec_pc_memory_cam[i]);
//        }
//    }

//    if(!vec_pc_memory_base.empty())
//    {
//        for(int i = 0;i<vec_pc_memory_base.size();++i)
//        {
//            pc_path = fullPath.toStdString() + "\\" + "m" + to_string(i) + "_base.pcd";
//            pcl::io::savePCDFileASCII (pc_path, vec_pc_memory_base[i]);
//        }
//    }

//    if(!vec_plant_memory_base.empty())
//    {
//        for(int i = 0;i<vec_plant_memory_base.size();++i)
//        {
//            pc_path = fullPath.toStdString() + "\\" + "m" + to_string(i) + "_base_plant.pcd";
//            pcl::io::savePCDFileASCII (pc_path, vec_plant_memory_base[i]);
//        }
//    }

//    if(!vec_pc_bbox_cam.empty())
//    {
//        for(int i = 0;i<vec_pc_bbox_cam.size();++i)
//        {
//            pc_path = fullPath.toStdString() + "\\" + "b" + to_string(i) + "_cam.pcd";
//            pcl::io::savePCDFileASCII (pc_path, vec_pc_bbox_cam[i]);
//        }
//    }

//    if(!vec_pc_bbox_base.empty())
//    {
//        for(int i = 0;i<vec_pc_bbox_base.size();++i)
//        {
//            pc_path = fullPath.toStdString() + "\\" + "b" + to_string(i) + "_base.pcd";
//            pcl::io::savePCDFileASCII (pc_path, vec_pc_bbox_base[i]);
//        }
//    }

//    if(!vec_plant_bbox_base.empty())
//    {
//        for(int i = 0;i<vec_plant_bbox_base.size();++i)
//        {
//            pc_path = fullPath.toStdString() + "\\" + "b" + to_string(i) + "_base_plant.pcd";
//            pcl::io::savePCDFileASCII (pc_path, vec_plant_bbox_base[i]);
//        }
//    }

//    if(!vec_pc_ok_cam.empty())
//    {
//        for(int i = 0;i<vec_pc_ok_cam.size();++i)
//        {
//            pc_path = fullPath.toStdString() + "\\" + "ok" + to_string(i) + "_cam.pcd";
//            pcl::io::savePCDFileASCII (pc_path, vec_pc_ok_cam[i]);
//        }
//    }

//    if(!vec_pc_ok_base.empty())
//    {
//        for(int i = 0;i<vec_pc_ok_base.size();++i)
//        {
//            pc_path = fullPath.toStdString() + "\\" + "ok" + to_string(i) + "_base.pcd";
//            pcl::io::savePCDFileASCII (pc_path, vec_pc_ok_base[i]);
//        }
//    }

//    if(!vec_plant_ok_base.empty())
//    {
//        for(int i = 0;i<vec_plant_ok_base.size();++i)
//        {
//            pc_path = fullPath.toStdString() + "\\" + "ok" + to_string(i) + "_base_plant.pcd";
//            pcl::io::savePCDFileASCII (pc_path, vec_plant_ok_base[i]);
//        }
//    }

//    pc_path = fullPath.toStdString() + "\\" + "memory" + "_plant.pcd";
//    pcl::io::savePCDFileASCII (pc_path, plant_memory);

    pc_path = fullPath.toStdString() + "\\" + "bbox" + "_plant.pcd";
    pcl::io::savePCDFileASCII (pc_path, plant_bbox);

    pc_path = fullPath.toStdString() + "\\" + "colored_bbox" + "_plant.pcd";
    pcl::io::savePCDFileASCII (pc_path, colored_cluster_cloud);

//    pc_path = fullPath.toStdString() + "\\" + "ok" + "_plant.pcd";
//    pcl::io::savePCDFileASCII (pc_path, plant_ok);

    if(!pc_clusters.empty())
    {
        for(int i = 0;i<pc_clusters.size();++i)
        {
            pc_path = fullPath.toStdString() + "\\" + "cluster_" + to_string(i) + ".pcd";
            pcl::io::savePCDFileASCII (pc_path, pc_clusters[i]);
        }
    }

    emit pcDataSavingDown();
}

void pcProc::transformMemoryPcCamToBase()
{
    vec_pc_memory_base = pcVectorTransform(vec_pc_memory_cam,T_memory_base2tcp,T_tcp2cam);
}

void pcProc::transformBboxPcCamToBase()
{
    vec_pc_bbox_base = pcVectorTransform(vec_pc_bbox_cam,T_bbox_base2tcp,T_tcp2cam);
}

void pcProc::transformOKPcCamToBase()
{
    vec_pc_ok_base = pcVectorTransform(vec_pc_ok_cam,T_ok_base2tcp,T_tcp2cam);
}

void pcProc::memoryTransformationAcquiring()
{
    T_memory_base2tcp.push_back(ur->T_base2tcp);
    T_tcp2cam = ur->T_tcp2cam;
}

void pcProc::bboxTransformationAcquiring()
{
    T_bbox_base2tcp.push_back(ur->T_base2tcp);
    T_tcp2cam = ur->T_tcp2cam;
}

void pcProc::okTransformationAcquiring()
{
    T_ok_base2tcp.push_back(ur->T_base2tcp);
    T_tcp2cam = ur->T_tcp2cam;
}

vector<pcl::PointCloud<PointType>> pcProc::pcVectorTransform(vector<pcl::PointCloud<PointType>> vec,vector<Eigen::Isometry3d> T_b2t,Eigen::Isometry3d T_t2c)
{
    vector<pcl::PointCloud<PointType>> vec_return;
    for (int i=0; i<vec.size(); ++i)
    {
        pcl::PointCloud<PointType> transformed_cloud;
        Eigen::Isometry3d T_b2c;
        T_b2c.matrix() = T_b2t[i].matrix() * T_t2c.matrix();
        pcl::transformPointCloud (vec[i], transformed_cloud, T_b2c.matrix());
        vec_return.push_back(transformed_cloud);
    }
    return vec_return;
}

void pcProc::dataClear()
{
    vec_pc_memory_cam.clear();
    vec_pc_memory_base.clear();
    vec_plant_memory_base.clear();
    vec_pc_bbox_cam.clear();
    vec_pc_bbox_base.clear();
    vec_plant_bbox_base.clear();
    vec_pc_ok_cam.clear();//best pos for data acquiring
    vec_pc_ok_base.clear();
    vec_plant_ok_base.clear();
    plant_memory.clear();
    plant_bbox.clear();
    plant_ok.clear();
    pc_iter_base.clear();
    pc_iter_cam.clear();
    pc_clusters.clear();
    colored_cluster_cloud.clear();

    isReachable_AABB.clear();
    isReachable_OBB.clear();
    posFromAABB.clear();
    posFromOBB.clear();
    orientation_AABB.clear();//x1 y1 z1 x2 y2 z2 ...
    orientation_OBB.clear();//x1 y1 z1 x2 y2 z2 ...

    //aabb for the whole plant
    moment_of_inertia.clear();
    eccentricity.clear();
    feature_values.clear();
    bboxPoints.clear();
    feature_vectors.clear();
    //obb for plant parts (vec)
    moment_of_inertia_vec.clear();
    eccentricity_vec.clear();
    feature_values_vec.clear();
    obbPoints_vec.clear();
    rotational_matrix_OBB_vec.clear();
    feature_vectors_vec.clear();

    T_memory_base2tcp.clear();
    T_bbox_base2tcp.clear();
    T_ok_base2tcp.clear();

    axis_ok.clear();
    T_base2tcp_ok.clear();
    pc_clusters_ok.clear();
    posFromOBB_ok.clear();
    obbPoints_vec_ok.clear();
    rotational_matrix_OBB_vec_ok.clear();
    //define a vec to figure out whether or not the pose need to be changed for occlusion
    isOccluded.clear();
    //define all the variables for iteration, the original variables will be not kept
    ur_pose_iter.clear();//after changed
    T_base2tcp_iter.clear();//after changed
    axis_iter.clear();//after changed
    pc_clusters_iter.clear();//after changed, fullfill the occluded cluster, the others kept
    //just for visualization, change them into global vars
    pc_b.clear();//bounding line pc data
    pc_clusters_single_frame.clear();//pc data
    pc_sp.clear();//sphere projected pc data
    idx_spa.clear();//sphere projected adjacent pc index

    //new MC method
    plant_convex_hull.clear();
    polygons.clear();
    orientation_MC.clear();
    posFromMC.clear();
    T_base2tcp_MC.clear();
}

void pcProc::dealURArrival()
{
    std::cout<<"UR arrival! Start pc acquiring & transformation acquiring!"<<std::endl;

    switch(flag)
    {
        case 0 :
            std::cout<<"UR arriving at the target pos without any action!"<<std::endl;
           break;
        case 1 :
            memoryTransformationAcquiring();
            memoryPcAcquiring();
           break;
        case 2 :
            bboxTransformationAcquiring();
            bboxPcAcquiring();
           break;
        case 3 :
            okTransformationAcquiring();
            okPcAcquiring();
        case 4 :
            iterPcAcquiring();
        default :
           break;
    }
}

void pcProc::memoryPcAcquiring()
{
    waitForStablePc();
    vec_pc_memory_cam.push_back(kinect->dk_cloud);
    isPcAcquiringFinished = 1;
}

void pcProc::bboxPcAcquiring()
{
    waitForStablePc();
    vec_pc_bbox_cam.push_back(kinect->dk_cloud);
    isPcAcquiringFinished = 1;
}

void pcProc::okPcAcquiring()
{
    waitForStablePc();
    vec_pc_ok_cam.push_back(kinect->dk_cloud);
    isPcAcquiringFinished = 1;
}

void pcProc::iterPcAcquiring()
{
    waitForStablePc();
    pc_iter_cam = kinect->dk_cloud;
    isPcAcquiringFinished = 1;
}

void pcProc::waitForStablePc()
{
    _sleep(2*1000); //delay 2 s for next arrival detection
}

void pcProc::waitForPcAcquiringFinished()
{
    while(!isPcAcquiringFinished)
        _sleep(0.1*1000);
}

void pcProc::vec3dNorm(Eigen::Vector3d & vec)
{
    double d = sqrt(vec(0)*vec(0)+vec(1)*vec(1)+vec(2)*vec(2));
    vec(0) = vec(0)/d;
    vec(1) = vec(1)/d;
    vec(2) = vec(2)/d;
}

void pcProc::vec3fNorm(Eigen::Vector3f & vec)
{
    float d = sqrt(vec(0)*vec(0)+vec(1)*vec(1)+vec(2)*vec(2));
    vec(0) = vec(0)/d;
    vec(1) = vec(1)/d;
    vec(2) = vec(2)/d;
}

float pcProc::vec3fDot(Eigen::Vector3f m, Eigen::Vector3f n)
{
    return m(0)*n(0)+m(1)*n(1)+m(2)*n(2);
}

Eigen::Vector3d pcProc::vec3dCross(Eigen::Vector3d m, Eigen::Vector3d n)
{
    Eigen::Vector3d res_vec;
    res_vec(0) = m(1)*n(2) - m(2)*n(1);
    res_vec(1) = (-1)*(m(0)*n(2) - m(2)*n(0));
    res_vec(2) = m(0)*n(1) - m(1)*n(0);
    return res_vec;
}

float pcProc::norm2f(Eigen::Vector3f vec)
{
    return sqrt(vec(0)*vec(0)+vec(1)*vec(1)+vec(2)*vec(2));
}

double pcProc::norm2d(Eigen::Vector3d vec)
{
    return sqrt(vec(0)*vec(0)+vec(1)*vec(1)+vec(2)*vec(2));
}

float pcProc::pointDistance(PointType p1, PointType p2)
{
    float d;
    d = sqrt((p1.x-p2.x)*(p1.x-p2.x)+(p1.y-p2.y)*(p1.y-p2.y)+(p1.z-p2.z)*(p1.z-p2.z));
    return d;
}

int pcProc::occlusionJudgeForPoint(PointType vp, PointType p1, PointType p2)
{
    float d1,d2;
    d1 = pointDistance(vp, p1);
    d2 = pointDistance(vp, p2);
    if(d1 < d2)
    {
        return 1;
    }
    else if(d1==d2)
    {
        return 0;
    }
    else
    {
        return -1;
    }
}

float pcProc::solidAngleForTriangle(PointType O, PointType A , PointType B, PointType C)
{
    Eigen::Vector3f OA(A.x - O.x,A.y - O.y,A.z - O.z);
    Eigen::Vector3f OB(B.x - O.x,B.y - O.y,B.z - O.z);
    Eigen::Vector3f OC(C.x - O.x,C.y - O.y,C.z - O.z);

    float dis_OA = norm2f(OA);
    float dis_OB = norm2f(OB);
    float dis_OC = norm2f(OC);

    double alpha = acos(double(vec3fDot(OB,OC)/(dis_OB*dis_OC)));
    double beta = acos(double(vec3fDot(OA,OC)/(dis_OA*dis_OC)));
    double gamma = acos(double(vec3fDot(OA,OB)/(dis_OA*dis_OB)));
    double s = 0.5 * (alpha + beta + gamma);

    float sigma = float(4 * atan(sqrt(tan(0.5*(s))*tan(0.5*(s-alpha))*tan(0.5*(s-beta))*tan(0.5*(s-gamma)))));

    return sigma;
}

template<typename T>
std::vector<int> pcProc::findItems(std::vector<T> const &v, int target)
{
    std::vector<int> indices;

    for (int i = 0; i < v.size(); i++)
    {
        if (v[i] == target)
        {
            indices.push_back(i);
        }
    }

    return indices;
}

template <typename T>
vector<size_t> pcProc::sort_indexes_e(vector<T> &v)
{
   vector<size_t> idx(v.size());
   iota(idx.begin(), idx.end(), 0);
   sort(idx.begin(), idx.end(),
       [&v](size_t i1, size_t i2) {return v[i1] > v[i2]; });
   return idx;
}
