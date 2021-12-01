//
// Created by caleb on 12.05.21.
//

#ifndef SRC_POINTCLOUD_PROCESSING_H
#define SRC_POINTCLOUD_PROCESSING_H

#include <iostream>
#include <vector>
#include <algorithm>
#include <chrono>

#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h> 
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <visualization_msgs/Marker.h>
#include "visualization_msgs/MarkerArray.h"
#include "std_msgs/Float32MultiArray.h"
#include <tf/tf.h>
#include "abstract_module.h"


typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT>::Ptr PtCdtr;

extern std::vector<PtCdtr> cloudClusters;
extern pcl::visualization::PCLVisualizer::Ptr viewer;
extern PtCdtr cp_filtered; // filtered point cloud for collision detection


struct Box // bounding box
{
    float x_min;
    float y_min;
    float z_min;
    float x_max;
    float y_max;
    float z_max;
};



class pointcloud_processing : public abstract_module{
    ros::NodeHandle nh;
    ros::Publisher modelPub;
    ros::Publisher imuPub;
    ros::Publisher boxPub;
    ros::Publisher locPub;
    ros::Publisher pcPub;
    ros::Publisher directPub;
    ros::Subscriber direct_sub;
    ros::Subscriber imu_sub;
    ros::Subscriber velocity_sub;
    ros::Subscriber point_sub;
    ros::Subscriber gps_sub;
    ros::Subscriber img_sub;
public:
    pointcloud_processing(int timeRate_in,boost::shared_ptr<pcl::visualization::PCLVisualizer>& pc_viewer); //initialize

    ~pointcloud_processing();

    void processStep();// main program of pointcloud processing

    void publish_decide(); // publish decision shown in rivz

private:
    PtCdtr FilterCloud(PtCdtr cloud, float filterRes); // downsampling by Voxel grid filter and Conditional Removal

    PtCdtr FilterCloud_1(PtCdtr cloud, int k_mean, float thres); // Statistical Outlier Removal

    std::pair<PtCdtr, PtCdtr>
    RansacSegmentPlane(PtCdtr cloud, int maxIterations, float distanceTol);  // Ground removal by RANSAC

    std::vector<PtCdtr>EuclidCluster(PtCdtr cloud, float clusterTolerance, int minSize,int maxSize); //Euclidean clustering

    void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg); //get GPS information

    void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg); //get pointcloud

    void renderPointCloud(pcl::visualization::PCLVisualizer::Ptr& viewer, const PtCdtr& cloud,std::string name);//visulization of point cloud

    void renderCluster(pcl::visualization::PCLVisualizer::Ptr& viewer, std::vector<PtCdtr> clusters);//visulization of cluster

    void publish_drone();//publish drone 3D-model, acceleration info and current safety volume

    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);//get imu info

    void publish_box();//publish bounding box for each cluster in rviz

    void location();//calculate the trajectory

    void publish_loc();//publish the trajectory

    Box BoundingBox(PtCdtr cluster);//build a bounding box for cluster

    Box createBox(Eigen::Vector3f uav_point, float width, float length, float height, const float offset[]);//build a bounding box for drone

    double getLength(const double *v);//calculate vector's length

    geometry_msgs::Vector3 toEulerAngle(geometry_msgs::Quaternion quat);//transform quaternion to euler angle

//    void attitude_callback(const geometry_msgs::QuaternionStamped::ConstPtr &msg);

    void velocityCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg);//get velocity info

    void directCallback(const std_msgs::Float32MultiArray::ConstPtr& msg);//get decision info


    //Parameters
    //Voxel grid and condition removal
    float filterRes = 0.2;
    float thres_x = 10;
    float thres_y = 3.5;
    float thres_z = 12;
    //StatisticalOutlierRemoval
    int k_mean = 50;
    float thres_stat = 1.0;
    //Segmentation
    float distanceThreshold = 0.2;
    int maxIterations = 50;
    float h_seg = 1.0; // wenn the point height is lower this value, segmentation will be performed.
    //Clustering
    float clusterTolerance = 0.5;
    int minsize = 100;
//    int minsize = 30;
    int maxsize = 5000;

    int gps_state = 0;
    sensor_msgs::NavSatFix init_gps;
    sensor_msgs::NavSatFix current_gps;
    sensor_msgs::Imu imu;
    sensor_msgs::Imu pre_imu;
    pcl::PointCloud<PointT> inputCloud;
    PtCdtr inputCloud_ptr;
    bool imu_state = false;
    geometry_msgs::Vector3 v;
    geometry_msgs::Point loc[100];
//    geometry_msgs::Point loc_t[100];
    int clusters_size = 0;
    int timeRate = 0;
    const float cameraPosition[3]={0,0,0};//{0,0.1,-0.2};


    float direct = 0;
    float ang = 0;
    float detec_l = 0;
    float minDis = 0;

};






#endif //SRC_POINTCLOUD_PROCESSING_H
