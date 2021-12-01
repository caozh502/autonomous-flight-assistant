//
// Created by caleb on 10.05.21.
//

#ifndef PCFILTER_COLLISIONDETECTION_H
#define PCFILTER_COLLISIONDETECTION_H

// Collision, Distance
#include <fcl/narrowphase/collision_object.h>
#include <fcl/narrowphase/distance.h>
#include <fcl/broadphase/broadphase_dynamic_AABB_tree.h>
#include "fcl/broadphase/default_broadphase_callbacks.h"
// Distance Request & Result
//#include <fcl/narrowphase/distance_request.h>
//#include <fcl/narrowphase/distance_result.h>

#include "pointcloud_processing.h"
#include <Eigen/Dense>
struct Color
{

    float r, g, b;

    Color(float setR, float setG, float setB)
            : r(setR), g(setG), b(setB)
    {}
};


Box createBox(Eigen::Vector3f uav_point, float width, float length, float height, const float offset[]);//build a bounding box for drone
//when there is a collision, let the safety volume become red in PCL
void CollisionVisualization(pcl::visualization::PCLVisualizer::Ptr& viewer, Box box, Color color,int id);
//safety volume shown in PCL
void UAVBoxVisualization(pcl::visualization::PCLVisualizer::Ptr& viewer, Box box, Color color,int id);


class collision_detection{
public:
    collision_detection(const PtCdtr &cluster, float width, float length, float height);//no required

    collision_detection(Box obj,Box drone);//initialize size of drone and object
    collision_detection(Box drone);//initialize size of drone
    ~collision_detection();

    void update_droneBox(Box drone_in);// update the size of drone
    void update_box(Box obj_in);// update the size of object

    void rough_detector();//rough detection: box and box
    void customize_detector(float angle);//no required
    void lr_detector(int fs_state);//no required
    void lrud_detector(int fs_state);//no required
    void ud_detector(int fs_state);//no required

    void nav_detector(float angle_in);//detection in navigation phase (turn to target)

    void lr_preDetector(int fs_state, float d_s, float minDis);// long-distance pre-detection in left-right direction
    void lrud_preDetector(int fs_state, float d_s);//no required
    void ud_preDetector(int fs_state, float d_s, float minDis);// long-distance pre-detection in up-down direction

    void lr_postDetector(int fs_state, float d_s, float minDis);// post-detection at close range in left-right direction
    void midDetector(float d_s1, float d_s2, float minDis);// mid-detection at close range in left-right direction

    int getFlightAngle_H();//get the flight angle in horizontal plane
    int getFlightAngle_V();//get the flight angle in vertical plane
    float getFlightDistance();//get the flight distance to avoid the obstacle

    float min_distance;
    bool isCollision = true;
    int direct;

private:

    Eigen::Matrix3f setRPY(Eigen::Vector3f rot);//transform the euler angle to rotation matrix

    fcl::CollisionObjectf createCollisionObject(pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_ptr);// build octree

    Box BoundingBox(PtCdtr cluster);

    // Distance Request and Result
    fcl::DistanceRequestf request;
    fcl::DistanceResultf result;
    fcl::CollisionRequestf C_request;
    fcl::CollisionResultf C_result;
    fcl::DefaultDistanceData<float> d_data1;
    fcl::DefaultCollisionData<float> c_data1;

    float cita_h,cita_v;
    int ang_h,ang_v; // flight angel: vertical, horizontal
    Box obj,drone;
    float drone_w, drone_h, drone_l, true_w;
    float detec_l;
    const float drone_l_t = 1.6;
//    const float cameraPosition[3]={0,0.1,-0.2};
    const float cameraPosition[3]={0,0,0};

};
#endif //PCFILTER_COLLISIONDETECTION_H
