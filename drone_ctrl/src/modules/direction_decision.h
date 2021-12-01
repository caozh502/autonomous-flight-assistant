//
// Created by caleb on 17.05.21.
//

#ifndef DRONE_CTRL_DIRECTION_DECISION_H
#define DRONE_CTRL_DIRECTION_DECISION_H


#include <cmath>
#include <string>
#include <iomanip>
#include <sstream>
#include <tf/tf.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Float32MultiArray.h>
#include "std_msgs/String.h"
#include <sensor_msgs/Joy.h>
#include "pointcloud_processing.h"
#include "CollisionDetection.h"
#include <pcl/pcl_macros.h>

constexpr float MODE_P = 10000.0;  // Value of mode_p normally transmitted by the remote
constexpr float MODE_A = 0.0;  // Value of mode_a normally transmitted by the remote
constexpr float MODE_F = -10000.0;  // Value of mode_f normally transmitted by the remote
struct mission_info // mission: execute the decision
{
    int direct = 0; // flight direct: up-down, left-right
    int angleH = 0; // angle in horizontal plane to flight
    int angleV = 0; // angle in vertical plane to flight
    float detecLength = 0; //length of safety volume (not required)
};

enum decision { l30 = 0, l60, r30, r60, u30, u60, d30, d60 }; //array vote for decision

class direction_decision : public abstract_module {
    ros::NodeHandle nh;
    ros::Publisher disAngPub;
    ros::Publisher directPub;
    ros::Publisher flightStatePub;
    ros::Subscriber gpsSub;
    ros::Subscriber attiSub;
    ros::Subscriber tarGpsSub;
    ros::Subscriber rcSub;
    DroneObjectROS my_drone;
public:

    direction_decision(DroneObjectROS &drone, int timeRate_in);//initialize

    ~direction_decision();

    void motionStep();// main program of direction decision
    void setTarget(float x, float y, float z); //set target poisition


private:
    ///parameters
    float target_x = 0;
    float target_y = 0;
    float target_z = 0;
    int takeoff_state = 2;
    float drone_x = 0;
    float drone_y = 0;
    float drone_z = 0;
    float drone_yaw = 0;
    const float cameraPosition[3]={0,0,0};//{0,0.1,-0.2};
    int timeRate;

    //speed value for sim
//    const float C_speed = 2.8;
//    float speed = 0.04;
//    const float v_min = 0.04;
//    const float v_max = 0.06;
//    float speed_ro = 0.05;
//    const float speed_emer = 0.01;

    //speed value for real
    const float C_speed = 2.8;
    float speed = 0.3;
    const float v_min = 0.3;
    const float v_max = 0.5;
    float speed_ro = 0.3;
    const float speed_emer = 0.15;
    
    int turn_state = 0;// 0: allow to turn to target; 1: no need to turn or obstacle found
    float rotate_counter = 0;
    int direct_state = 0; // fs_pre valid
    int obs_state = 0;// 0: no obstalce; 1: obstacle found
    int noObs_counter = 0;// if greater than noObs_thres: safe flight confirm
    const int noObs_thres = 10;
    const float d_safe = 2.5;
    float d_preEnd = 0;
    float d_preBegin = 0;


    int fs_counter = 0; // counter the number of "no FreeSpace"
    const int noFS_thres = 5; // fs_counter > noFS_thres ---> safe flying (avoidance phase)
    int vote_state = 0; // 1: detector find no obstacle in path; 0: find obstacle in path
    const int vote_thres = 10; // maximal vote value is reliable, when it is greater than vote_thres.

    const std::vector<Color> colors = {Color(1, 0, 0), Color(0, 1, 0), Color(0, 0, 1)};
    sensor_msgs::NavSatFix current_gps;
    sensor_msgs::NavSatFix init_gps;
    geometry_msgs::Quaternion current_atti;
    mission_info info;
    mission_info info_viewer;
    Box droneBox, droneBox_t;
    std::vector<float> rc_info;
    bool assistant_active;

    int vote[8]= {0};
    float minDistance_pre = 0;

    const float y_max = 5;
    const float y_min = -5;
    const float x_max = 10;
    const float x_min = -10;

    void mission(int direct, int angleH, int angleV);// execute the final decision

//    void motionCtrl(Box box,float width, float length, float height);

    void drone_forward();//(not required)

    void drone_stop(float time);//(not required)

    void drone_rotation(float time);//(not required)

    void drone_updown(float time);//(not required)



    void setDronePos(float x, float y, float z, float yaw); // set current drone position with GPS info

    float getAimAngle();//get the angle to rotate (navigation)

    float judgeTurnDirection(const float *pos1, const float *pos2);//judge the direction of the target point in current heading direction: left of right

    float getAngle(const float *v1, const float *v2);//calculate the included angle of two vectors

    float getLength(const float *v); //get length of a vector

    geometry_msgs::Vector3 toEulerAngle(geometry_msgs::Quaternion quat);//transform quaternion to euler angle

    void attitude_callback(const geometry_msgs::QuaternionStamped::ConstPtr &msg);//get attitude info

    void target_gpsCallback(const std_msgs::Float32MultiArray::ConstPtr& msg);// get input of target point position

    void current_gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg);//get current gps info

    void reset();//(not required)

    void receive_rc_info(const sensor_msgs::JoyConstPtr &joy_msg);//get remote control info
//    void publish_flightState(std::string str);
};

void renderBox(pcl::visualization::PCLVisualizer::Ptr& viewer, Box box, int id);//visualize the bounding box in PCL

Box BoundingBox(PtCdtr cluster);//build a bounding box for a cluster


#endif //DRONE_CTRL_DIRECTION_DECISION_H
