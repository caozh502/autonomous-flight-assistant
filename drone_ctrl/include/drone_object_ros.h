#ifndef ARDRONE_ROS_H
#define ARDRONE_ROS_H

#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Joy.h>
#include <dji_sdk/DroneTaskControl.h>
#include <dji_sdk/SDKControlAuthority.h>
#include <djiosdk/dji_version.hpp>
#include <std_msgs/UInt8.h>
#include "dji_sdk/dji_sdk.h"



/**
 * @brief A simple class to send the commands to the drone through 
 * the corresponding topics
 */

class DroneObjectROS{
protected:
    DroneObjectROS(){}
public:
    
    DroneObjectROS(ros::NodeHandle& node){
        initROSVars(node);
    }

    bool isFlying;
    bool isPosctrl;

    uint8_t flight_status = 255;
    uint8_t display_mode  = 255;

    uint8_t joy_cmd_flag;
    float move_speed_limit = 0.7; //in m/s
    
    ros::Publisher pubTakeOff;
    ros::Publisher pubLand;
    ros::Publisher pubCmd;

    ros::ServiceClient drone_task_service;
    ros::ServiceClient sdk_ctrl_authority_service;

    void initROSVars(ros::NodeHandle& node);
    void obtain_control(ros::NodeHandle & node) ;
    void give_up_control(ros::NodeHandle & node) ;
    bool takeoff_land(int task);
    bool monitoredTakeoff();
    void display_mode_callback(const std_msgs::UInt8::ConstPtr& msg);

    void flight_status_callback(const std_msgs::UInt8::ConstPtr& msg);


    void takeOff();
    void land();
    void hover();

    // commands for controling ARDrone
    // pitch_lr = left-right tilt		(-1) to right		(+1)
    // roll_fb = front-back tilt 		(-1) to backwards	(+1)
    // v_du = velocity downwards	(-1) to upwards		(+1)
    // w_lr = angular velocity left (-1) to right		(+1)

    void pitch(float speed);
    void roll(float speed);
    void rise(float speed);
    void yaw(float speed);
    void set_speed_limit(float lim);
    float limit_speed(float v) const;
    void move(float v_horizontal_forward, float v_horizontal_left, float v_vertical_up, float yaw_rate_left) const;


};

#endif // ARDRONE_ROS_H
