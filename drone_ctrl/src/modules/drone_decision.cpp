// Reacts to incoming preprocessed information. Sends flight control messages

#include "drone_decision.h"

drone_decision::drone_decision(DroneObjectROS &drone) : my_drone(drone), nh() {
    sub = nh.subscribe("/drone/env_info",1000,
                                       &drone_decision::env_proc_callback, this);
}

drone_decision::~drone_decision(){
    ROS_INFO_STREAM("drone decision exits");
}


void drone_decision::env_proc_callback(const std_msgs::String::ConstPtr& msg) {
    ROS_INFO_STREAM("I heard:" << msg->data);

    if (msg->data == "dark"){
        my_drone.rise(0.1);
        ROS_INFO_STREAM("too dark, going up");

    }
    if (msg->data == "bright"){
        my_drone.rise(-0.1);
        ROS_INFO_STREAM("too bright, going down");
    }
    else{
        ROS_INFO_STREAM("Command not reckognized");

    }

}
