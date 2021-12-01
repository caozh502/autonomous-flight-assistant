#ifndef SRC_DRONE_DECISION_H
#define SRC_DRONE_DECISION_H
#include "abstract_module.h"

class drone_decision : public abstract_module {
    ros::NodeHandle nh;
    DroneObjectROS my_drone;
    ros::Subscriber sub;

    void env_proc_callback(const std_msgs::String::ConstPtr& msg);

public:
    drone_decision(DroneObjectROS &drone);
    ~drone_decision();
};


#endif //SRC_DRONE_DECISION_H
