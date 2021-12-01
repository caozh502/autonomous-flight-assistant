#include <sstream>
#include "ros/ros.h"
#include "../../include/drone_object_ros.h"
#include "std_msgs/String.h"


#ifndef SRC_ABSTRACT_MODULE_H
#define SRC_ABSTRACT_MODULE_H


class abstract_module {

public:
    ros::NodeHandle nh;
    abstract_module();

};


#endif //SRC_ABSTRACT_MODULE_H
