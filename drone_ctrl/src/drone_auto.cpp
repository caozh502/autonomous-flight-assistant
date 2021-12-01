//Initializes the autonomous drone control. Starts modules. No Processing here

#include <sstream>
#include "ros/ros.h"
#include "../include/drone_object_ros.h"
#include "modules/environment_processing.h"
#include "modules/drone_decision.h"

const int speed = 1;

int main(int argc, char **argv) {
    ros::init(argc, argv, "drone_auto");
    ros::NodeHandle flight_cmd_nh;

    DroneObjectROS drone(flight_cmd_nh);
    ros::Duration(0.1).sleep(); // wait for dji rosnode
    drone.monitoredTakeoff();
    drone.takeOff();

    // start modules
    environment_processing env_proc;
    drone_decision drone_dec(drone);

    ros::Rate loop_rate(30);
    while (ros::ok()) {
        // do something regularly
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
