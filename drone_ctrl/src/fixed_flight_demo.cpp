#include "ros/ros.h"
#include "../include/drone_object_ros.h"
#include <sstream>

//模拟环境转指定角度：v=speed, t=phi*pi/180/(speed*5)
//模拟环境飞行指定距离：v=speed, t=distance*(1/(speed*15)) (可能）
//v=speed, t=distance/(speed*5)) (未验证）
#define DEG2RAD(x) ((x)*0.017453293)
const float speed = 0.06;
const float pi = 3.14159265;
void fixed_flight_demo(DroneObjectROS &drone) {
    drone.monitoredTakeoff();
    drone.takeOff();


//    drone.yaw(speed);
//    ros::Duration(6.28).sleep();// 1 round
    drone.hover();
    ros::Duration(2.0).sleep();

    drone.move(speed, 0, -speed*tan(DEG2RAD(-60))*2.7, 0);//up
    ros::Duration(0.1).sleep();
    drone.move(speed, 0, -speed*tan(DEG2RAD(-60))*2.7, 0);
    ros::Duration(3).sleep();
    drone.move(speed, 0, -speed*tan(DEG2RAD(60))*2.7, 0);//down
    ros::Duration(0.1).sleep();
    drone.move(speed, 0, -speed*tan(DEG2RAD(60))*2.7, 0);
    ros::Duration(3).sleep();
    drone.hover();
//    ros::Duration(2).sleep();
//    drone.move(0, 0, 0, 0.5);
//    ros::Duration(2).sleep();
//    drone.move(0, 0, 0, 0.5);
//    ros::Duration(2).sleep();
//    drone.move(0, 0, 0, 0.5);
//    ros::Duration(2).sleep();
//    drone.move(0, 0, 0, 0.5);
//    ros::Duration(2).sleep();

//    drone.hover();
//    ros::Duration(3).sleep();
//    drone.land();
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "fixed_flight_demo");
    ros::NodeHandle node;
    DroneObjectROS drone(node);
    ros::Duration(1.0).sleep(); // wait for receiving rosnode

    fixed_flight_demo(drone);

    ros::Rate loop_rate(1); // rate in Hz
    while (ros::ok()) {

        ROS_INFO_STREAM("do something regularly");
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
