#ifndef QT_NODE_H
#define QT_NODE_H

#include <std_msgs/String.h>
#include "std_msgs/Float32MultiArray.h"
#include <sensor_msgs/NavSatFix.h>
#include <ros/ros.h>
#include <QThread>
#include <pcl/visualization/pcl_visualizer.h>


#include "modules/pointcloud_processing.h"
#include "modules/direction_decision.h"
#include "modules/img_process.h"

class QNode : public QThread
{
    Q_OBJECT
public:
    QNode(int argc, char** argv);
    virtual ~QNode();
    void run();
    bool init();
    void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg);
    void infoCallback(const std_msgs::Float32MultiArray::ConstPtr& msg);
    void flightStateCallback(const std_msgs::String::ConstPtr& msg);

    float current_lo;
    float current_la;
    float distance;
    float angle;
    float speed;
    float minDis;
    bool control_state;
    std::string flight_state;
private:
//    ros::Publisher chatter_publisher;
    ros::Subscriber disAngSub;
    ros::Subscriber gpsSub;
    ros::Subscriber flightStateSub;
//    sensor_msgs::NavSatFix current_gps;
    std_msgs::Float32MultiArray target_gps;


    int init_argc;
    char** init_argv;

};

#endif // QT_NODE_H
