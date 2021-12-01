//
// Created by caleb on 15.07.21.
//

#include "qnode.h"
QNode::QNode(int argc, char** argv )
    : init_argc(argc), init_argv(argv)
{}

QNode::~QNode()
{
    if(ros::isStarted())
    {
        ros::shutdown();
        ros::waitForShutdown();
    }
    wait();
}

bool QNode::init()
{
    ros::init(init_argc,init_argv, "drone_Pathfinder");
    ros::start();
    ros::NodeHandle nd;
    gpsSub = nd.subscribe("dji_sdk/gps_position", 1, &QNode::gpsCallback,this);
    disAngSub = nd.subscribe("distance_angle2ro",1,&QNode::infoCallback,this);
    flightStateSub = nd.subscribe("flight_state",1,&QNode::flightStateCallback,this);
  start();
  return true;
}

void QNode::infoCallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
  distance = msg->data.at(0);
  angle = msg->data.at(1);
  speed = msg->data.at(2);
  minDis = msg->data.at(3);
  control_state = msg->data.at(4);
}
void QNode::gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
   current_la = msg->latitude;
   current_lo = msg->longitude;
}
void QNode::flightStateCallback(const std_msgs::String::ConstPtr& msg)
{
    flight_state = msg->data;
}

void QNode::run()
{

    ros::NodeHandle flight_cmd_nh;
    DroneObjectROS drone(flight_cmd_nh);
    ros::Duration(0.1).sleep(); // wait for dji rosNode

    // start modules
    boost::shared_ptr<pcl::visualization::PCLVisualizer> pc_viewer(new pcl::visualization::PCLVisualizer("PointCloud view"));
    pc_viewer->setCameraPosition(-40, -30, -20, 0, 0, 15,1,-7,-1);
    pc_viewer->addCoordinateSystem (1.0);
//    pc_viewer->setBackgroundColor(0.2,0.2,0.2);
    pc_viewer->setBackgroundColor(0,0,0);


    int timeRate =10;
    pointcloud_processing pc_proc(timeRate,pc_viewer);
    direction_decision dir_dec(drone,timeRate);
//    img_process img_proc;
    //set Target point
    dir_dec.setTarget(113.003,22.003,0);


    ros::Rate loop_rate(timeRate);

    while(ros::ok()) {//&&!pc_viewer->wasStopped()
        pc_viewer->removeAllPointClouds();
        pc_viewer->removeAllShapes();

        pc_proc.processStep();
        dir_dec.motionStep();
        pc_proc.publish_decide();
        ros::spinOnce();
        loop_rate.sleep();
        pc_viewer->spinOnce();
    }

//    img_proc.destroywindow();
    std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;

}
