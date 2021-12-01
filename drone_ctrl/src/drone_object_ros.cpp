#include "../include/drone_object_ros.h"
#include "../include/dji_sdk_enums.h"


void DroneObjectROS::initROSVars(ros::NodeHandle& node){
    isPosctrl = false;

    joy_cmd_flag = (DJISDK::HORIZONTAL_VELOCITY |
                    DJISDK::VERTICAL_VELOCITY   |
                    DJISDK::YAW_RATE            |
                    DJISDK::HORIZONTAL_BODY     |
                    DJISDK::STABLE_ENABLE);

    pubTakeOff = node.advertise<std_msgs::Empty>("/drone/takeoff",1024);
    pubLand = node.advertise<std_msgs::Empty>("/drone/land",1024);
    pubCmd = node.advertise<sensor_msgs::Joy>("/dji_sdk/flight_control_setpoint_generic",1024);


//    sdk_ctrl_authority_service = node.serviceClient<dji_sdk::SDKControlAuthority> ("dji_sdk/sdk_control_authority");
    drone_task_service         = node.serviceClient<dji_sdk::DroneTaskControl>("dji_sdk/drone_task_control");

//    dji_sdk::SDKControlAuthority authority;
//    authority.request.control_enable=1;
//    sdk_ctrl_authority_service.call(authority);

//    if(!authority.response.result)
//    {
//        ROS_ERROR("DJI OSDK obtain control failed! (ignore this when SIL-simulating)");
//    }
//    else{
//        ROS_INFO("DJI OSDK obtain control success!");
//    }
}
void DroneObjectROS::give_up_control(ros::NodeHandle &node)  {
    sdk_ctrl_authority_service = node.serviceClient < dji_sdk::SDKControlAuthority > ("dji_sdk/sdk_control_authority");
    dji_sdk::SDKControlAuthority authority;
    authority.request.control_enable = 0;
    sdk_ctrl_authority_service.call(authority);


    if (!authority.response.result) {
        ROS_INFO("Can't give up DJI OSDK authority (ignore this when SIL-simulating)");
    } else {
        ROS_INFO("DJI OSDK give up control success!");
    }
}
void DroneObjectROS::obtain_control(ros::NodeHandle & node) {

    sdk_ctrl_authority_service = node.serviceClient < dji_sdk::SDKControlAuthority > ("dji_sdk/sdk_control_authority");
    dji_sdk::SDKControlAuthority authority;
    authority.request.control_enable = 1;
    sdk_ctrl_authority_service.call(authority);

    if (!authority.response.result) {
        ROS_INFO("Can't obtain DJI OSDK authority (ignore this when SIL-simulating)");
    } else {
        ROS_INFO("DJI OSDK obtain control success!");
    }
}
void DroneObjectROS::takeOff(){

    pubTakeOff.publish(std_msgs::Empty());
    ROS_INFO("Taking Off...");
}

void DroneObjectROS::land(){

    pubLand.publish(std_msgs::Empty());
    ROS_INFO("Landing...");
}


void DroneObjectROS::hover(){
    sensor_msgs::Joy joy_msg;
    joy_msg.axes.push_back(0); //v_horizontal_forward
    joy_msg.axes.push_back(0); //v_horizontal_sideways
    joy_msg.axes.push_back(0); //v_vertical
    joy_msg.axes.push_back(0); //yaw_rate
    joy_msg.axes.push_back(joy_cmd_flag); //flag

    pubCmd.publish(joy_msg);

    ROS_INFO("sent Hovering...");

}

void DroneObjectROS::pitch(float speed){
    sensor_msgs::Joy joy_msg;

    //range: -1,1

    joy_msg.axes.push_back(speed); //v_horizontal_forward
    joy_msg.axes.push_back(0); //v_horizontal_sideways
    joy_msg.axes.push_back(0); //v_vertical
    joy_msg.axes.push_back(0); //yaw_rate
    joy_msg.axes.push_back(joy_cmd_flag); //flag

    pubCmd.publish(joy_msg);
    ROS_INFO("sent Pitching...");

}

void DroneObjectROS::roll(float speed){
    sensor_msgs::Joy joy_msg;

    //range: -1,1
    joy_msg.axes.push_back(0); //v_horizontal_forward
    joy_msg.axes.push_back(speed); //v_horizontal_sideways
    joy_msg.axes.push_back(0); //v_vertical
    joy_msg.axes.push_back(0); //yaw_rate
    joy_msg.axes.push_back(joy_cmd_flag); //flag

    pubCmd.publish(joy_msg);
    ROS_INFO("sent Rolling...");
}

void DroneObjectROS::rise(float speed){
    sensor_msgs::Joy joy_msg;

    joy_msg.axes.push_back(0); //v_horizontal_forward
    joy_msg.axes.push_back(0); //v_horizontal_sideways
    joy_msg.axes.push_back(speed); //v_vertical
    joy_msg.axes.push_back(0); //yaw_rate
    joy_msg.axes.push_back(joy_cmd_flag); //flag
    pubCmd.publish(joy_msg);

    ROS_INFO("sent Rising/Down...");
}

void DroneObjectROS::yaw(float speed){
    sensor_msgs::Joy joy_msg;

    joy_msg.axes.push_back(0); //v_horizontal_forward
    joy_msg.axes.push_back(0); //v_horizontal_sideways
    joy_msg.axes.push_back(0); //v_vertical
    joy_msg.axes.push_back(speed); //yaw_rate
    joy_msg.axes.push_back(joy_cmd_flag); //flag
    pubCmd.publish(joy_msg);

    ROS_INFO("sent Turning head...");
}

void DroneObjectROS::move(float v_horizontal_forward, float v_horizontal_left, float v_vertical_up, float yaw_rate_left) const {
    sensor_msgs::Joy joy_msg;
    joy_msg.axes = {limit_speed(v_horizontal_forward),
                    limit_speed(v_horizontal_left),
                    limit_speed(v_vertical_up),
                    limit_speed(yaw_rate_left),
                    static_cast<float>(joy_cmd_flag)};

    pubCmd.publish(joy_msg);
}

float DroneObjectROS::limit_speed(float v) const {
    float limited_speed; // if v is NaN, default to 0.0

    if (v < -this -> move_speed_limit){
        limited_speed = -this -> move_speed_limit;
    }
    else if (v > +this -> move_speed_limit){
        limited_speed = +this -> move_speed_limit;
    }
    else if (std::isnan(v)) {
        limited_speed = 0.0f;
        ROS_INFO("Nan Recieved in drone_interface, defaulting to 0");
    }

    else{
        limited_speed = v;
    }
    return limited_speed;
}

void DroneObjectROS::set_speed_limit(float lim) {
    this -> move_speed_limit = lim;
}

bool DroneObjectROS::takeoff_land(int task)
{
    dji_sdk::DroneTaskControl droneTaskControl;

    droneTaskControl.request.task = task;

    this->drone_task_service.call(droneTaskControl);

    if(!droneTaskControl.response.result)
    {
        ROS_ERROR("DJI OSDK takeoff_land fail (ignore this in SIL simulation");
        return false;
    }

    return true;
}


bool DroneObjectROS::monitoredTakeoff()
{
    ros::Time start_time = ros::Time::now();

    if(!takeoff_land(dji_sdk::DroneTaskControl::Request::TASK_TAKEOFF)) {
        return false;
    }

    ros::Duration(0.01).sleep();
    ros::spinOnce();

    // Step 1.1: Spin the motor
    while (flight_status != DJISDK::FlightStatus::STATUS_ON_GROUND &&
           display_mode != DJISDK::DisplayMode::MODE_ENGINE_START &&
           ros::Time::now() - start_time < ros::Duration(5)) {
        ros::Duration(0.01).sleep();
        ros::spinOnce();
    }

    if(ros::Time::now() - start_time > ros::Duration(5)) {
        ROS_ERROR("Takeoff failed. Motors are not spinnning.");
        return false;
    }
    else {
        start_time = ros::Time::now();
        ROS_INFO("Motor Spinning ...");
        ros::spinOnce();
    }


    // Step 1.2: Get in to the air
    while (flight_status != DJISDK::FlightStatus::STATUS_IN_AIR &&
           (display_mode != DJISDK::DisplayMode::MODE_ASSISTED_TAKEOFF || display_mode != DJISDK::DisplayMode::MODE_AUTO_TAKEOFF) &&
           ros::Time::now() - start_time < ros::Duration(20)) {
        ros::Duration(0.01).sleep();
        ros::spinOnce();
    }

    if(ros::Time::now() - start_time > ros::Duration(20)) {
        ROS_ERROR("Takeoff failed. Aircraft is still on the ground, but the motors are spinning.");
        return false;
    }
    else {
        start_time = ros::Time::now();
        ROS_INFO("Ascending...");
        ros::spinOnce();
    }

    // Final check: Finished takeoff
    while ( (display_mode == DJISDK::DisplayMode::MODE_ASSISTED_TAKEOFF || display_mode == DJISDK::DisplayMode::MODE_AUTO_TAKEOFF) &&
            ros::Time::now() - start_time < ros::Duration(20)) {
        ros::Duration(0.01).sleep();
        ros::spinOnce();
    }

    if ( display_mode != DJISDK::DisplayMode::MODE_P_GPS || display_mode != DJISDK::DisplayMode::MODE_ATTITUDE)
    {
        ROS_INFO("Successful takeoff!");
        start_time = ros::Time::now();
    }
    else
    {
        ROS_ERROR("Takeoff finished, but the aircraft is in an unexpected mode. Please connect DJI GO.");
        return false;
    }

    return true;
}

void DroneObjectROS::flight_status_callback(const std_msgs::UInt8::ConstPtr& msg)
{
    this->flight_status = msg->data;
}

void DroneObjectROS::display_mode_callback(const std_msgs::UInt8::ConstPtr& msg)
{
    this->display_mode = msg->data;
}
