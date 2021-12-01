#include "plugin_drone.h"


#include <cmath>
#include <stdlib.h>
#include <iostream>


namespace gazebo {

    DroneSimpleController::DroneSimpleController()
    {
        navi_state = LANDED_MODEL;
        joy_cmd_val.axes = {0,0,0,0,0};
        this->speed_multiplier_roll = 5.;
        this->speed_multiplier_pitch = 15.;
        this->speed_multiplier_yaw = 5.;
        this->speed_multiplier_vz = 5.;


    }

////////////////////////////////////////////////////////////////////////////////
// Destructor
    DroneSimpleController::~DroneSimpleController()
    {
        // Deprecated since Gazebo 8.
        //event::Events::DisconnectWorldUpdateBegin(updateConnection);

        node_handle_->shutdown();
        delete node_handle_;
    }

////////////////////////////////////////////////////////////////////////////////
// Load the controller
    void DroneSimpleController::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {

        if(!ros::isInitialized()){
            ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
                                     << "Load the Gazebo system plugin 'libgazebo_ros_init.so' in the gazebo_ros package)");
        }

        world = _model->GetWorld();
        ROS_INFO("The drone plugin is loading!");

        //load parameters
        cmd_normal_topic_ = "dji_sdk/flight_control_setpoint_generic";
        takeoff_topic_ = "drone/takeoff";
        land_topic_ = "drone/land";
        attitude_topic_ = "dji_sdk/attitude";

        if (!_sdf->HasElement("imuTopic"))
            imu_topic_.clear();
        else
            imu_topic_ = _sdf->GetElement("imuTopic")->Get<std::string>();


        if (!_sdf->HasElement("bodyName"))
        {
            link = _model->GetLink();
            link_name_ = link->GetName();
        }
        else {
            link_name_ = _sdf->GetElement("bodyName")->Get<std::string>();
            link = boost::dynamic_pointer_cast<physics::Link>(world->EntityByName(link_name_));
        }

        if (!link)
        {
            ROS_FATAL("gazebo_ros_baro plugin error: bodyName: %s does not exist\n", link_name_.c_str());
            return;
        }

        if (!_sdf->HasElement("maxForce"))
            max_force_ = -1;
        else
            max_force_ = _sdf->GetElement("maxForce")->Get<double>();


        // get inertia and mass of quadrotor body
        inertia = link->GetInertial()->PrincipalMoments();
        mass = link->GetInertial()->Mass();

        node_handle_ = new ros::NodeHandle;


        // subscribe command: control command
        if (!cmd_normal_topic_.empty())
        {

            ros::SubscribeOptions ops = ros::SubscribeOptions::create<sensor_msgs::Joy>(
                    cmd_normal_topic_, 1,
                    boost::bind(&DroneSimpleController::CmdCallback, this, _1),
                    ros::VoidPtr(), &callback_queue_);
            cmd_subscriber_ = node_handle_->subscribe(ops);

            if( cmd_subscriber_.getTopic() != "")
                ROS_INFO_NAMED("quadrotor_simple_controller", "Using cmd_topic %s.", cmd_normal_topic_.c_str());
            else
                ROS_INFO("cannot find the command topic!");
        }


        // subscribe imu
        if (!imu_topic_.empty())
        {
            ros::SubscribeOptions ops = ros::SubscribeOptions::create<sensor_msgs::Imu>(
                    imu_topic_, 1,
                    boost::bind(&DroneSimpleController::ImuCallback, this, _1),
                    ros::VoidPtr(), &callback_queue_);
            imu_subscriber_ = node_handle_->subscribe(ops);

            if(imu_subscriber_.getTopic() !="")
                ROS_INFO_NAMED("quadrotor_simple_controller", "Using imu information on topic %s as source of orientation and angular velocity.", imu_topic_.c_str());
            else
                ROS_INFO("cannot find the IMU topic!");
        }

        // subscribe command: take off command
        if (!takeoff_topic_.empty())
        {
            ros::SubscribeOptions ops = ros::SubscribeOptions::create<std_msgs::Empty>(
                    takeoff_topic_, 1,
                    boost::bind(&DroneSimpleController::TakeoffCallback, this, _1),
                    ros::VoidPtr(), &callback_queue_);
            takeoff_subscriber_ = node_handle_->subscribe(ops);
            if( takeoff_subscriber_.getTopic() != "")
                ROS_INFO("find the takeoff topic");
            else
                ROS_INFO("cannot find the takeoff topic!");
        }

        // subscribe command: land command
        if (!land_topic_.empty())
        {
            ros::SubscribeOptions ops = ros::SubscribeOptions::create<std_msgs::Empty>(
                    land_topic_, 1,
                    boost::bind(&DroneSimpleController::LandCallback, this, _1),
                    ros::VoidPtr(), &callback_queue_);
            land_subscriber_ = node_handle_->subscribe(ops);
            ROS_INFO("find the land topic");
        }


        if (!attitude_topic_.empty()){
            pub_attitude = node_handle_->advertise<geometry_msgs::QuaternionStamped>("dji_sdk/attitude", 1024);
        }

        pub_gt_vec_ = node_handle_->advertise<geometry_msgs::Twist>("drone/gt_vel", 1024);
        pub_gt_acc_ = node_handle_->advertise<geometry_msgs::Twist>("drone/gt_acc", 1024);


        LoadControllerSettings(_model, _sdf);

        // New Mechanism for Updating every World Cycle
        // Listen to the update event. This event is broadcast every
        // simulation iteration.
        updateConnection = event::Events::ConnectWorldUpdateBegin(
                boost::bind(&DroneSimpleController::Update, this));
        ROS_INFO("plugin_drone load complete");

    }

    void DroneSimpleController::LoadControllerSettings(physics::ModelPtr _model, sdf::ElementPtr _sdf){
        controllers_.roll.Load(_sdf, "rollpitch");
        controllers_.pitch.Load(_sdf, "rollpitch");
        controllers_.yaw.Load(_sdf, "yaw");
        controllers_.velocity_x.Load(_sdf, "velocityXY");
        controllers_.velocity_y.Load(_sdf, "velocityXY");
        controllers_.velocity_z.Load(_sdf, "velocityZ");

        controllers_.pos_x.Load(_sdf, "positionXY");
        controllers_.pos_y.Load(_sdf, "positionXY");
        controllers_.pos_z.Load(_sdf, "positionZ");
        //ROS_INFO_STREAM("plugin_drone LoadControllerSettings complete");

    }

////////////////////////////////////////////////////////////////////////////////
// Callbacks
    void DroneSimpleController::CmdCallback(const sensor_msgs::JoyConstPtr& cmd)
    {
        this->joy_cmd_val = *cmd;
        static common::Time last_sim_time = world->SimTime();
        // Get simulator time
        common::Time cur_sim_time = world->SimTime();
        double dt = (cur_sim_time - last_sim_time).Double();
        // save last time stamp
        last_sim_time = cur_sim_time;
        //ROS_INFO_STREAM("end cmd_callback");
    }

    void DroneSimpleController::ImuCallback(const sensor_msgs::ImuConstPtr& imu)
    {
        //directly read the quternion from the IMU data
        pose.Rot().Set(imu->orientation.w, imu->orientation.x, imu->orientation.y, imu->orientation.z);
        euler = pose.Rot().Euler();
        angular_velocity = pose.Rot().RotateVector(ignition::math::Vector3d(imu->angular_velocity.x, imu->angular_velocity.y, imu->angular_velocity.z));
    }

    void DroneSimpleController::TakeoffCallback(const std_msgs::EmptyConstPtr& msg)
    {
        if(navi_state == LANDED_MODEL)
        {
            navi_state = TAKINGOFF_MODEL;
            m_timeAfterCmd = 0;
            ROS_INFO_STREAM("Quadrotor takes off!!");
        }
    }

    void DroneSimpleController::LandCallback(const std_msgs::EmptyConstPtr& msg)
    {
        if(navi_state == FLYING_MODEL||navi_state == TAKINGOFF_MODEL)
        {
            navi_state = LANDING_MODEL;
            m_timeAfterCmd = 0;
            ROS_INFO_STREAM("Quadrotor lands!!");
        }

    }

////////////////////////////////////////////////////////////////////////////////
// Update the controller
    void DroneSimpleController::Update()
    {
        // ROS_INFO_STREAM("start DroneSimpleController::Update()");
        // Get new commands/state
        callback_queue_.callAvailable();

        // Get simulator time
        common::Time sim_time = world->SimTime();
        double dt = (sim_time - last_time).Double();
        if (dt == 0.0) return;

        UpdateState(dt);
        UpdateDynamics(dt);

        // save last time stamp
        last_time = sim_time;
    }

    void DroneSimpleController::UpdateState(double dt){
        if(navi_state == TAKINGOFF_MODEL){
            m_timeAfterCmd += dt;
            if (m_timeAfterCmd > 0.5){
                navi_state = FLYING_MODEL;
                std::cout << "Entering flying model!" << std::endl;
            }
        }else if(navi_state == LANDING_MODEL){
            m_timeAfterCmd += dt;
            if(m_timeAfterCmd > 1.0){
                navi_state = LANDED_MODEL;
                std::cout << "Landed!" <<std::endl;
            }
        }else
            m_timeAfterCmd = 0;
    }

    void DroneSimpleController::UpdateDynamics(double dt){
        // ROS_INFO_STREAM("start UpdateDynamics");

        ignition::math::Vector3d force, torque;

        // Get Pose/Orientation from Gazebo (if no state subscriber is active)
        //  if (imu_subscriber_.getTopic()=="")
        {
            pose = link->WorldPose();
            angular_velocity = link->WorldAngularVel();
            euler = pose.Rot().Euler();
        }
        // if (state_topic_.empty())
        {
            acceleration = (link->WorldLinearVel() - velocity) / dt;
            velocity = link->WorldLinearVel();
        }

        //publish the ground truth pose of the drone to the ROS topic
        geometry_msgs::QuaternionStamped attitude_q;
        attitude_q.header.stamp    = ros::Time::now();
        // TODO: check for sign errors!
        attitude_q.header.frame_id = "body_FLU";
        attitude_q.quaternion.w = pose.Rot().W();
        attitude_q.quaternion.x = -pose.Rot().X();
        attitude_q.quaternion.y = -pose.Rot().Y();
        attitude_q.quaternion.z = pose.Rot().Z();
        pub_attitude.publish(attitude_q);

        //convert the acceleration and velocity into the body frame
        ignition::math::Vector3d body_vel = pose.Rot().RotateVector(velocity);
        ignition::math::Vector3d body_acc = pose.Rot().RotateVector(acceleration);

        //publish the velocity
        geometry_msgs::Twist tw;
        tw.linear.x = body_vel.X();
        tw.linear.y = body_vel.Y();
        tw.linear.z = body_vel.Z();
        pub_gt_vec_.publish(tw);

        //publish the acceleration
        tw.linear.x = body_acc.X();
        tw.linear.y = body_acc.Y();
        tw.linear.z = body_acc.Z();
        pub_gt_acc_.publish(tw);


        ignition::math::Vector3d poschange = pose.Pos() - position;
        position = pose.Pos();


        // Get gravity
        ignition::math::Vector3d gravity_body = pose.Rot().RotateVector(world->Gravity());
        double gravity = gravity_body.Length();
        double load_factor = gravity * gravity / world->Gravity().Dot(gravity_body);  // Get gravity

        // Rotate vectors to coordinate frames relevant for control
        ignition::math::Quaterniond heading_quaternion(cos(euler.Z()/2),0,0,sin(euler.Z()/2));
        ignition::math::Vector3d velocity_xy = heading_quaternion.RotateVectorReverse(velocity);
        ignition::math::Vector3d acceleration_xy = heading_quaternion.RotateVectorReverse(acceleration);
        ignition::math::Vector3d angular_velocity_body = pose.Rot().RotateVectorReverse(angular_velocity);

        // update controllers
        force.Set(0.0, 0.0, 0.0);
        torque.Set(0.0, 0.0, 0.0);

        //normal control
        if( navi_state == FLYING_MODEL )//&& cmd_val.linear.x >= 0 && cmd_val.linear.y >= 0)
        {
            //hovering
            //ROS_INFO_STREAM("parsing joy message");
            double pitch_command =  controllers_.velocity_x.update(joy_cmd_val.axes[0]*this->speed_multiplier_pitch, velocity_xy.X(), acceleration_xy.X(), dt) / gravity;
            double roll_command  = -controllers_.velocity_y.update(joy_cmd_val.axes[1]*this->speed_multiplier_roll, velocity_xy.Y(), acceleration_xy.Y(), dt) / gravity;
            torque.X() = inertia.X() *  controllers_.roll.update(roll_command, euler.X(), angular_velocity_body.X(), dt);
            torque.Y() = inertia.Y() *  controllers_.pitch.update(pitch_command, euler.Y(), angular_velocity_body.Y(), dt);
        }

        torque.Z() = inertia.Z() *  controllers_.yaw.update(joy_cmd_val.axes[3]*this->speed_multiplier_yaw, angular_velocity.Z(), 0, dt);
        force.Z()  = mass      * (controllers_.velocity_z.update(joy_cmd_val.axes[2]*this->speed_multiplier_vz,  velocity.Z(), acceleration.Z(), dt) + load_factor * gravity);
        //ROS_INFO_STREAM("parsing joy message complete");

        if (max_force_ > 0.0 && force.Z() > max_force_) force.Z() = max_force_;
        if (force.Z() < 0.0) force.Z() = 0.0;


        // process robot state information
        if(navi_state == LANDED_MODEL)
        {

        }
        else if(navi_state == FLYING_MODEL)
        {
            link->AddRelativeForce(force);
            link->AddRelativeTorque(torque);
        }
        else if(navi_state == TAKINGOFF_MODEL)
        {
            link->AddRelativeForce(force*1.5);
            link->AddRelativeTorque(torque*1.5);
        }
        else if(navi_state == LANDING_MODEL)
        {
            link->AddRelativeForce(force*0.8);
            link->AddRelativeTorque(torque*0.8);
        }

    }
// Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(DroneSimpleController)

} // namespace gazebo
