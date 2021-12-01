#ifndef ARDRONE_SIMPLE_CONTROL_H
#define ARDRONE_SIMPLE_CONTROL_H

#include "gazebo/common/Events.hh"
#include "gazebo/gazebo.hh"
#include "gazebo/physics/physics.hh"
#include <ignition/math.hh>
#include <ros/callback_queue.h>
#include <ros/ros.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/Imu.h>

#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>

#include "pid_controller.h"

#define LANDED_MODEL 0
#define FLYING_MODEL 1
#define TAKINGOFF_MODEL 2
#define LANDING_MODEL 3

#define EPS 1E-6
namespace gazebo {
class DroneSimpleController : public ModelPlugin {
public:
  DroneSimpleController();
  virtual ~DroneSimpleController();

protected:
  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  virtual void LoadControllerSettings(physics::ModelPtr _model,
                                      sdf::ElementPtr _sdf);
  virtual void Update();
  void UpdateDynamics(double dt);
  void UpdateState(double dt);

private:
    float speed_multiplier_roll, speed_multiplier_pitch, speed_multiplier_yaw, speed_multiplier_vz;
  double m_timeAfterCmd;
  unsigned int navi_state;

  /// \brief The parent World
  physics::WorldPtr world;

  /// \brief The link referred to by this plugin
  physics::LinkPtr link;

  ros::NodeHandle *node_handle_;
  ros::CallbackQueue callback_queue_;
  ros::Subscriber cmd_subscriber_;
  ros::Subscriber land_subscriber_;
  ros::Subscriber imu_subscriber_;

  // extra robot control command
  ros::Subscriber takeoff_subscriber_;

  ros::Publisher pub_attitude; // for publishing ground truth pose
  ros::Publisher pub_gt_vec_;  // ground truth velocity in the body frame
  ros::Publisher pub_gt_acc_;  // ground truth acceleration in the body frame

  sensor_msgs::Joy joy_cmd_val;

  // callback functions for subscribers
  void CmdCallback(const sensor_msgs::JoyConstPtr &);
  void ImuCallback(const sensor_msgs::ImuConstPtr &);
  void TakeoffCallback(const std_msgs::EmptyConstPtr &);
  void LandCallback(const std_msgs::EmptyConstPtr &);

  ros::Time state_stamp;
  ignition::math::Pose3d pose;
  ignition::math::Vector3d euler, velocity, acceleration, angular_velocity,
      position;

  std::string link_name_;
  std::string cmd_normal_topic_;
  std::string imu_topic_;
  std::string takeoff_topic_;
  std::string land_topic_;
  std::string attitude_topic_; // ground truth

  double max_force_;

  struct Controllers {
    PIDController roll;
    PIDController pitch;
    PIDController yaw;
    PIDController velocity_x;
    PIDController velocity_y;
    PIDController velocity_z;
    PIDController pos_x;
    PIDController pos_y;
    PIDController pos_z;
  } controllers_;

  ignition::math::Vector3d inertia;
  double mass;

  /// \brief save last_time
  common::Time last_time;

  // Pointer to the update event connection
  event::ConnectionPtr updateConnection;
};

} // namespace gazebo

#endif // ARDRONE_SIMPLE_CONTROL_H
