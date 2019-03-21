#include "avc_vehicle.h"

#define PI 3.14159

using namespace avc_sim;

AVC_vehicle::AVC_vehicle() { init(); }
AVC_vehicle::~AVC_vehicle() {}
bool AVC_vehicle::init() {
  // initialize ROS parameter

  nh_.param("wheel_front_left_joint_name", joint_states_name_[LEFT],
            std::string("left_front_wheel_joint"));
  nh_.param("wheel_front_right_joint_name", joint_states_name_[RIGHT],
            std::string("right_front_wheel_joint"));
  nh_.param("wheel_back_left_joint_name", joint_states_name_[B_LEFT],
            std::string("left_back_wheel_joint"));
  nh_.param("wheel_back_right_joint_name", joint_states_name_[B_RIGHT],
            std::string("right_back_wheel_joint"));

  nh_.param("axil_front_right_joint_name", joint_states_name_[4],
            std::string("right_front_axle_joint"));
  nh_.param("axil_front_left_joint_name", joint_states_name_[5],
            std::string("left_front_axle_joint"));
  nh_.param("axil_back_right_joint_name", joint_states_name_[6],
            std::string("right_back_axle_joint"));
  nh_.param("axil_back_left_joint_name", joint_states_name_[7],
            std::string("left_back_axle_joint"));


  nh_.param("joint_states_frame", joint_states_.header.frame_id,
            std::string("base_footprint"));

  // initialize variables
  wheel_speed_cmd_[LEFT] = 0.0;
  wheel_speed_cmd_[RIGHT] = 0.0;
  last_position_[LEFT] = 0.0;
  last_position_[RIGHT] = 0.0;
  last_velocity_[LEFT] = 0.0;
  last_velocity_[RIGHT] = 0.0;


  last_position_[B_LEFT] = 0.0;
  last_position_[B_RIGHT] = 0.0;
  last_velocity_[B_LEFT] = 0.0;
  last_velocity_[B_RIGHT] = 0.0;

  last_position_[4] = 0.0;
  last_position_[5] = 0.0;
  last_position_[6] = 0.0;
  last_position_[7] = 0.0;

  joint_states_.name.push_back(joint_states_name_[LEFT]);
  joint_states_.name.push_back(joint_states_name_[RIGHT]);
  joint_states_.name.push_back(joint_states_name_[B_LEFT]);
  joint_states_.name.push_back(joint_states_name_[B_RIGHT]);
  joint_states_.name.push_back(joint_states_name_[4]);
  joint_states_.name.push_back(joint_states_name_[5]);
  joint_states_.name.push_back(joint_states_name_[6]);
  joint_states_.name.push_back(joint_states_name_[7]);

  joint_states_.position.resize(8, 0.0);
  joint_states_.velocity.resize(4, 0.0);
  joint_states_.effort.resize(4, 0.0);

  // initialize publishers
  joint_states_pub_ =
      nh_.advertise<sensor_msgs::JointState>("joint_states", 100);
  // initialize subscriber
  odom_sub_ = nh_.subscribe("odom", 100, &AVC_vehicle::odometryCallback, this);

  prev_update_time_ = ros::Time::now();
  return true;
}
bool AVC_vehicle::update() {
  ros::Time time_now = ros::Time::now();
  ros::Duration step_time = time_now - prev_update_time_;
  prev_update_time_ = time_now;

  // joint_states
  updateJoint(step_time);
  joint_states_.header.stamp = time_now;
  joint_states_pub_.publish(joint_states_);

  return true;
}

void AVC_vehicle::updateJoint(ros::Duration diff_time) {
  double wheel_l, wheel_r, wheel_rot;  // rotation value of wheel [rad]
  double v[2], w[2];

  wheel_l = wheel_r = 0.0;

  v[LEFT] = wheel_speed_cmd_[LEFT];
  w[LEFT] = v[LEFT] / WHEEL_RADIUS;  // w = v / r
  v[RIGHT] = wheel_speed_cmd_[LEFT];
  w[RIGHT] = v[RIGHT] / WHEEL_RADIUS;

  last_velocity_[LEFT] = w[LEFT];
  last_velocity_[RIGHT] = w[RIGHT];
  last_velocity_[B_LEFT] = w[LEFT];
  last_velocity_[B_RIGHT] = w[RIGHT];

  wheel_l = w[LEFT] * diff_time.toSec();
  wheel_r = w[RIGHT] * diff_time.toSec();

  wheel_rot = wheel_speed_cmd_[RIGHT];

  last_position_[LEFT] += wheel_l;
  last_position_[RIGHT] += wheel_r;
  last_position_[B_LEFT] += wheel_l;
  last_position_[B_RIGHT] += wheel_r;

  last_position_[4] = wheel_rot;

  if(last_position_[4] > PI/6){
    last_position_[4] = PI/6;
  } else if(last_position_[4] < -PI/6){
    last_position_[4] = -PI/6;
  }

  joint_states_.position[LEFT] = last_position_[LEFT];
  joint_states_.position[RIGHT] = last_position_[RIGHT];
  joint_states_.velocity[LEFT] = last_velocity_[LEFT];
  joint_states_.velocity[RIGHT] = last_velocity_[RIGHT];

  joint_states_.position[B_LEFT] = last_position_[B_LEFT];
  joint_states_.position[B_RIGHT] = last_position_[B_RIGHT];
  joint_states_.velocity[B_LEFT] = last_velocity_[B_LEFT];
  joint_states_.velocity[B_RIGHT] = last_velocity_[B_RIGHT];


  joint_states_.position[4] = last_position_[4];
  joint_states_.position[5] = last_position_[4];
}

void AVC_vehicle::odometryCallback(const nav_msgs::OdometryConstPtr odom) {
  double linear_velocity = odom->twist.twist.linear.x;
  double angular_velocity = odom->twist.twist.angular.x;

  wheel_speed_cmd_[LEFT] =
      linear_velocity;
  wheel_speed_cmd_[RIGHT] =
      angular_velocity;
}

/*******************************************************************************
 * Main function
 *******************************************************************************/
int main(int argc, char* argv[]) {
  ros::init(argc, argv, "AVC_vehicle_node");
  AVC_vehicle vehicle;

  ros::Rate loop_rate(30);

  while (ros::ok()) {
    vehicle.update();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
