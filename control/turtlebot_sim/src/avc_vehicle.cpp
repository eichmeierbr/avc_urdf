#include "turtlebot_sim/avc_vehicle.h"

using namespace turtlebot_sim;


AVC_vehicle::AVC_vehicle(){
    init();
}
AVC_vehicle::~AVC_vehicle(){

}
bool AVC_vehicle::init(){
    // initialize ROS parameter

    std::string robot_model = nh_.param<std::string>("tb3_model", "");

    wheel_seperation_ = 0.243;

    nh_.param("front_wheel_left_joint_name", joint_states_name_[LEFT],  std::string("left_front_wheel_joint"));
    nh_.param("front_wheel_right_joint_name", joint_states_name_[RIGHT],  std::string("right_front_wheel_joint"));
    nh_.param("back_wheel_left_joint_name", joint_states_name_[LEFT],  std::string("left_back_wheel_joint"));
    nh_.param("back_wheel_right_joint_name", joint_states_name_[RIGHT],  std::string("right_back_wheel_joint"));
    nh_.param("joint_states_frame", joint_states_.header.frame_id, std::string("base_link")); //base_footprint

    // initialize variables
    wheel_speed_cmd_[LEFT]  = 0.0;
    wheel_speed_cmd_[RIGHT] = 0.0;
    last_position_[LEFT]    = 0.0;
    last_position_[RIGHT]   = 0.0;
    last_velocity_[LEFT]    = 0.0;
    last_velocity_[RIGHT]   = 0.0;
    
    // Extra Wheel Varialbes
    wheel_speed_cmd_[BLEFT]  = 0.0;
    wheel_speed_cmd_[BRIGHT] = 0.0;
    last_position_[BLEFT]    = 0.0;
    last_position_[BRIGHT]   = 0.0;
    last_velocity_[BLEFT]    = 0.0;
    last_velocity_[BRIGHT]   = 0.0;

    joint_states_.name.push_back(joint_states_name_[LEFT]);
    joint_states_.name.push_back(joint_states_name_[RIGHT]);
    joint_states_.name.push_back(joint_states_name_[BLEFT]);
    joint_states_.name.push_back(joint_states_name_[BRIGHT]);

    joint_states_.position.resize(4,0.0);
    joint_states_.velocity.resize(4,0.0);
    joint_states_.effort.resize(4,0.0);

    // initialize publishers
    joint_states_pub_ = nh_.advertise<sensor_msgs::JointState>("joint_states", 100);

    prev_update_time_ = ros::Time::now();
    return true;

}
bool AVC_vehicle::update(){
    ros::Time time_now = ros::Time::now();
    ros::Duration step_time = time_now - prev_update_time_;
    prev_update_time_ = time_now;

    // joint_states
    updateJoint(step_time);
    joint_states_.header.stamp = time_now;
    joint_states_pub_.publish(joint_states_);

    return true;
}

void AVC_vehicle::updateJoint(ros::Duration diff_time){

    double wheel_l, wheel_r; // rotation value of wheel [rad]
    double delta_s, delta_theta;
    double v[2], w[2];

    wheel_l = wheel_r     = 0.0;
    delta_s = delta_theta = 0.0;

    v[LEFT]  = wheel_speed_cmd_[LEFT];
    w[LEFT]  = v[LEFT] / WHEEL_RADIUS;  // w = v / r
    v[RIGHT] = wheel_speed_cmd_[RIGHT];
    w[RIGHT] = v[RIGHT] / WHEEL_RADIUS;
    // Our Edits
    v[BLEFT]  = wheel_speed_cmd_[BLEFT];
    w[BLEFT]  = v[BLEFT] / WHEEL_RADIUS;  // w = v / r
    v[BRIGHT] = wheel_speed_cmd_[BRIGHT];
    w[BRIGHT] = v[BRIGHT] / WHEEL_RADIUS;

    last_velocity_[LEFT]  = w[LEFT];
    last_velocity_[RIGHT] = w[RIGHT];
    
    last_velocity_[BLEFT]  = w[BLEFT];
    last_velocity_[BRIGHT] = w[BRIGHT];

    wheel_l = w[LEFT]  * diff_time.toSec();
    wheel_r = w[RIGHT] * diff_time.toSec();

    // wheel_l = w[BLEFT]  * diff_time.toSec();
    // wheel_r = w[BRIGHT] * diff_time.toSec();

    if(isnan(wheel_l))
    {
      wheel_l = 0.0;
    }

    if(isnan(wheel_r))
    {
      wheel_r = 0.0;
    }

    last_position_[LEFT]  += wheel_l;
    last_position_[RIGHT] += wheel_r;

    last_position_[BLEFT]  += wheel_l;
    last_position_[BRIGHT] += wheel_r;


    joint_states_.position[LEFT]  = last_position_[LEFT];
    joint_states_.position[RIGHT] = last_position_[RIGHT];
    joint_states_.velocity[LEFT]  = last_velocity_[LEFT];
    joint_states_.velocity[RIGHT] = last_velocity_[RIGHT];

    joint_states_.position[BLEFT]  = last_position_[BLEFT];
    joint_states_.position[BRIGHT] = last_position_[BRIGHT];
    joint_states_.velocity[BLEFT]  = last_velocity_[BLEFT];
    joint_states_.velocity[BRIGHT] = last_velocity_[BRIGHT];
}

void AVC_vehicle::odometryCallback(const nav_msgs::OdometryConstPtr odom){
    double linear_velocity  = odom->twist.twist.linear.x;
    double angular_velocity = odom->twist.twist.angular.z;

    wheel_speed_cmd_[LEFT]  = linear_velocity - (angular_velocity * wheel_seperation_ / 2);
    wheel_speed_cmd_[RIGHT] = linear_velocity + (angular_velocity * wheel_seperation_ / 2);

    wheel_speed_cmd_[BLEFT]  = linear_velocity - (angular_velocity * wheel_seperation_ / 2);
    wheel_speed_cmd_[BRIGHT] = linear_velocity + (angular_velocity * wheel_seperation_ / 2);
}

/*******************************************************************************
* Main function
*******************************************************************************/
int main(int argc, char* argv[])
{
  ros::init(argc, argv, "avc_vehicle_node");
  AVC_vehicle vehicle;

  ros::Rate loop_rate(30);

  while (ros::ok())
  {
    vehicle.update();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
