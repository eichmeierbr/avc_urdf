#include "dynamic_models/bicycle.h"

using namespace dynamic_models;

Bicycle::Bicycle() {
    init();
}

Bicycle::~Bicycle(){

}

bool Bicycle::init(){
    // Initialize the odometry position
    odom_.pose.pose.position.x = 0.0;
    odom_.pose.pose.position.y = 0.0;
    odom_.pose.pose.position.z = 0.0;
    yaw_ = 0.0;
    odom_.pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw_);

    // Initialize bike
    wheel_sep_ = 0.3298;

    // Intialize the odometry twist
    linear_velocity_ = 0.0;
    angular_velocity_ = 0.0;
    odom_.twist.twist.linear.x = linear_velocity_;
    odom_.twist.twist.linear.y = 0.0;
    odom_.twist.twist.linear.z = 0.0;
    odom_.twist.twist.angular.x = 0.0;
    odom_.twist.twist.angular.y = 0.0;
    odom_.twist.twist.angular.z = angular_velocity_;

    nh_.param("odom_frame", odom_.header.frame_id, std::string("odom"));
    nh_.param("base_frame", odom_.child_frame_id, std::string("base_footprint"));

    double pcov[36] = { 0.1,   0,   0,   0,   0, 0,
                          0, 0.1,   0,   0,   0, 0,
                          0,   0, 1e6,   0,   0, 0,
                          0,   0,   0, 1e6,   0, 0,
                          0,   0,   0,   0, 1e6, 0,
                          0,   0,   0,   0,   0, 0.2};
    memcpy(&(odom_.pose.covariance),pcov,sizeof(double)*36);
    memcpy(&(odom_.twist.covariance),pcov,sizeof(double)*36);

    // initialize publishers
    odom_pub_         = nh_.advertise<nav_msgs::Odometry>("odom", 100);

    // initialize subscribers
    cmd_vel_sub_  = nh_.subscribe("cmd_vel", 100,  &Bicycle::commandVelocityCallback, this);

    prev_update_time_ = ros::Time::now();
    return true;
}

bool Bicycle::update(){
    ros::Time time_now = ros::Time::now();
    ros::Duration step_time = time_now - prev_update_time_;
    prev_update_time_ = time_now;

    // odom
    updateOdometry(step_time);
    odom_.header.stamp = time_now;
    odom_pub_.publish(odom_);

    // tf
    geometry_msgs::TransformStamped odom_tf;
    updateTF(odom_tf);
    tf_broadcaster_.sendTransform(odom_tf);

    return true;
}

void Bicycle::commandVelocityCallback(const geometry_msgs::TwistConstPtr cmd_vel_msg){
    linear_velocity_ = cmd_vel_msg->linear.x;
    double angLimit = M_PI/6;
    
    if(cmd_vel_msg->angular.z >= angLimit){
        angular_velocity_ = angLimit;
    } else if(cmd_vel_msg->angular.z <= -angLimit){
        angular_velocity_ = -angLimit;
    } else {
        angular_velocity_ = cmd_vel_msg->angular.z;
    }
}

bool Bicycle::updateOdometry(ros::Duration diff_time){
    double dt = diff_time.toSec();

    // Use Euler integration for updating the odometry
    double ang = linear_velocity_/wheel_sep_*std::tan(angular_velocity_);  

    odom_.pose.pose.position.x = odom_.pose.pose.position.x + dt * linear_velocity_ * std::cos(yaw_);
    odom_.pose.pose.position.y = odom_.pose.pose.position.y + dt * linear_velocity_ * std::sin(yaw_);
    yaw_ = yaw_ + dt * ang;
    odom_.pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw_);

    // Update the odometry twist
    odom_.twist.twist.linear.x = linear_velocity_;
    odom_.twist.twist.angular.z =  ang;

    return true;
}
void Bicycle::updateTF(geometry_msgs::TransformStamped& odom_tf){
    odom_tf.header = odom_.header;
    odom_tf.child_frame_id = odom_.child_frame_id;
    odom_tf.transform.translation.x = odom_.pose.pose.position.x;
    odom_tf.transform.translation.y = odom_.pose.pose.position.y;
    odom_tf.transform.translation.z = odom_.pose.pose.position.z;
    odom_tf.transform.rotation = odom_.pose.pose.orientation;
}

/*******************************************************************************
* Main function
*******************************************************************************/
int main(int argc, char* argv[])
{
  ros::init(argc, argv, "unicycle_dynamics_node");
  Bicycle bicycle;

  ros::Rate loop_rate(30);

  while (ros::ok())
  {
    bicycle.update();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
