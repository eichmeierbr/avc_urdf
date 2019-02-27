/*******************************************************************************
* unicycle.h and unicycle.cpp hav been adapted from turtlebot3_fake.h and turtlebot3_fake.cpp
* See the ROBOTIS copyright below:
*
* Copyright 2016 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

#ifndef UNICYCLE_H_
#define UNICYCLE_H_

#include  <ros/ros.h>
#include <ros/time.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

namespace dynamic_models{


class Unicycle
{
public:
    Unicycle();
    ~Unicycle();
    bool init();
    bool update();

private:
    // ROS NodeHandle
    ros::NodeHandle nh_;

    // ROS Topic Publishers
    ros::Publisher odom_pub_;

    // ROS Topic Subscribers
    ros::Subscriber cmd_vel_sub_;

    // ROS time for updates
    ros::Time prev_update_time_;

    // Variables
    double linear_velocity_;
    double angular_velocity_;
    double yaw_;
    nav_msgs::Odometry odom_;
    tf::TransformBroadcaster tf_broadcaster_;

    // Function prototypes
    void commandVelocityCallback(const geometry_msgs::TwistConstPtr cmd_vel_msg);
    bool updateOdometry(ros::Duration diff_time);
    void updateTF(geometry_msgs::TransformStamped& odom_tf);
};

}


#endif // UNICYCLE_H_
