/*******************************************************************************
* avc_vehicle.h and avc_vehicle.cpp hav been adapted from avc3_fake.h and avc3_fake.cpp
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

#ifndef AVC_VEHICLE_H_
#define AVC_VEHICLE_H_
#include <math.h>

#include <ros/ros.h>
#include <ros/time.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>

namespace avc_sim
{

#define WHEEL_RADIUS                    0.033     // meter

#define LEFT                            0
#define RIGHT                           1
#define B_LEFT                          2
#define B_RIGHT                         3

class AVC_vehicle
{
public:
    AVC_vehicle();
    ~AVC_vehicle();
    bool init();
    bool update();

private:
    // ROS NodeHandle
    ros::NodeHandle nh_;

    // ROS Topic Publishers
    ros::Publisher joint_states_pub_;
  
    // ROS Topic Subscribers
    ros::Subscriber odom_sub_;

    // ROS time for updates
    ros::Time prev_update_time_;

    sensor_msgs::JointState joint_states_;
    double wheel_speed_cmd_[2];

    std::string joint_states_name_[4];

    double last_position_[4];
    double last_velocity_[4];

    double wheel_seperation_;
    double turning_radius_;
    double robot_radius_;

    void updateJoint(ros::Duration diff_time);
    void odometryCallback(const nav_msgs::OdometryConstPtr odom);

};
} // end namespace avc_sim

#endif // AVC_VEHICLE_H_
