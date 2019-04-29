#!/usr/bin/env python

import roslib; roslib.load_manifest('sound_play')
import rospy
from math import sqrt
from sound_play.libsoundplay import SoundClient
from geometry_msgs.msg import PoseStamped, Quaternion

def goalCallback(msg):
    x_dist = (msg.pose.position.x)
    y_dist = (msg.pose.position.y)
    dist = sqrt(x_dist **2 + y_dist**2)
    if( dist > 2):
        sound_client = SoundClient()
        sound_client.playWave('/home/snuc/catkin_ws/src/avc_urdf/avc_motion/sounds/dog.wav')
        

if __name__=='__main__':
    rospy.init_node('play_sound_file')

    sleep_time = 3

    # Wait for sound_play to connect to publishers
    rospy.sleep(sleep_time)

    while not rospy.is_shutdown():
        rospy.Subscriber('/goal', PoseStamped, goalCallback)
        rospy.sleep(sleep_time)

    rospy.spin()