#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped, Quaternion
from math import cos, asin, sqrt, atan2, sin, pi, exp
import tf
import requests 
import json
requests.packages.urllib3.disable_warnings() 

# Calculate the distance between 2 GPS points
def getDistance(lat1, lon1, lat2, lon2):
    p = 0.017453292519943295     #Pi/180
    a = 0.5 - cos((lat2 - lat1) * p)/2 + cos(lat1 * p) * cos(lat2 * p) * (1 - cos((lon2 - lon1) * p)) / 2
    return 12742 * asin(sqrt(a)) #2*R*asin..

# Calculate the orientation between 2 GPS points
def getOrientation(lat1, lon1, lat2, lon2):
    numer = sin(lon2-lon1)
    denom = cos(lat1)*sin(lat2)-sin(lat1)*cos(lat2)*cos(lon2-lon1)
    return atan2(numer,denom)


def getGPSpoints(thingName, propName):
    ####### Get Data   #####################
    url = 'https://pp-1904181900im.devportal.ptc.io/Thingworx'
    headers = { 'Accept': 'application/json'}#, 'appKey': '1c6026f9-2310-4e36-98b5-8b4aa0e35dfc' }
    response = requests.get(url + '/Things/'+thingName+'/Properties/'+propName, headers=headers, verify=False)
    raw = response.json()

    ### Extract Data ###########
    data = raw['rows'][0][propName]
    lat = data['latitude']
    lon = data['longitude']
    pos = [lat, lon]
    return pos


def shaper(dist):
    sig =2.5
    return (1-exp(-dist*dist/sig/sig))


def updateGPS():
    # Initialize ROS publisher and node
    pub = rospy.Publisher('goal', PoseStamped, queue_size=10)
    goal_msg = PoseStamped()
    rospy.init_node('gps_reciever', anonymous=True)
    rate = rospy.Rate(0.5) # hz

	# Loop GPS collection
    while not rospy.is_shutdown():
        try:
            # Collect GPS data from the server
            [lat1, lon1] = [41.7502065,-111.8151294]
            [lat2, lon2] = [40.7517753,-111.8146574]
            # [lat1, lon1] = getGPSpoints('medicalFake1','GPS_goal')
            # [lat2, lon2] = getGPSpoints('fakeLocation2','GPS_car')
        except:
            rospy.loginfo('No coordinates received.')
        else:
            # Calculate Distance
            dist = getDistance(lat1, lon1, lat2, lon2)
            # Calculate Orientation
            orient = getOrientation(lat1, lon1, lat2, lon2)
            
            goal_msg.header.frame_id = "map"
            goal_msg.header.stamp = rospy.get_rostime()
            goal_msg.pose.position.x = dist*cos(orient)
            goal_msg.pose.position.y = dist*sin(orient)
            goal_msg.pose.position.z = 0.0
            goal_msg.pose.orientation.x = 0.0
            goal_msg.pose.orientation.y = 0.0
            goal_msg.pose.orientation.z = 1.0
            goal_msg.pose.orientation.w = 0.0


            pub.publish(goal_msg)
        finally:
            rate.sleep()

if __name__ == '__main__':
    try:
        updateGPS()
    except rospy.ROSInterruptException:
        pass
