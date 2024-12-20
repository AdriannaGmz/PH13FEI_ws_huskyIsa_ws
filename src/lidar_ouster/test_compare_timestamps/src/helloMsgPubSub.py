#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from tf2_msgs.msg import TFMessage
from diagnostic_msgs.msg import DiagnosticArray

LIDAR_time_secs = 0
SCAN_time_secs = 0
ROS_time_secs = 0

def callback(data):
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    # rospy.loginfo(rospy.get_caller_id() + "I heard %i", data.header.frame_id)
    rospy.loginfo(" Lidar time stamp %i.%i", data.header.stamp.secs, data.header.stamp.nsecs)
    global LIDAR_time_secs
    global SCAN_time_secs
    LIDAR_time_secs = data.header.stamp.secs
    diff = LIDAR_time_secs - SCAN_time_secs
    rospy.loginfo(" \t\t Lidar - ROS time = %i", diff)


def callback1(data):
    # rospy.loginfo(" Scan  time stamp %i.%i", data.header.stamp.secs, data.header.stamp.nsecs)
    global SCAN_time_secs
    SCAN_time_secs = data.header.stamp.secs

def callback2(data):
    rospy.loginfo(" Map  time stamp %i.%i", data.header.stamp.secs, data.header.stamp.nsecs)

def callback3(data):
    rospy.loginfo(" TF   time stamp %i.%i", data.transforms.header.stamp.secs, data.transforms.header.stamp.nsecs)

def callback4(data):
    # rospy.loginfo(" Diag time stamp %i.%i", data.header.stamp.secs, data.header.stamp.nsecs)
    global ROS_time_secs
    ROS_time_secs = data.header.stamp.secs


    
def listener():

    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/os_cloud_node/points", PointCloud2, callback)
    rospy.Subscriber("/scan", LaserScan, callback1)
    # rospy.Subscriber("map", OccupancyGrid, callback2)
    # rospy.Subscriber("tf", OccupancyGrid, callback3)
    rospy.Subscriber("diagnostics", DiagnosticArray, callback4)




    rospy.spin()



if __name__ == '__main__':
    listener()
