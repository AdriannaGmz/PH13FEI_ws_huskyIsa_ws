#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from tf2_msgs.msg import TFMessage
from diagnostic_msgs.msg import DiagnosticArray

LIDAR_time_secs = 0
LIDAR_time_nsecs = 0
ROS_time_secs = 0
ROS_time_nsecs = 0

def callback(data):
    # rospy.loginfo(" Lidar time stamp %i.%i", data.header.stamp.secs, data.header.stamp.nsecs)
    global LIDAR_time_secs
    global LIDAR_time_nsecs
    global ROS_time_secs
    global ROS_time_nsecs
    LIDAR_time_secs = data.header.stamp.secs
    LIDAR_time_nsecs = data.header.stamp.nsecs

    diff = LIDAR_time_secs - ROS_time_secs
    diffn = LIDAR_time_nsecs - ROS_time_nsecs

    rospy.loginfo(" \t\t Lidar - ROS time  [secs] = %i", diff)
    rospy.loginfo(" \t\t Lidar - ROS time [nsecs] = %i", diffn)



def callback2(data):
    # rospy.loginfo(" Diag time stamp %i.%i", data.header.stamp.secs, data.header.stamp.nsecs)
    global ROS_time_secs
    global ROS_time_nsecs
    ROS_time_secs = data.header.stamp.secs
    ROS_time_nsecs = data.header.stamp.nsecs


    
def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/ouster/points", PointCloud2, callback)
    rospy.Subscriber("/diagnostics", DiagnosticArray, callback2)
    rospy.spin()



if __name__ == '__main__':
    listener()
