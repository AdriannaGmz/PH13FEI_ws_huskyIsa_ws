#!/usr/bin/env python

import sys
import rospy
from flir_adk_ros.srv import Info
from flir_adk_ros.srv import InfoRequest
from flir_adk_ros.srv import InfoResponse

def info_client(device):
    rospy.loginfo("waiting for service")
    rospy.wait_for_service('info')
    rospy.loginfo("found service")
    try:
        info = rospy.ServiceProxy('info', Info)
        resp = info(device)
        return resp.response
    except rospy.ServiceException, e:
        rospy.loginfo("Service call failed: %s"%e)

if __name__ == "__main__":
    cam_count = rospy.get_param('/flir_boson_services/cam_count', 1)
    device = 0
    if cam_count >1 :
        if len(sys.argv) == 2:
            device = int(sys.argv[1])
        else:
            rospy.loginfo("param cam_count > 1, specify camera index (0,1,...)")
            sys.exit(1)

    print("requesting info on cam[{}]".format(device))
    print(info_client(device))


