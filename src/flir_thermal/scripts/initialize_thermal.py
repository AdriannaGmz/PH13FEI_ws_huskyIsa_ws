#!/usr/bin/env python

import rospy

from BosonSDK.ClientFiles_Python import Client_API as pyClient
from BosonSDK.ClientFiles_Python import EnumTypes

from flir_adk_ros.srv import FlatFieldCorrection
from flir_adk_ros.srv import FlatFieldCorrectionRequest
from flir_adk_ros.srv import FlatFieldCorrectionResponse
from flir_adk_ros.srv import Sync
from flir_adk_ros.srv import SyncRequest
from flir_adk_ros.srv import SyncResponse
 
def set_sync_slave(index):
    rospy.loginfo("waiting for service")
    rospy.wait_for_service('sync_mode')
    # print("found service")    
    rospy.loginfo("found service")
    try:
        sync = rospy.ServiceProxy('sync_mode', Sync)
        # request : set dev[index] to slave
        req = SyncRequest()
        req.index = index
        req.sync_mode = "slave"
        response = sync(req).response
        print("Here goes the response")
        print(response)
        rospy.loginfo(response)
        return response
    except rospy.ServiceException, e:
        # print("Failed")
        rospy.loginfo("Service call failed: %s"%e)



# def set_manual_ffc(index):
#     # port = rospy.get_param('/flir_boson_services/serial_device', '/dev/ttyACM')
#     port = rospy.get_param('/flir_boson_services/serial_device')
#     print (port)
#     port += str(index)
#     so_path = rospy.get_param('/flir_boson_services/lib_path', None)
#     myport = pyClient.Initialize(lib_path=so_path, manualport=port,useDll=True)
#     result = pyClient.bosonSetFFCMode(EnumTypes.FLR_BOSON_FFCMODE_E.FLR_BOSON_MANUAL_FFC)
#     return ("set device[{}] to manual FFC with result {}".format(index, result))

if __name__ == "__main__":
    cam_count = rospy.get_param('/flir_boson_services/cam_count', 1)
    rospy.loginfo("camera count : {}".format(cam_count))
    rospy.loginfo(set_sync_slave(0))
    # rospy.loginfo(set_sync_slave(1))
    # rospy.loginfo(set_manual_ffc(0))
    # rospy.loginfo(set_manual_ffc(1))


