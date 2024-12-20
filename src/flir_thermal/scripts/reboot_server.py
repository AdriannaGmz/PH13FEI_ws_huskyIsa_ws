#!/usr/bin/env python

from BosonSDK.ClientFiles_Python import Client_API as pyClient
from flir_adk_ros.srv import Reboot
from flir_adk_ros.srv import RebootRequest
from flir_adk_ros.srv import RebootResponse

import rospy

def handle(req):
    # get device index from request
    device = 0
    device += req.index
    port = rospy.get_param('/flir_boson_services/serial_device', '/dev/ttyACM')
    port += str(device)

    so_path = rospy.get_param('/flir_boson_services/lib_path', None)
    myport = pyClient.Initialize(lib_path=so_path, manualport=port,useDll=True) 
    
    result_ = pyClient.bosonReboot()
    pyClient.Close(myport)
    ret_str = "reboot req. performed on cam[{}]".format(device)
    if result_ != 0 :
        ret_str = "failed with code :" + str(result_)
    return RebootResponse(ret_str)

def reboot_server():
    rospy.init_node('reboot_server')
    rospy.Service('reboot', Reboot, handle)
    rospy.loginfo("waiting for reboot requests")
    rospy.spin()

if __name__ == "__main__":
    reboot_server()
