#!/usr/bin/env python

from BosonSDK.ClientFiles_Python import Client_API as pyClient
from flir_adk_ros.srv import FlatFieldCorrection
from flir_adk_ros.srv import FlatFieldCorrectionRequest
from flir_adk_ros.srv import FlatFieldCorrectionResponse

import rospy
import sys

def handle(req):
    # get device index from request
    device = 0
    device += req.index
    # port = rospy.get_param('/flir_boson_services/serial_device', '/dev/ttyACM')
    port = rospy.get_param('/flir_boson_services/serial_device')
    port += str(device)

    so_path = rospy.get_param('/flir_boson_services/lib_path', None)
    myport = pyClient.Initialize(lib_path=so_path, manualport=port,useDll=True) 
    result_ = pyClient.bosonRunFFC()
    pyClient.Close(myport)
    ret_str = "FFC req. performed on cam[{}]".format(device)
    if result_ != 0 :
        ret_str = "failed with code :" + str(result_)
    return FlatFieldCorrectionResponse(ret_str)

def ffc_server():
    rospy.init_node('flat_field_correction_server')
    rospy.Service('flat_field_correction', FlatFieldCorrection, handle)
    rospy.loginfo("waiting for FFC requests")
    rospy.spin()

if __name__ == "__main__":
    ffc_server()
