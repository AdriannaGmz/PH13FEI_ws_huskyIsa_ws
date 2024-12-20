#!/usr/bin/env python

from BosonSDK.ClientFiles_Python import Client_API as pyClient
from BosonSDK.ClientFiles_Python import EnumTypes
from flir_adk_ros.srv import Sync
from flir_adk_ros.srv import SyncRequest
from flir_adk_ros.srv import SyncResponse

import sys
import rospy

def handle(req):
    if req.sync_mode not in ['disabled', 'master', 'slave'] :
        return SyncResponse("usage : rosservice call sync_mode [disabled|master|slave] [device index]")
    
    # get device index from request
    device = 0
    device += req.index
    port = rospy.get_param('/flir_boson_services/serial_device', '/dev/ttyACM0')
    # port = rospy.get_param('/flir_boson_services/serial_device')
    rospy.loginfo("This is the port tty or whatever :")
    rospy.loginfo(port)
    print("This is theeee port")    
    print (port)
    # port += str(device)
    
    so_path = rospy.get_param('/flir_boson_services/lib_path', None)
    myport = pyClient.Initialize(lib_path=so_path, manualport=port,useDll=True) 

    result_0 = pyClient.bosonSetExtSyncMode(EnumTypes.FLR_BOSON_EXT_SYNC_MODE_E.FLR_BOSON_EXT_SYNC_MASTER_MODE if req.sync_mode == "master"  else \
                                            EnumTypes.FLR_BOSON_EXT_SYNC_MODE_E.FLR_BOSON_EXT_SYNC_SLAVE_MODE if req.sync_mode == "slave" else \
                                            EnumTypes.FLR_BOSON_EXT_SYNC_MODE_E.FLR_BOSON_EXT_SYNC_DISABLE_MODE if req.sync_mode == "disabled" else \
                                            EnumTypes.FLR_BOSON_EXT_SYNC_MODE_E.FLR_BOSON_EXT_SYNC_END)
    display_mode = "continouous"
    if req.sync_mode != "disabled" :
        display_mode = "one shot"
    result_1 = pyClient.dvoSetDisplayMode(EnumTypes.FLR_DVO_DISPLAY_MODE_E.FLR_DVO_ONE_SHOT if req.sync_mode == "master" else \
                                          EnumTypes.FLR_DVO_DISPLAY_MODE_E.FLR_DVO_ONE_SHOT if req.sync_mode == "slave" else \
                                          EnumTypes.FLR_DVO_DISPLAY_MODE_E.FLR_DVO_CONTINUOUS)

    ret_str = "sync mode on cam [{}]set to {} with result {}\n display mode set to {} with result {}".format(device,req.sync_mode, result_0, display_mode, result_1)
    
    pyClient.Close(myport)
    
    return SyncResponse(ret_str)

def sync_server():
    
    rospy.init_node('sync_mode_server')
    rospy.Service('sync_mode', Sync, handle)
    print("waiting for sync requests")
    rospy.spin()

if __name__ == "__main__":
    sync_server()
