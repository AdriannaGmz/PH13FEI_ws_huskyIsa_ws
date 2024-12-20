#!/usr/bin/env python

from BosonSDK.ClientFiles_Python import Client_API as pyClient
from BosonSDK.ClientFiles_Python import EnumTypes
from flir_adk_ros.srv import Info
from flir_adk_ros.srv import InfoRequest
from flir_adk_ros.srv import InfoResponse

import rospy

def handle(req):
    #some init
    ret_str = ""
    r = [0, 0, 0, 0, 0, 0, 0, 0, 0]
    
    # get device index from request
    device = 0
    device += req.index
    # port = rospy.get_param('/flir_boson_services/serial_device', '/dev/ttyACM')
    port = rospy.get_param('/flir_boson_services/serial_device')
    print (port)
    port += str(device)
    ret_str += "device : \t {}\n".format(device)

    # pyClient searches __file__ to find FSLP_64.so. if it fails lib_path should be set in launch file
    so_path = rospy.get_param('/flir_boson_services/lib_path', None)
    myport = pyClient.Initialize(lib_path=so_path, manualport=port,useDll=True) 
    
    
    r[0], type = pyClient.dvoGetType()
    type_str = "analog" if type == EnumTypes.FLR_DVO_TYPE_E.FLR_DVO_TYPE_ANALOG else \
                 "8bit" if type == EnumTypes.FLR_DVO_TYPE_E.FLR_DVO_TYPE_MONO8 else \
                "16bit" if type == EnumTypes.FLR_DVO_TYPE_E.FLR_DVO_TYPE_MONO16 else "0"
    ret_str += "type : \t\t {}\n".format(type_str)

    r[1], format = pyClient.dvoGetOutputFormat()
    format_str = "IR16" if format == EnumTypes.FLR_DVO_OUTPUT_FORMAT_E.FLR_DVO_IR16 else \
                  "RGB" if format == EnumTypes.FLR_DVO_OUTPUT_FORMAT_E.FLR_DVO_RGB else \
                "YCBCR" if format == EnumTypes.FLR_DVO_OUTPUT_FORMAT_E.FLR_DVO_YCBCR else \
                "default" if format == EnumTypes.FLR_DVO_OUTPUT_FORMAT_E.FLR_DVO_DEFAULT_FORMAT else "0"
    ret_str += "output format: \t {}\n".format(format_str)

    r[2], mode = pyClient.dvoGetDisplayMode()
    mode_str = "continuous" if mode == EnumTypes.FLR_DVO_DISPLAY_MODE_E.FLR_DVO_CONTINUOUS else \
                 "one shot" if mode == EnumTypes.FLR_DVO_DISPLAY_MODE_E.FLR_DVO_ONE_SHOT else "0"
    ret_str += "display mode: \t {}\n".format(mode_str)
        
    r[3], gamma = pyClient.agcGetGamma()
    ret_str += "gamma : \t {}\n".format(gamma)

    r[4], temp = pyClient.bosonlookupFPATempDegCx10()
    ret_str += "core temp: \t {}\n".format(float(temp/10))

    r[5], frame_count = pyClient.roicGetFrameCount()
    ret_str += "frame count: \t {}\n".format(frame_count)
    
    r[6], sync = pyClient.bosonGetExtSyncMode()
    sync_str = "disabled" if sync == EnumTypes.FLR_BOSON_EXT_SYNC_MODE_E.FLR_BOSON_EXT_SYNC_DISABLE_MODE else \
               "master" if sync == EnumTypes.FLR_BOSON_EXT_SYNC_MODE_E.FLR_BOSON_EXT_SYNC_MASTER_MODE else \
               "slave" if sync == EnumTypes.FLR_BOSON_EXT_SYNC_MODE_E.FLR_BOSON_EXT_SYNC_SLAVE_MODE else "0"
    ret_str += "sync mode: \t {}\n".format(sync_str)

    r[7], frate = pyClient.sysctrlGetCameraFrameRate()
    ret_str += "frame rate: \t {}\n".format(frate)
    
    r[8], lnr = pyClient.bosonGetLensNumber()
    ret_str += "lens number: \t {}\n".format(lnr)

    pyClient.Close(myport)
    ret_str = "results returned with codes {} {} {} {} {} {} {} {} {}\n".format(r[0],r[1],r[2],r[3],r[4],r[5],r[6],r[7],r[8]) + ret_str

    return InfoResponse(ret_str)

def info_server():
    rospy.init_node('info_server')
    rospy.Service('info', Info, handle)
    rospy.loginfo("waiting for info requests")
    rospy.spin()

if __name__ == "__main__":
    info_server()
    
