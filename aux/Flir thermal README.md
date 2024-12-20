General setup information on the Thermal camera setup



# FLIR thermal: Boson USB

[Webpage](https://groupgets.com/manufacturers/teledyne-flir/products/boson)

When connected to your computer, the USB ADK shows up as two different devices:

*  The first is a USB Vision Camera used for streaming video. 
*  The second is a com port used for command and control
*  ls /dev/video0  and 1

Power **Window heater**: Connect the green lead of the provided cable to positive and the black lead to ground.



### External Sync

The BNC coaxial connector on the USB ADK cable is for optional external sync. The ADK triggers on the falling edge of a 3-12 volt signal with a minimum pulse width of 90nS. The sync pulse should be 55-60hz even if the camera is in 30hz output mode. The camera will need to be configured for external sync. Failure to provide sync pulse s to a camera configured for external sync may crash the camera requiring a reboot. 

The start of readout from the sensor occurs **0.5 msec** after the rising edge of EXT_SYNC.  Readout from the sensor is the input to Boson’s signal-processing pipeline; the lag between that and the output from the pipeline is referred to as latency.



**Flir_adk_ros** is the pkg provided by AIS to synch with external trigger:

```bash
roslaunch flir_ros_adk sing_synch.launch
```

