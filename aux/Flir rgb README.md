General setup information on the rgb cameras setup



# FLIR rgb cameras: Spinnaker SDK



![img](https://www.flir.com/globalassets/support/iis/application-notes/9hjdn56z.bmp)



| Color     | Pin  | Line | Function | Description                      |
| --------- | ---- | ---- | -------- | -------------------------------- |
| **Green** | 1    | 3    | **VAUX** | **Auxiliary Input Voltage (DC)** |
|           | 1    | 3    | GPI      | Non-isolated Input               |
| Black     | 2    | 0    | OPTOIN   | Opto-isolated Input              |
| Red       | 3    | 2    | VOUT     | Camera Power Output              |
|           | 3    | 2    | GPIO     | Non-isolated Input/Ouput         |
| White     | 4    | 1    | OPTOOUT  | Opto-isolated Output             |
| Blue      | 5    | N/A  | Opto GND | Opto-isolated Ground             |
| **Brown** | 6    | N/A  | **GND**  | **Camera Power Ground**          |





Download the Spinnaker SDK and update firmware:

* https://flir.box.com/s/nzqrxcio955u9otsfjbdbl7wms2usrml

The firmware upgrade on Ubuntu can be done using the SpinUpdateConsole tool

```bash
$ SpinUpdateConsole -RXXXXXXXX xxxx.ez2
```

where XXXXXXXX is serial number of the camera, and xxxx.ez2 is firmware file name.





## Linux settings


Since both cameras are handled by a common Switch/Router, bandwidth must be managed by adjusting packet size and packet delay, based on desired resolution and frame rate.


#### Set maximum receive buffer size (Enable jumbo packets)

To check the current receive buffer size (max and default), run the following commands:

```
$ cat /proc/sys/net/core/rmem_max
$ cat /proc/sys/net/core/rmem_default
```

And to increase it, add the following lines to the bottom of the **`/etc/sysctl.conf`** file:

```
net.core.rmem_max=10485760
net.core.rmem_default=10485760
```



#### Increase Packet Size (MTU, Maximum transmission unit)

* Packet size influences the number of interrupts generated (which affects CPU usage). 
* The larger the packet size, the fewer the interrupts for the same amount of data. 
* To minimize CPU usage, increase the packet size. The upper limit depends on your host adapter, your Ethernet switches (if used), and the camera.
* Adjust the packet size (MTU) of your host adapter to ~9000 (the standard **jumbo** packet size), by
  * Manually changing MTU per interface (br0 takes the smallest value)

```bash
$ sudo ifconfig br0 mtu 9000  		
$ sudo ifconfig eno1 mtu 9000
$ sudo ifconfig enp1s0 mtu 9000   	#not required atm 
```

* To increase **permanently**
  * add in **`/etc/rc.local`:**

```
# adgr - increase mtu size to 9000 for flir images
sudo ifconfig br0 mtu 9000
sudo ifconfig eno1 mtu 9000
sudo ifconfig enp1s0 mtu 9000
```

#### Packet Delay

The Packet delay acts like a gap between packets during transmission. This delay allows the host to process the current packet before the arrival of the next one. When you increase the packet delay value from zero, you reduce the effective bandwidth assigned to the camera and thereby also reduce the possibility of dropped frames.




_Modify **Packet Delay** and **Packet size** in Spinview by filtering with "packet" keyword_:

* SCPS Packet Size changed from 9000 to 1400
* Stream Channel Packet Delay changed from 1000 to 900







#### Required Bandwidth

The **maximum bandwidth available is 125 MB**. This includes image data, control data and image resends, which occur when frames are being dropped. Each image and each packet has a certain amount of overhead that will use some bandwidth. Therefore, when calculating your bandwidth requirements, you should *not attempt to use the full maximum of 125 MB*. If the packet size and packet delay combination exceeds the available bandwidth, frames will be dropped. 

To calculate your bandwidth requirements, determine your **required resolution, frame rate, and pixel format (bytes per pixel)**

> Height x Width x Frame Rate x Bytes per Pixel = Bandwidth in MB

*Example*, image that is VGA, 82 FPS, Mono8:

* 640 (H) x 480 (W) x 82 (FPS) x 1 (BPP) = 25,190,400 = ~25 MB





 **In our case**, bandwidth of both cameras with binning 4, 39 FPS according to spinview and the driver, Pixel format is  BGR8 (8 bits of data per pixel, 3 channels, 3byte per pixel):

> 2 BFS cameras x 384 (H) x 512 (W) x 39 (FPS) x 3 (BPP) = 46,006,272= 43.875 MB = ~ 44MB



In reality bandwidth reported to be ~22 MB of bandwidth per camera





#### More on setup and problems with frame size and incomplete frames

* [Configure Synched capture with multiple FLIR cams](https://www.flir.com/support-center/iis/machine-vision/application-note/configuring-synchronized-capture-with-multiple-cameras/).
* [Setup multiple GigE cams](https://www.flir.eu/support-center/iis/machine-vision/application-note/setting-up-multiple-gige-cameras/)
* [Problems FaQ, Jumbo Packets](https://flir.custhelp.com/app/answers/detail/a_id/2990/kw/two%20cameras/related/1)
* [FLIR Lost-ethernet-data-packets-on-linux-systems](https://www.flir.com/support-center/iis/machine-vision/knowledge-base/lost-ethernet-data-packets-on-linux-systems/)
* [FLIR Troubleshooting-image-consistency-errors](https://www.flir.com/support-center/iis/machine-vision/application-note/troubleshooting-image-consistency-errors/)

------









## **Spinview**

### Single and Multiple cameras (Master-slave and slaves-to-triggered signal)

Check docs in **aux/FlirRgb** for  master-slave and slaves triggered by arduino signal (5v)  configurations







## ROS Driver, neufieldrobotics: spinnaker_sdk

There are several drivers for ROS and the FLIR Blackfly S (BFS) cameras.. the ones we chose:

* [Spinnaker sdk camera driver](http://wiki.ros.org/spinnaker_sdk_camera_driver) 

* **[github for the driver](https://github.com/neufieldrobotics/spinnaker_sdk_camera_driver)** 

  

Remarks:

* The second one is being used atm, Nov 2021
* Parameters from launch file override parameters from yaml





### Single camera

Modify  `params/single_params.yaml`  replacing the cam-id serial number. Then:

* Node version of driver

```
roslaunch spinnaker_sdk_camera_driver acquisition_single.launch
```

* Test that the images are being published by running

``` bash
rqt_image_view
```





### Multiple cameras, master-slave

Never worked with this driver.









### Multiple cameras, slaves-to-triggered signal

* Connect according to **aux/FlirRgb/BothSlave/SlaveSlaveSchem.png**  and have Arduino sending 5v trigger signal.

* Modify `params/multi_external_trigger_params.yaml` accordingly
* Then

```bash
roslaunch spinnaker_sdk_camera_driver acquisition_multi_external_trigger.launch
```



* cam0 is ip28 cam on the left (Husky perspective)

* cam1 is ip29 cam on the right (Husky perspective)





In Reality, maximum fps available from spinview is 38.5-39 Hz 



------

### Notes on this ROS driver

| Remarks | Single camera                                         | Multiple, master-slave                                       |
| ------- | ----------------------------------------------------- | ------------------------------------------------------------ |
| launch  | *acquisition-single.launch* uses *single_params.yaml* | *acquisition_external_trigger.launch* uses *external_trigger_params.yaml* |



If `max_rate_save: true`  in the Launch file

* do not publish to ROS
* saves imgs without explicitly asking
* and goes to the "first IF" (bc MAX_RATE_SAVE_=true) in the following code:    ***capture.cpp**, line ~740* 

```c++
if (cams.is_master()) { 
    if (MAX_RATE_SAVE_){
      "MSG: Master first IF, max_rate_save set to TRUE"
      LineSelector-»Line1
      LineMode-»Output
      AcquisitionFrameRateEnable-»False
    } else{
      "MSG: Master second IF, max_rate_save set to FALSE"
      TriggerMode-»On
      LineSelector-»Line1
      LineMode-»Output
      TriggerSource-»Software
    }
else { //setup for slave cameras
      TriggerMode-»On
      LineSelector-»Line2
      TriggerSource-»Line2
      TriggerSelector-»FrameStart
      LineMode-»Input
      TriggerOverlap-»ReadOut
      TriggerActivation-»RisingEdge
    }
```







# Rectification node

Normally, we not only want the raw images, but the rectified imgs (processed from the cam_info published along with the images).  Refer to this [Ros pkg for rectified images](http://wiki.ros.org/image_proc)

### Commands 

Run in different terminals. This is also included in the *ais_init_launch* file (in *publish_rectifications.launch*)

```bash
ROS_NAMESPACE=/camera_array/cam0Left rosrun image_proc image_proc
ROS_NAMESPACE=/camera_array/cam1Right rosrun image_proc image_proc
ROS_NAMESPACE=/camThermal rosrun image_proc image_proc
```
