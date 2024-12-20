# FLIR ROS ADK

This is a ROS wrapper for the FLIR Boson ADK thermal camera. The code is mostly based on [this repository](https://github.com/astuff/flir_boson_usb).

## Features
- Multiple camera support
- Multiple output 
    - 16bit binary
    - 8bit binary
    - 8bit colormapped RGB
- Camera control
    - Flat field correction
    - Reboot
    - Info. output
    - External sync mode

## Dependencies
ROS package dependencies can be found in CMakeLists.txt.

## Usage

### Serial Port
The code uses a local library provided by FLIR to interact with the serial port. It should automatically find the .so file under flir_ros_adk/scripts/BosonSDK/FSLP_Files but in case of errors, the lib_path parameter should be set in the launch file.

The serial device is part of the `dialout` group. This can be checked using the following command :

```bash
$ ls -l /dev/ttyACM*
```

To be able to communicate with them, the user should also be a part of this group or have root access. To add the current user to the `dialout` group, the following command needs to be executed :

```bash
$ usermod -a -G dialout <username>
```

### Compile and Run
```bash
$ mkdir -p catkin_ws/src
$ cd catkin_ws/src
$ git clone https://aisgit.informatik.uni-freiburg.de/vertensj/flir_adk_ros.git
$ cd ..
$ catkin_make
$ source devel/setup.bash
$ roslaunch flir_ros_adk flir_boson_mul.launch
```
The launch file should be edited depending on the number of cameras being driven.

#### Note
The devices are assumed to be at `/dev/ttyACM0`, `/dev/ttyACM1` and `/dev/video0`, `/dev/video1`. After each device reboot, the indices may shift by two e.g. `/dev/ttyACM2` and `/dev/ttyACM3`. Naturally this requires changes to be made to the launch file.

## Services
These services control the connected devices. If more than one device is connected, the device index should be sent as an argument, otherwise it is assumed to be 0. This should be the same device index under `/dev/ttyACM*`.

### Info. Server
This service can be called to fetch information about connected devices. 
```bash
$ rosrun flir_ros_adk info_client.py [device index]
OR
$ rosservice call info [device index]
```
The former is easier to read.
### Reboot Server
This service can be used to reboot each device. 

```bash
$ rosservice call reboot [device index]
```

### Flat Field Correction Server
This service can be used to perform flat field correction on each device. 

```bash
$ rosservice call flat_field_correction [device index]
```

### External Sync Server
This service can be used to choose the external sync. mode. By default this feature is disabled and the device uses an internal clock to run at 60Hz.

```bash
$ rosservice call sync_mode [disabled|master|slave]
```

## Parameters

+ `lib_path` : path to .so file, only assign if errors occur
+ `cam_count` : used by the server scripts
+ `serial_device` : name of the serial device under /dev/
+ `dev` : name of the video device under /dev/
+ `video_mode` : leave at default (RAW16)
+ `zoom_enable` : leave at default (FALSE)
+ `color_lut_enable` : publishes a converted 8bit colormapped RGB image on /image_raw_color
+ `publish_8bit_enable` : publishes a converted 8bit binary image with AGC on /image_raw
+ `sensor_type` : leave at default (Boson_640)
+ `camera_info_url` : path to the .yaml file


## License
