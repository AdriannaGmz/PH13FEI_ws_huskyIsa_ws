General setup information on the Lidar


@2023 , updated Firmware revision to the newest available (Firmware 2.4) and used the up-to-that-date latest ouster driver from github, Which had the option for ROS_TIMESTAMP and solved the problem with the PTP synchronization! :D




# LIDAR Ouster

Pkg provided by Ouster. See README

## Building


```bash
sudo apt install build-essential cmake libglfw3-dev libglew-dev libeigen3-dev \
         libjsoncpp-dev libtclap-dev libtins-dev libpcap-dev
sudo apt install ros-melodic-ros-core ros-melodic-pcl-ros \
         ros-melodic-tf2-geometry-msgs ros-melodic-rviz
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin build
```

## Testing

```bash
roslaunch ouster_ros ouster.launch sensor_hostname:=<sensor hostname> \
                                   udp_dest:=<udp data destination> \
                                   metadata:=<path to metadata json> \
                                   lidar_mode:=<lidar mode> viz:=<viz>
```

where:

* ``<sensor hostname>`` can be the hostname (os-99xxxxxxxxxx) or IP of the sensor
  * 192.168.132.30
* ``<udp data destination>`` is the hostname or IP to which the sensor should send data
  * 192.169.132.1
* ``<path to metadata json>`` is an optional path to json file to save calibration metadata
* ``<lidar mode>`` is one of ``512x10``, ``512x20``, ``1024x10``, ``1024x20``, or ``2048x10``, and
* ``<viz>`` is either ``true`` or ``false``: if true, a window should open and start displaying data after a few seconds.

Note that if the ``metadata`` parameter is not specified, this command will write metadata to ``${ROS_HOME}``. By default, the name of this file is based on the hostname of the sensor, e.g. ``os-99xxxxxxxxxx.json``.

For our current setup:

```bash
roslaunch ouster_ros ouster.launch sensor_hostname:=192.168.132.30 udp_dest:=192.168.132.59 metadata:=/home/adrianna/ISAsense/met.json lidar_mode:=2048x10 viz:=true

roslaunch ouster_ros ouster.launch sensor_hostname:=192.168.132.30 udp_dest:=192.168.132.1 metadata:=/home/husky/ISA4/tmp/met.json lidar_mode:=512x10 viz:=false

```



## PTP synchronization

* [Help on github](https://github.com/ouster-lidar/ouster_example/issues/78)

Before using the data, verify that PTP server is working in the husky, else,there will be error messages of unsynchronized timestamps and tf. To check

`sudo service ptp4l status`

If problems with this, reboot and verify the configuration (to have it in software mode). Check note on **Lidar and PTP-disable br0** (in **Dropbox/0-implementation notes**).

