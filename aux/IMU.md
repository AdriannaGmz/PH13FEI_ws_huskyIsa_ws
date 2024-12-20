General setup information on the IMU Xsens



# Xsens MTI IMU

Default from the Xsens MTI Software Suite.

* [ROS MTI driver](http://wiki.ros.org/xsens_mti_driver)

1. Launch the Xsens MTi driver from your catkin workspace:
   ``roslaunch xsens_mti_driver xsens_mti_node.launch``
2. After the device has been detected, you can communicate with it from another process / terminal window, for example:
   ``rostopic echo /filter/quaternion``
3. There is also an example that shows a 3D visualization of the device (orientation data should be enabled in the device). Modify the launch to access IMU plugged in another host (not posible in diffs machines atm)
   `roslaunch xsens_mti_driver display.launch`

