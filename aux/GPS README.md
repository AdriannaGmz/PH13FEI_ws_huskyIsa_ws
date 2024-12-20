General setup information







<br><a name="c_gps"></a>

# GPS: Emlid Reach RS2

* Reach creates its own Wifi Hotspot.
* Reach provides NMEA msgs over WiFi, bluetooth or serial
* [ROS driver](https://github.com/enwaytech/reach_rs_ros_driver) expects data from wifi connection

## Configuration

1. With an android phone, connect to the Reach Hotspot. It appears as _reach:XX:XX_.
2. Hotspot password *emlidreach*

  * alternative: web browser and go to *192.168.42.1*

3. In the android phone open the [ReachView app](https://play.google.com/store/apps/details?id=com.reachview&hl=en&gl=US) -deprecated- and find the reach rs2
4. If there's an available internet wifi network, *write the network credentials to the reach rs2 through the app* (it will update the firmware and will register the reach in the other wifi network). This should show the reach's *ip address*
5. Connect back to the internet wifi network and ping ReachRS2

* https://computingforgeeks.com/connect-to-bluetooth-device-from-linux-terminal/)

## Wifi connection, testing with [ROS driver](https://github.com/enwaytech/reach_rs_ros_driver)

* If RS2 ip address is known, open settings in a web browser, else, do with the android app.
* On the Position output tab under *Output1* choose TCP.
* Set the role to *Server*, the address to *localhost* and choose a port number. From the formats select *NMEA*.
* Note IP address and port number in order to configure the ROS driver node correctly.
  * asusvivob config: 10.42.0.252 , port 9001
* `catkin build` and `source` pkg . Does not work in ROS Noetic due to the python3 version for ROS. Tested only in ROS melodic

## SSH

* login: reach
* password: emlidreach

 others..

* login: reach / root
* password: reach / emlidreach











