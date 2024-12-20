Contents

- [Microphone AoA](#c_mic)
- [eNose](#c_nose)
- [IR-Kam](#c_irkam)
- [UV-Module](#c_uvmod)
- [Dotcube](#c_dotcube)



<br><a name="c_mic"></a>

# Microphone array

**listen_microphone** has the messages definition _audio_with_timestamp.msg_ (which is the same as AudioMics.msg, msg type sent from Raspberry). receiverSoftware is just to test.

* `rosrun listen_microphone simpleSubscriberMicroph_node`
* `rosrun receiverSoftware simpleSubsc_node`
* `rostopic echo /audio_packets`   (needs built msgs)

UPDATE:::::  Pkg ros-gasleak-localization, verify the simple instructions (adgr)

### Configuration

1. Set ros master - on the raspberry (pi **192.168.132.27**) edit the file **/etc/environment**
2. Interesting to see if everything is working correctly > 
   1.  ssh
   2.  use the command "**screen -r**" , this will show some information      about the state of the software
   3.  exit the screen again with **ctrl+a+d**.

### Commands

telocate stop (will stop the sending of messages) 

telocate start (will start the sending of messages) 

telocate status (will output if it is running or not) 

telocate restart (will stop and start)



<br><a name="c_nose"></a>

# Electrical Nose, Endress+Hauser

ip: 192.168.132.23

## Run service

```
sudo systemctl status roslaunch_enose.service 
rostopic echo /enose/chatter
```

UPDATE::::::

Test  with 

husky@husky-z1:~/isadata/ROSbag$ rosbag play 2022-03-30-17-24-13.bag
rostopic echo /enose/chatter


<br><a name="c_irkam"></a>

# IR KAM, Endress+Hauser

ip: 192.168.132.24

## Run service

`sudo systemctl status roslaunch_irkam.service `

**rqt_plot**: `/gas_camera/ir/image_raw`

## Particularities about the node (Python3 with ROS melodic)

This ROS node is executed with python3 support for ROS melodic (which by default uses python2). In order to execute rosrun with python3 support for the python nodes, the following must be observed:

* **Python Script**: Modify the shebang accordingly (at the file where python3 should be executed): `#!/usr/bin/env python3`
* **Pkg building**: Needs python 3.8 at least. Declare env. vars and modify building accordingly:

```
$ export ROS_PYTHON_VERSION=3
$ catkin config --cmake-args -DPYTHON_VERSION=3
$ catkin build 
$ source devel/setup.bash
$ rosrun my_pkg my_script.py
```

* **setcap**: Run just once

```
cd /usr/bin
setcap 'cap_net_raw,cap_net_admin+eip' /usr/bin/python3.8
getcap /usr/bin/python3.8


#and to remove it (when needed)
setcap -r /usr/bin/python3.8
```

* Pkgs required (and how to install opencv without the library error)

```
pip3 install numpy PyQt5 pyqtgraph
sudo pip3 install --upgrade pip
sudo pip3 install opencv-python
```

* **Problem** "not finding python3/r in /usr/bin/env". This happens when the python format file is not recognized when  copying the script from Windows and use it directly in Linux (ff for file format with values from macs and unix)
  * To solve, open file in vim or vi and execute `:set ff=unix` Save and exit `:wq` and try again :) 




Update. Sept 2022

source the ir_kam_ws
export DISPLAY=:0
roslaunch gas_cam_ros gas_cam_ros.launch 
roslaunch gas_algorithms gas_algorithms.launch
roslaunch gas_algorithms tf_static_ir.launch


topics:
  /ir_img  as cam rviz type
  /gas_ir_overlay  as cam rviz type




<br><a name="c_uvmod"></a>

# UV Module - Oil detector, Endress+Hauser

ip: 192.168.132.25

## Run service

` sudo systemctl status roslaunch_uvmod.service `

**rqt_plot**: `/uv_mod/img_blend` or `rostopic echo /uv_mod/img_raw`





<br><a name="c_dotcube"></a>

# Lidar Dotcube, DotScene

ip: 192.168.132.26

## Run service

`sudo systemctl status roslaunch_dotcube.service `

Visualize in **rviz**







