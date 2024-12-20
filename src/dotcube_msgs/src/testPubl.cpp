/** PUBLISHER* sending messages, similar to Dotcube.
 publishes customized messages of type VelodyneScan.msg:

# Velodyne LIDAR scan packets.
Header           header         # Stamp marks the start time of data collection. 
VelodynePacket[] packets        # vector of raw packets

# VelodynePacket.msg: Raw Velodyne LIDAR packet.
time stamp              # Start of data collection + TCP transfer time +  calibration offset
uint8[1206] data        # Velodyne data blocks + timestamp + factory bytes (= one velodyne packet - header)

 */

#include "ros/ros.h"
#include "std_msgs/Header.h"

// customized msgs Dotcube
#include <dotcube_msgs/VelodynePacket.h>  // custom outgoing msgs, single packet
#include <dotcube_msgs/VelodyneScan.h>    // custom outgoing msgs, includes array of VelodynePacket ("wrapper")



int main(int argc, char **argv){
  ros::init(argc, argv, "talker");    /** name of this node if not overriden */
  ros::NodeHandle n;

  //Publish on topic: chatter_dotcube
  ros::Publisher chatter_dotcube_pub = n.advertise<dotcube_msgs::VelodyneScan>("chatter_dotcube", 1000);


  ros::Rate loop_rate(10);

  int count = 0;    //A count of how many messages we have sent. 
  while (ros::ok())  {
    // PUBLISH TOPIC: Dotcube "wrapper" message
    dotcube_msgs::VelodyneScan msgDotcubeOut;
    dotcube_msgs::VelodynePacket msgVelPacket;
    std_msgs::Header msgHeader;


    // Filling raw Velodyne LIDAR packet
    // msgVelPacket.stamp  type:time ; Start of data collection + TCP transfer time +  calibration offset
    msgVelPacket.stamp =  ros::Time::now();  //"dummy" time data type    
    boost::array<unsigned char, 1206> arrayForData = {};  //initialize array with 0s
    msgVelPacket.data = arrayForData; // type:uint8[1206]


    // Raw Header with current time, counter for sequence and string for frame_id
    msgHeader.stamp =  ros::Time::now();  // time data type
    msgHeader.seq = count;          
    msgHeader.frame_id = "This is a test";


    // // Fill wrapper message with array of VelodynePackets
    msgDotcubeOut.packets = {msgVelPacket, msgVelPacket, msgVelPacket}; // type: VelodynePacket[] ; vector of raw packets
    msgDotcubeOut.header = msgHeader; // type: Header ; Stamp marks the start time of data collection. 



    ROS_INFO("From Publisher, I send count %i", msgHeader.seq);
    chatter_dotcube_pub.publish(msgDotcubeOut);



    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}