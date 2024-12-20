/** PUBLISHER* sending of messages over the ROS system. Publishes:
  1. current ROS time
 */
#include "ros/ros.h"

#include "std_msgs/Header.h"
#include <sstream>          




int main(int argc, char **argv){
  ros::init(argc, argv, "talker_time_speed");    /** name of this node if not overriden */
  ros::NodeHandle n;


  //chatter_time is the name of the TOPIC 1
  ros::Publisher chatter_time_pub = n.advertise<std_msgs::Header>("/chatter_time_speed", 1000);

  ros::Rate loop_rate(10);

  int count = 0;    //A count of how many messages we have sent. 
  while (ros::ok())  {


    // PUBLISH TOPIC with Header msgs
    std_msgs::Header msg;         
    
    // filling with current time, counter for sequence and string for frame_id
    msg.stamp =  ros::Time::now();  //expects "time" data type
    msg.seq = count;          
    msg.frame_id = "Time test";

    ROS_INFO("Sequence nr %d", msg.seq);

    // publish msg
    chatter_time_pub.publish(msg);     


    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}