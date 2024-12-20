/** LISTENER receipt of Header messages
Listens to topics:
  1. chatter_time with Header messages (with current ros time)
 */

#include "ros/ros.h"
#include "std_msgs/Header.h"              


void chatterCallback(const std_msgs::Header::ConstPtr& msg){
  // ROS_INFO("Seq_id : %d", msg->seq);
  // ROS_INFO("\t frame_id: %s", msg->frame_id.c_str());
  // ROS_INFO("\t timestamp secs: %d", msg->stamp.sec);
  ROS_INFO("Husky - master timestamp secs: %d", msg->stamp.sec);
  ROS_INFO("Husky - master timestamp nsec: %d", msg->stamp.nsec);
}

void chatterCallback_speed(const std_msgs::Header::ConstPtr& msg){
  ROS_INFO("\t Speed timestamp secs: %d", msg->stamp.sec);
  ROS_INFO("\t Speed timestamp nsec: %d", msg->stamp.nsec);
}

void chatterCallback_aislap58(const std_msgs::Header::ConstPtr& msg){
  ROS_INFO("\t Aislap58 timestamp secs: %d", msg->stamp.sec);
  ROS_INFO("\t Aislap58 timestamp nsec: %d", msg->stamp.nsec);
}


int main(int argc, char **argv){
  ros::init(argc, argv, "listener_time");
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("/chatter_time", 1000, chatterCallback);
  ros::Subscriber sub_speed = n.subscribe("/chatter_time_speed", 1000, chatterCallback_speed);
  ros::Subscriber sub_aislap58 = n.subscribe("/chatter_time_aislap58", 1000, chatterCallback_aislap58);

  ros::spin();

  return 0;
}
