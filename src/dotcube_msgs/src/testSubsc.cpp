/** LISTENER receipt of customized messages from Dotcube
Listens to topic: chatter_dotcube
 */
#include "ros/ros.h"

#include "std_msgs/Header.h"

// customized msgs Dotcube
#include <dotcube_msgs/VelodynePacket.h>  // custom single outgoing msgs
#include <dotcube_msgs/VelodyneScan.h>    // custom array outgoing msgs




void chatterCb(const dotcube_msgs::VelodyneScan::ConstPtr& msg){
  ROS_INFO("In Subscriber, I read count: %u", msg->header.seq);

    // msgDotcubeOut has:
      // msgDotcubeOut.header = msgHeader; 
      // msgDotcubeOut.packets = {msgVelPacket, msgVelPacket, msgVelPacket}; 
    // where the header is std_msgs::Header msgHeader and has:
      // msgHeader.stamp =  ros::Time::now() 
      // msgHeader.seq = count
      // msgHeader.frame_id = "This is a test"
    // and where packets is VelodynePacket[] , ie vector of raw packets.
    // where each VelodynePacket has
      // msgVelPacket.stamp  type:time ; Start of data collection + TCP transfer time +  calibration offset
      // msgVelPacket.data = array type:uint8[1206]
}




int main(int argc, char **argv){
  ros::init(argc, argv, "listenerOfDotcube");
  ros::NodeHandle n;

  /* The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */
  ros::Subscriber sub = n.subscribe("chatter_dotcube", 1000, chatterCb);

  ros::spin();

  return 0;
}




