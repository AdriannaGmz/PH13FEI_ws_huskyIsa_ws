/** == For a better understanding, see the "listen_microphone" pkg ==  */

#include "ros/ros.h"
#include <receiverSoftware/audio_with_timestamp.h> // custom ingoing msgs


void chatterCallbackAudio(const receiverSoftware::audio_with_timestamp::ConstPtr& msgAudio){
  ROS_INFO("This is the data from ROS msg:");  
  ROS_INFO("\tseq: [%i]", msgAudio->seq);  
  ROS_INFO("\tsampleRate: [%f]", msgAudio->sampleRate);  
  ROS_INFO("\tlength: [%i]", msgAudio->length);  
  // ROS_INFO("\tsamples: [%f]", msgAudio->samples);   //This is an array, retrieve data accordingly
}



int main(int argc, char **argv){
  ros::init(argc, argv, "simpleSubscriberrr");
  ros::NodeHandle n;
  ros::Subscriber audioMics_sub = n.subscribe("/audio_packets", 1000, chatterCallbackAudio);

  ros::spin();

  return 0;
}