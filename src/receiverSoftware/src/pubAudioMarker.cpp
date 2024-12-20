/*
listens from measurements published in  /doa_results (doa_results.msg):
      which seem to be in radians: [...]   :/
        msg.seq = CalcSeq++;
        msg.azimuth = res.phi;
        msg.elevation = res.theta;

              seq: 78
              elevation: 1.57079637051
              azimuth: -2.62008833885

And from this info, publish a Marker in topic 
      /audio_source_marker   to be seen in rviz

*/

#include "ros/ros.h"
#include <receiverSoftware/doa_result.h> // custom msgs from DoaEstimator (Fraunhofer)
#include <visualization_msgs/Marker.h>
#include <tf2/LinearMath/Quaternion.h>
// #include <cmath>

float azimuth_rcvd = 0.0;
float elevation_rcvd = 0.0;

void cbAudioDoa(const receiverSoftware::doa_result::ConstPtr& msgDoaResults){
  ROS_INFO("Data from DOA msg:");  
  ROS_INFO("\tseq: [%i]", msgDoaResults->seq);  
  ROS_INFO("\tAzimuth [rad]: [%f]", msgDoaResults->azimuth);  
  ROS_INFO("\tElevation [rad]: [%f]", msgDoaResults->elevation);  
  azimuth_rcvd = msgDoaResults->azimuth;
  elevation_rcvd = msgDoaResults->elevation;
}


int main(int argc, char **argv){
  ros::init(argc, argv, "audioDoa2marker");
  ros::NodeHandle n;
  ros::Subscriber audioDoa_sub = n.subscribe("/doa_results", 1000, cbAudioDoa);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("/doa_view_marker", 1000);
  tf2::Quaternion q;
  // ros::spin();
  // return 0;
  ros::Rate loop_rate(10);

  while(ros::ok()){
    visualization_msgs::Marker marker;  
    // marker.header.frame_id = "/map"; 
    marker.header.frame_id = "/base_link";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "audioDoa_shape";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::ARROW;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;


    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    //      correspondence: roll    pitch(y)->-elevation     yaw(Z)->-azimuth    to be reviewed
    // q.setRPY(0, -elevation_rcvd, -azimuth_rcvd);        //  roll pitch yaw in radians
    q.setRPY(0, 0, -azimuth_rcvd);        //  roll pitch yaw in radians
    marker.pose.position.x = -0.232;//similar position to AoA
    marker.pose.position.y = -0.05;
    marker.pose.position.z = 0.80;
    marker.pose.orientation.x = q.x();//0.0;
    marker.pose.orientation.y = q.y();//0.0;
    marker.pose.orientation.z = q.z();//0.0;
    marker.pose.orientation.w = q.w();//1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.6; 
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 1.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    // Publish the marker
    while (marker_pub.getNumSubscribers() < 1)    {
      if (!ros::ok()) { return 0;  }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }
    marker_pub.publish(marker);

    ros::spinOnce();
    loop_rate.sleep();
  }  //while ros ok

  return 0;
}