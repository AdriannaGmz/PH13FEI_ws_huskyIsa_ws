/*    Static tf2 Broadcaster
publishes transform:      base_link → cam0l_link
cam0L is the Left camera, ip 28

*/

#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>


int main(int argc, char** argv){
  ros::init(argc, argv, "tf2_static_husky_cam0l");

  static tf2_ros::StaticTransformBroadcaster st_br;
  geometry_msgs::TransformStamped transformStamped;

  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = "base_link";
  transformStamped.child_frame_id = "cam_0_optical_frame";
  // transformStamped.child_frame_id = "cam0l_link";

  transformStamped.transform.translation.x = 0.260;  
  transformStamped.transform.translation.y = 0.20;
  transformStamped.transform.translation.z = 0.650;

  tf2::Quaternion q;
  q.setRPY(-1.571, 0, -1.571);    //  roll pitch yaw in radians
  transformStamped.transform.rotation.x = q.x();
  transformStamped.transform.rotation.y = q.y();
  transformStamped.transform.rotation.z = q.z();
  transformStamped.transform.rotation.w = q.w();            

  st_br.sendTransform(transformStamped);

  ros::spin();
  return 0;
};
