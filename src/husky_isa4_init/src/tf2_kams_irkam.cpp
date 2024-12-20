/*    Static tf2 Broadcaster
publishes transform:      base_link → ir_kam_link
base_link is Husky origin

*/

#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>


int main(int argc, char** argv){
  ros::init(argc, argv, "tf2_static_irkam");

  static tf2_ros::StaticTransformBroadcaster st_br_irkam;
  geometry_msgs::TransformStamped transformKam;

  transformKam.header.stamp = ros::Time::now();
  transformKam.header.frame_id = "base_link";
  transformKam.child_frame_id = "ir_kam_link";

  transformKam.transform.translation.x = 0.196;  
  transformKam.transform.translation.y = -0.73;
  transformKam.transform.translation.z = 0.286;

  tf2::Quaternion q;
  q.setRPY(0, 0, 3.14);    //  roll pitch yaw in radians
  transformKam.transform.rotation.x = q.x();
  transformKam.transform.rotation.y = q.y();
  transformKam.transform.rotation.z = q.z();
  transformKam.transform.rotation.w = q.w();            

  st_br_irkam.sendTransform(transformKam);

  ros::spin();
  return 0;
};
