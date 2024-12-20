/*    Static tf2 Broadcaster
publishes transform:      base_link → os_sensor
  from     base_link (origin/base coordinate system of husky)
  to   os_sensor (origin coordinate system of lidar readings (os_imu and os_lidar) 
*/

#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>


int main(int argc, char** argv){
  ros::init(argc, argv, "tf2_static_husky_lidar");

  static tf2_ros::StaticTransformBroadcaster st_br;
  geometry_msgs::TransformStamped transformStamped;

  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = "base_link";
  transformStamped.child_frame_id = "os_sensor";

  transformStamped.transform.translation.x = -0.232;  
  transformStamped.transform.translation.y = 0.138;
  transformStamped.transform.translation.z = 1.057;

  tf2::Quaternion q;
  q.setRPY(0, 0, -1.571);    //  roll pitch yaw in radians
  transformStamped.transform.rotation.x = q.x();
  transformStamped.transform.rotation.y = q.y();
  transformStamped.transform.rotation.z = q.z();
  transformStamped.transform.rotation.w = q.w();            

  st_br.sendTransform(transformStamped);

  ros::spin();
  return 0;
};
