/*    Static tf2 Broadcaster
publishes transform:      base_link → camth_link
Thermal camera Boson Flir

*/

#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>


int main(int argc, char** argv){
  ros::init(argc, argv, "tf2_static_husky_camth");

  static tf2_ros::StaticTransformBroadcaster st_br;
  geometry_msgs::TransformStamped transformStamped;

  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = "base_link";
  transformStamped.child_frame_id = "boson_camera";
  // transformStamped.child_frame_id = "camth_link";

  transformStamped.transform.translation.x = 0.260;  
  transformStamped.transform.translation.y = 0.0;
  transformStamped.transform.translation.z = 0.650;//0.800

  tf2::Quaternion q;
  // q.setRPY(0, 1.571, 0);    //  roll pitch yaw in radians
  q.setRPY(-1.571, 0, -1.571);    //  roll pitch yaw in radians
  transformStamped.transform.rotation.x = q.x();
  transformStamped.transform.rotation.y = q.y();
  transformStamped.transform.rotation.z = q.z();
  transformStamped.transform.rotation.w = q.w();            

  st_br.sendTransform(transformStamped);

  ros::spin();
  return 0;
};

// rosrun tf static_transform_publisher x y z yaw pitch roll frame_id child_frame_id period_in_ms
// rosrun tf static_transform_publisher 0 0 0 0 0 0 base_link boson_camera 100
// rosrun tf static_transform_publisher 0.26 0 0.5 -1.571 0 -1.571 base_link boson_camera 100   // parece q es esta

// rosrun tf static_transform_publisher 0.26 0.0   0.65 -1.571 0 -1.571 base_link boson_camera 100
// rosrun tf static_transform_publisher 0.26 0.20  0.65 -1.571 0 -1.571 base_link cam_0_optical_frame 100   
// rosrun tf static_transform_publisher 0.26 -0.05 0.65 -1.571 0 -1.571 base_link cam_1_optical_frame 100   
