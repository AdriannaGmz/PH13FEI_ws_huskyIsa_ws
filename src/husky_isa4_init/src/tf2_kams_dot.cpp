/*    Static tf2 Broadcaster
publishes transform:      base_link → dotcube_link
base_link is Husky origin

*/

#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>


int main(int argc, char** argv){
  ros::init(argc, argv, "tf2_static_dot");

  static tf2_ros::StaticTransformBroadcaster st_br_dotcube;
  geometry_msgs::TransformStamped transformDotcube;

  transformDotcube.header.stamp = ros::Time::now();
  transformDotcube.header.frame_id = "base_link";
  transformDotcube.child_frame_id = "dotcube_link";

  transformDotcube.transform.translation.x = 0;  
  transformDotcube.transform.translation.y = -0.103;
  transformDotcube.transform.translation.z = 0.286;    

  tf2::Quaternion q;
  q.setRPY(0, 0, 0);    //  roll pitch yaw in radians
  transformDotcube.transform.rotation.x = q.x();
  transformDotcube.transform.rotation.y = q.y();
  transformDotcube.transform.rotation.z = q.z();
  transformDotcube.transform.rotation.w = q.w();            

  st_br_dotcube.sendTransform(transformDotcube);

  ros::spin();
  return 0;
};
