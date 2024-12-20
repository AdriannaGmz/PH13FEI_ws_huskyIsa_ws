/** Broadcaster of tf tree that relates 
 * os_sensor (origin coordinate system of lidar readings (os_imu and os_lidar) 
 * to the base_link (origin/base coordinate system of husky)

os_sensor is located x: -0.232m, y: 0.138m, z: 1.057m from the base_link of the husky and -90deg around Z axis, (pi/2 = 1.571 rad)

Node that publishes the os_sensor → base_link transform 
every time that a new message is published in IMU topic
*/

#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>



// void pointsCallback(const sensor_msgs::Imu::ConstPtr &msg){
void pointsCallback(const sensor_msgs::PointCloud2ConstPtr& msg){

  static tf2_ros::TransformBroadcaster br;

  geometry_msgs::TransformStamped transformStamped;
  
  // transformStamped.header.stamp = ros::Time::now();
  transformStamped.header=msg->header;

// instead of base_link (robot), we use odom.. and adding its difference..
  transformStamped.header.frame_id = "base_link";



  transformStamped.child_frame_id = "os_sensor";
  transformStamped.transform.translation.x = -0.232;  // this is raw from base_link



// from odom to base_link: rosrun tf tf_echo odom base_link
// - Translation: [0.425, -0.000, 0.000]
// - Rotation: in RPY (radian) [0.000, 0.000, -0.000]

  // transformStamped.header.frame_id = "odom";  
  // transformStamped.transform.translation.x = 0.193;  //+0.425-0.232   //from odom to base_link to lidar




  transformStamped.transform.translation.y = 0.138;
  transformStamped.transform.translation.z = 1.057;
  
  tf2::Quaternion q;
  q.setRPY(0, 0, -1.571);    //  roll pitch yaw in radians

  transformStamped.transform.rotation.x = q.x();
  transformStamped.transform.rotation.y = q.y();
  transformStamped.transform.rotation.z = q.z();
  transformStamped.transform.rotation.w = q.w();            

  br.sendTransform(transformStamped);





// //publishing tf from base_link to scan topic

//   static tf2_ros::TransformBroadcaster br_scan;  

//   geometry_msgs::TransformStamped transformStamped_scan;
//   transformStamped_scan = transformStamped;
//   transformStamped_scan.child_frame_id = "scan";

//   br_scan.sendTransform(transformStamped);



}

int main(int argc, char** argv){
  ros::init(argc, argv, "tf2_broadcaster_husky_lidar");
    
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/os_cloud_node/points", 1000, &pointsCallback);
  // ros::Subscriber sub = n.subscribe("/os_cloud_node/imu", 1000, &pointsCallback);


  ros::spin();
  return 0;
};
