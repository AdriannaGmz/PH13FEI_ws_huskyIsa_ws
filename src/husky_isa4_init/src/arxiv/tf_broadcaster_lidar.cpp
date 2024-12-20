/** Broadcaster of tf tree that relates 

 * os_sensor (origin coordinate system of lidar readings (os_imu and os_lidar) 
 * to the base_link (origin/base coordinate system of husky)

os_sensor is located x: -0.232m, y: 0.138m, z: 1.057m from the base_link of the husky and -90deg around Z axis, (pi/2 = 1.571 rad)


Node that publishes the os_sensor → base_link transform over ROS at a certain rate
this eventually gets unsynched, prefer the tf2_broadcaster_lidar.cpp
*/


#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "husky_lidar_tf_pub");
  ros::NodeHandle n;

  ros::Rate r(10000);

  tf::TransformBroadcaster broadcaster;

  while(n.ok()){

// Sending a transform with a TransformBroadcaster requires 4 arguments:
// 1. Rotation transform 
//      a. specified by a btQuaternion for any rotation that needs to occur between the two coordinate frames
//      b. A btVector3 for any translation that we'd like to apply. 
// 2. Timestamp
// 3. name of the parent node of the link we're creating, in this case "base_link."
// 4. name of the child node of the link we're creating, in this case "base_laser", "base_lidar", "os_sensor"

    broadcaster.sendTransform(
      tf::StampedTransform(

        //Transform from base_link tf to os_sensor tf:
        tf::Transform(
                // no rotation-->   tf::Quaternion(0, 0, 0, 1), 
                // but we need   -pi/2 rotation around Z 
                tf::Quaternion(0, 0, -0.7071788, 0.7070348), 
                tf::Vector3(-0.232, 0.138, 1.057)       
                ),
        ros::Time::now(),
        "base_link", "os_sensor"
        )
      );


    r.sleep();
  }
}