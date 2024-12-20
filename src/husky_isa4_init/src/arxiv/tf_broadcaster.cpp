/** Broadcaster of tf tree that relates 

 * lidar readings (from its own coordinate system base_lidar) 
 * to the base_link (origin/base coordinate system of husky)

base_lidar is located x: -0.232m, y: 0.138m, z: 1.057m from the base_link of the husky

Node that publishes the base_lidar → base_link transform over ROS.

From http://wiki.ros.org/navigation/Tutorials/RobotSetup/TF
 */
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "husky_tf_publisher");
  ros::NodeHandle n;

  ros::Rate r(100);

  tf::TransformBroadcaster broadcaster;

  while(n.ok()){

// Sending a transform with a TransformBroadcaster requires five arguments:
// 1. Rotation transform, which is specified by a btQuaternion for any rotation that needs to occur between the two coordinate frames
// 2. A btVector3 for any translation that we'd like to apply. 
// 3. Timestamp
// 4. name of the parent node of the link we're creating, in this case "base_link"
// 5. name of the child node of the link we're creating, in this case "base_laser", "base_lidar", "os_sensor"

    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), 
        tf::Vector3(-0.232, 0.138, 1.057)),
        ros::Time::now(),"base_link", "base_lidar"));

    r.sleep();
  }
}