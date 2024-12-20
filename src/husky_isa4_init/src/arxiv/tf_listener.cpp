/** Listener of broadcasted tf

Using the info from the node that publishes 
the base_lidar → base_link transform (tf_broadcaster.cpp, husky_tf_publisher node),

Now, this node uses that transform to take a point in the "base_lidar" frame
and transform it to a point in the "base_link" frame. 
 */

#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>

// This function acts:
// given a TransformListener, takes a point in the "base_lidar" frame and transforms it to the "base_link" frame.
// This function serves as a callback for the ros::Timer created in the main() and will fire every second.

void transformPoint(const tf::TransformListener& listener){
  //we'll create a point in the base_lidar frame that we'd like to transform to the base_link frame
  //it has to be Stamped bc it needs time header
  geometry_msgs::PointStamped lidar_point;

  //Filling it in..
  lidar_point.header.frame_id = "base_lidar";

  //we'll just use the most recent transform available for our simple example
  lidar_point.header.stamp = ros::Time();

  //just an arbitrary point in space
  lidar_point.point.x = 1.0;
  lidar_point.point.y = 0.2;
  lidar_point.point.z = 0.5;


// Now that we have the point in the "base_lidar" frame we want to transform it into the "base_link" frame.
// To do this, we'll use the TransformListener object, and call transformPoint() with three arguments: 
//    1. frame we want to transform the point to ("base_link" in our case), 
//    2. point we're transforming, 
//    3. storage for the transformed point. 
// 
// So, after the call to transformPoint(), base_point holds the same information as laser_point did before only now in the "base_link" frame. 

  try{
    geometry_msgs::PointStamped base_point;
    listener.transformPoint("base_link", lidar_point, base_point);

    ROS_INFO("base_lidar: (%.2f, %.2f. %.2f) -----> base_link: (%.2f, %.2f, %.2f) at time %.2f",
        lidar_point.point.x, lidar_point.point.y, lidar_point.point.z,
        base_point.point.x, base_point.point.y, base_point.point.z, base_point.header.stamp.toSec());
  }

// If for some reason the base_laser → base_link transform is unavailable 
// (perhaps the tf_broadcaster is not running), the TransformListener may throw an exception
// when we call transformPoint(). 
// To make sure we handle this gracefully, we'll catch the exception and print out an error for the user. 
  catch(tf::TransformException& ex){
    ROS_ERROR("Received an exception trying to transform a point from \"base_lidar\" to \"base_link\": %s", ex.what());
  }
}




int main(int argc, char** argv){
  ros::init(argc, argv, "husky_tf_listener");
  ros::NodeHandle n;
 
 // Creation of TransformListener object, will be input argument to transformPoint function
  tf::TransformListener listener(ros::Duration(10));

  //we'll transform a point once every second
  ros::Timer timer = n.createTimer(ros::Duration(1.0), boost::bind(&transformPoint, boost::ref(listener)));

  ros::spin();

}