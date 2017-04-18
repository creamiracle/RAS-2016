#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

#define PI 3.14159265359
int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "laser_braodcaster");
  ros::NodeHandle node;
  ros::Rate rate = ros::Rate(10); //Hz
  while (node.ok()) {
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(-0.065,0,0.0));
    tf::Quaternion q;
    q.setRPY( 0, 0, -PI/2);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "robot_center", "laser"));
    rate.sleep();
  }
  return 0;
}
