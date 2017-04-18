#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

#include <cmath>

int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "uarm_braodcaster");
  ros::NodeHandle node;
  ros::Rate rate = ros::Rate(10); //Hz
  while (node.ok()) {
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(0.070,0.02,0.08));
    tf::Quaternion q;
    q.setRPY( 0, 0,-M_PI/2);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "robot_center", "uarm"));
    rate.sleep();
  }
  return 0;
}
