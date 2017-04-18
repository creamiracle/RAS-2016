#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

#define PI 3.1415
int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "laser_braodcaster");
  ros::NodeHandle node;
  ros::Rate rate = ros::Rate(10); //Hz
  while (node.ok()) {
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(0.11,0.0,0.09));
    tf::Quaternion q;
    q.setRPY(0,PI/180*17.5,0);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "robot_center", "camera_link"));
    rate.sleep();
  }
  return 0;
}
