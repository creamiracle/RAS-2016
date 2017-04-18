#include <ros/ros.h>
#include <tf/transform_broadcaster.h>


void poseCallback(const geometry_msgs::TwistConstPtr &msg){
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(msg->linear.x, msg->linear.y, msg->linear.z) );
  tf::Quaternion q;
  q.setRPY( msg->angular.x,  msg->angular.y, msg->angular.z);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "odometry"));
}

int main(int argc, char** argv){
  ros::init(argc, argv, "pose_broadcaster");
  ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe("robot8/pose", 10, &poseCallback);
  ros::spin();
  return 0;
};
