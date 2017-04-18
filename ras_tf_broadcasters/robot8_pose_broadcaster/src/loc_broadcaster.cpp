#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>

void poseCallback(const geometry_msgs::PoseStampedConstPtr &msg){
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z) );
  tf::Quaternion q;
  tf::quaternionMsgToTF(msg->pose.orientation, q);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "robot_center"));
}

int main(int argc, char** argv){
  ros::init(argc, argv, "pose_broadcaster");
  ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe("localization/pose", 10, &poseCallback);
  ros::spin();
  return 0;
};
