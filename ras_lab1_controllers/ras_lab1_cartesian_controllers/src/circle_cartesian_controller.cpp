#include<ros/ros.h>
#include<geometry_msgs/Twist.h>
#include<cmath>

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "circle_control");
  ros::NodeHandle handle;
  ros::Publisher twist_pub = handle.advertise<geometry_msgs::Twist>("motor_controller/twist", 1000);
  geometry_msgs::Twist twist_;
  
  double circleRad;
  double time_;

  handle.param<double>("circle/radius", circleRad, 0.5);
  handle.param<double>("circle/time", time_, 10.0);
  
  twist_.linear.x = (2*M_PI*circleRad)/time_;
  twist_.angular.z = (2*M_PI)/time_;
  ROS_INFO_STREAM("TIME"<<time_<<"radius"<<circleRad);
  ROS_INFO_STREAM("linear"<< twist_.linear.x);
  ROS_INFO_STREAM("angular"<< twist_.angular.z);
  ros::Rate loopRate(10);

  while(ros::ok())
    {
      twist_pub.publish(twist_);
      //ros::spinOnce();
      loopRate.sleep();
    }
  return 0;
}
