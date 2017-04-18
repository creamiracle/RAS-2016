#include<ros/ros.h>
#include<geometry_msgs/Twist.h>
#include<cmath>
#include<ras_lab1_msgs/ADConverter.h>

double ANGULAR_VEL;
double LINEAR_VEL;
double ANGULAR_ALPHA;

void adcFeedback(const ras_lab1_msgs::ADConverter::ConstPtr& msg)
{
  ANGULAR_VEL = ANGULAR_ALPHA * (msg->ch2 - msg->ch1);
  std::cout<<"ch1: "<<msg->ch1<<"ch2: "<< msg->ch2<< "vel"<<ANGULAR_VEL<<std::endl;
}

int main(int argc, char *argv[])
{
  ros::init(argc,argv,"wall_follow");
  ros::NodeHandle handle;
  ros::Subscriber adc_scrib = handle.subscribe("/kobuki/adc",1000,adcFeedback);
  ros::Publisher twist_pub = handle.advertise<geometry_msgs::Twist>("motor_controller/twist", 1000);
  
  geometry_msgs::Twist twist_;
  
  handle.param<double>("wall_follow/linear_vel", LINEAR_VEL,0.3);
  handle.param<double>("wall_follow/angular_alpha", ANGULAR_ALPHA,0.002);
  ROS_INFO_STREAM("LINEAR VEL"<<LINEAR_VEL);
  ros::Rate loopRate(10);
  while(ros::ok())
    {
      twist_.linear.x = LINEAR_VEL;
      twist_.angular.z = ANGULAR_VEL;
      twist_pub.publish(twist_);

      ros::spinOnce();
      loopRate.sleep();
    }
  return 0;
}
