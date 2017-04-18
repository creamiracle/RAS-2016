#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "phidgets/motor_encoder.h"
#include "geometry_msgs/Twist.h"


double desired_w1 = 0;
double desired_w2 = 0;
double estimated_w1 = 0;
double estimated_w2 = 0;
double control_frequency = 125;
double ticks_per_rev = 900;
double wheel_radius = 0.036;
double base = 0.228;
double pi = 3.1415926;

static void velocityCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
  double velocity = msg->linear.x;
  double angular_velocity = msg->angular.z;
  desired_w1 = (velocity - (base / 2)*angular_velocity)/wheel_radius;
  desired_w2 = -(velocity + (base / 2)*angular_velocity)/wheel_radius;

}
static void encoderCallbackleft(const phidgets::motor_encoder::ConstPtr &msg)
{
  double dencoder1 = msg->count_change;
  estimated_w1 = (dencoder1*2*pi*control_frequency)/(ticks_per_rev);
}
static void encoderCallbackright(const phidgets::motor_encoder::ConstPtr &msg)
{
  double dencoder2 = msg->count_change;
  estimated_w2 = (dencoder2*2*pi*control_frequency)/(ticks_per_rev);
}



int main(int argc, char **argv)
{
	double w1_int_error = 0;
	double w2_int_error = 0;
  ros::init(argc, argv,"motor_controller");
  ros::NodeHandle n_("~");
  ros::Publisher publeft = n_.advertise<std_msgs::Float32>("/left/cmd_vel",10);
  ros::Publisher pubright = n_.advertise<std_msgs::Float32>("/right/cmd_vel",10);
  ros::Subscriber twist_subscriber = n_.subscribe("/cmd_vel",10, velocityCallback);
  ros::Subscriber encoder_subscriber_left = n_.subscribe("/left/encoder", 10, encoderCallbackleft);
  ros::Subscriber encoder_subscriber_right = n_.subscribe("/right/encoder", 10, encoderCallbackright);

  double f = 10;
  n_.getParam("f", f);
  ros::Rate loop_rate(f);
  std_msgs::Float32 leftPWM_msgs;
  std_msgs::Float32 rightPWM_msgs;
  leftPWM_msgs.data = 0.0;
  rightPWM_msgs.data = 0.0;
  double I = 0.1;
  n_.getParam("/motor_controller/I", I);
  double c = 20/4.926;
  n_.getParam("c", c);
  double P;
  n_.getParam("P", P);

  while(ros::ok())
    {
    	w1_int_error += (desired_w1 - estimated_w1);
    	w2_int_error += (desired_w2 - estimated_w2);
   	  if (desired_w2 == 0)
        w2_int_error = 0;
   	  if (desired_w1 == 0)
        w1_int_error = 0;
      leftPWM_msgs.data = c * desired_w1 + I * w1_int_error + P * (desired_w1 - estimated_w1);
      rightPWM_msgs.data = c * desired_w2 + I * w2_int_error + P * (desired_w2 - estimated_w2);
      publeft.publish(leftPWM_msgs);
      pubright.publish(rightPWM_msgs);
      ROS_INFO("Des = %f, Est = %f", desired_w1, estimated_w1);
      ros::spinOnce();
      loop_rate.sleep();
     }
  return 0;
}
