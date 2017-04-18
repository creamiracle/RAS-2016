#include<ros/ros.h>
#include<ras_lab1_msgs/PWM.h>
#include<ras_lab1_msgs/Encoders.h>
#include<cmath>
#include<geometry_msgs/Twist.h>



enum WHEEL
  {
    LEFT_WHEEL,
    RIGHT_WHEEL,
  };

//global var
ros::Publisher pwm_pub;
ros::Subscriber enc_sub;
ros::Subscriber twi_sub;

int CONTROL_FREQ;
int TICKS_PER_REV;
double WHEEL_BASELINE;
double WHEEL_RADIUS;

//real vel
float LEFT_ANGULAR_VEL;
float RIGHT_ANGULAR_VEL;

//estimate vel
float LEFT_EST_VEL;
float RIGHT_EST_VEL;

//alpha
double LEFT_ALPHA;
double RIGHT_ALPHA;

float computeAngularVel(float linearVel, float angularVel, WHEEL wheel)
{
  if(wheel == LEFT_WHEEL)
    {
      return ((linearVel - (WHEEL_BASELINE/2) * angularVel)) / WHEEL_RADIUS;
    }
  else
    {
      return ((linearVel + (WHEEL_BASELINE/2) * angularVel)) / WHEEL_RADIUS;
    }
}

void encoderFeedback(const ras_lab1_msgs::Encoders::ConstPtr& msg)
{
  LEFT_EST_VEL = (float)(msg->delta_encoder1 * 2 * M_PI * CONTROL_FREQ) / TICKS_PER_REV;
  RIGHT_EST_VEL = (float)(msg->delta_encoder2 * 2 *M_PI * CONTROL_FREQ) / TICKS_PER_REV;
  std::cout<<"left estimate vel is:" << LEFT_EST_VEL << "right is:" << RIGHT_EST_VEL <<std::endl;
}

void twistFeedback(const geometry_msgs::Twist::ConstPtr& msg)
{
  LEFT_ANGULAR_VEL = computeAngularVel(msg->linear.x, msg->angular.z, LEFT_WHEEL);
  RIGHT_ANGULAR_VEL = computeAngularVel(msg->linear.x, msg->angular.z, RIGHT_WHEEL);
  std::cout<<"left angular vel is:"<<LEFT_ANGULAR_VEL<<"right is :" << RIGHT_ANGULAR_VEL<<std::endl;
}

void initParameter(ros::NodeHandle handle)
{
  handle.param<double>("/robot/wheel_baseline",WHEEL_BASELINE, 0.23);
  handle.param<double>("/robot/wheel_radius", WHEEL_RADIUS, 0.0352);
  handle.param<double>("/robot/left_alpha", LEFT_ALPHA, 0.5);
  handle.param<double>("/robot/right_alpha", RIGHT_ALPHA, 0.5);
  handle.param<int>("/controller/control_freq", CONTROL_FREQ, 10);
  handle.param<int>("/controller/ticks_per_rev", TICKS_PER_REV, 360);
}

int main(int argc, char *argv[])
{
  ros::init(argc,argv,"motor_controller");
  ros::NodeHandle handle;
  initParameter(handle);

  pwm_pub = handle.advertise<ras_lab1_msgs::PWM>("/kobuki/pwm", 1000);
  enc_sub = handle.subscribe("/kobuki/encoders", 1000, encoderFeedback);
  twi_sub = handle.subscribe("/motor_controller/twist", 1000, twistFeedback);
  
  ros::Rate loopRate(CONTROL_FREQ);
  ras_lab1_msgs::PWM PWM_;

  while(ros::ok())
    {
      
      float PWM1_ = PWM_.PWM1 + LEFT_ALPHA * (LEFT_ANGULAR_VEL - LEFT_EST_VEL);
      float PWM2_ = PWM_.PWM2 + RIGHT_ALPHA * (RIGHT_ANGULAR_VEL - RIGHT_EST_VEL);
      std::cout<<"PWM1"<< PWM1_<<"PWM2"<<PWM2_<<std::endl;
      if(PWM1_ >255 || PWM2_ > 255)
	{
	  PWM1_ = PWM1_/(std::max(PWM1_,PWM2_)) * 255;
	  PWM2_ = PWM2_/(std::max(PWM1_,PWM2_)) * 255;
	}
      PWM_.PWM1 = PWM1_;
      PWM_.PWM2 = PWM2_;
      std::cout << "publish PWM1:" << PWM_.PWM1 << "PWM2:" << PWM_.PWM2 << std::endl;
      pwm_pub.publish(PWM_);
      
      ros::spinOnce();
      loopRate.sleep();
    }


  return 0;
}
