#include "ros/ros.h"
#include "ras_lab1_msgs/PWM.h"

ros::Publisher pub_PWM;


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "open_loop");
    ros::NodeHandle handle;
    
    pub_PWM = handle.advertise<ras_lab1_msgs::PWM>("/kobuki/pwm", 1000);
    
    ros::Rate loopRate(10);
    
    ras_lab1_msgs::PWM pwm_; // header is automatically filled
    handle.param<int>("/open_loop/left_set", pwm_.PWM1, 255);
    handle.param<int>("/open_loop/right_set", pwm_.PWM2, 255);
    

    while (ros::ok()){
	pub_PWM.publish(pwm_);
	loopRate.sleep();
    }
    
    return 0;
}
