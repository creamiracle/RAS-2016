#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "line_control");
    ros::NodeHandle handle;
    ros::Publisher pub_twist = handle.advertise<geometry_msgs::Twist>("/motor_controller/twist", 1000);
    
    geometry_msgs::Twist t;
    t.linear.x = 0.2;
        
    ros::Rate loopRate(10);
    
    while (ros::ok()){
	pub_twist.publish(t);
	loopRate.sleep();
    }

    return 0;
}

