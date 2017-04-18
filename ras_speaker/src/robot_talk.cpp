#include "ros/ros.h"
#include "std_msgs/String.h"
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <string>

using namespace std;

void tts(string say)
{
    string say_out = string("espeak \"") + say + string("\"");
    system(say_out.c_str());
    ROS_INFO("%s", say_out.c_str());

} 

void chatterCallback(const std_msgs::String::ConstPtr& msg){tts(msg->data);}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "robot_talk");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("robot/talk", 5, chatterCallback);
  ros::spin();
  return 0;
}

