#include<ros/ros.h>
#include<std_msgs/Float64.h>


int main(int argc,char **argv)
{
  ros::init(argc, argv, "sensor_calib");
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<std_msgs::Float64>("/distance_sensor_test_node/distance",1000);
  ros::Rate loop_rate(10);
  float count = 0.0;
  while(ros::ok())
    {
      std_msgs::Float64 msg;
      if(count==81.0)
	{
	  count = 0.0;
	}
      msg.data = count/100.00;
      ROS_INFO("%f",msg.data);
      pub.publish(msg);
      ros::spinOnce();
      loop_rate.sleep();
      ++count;
    }
  return 0;
}
