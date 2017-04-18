/*
 *  distance_sensor_test_node.cpp
 *
 *
 *  Created on: Aug 4, 2014
 *  Authors:   Francisco Viña
 *            fevb <at> kth.se
 */

/* Copyright (c) 2014, Francisco Viña, CVAP, KTH
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:
      * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
      * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
      * Neither the name of KTH nor the
        names of its contributors may be used to endorse or promote products
        derived from this software without specific prior written permission.

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
   ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
   WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
   DISCLAIMED. IN NO EVENT SHALL KTH BE LIABLE FOR ANY
   DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
   (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
   ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
   SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <ras_lab1_msgs/ADConverter.h>
#include <lab1_distance_sensor/distance_sensor.h>

class DistanceSensorTestNode
{

public:

    ros::NodeHandle n_;
    ros::Subscriber distance_subscriber_;
    ros::Publisher sensor_value_publisher_;

    DistanceSensorTestNode()
    {
        n_ = ros::NodeHandle("~");
        distance_sensor_ = NULL;
    }

    ~DistanceSensorTestNode()
    {
        delete distance_sensor_;
    }

    void init()
    {
        distance_sensor_ = new DistanceSensor();
        distance_subscriber_ = n_.subscribe("distance", 1, &DistanceSensorTestNode::topicCallbackDistance, this);
        sensor_value_publisher_ = n_.advertise<ras_lab1_msgs::ADConverter>("adc", 1);
    }

    void topicCallbackDistance(const std_msgs::Float64::ConstPtr &msg)
    {
        double distance = msg->data;
        double sensor_value = distance_sensor_->sample(distance);

        ras_lab1_msgs::ADConverter adc_msg;
        adc_msg.ch1 = (unsigned int)(sensor_value*(1023/5.0));
        adc_msg.ch3 = 0.0;
        adc_msg.ch4 = 0.0;
        adc_msg.ch5 = 0.0;
        adc_msg.ch6 = 0.0;
        adc_msg.ch7 = 0.0;
        adc_msg.ch8 = 0.0;

        sensor_value_publisher_.publish(adc_msg);
    }

private:

    double distance_;
    DistanceSensor *distance_sensor_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "distance_sensor_test_node");

    DistanceSensorTestNode test_node;


    test_node.init();

    ros::Rate loop_rate(10.0);

    while(test_node.n_.ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
