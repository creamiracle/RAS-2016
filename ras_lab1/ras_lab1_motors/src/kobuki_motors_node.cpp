/*
 *  kobuki_motors_node.cpp
 *
 *
 *  Created on: Aug 26, 2014
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
#include <ras_lab1_msgs/PWM.h>
#include <ras_lab1_msgs/Encoders.h>
#include <ras_lab1_motors/kobuki_motors.h>
#include <geometry_msgs/Twist.h>



class KobukiMotorsNode
{
public:

    ros::NodeHandle n_;
    ros::Subscriber pwm_subscriber_;
    ros::Publisher encoders_publisher_;
    ros::Publisher twist_publisher_;

    KobukiMotorsNode()
    {
        n_ = ros::NodeHandle("~");
        kobuki_motors_ = new KobukiMotors();

        pwm_ = std::vector<int>(2, 0);
        t_pwm_ = ros::Time::now();

        wheel_radius_ = 0.0352;
        base_ = 0.23;

        pwm_subscriber_ = n_.subscribe("pwm", 1, &KobukiMotorsNode::pwmCallback, this);
        encoders_publisher_ = n_.advertise<ras_lab1_msgs::Encoders>("encoders", 1);
        twist_publisher_ = n_.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);
    }

    ~KobukiMotorsNode()
    {
        delete kobuki_motors_;
    }


    // [0] corresponds to left wheel, [1] corresponds to right wheel
    void pwmCallback(const ras_lab1_msgs::PWM::ConstPtr &msg)
    {
        pwm_[0] = msg->PWM1;
        pwm_[1] = msg->PWM2;
        t_pwm_ = ros::Time::now();
    }

    void updateMotors()
    {

        ras_lab1_msgs::Encoders encoders_msg;

        // [0] corresponds to left wheel, [1] corresponds to right wheel
        std::vector<double> wheel_angular_velocities(2, 0.0);
        std::vector<int> abs_encoders(2, 0);
        std::vector<int> diff_encoders(2, 0);


        // if more than 2 seconds have passed and no messages have been received,
        // shutdown the motors
        if((ros::Time::now()-t_pwm_).toSec()>2.0)
        {
            pwm_[0] = 0;
            pwm_[1] = 0;
        }

        kobuki_motors_->update(pwm_, wheel_angular_velocities,
                               abs_encoders, diff_encoders);


        // publish encoders
        encoders_msg.encoder1 = abs_encoders[0];
        encoders_msg.encoder2 = abs_encoders[1];

        encoders_msg.delta_encoder1 = diff_encoders[0];
        encoders_msg.delta_encoder2 = diff_encoders[1];


        encoders_publisher_.publish(encoders_msg);


        // calculate kinematics and send twist to robot simulation node
        geometry_msgs::Twist twist_msg;

        double linear_vel = (wheel_angular_velocities[1] + wheel_angular_velocities[0])*0.5*wheel_radius_;
        double angular_vel = (wheel_angular_velocities[1] - wheel_angular_velocities[0])*wheel_radius_/base_;

        twist_msg.linear.x = linear_vel;
        twist_msg.linear.y = 0.0;
        twist_msg.linear.z = 0.0;

        twist_msg.angular.x = 0.0;
        twist_msg.angular.y = 0.0;
        twist_msg.angular.z = angular_vel;

        twist_publisher_.publish(twist_msg);

    }

private:
    KobukiMotors *kobuki_motors_;

    // [0] corresponds to left wheel, [1] corresponds to right wheel
    std::vector<int> pwm_;
    ros::Time t_pwm_;

    double wheel_radius_;
    double base_;
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "kobuki_motors");

    KobukiMotorsNode kobuki_motors_node;

    // Control @ 10 Hz
    double control_frequency = 10.0;

    ros::Rate loop_rate(control_frequency);

    while(kobuki_motors_node.n_.ok())
    {
        kobuki_motors_node.updateMotors();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
