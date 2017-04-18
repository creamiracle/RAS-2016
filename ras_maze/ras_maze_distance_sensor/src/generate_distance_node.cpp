/*
 *  generate_distance_node.cpp
 *
 *
 *  Created on: Aug 25, 2014
 *  Authors:   Rares Ambrus
 *            raambrus <at> kth.se
 */

/* Copyright (c) 2014, Rares Ambrus, CVAP, KTH
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
#include <tf/transform_listener.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <ras_lab1_msgs/ADConverter.h>
#include <algorithm>
#include <kdl/frames.hpp>
#include <kdl_conversions/kdl_msg.h>

#include <maze_distance_sensor/distance_sensor.h>


class GenerateDistanceNode
{

public:

    ros::NodeHandle n_;
    ros::Subscriber world_subscriber_;
    ros::Publisher dist_sensor_back_vis_pub_, adc_pub_;
    tf::TransformListener transform_listener_;

    std::vector<std::pair<geometry_msgs::Point,geometry_msgs::Point> > maze_map_;
    double max_sensor_range_; // in meters
    double min_sensor_range_;
    double sensor_height_;
    bool wall_initialized_;

    GenerateDistanceNode()
    {
        n_ = ros::NodeHandle("");
        distance_sensor_ = NULL;
        wall_initialized_ = false;

    }

    ~GenerateDistanceNode()
    {
        delete distance_sensor_;
    }

    void init()
    {
        distance_sensor_ = new DistanceSensor();
        world_subscriber_ = n_.subscribe("/maze_map", 1, &GenerateDistanceNode::topicCallbackWallMarker, this);
        dist_sensor_back_vis_pub_ = n_.advertise<visualization_msgs::MarkerArray>( "dist_sensor_markers", 0 );
        max_sensor_range_ = 0.8; // meters
        min_sensor_range_ = 0.1;
        sensor_height_ = 0.1;
        adc_pub_ = n_.advertise<ras_lab1_msgs::ADConverter>("adc", 1);
    }

    void topicCallbackWallMarker(const visualization_msgs::MarkerArray::ConstPtr &msg)
    {

        if (!wall_initialized_)
        {
            if (msg->markers.size()){
                wall_initialized_ = true;
            }
            for (size_t i=0; i<msg->markers.size(); i++){
                // get segment points from marker text (nice hack!!)
                std::istringstream text_stream(msg->markers[i].text);
                double x1, y1, x2, y2;
                text_stream>>x1>>y1>>x2>>y2;
                visualization_msgs::Marker marker = msg->markers[i];

                geometry_msgs::Point wall_start, wall_end;
                wall_start.x = round(x1); wall_start.y = round(y1);
                wall_end.x = round(x2); wall_end.y = round(y2);
                // add to maze
                maze_map_.push_back(std::make_pair(wall_start, wall_end));
            }
        }
    }

    void generateDistance()
    {
        visualization_msgs::MarkerArray marker_array;
        ras_lab1_msgs::ADConverter adc_msg;
        adc_msg.ch1 = 0;
        adc_msg.ch2 = 0;
        adc_msg.ch3 = 0;
        adc_msg.ch4 = 0;
        adc_msg.ch5 = 0;
        adc_msg.ch6 = 0;

        if (!wall_initialized_){
            return;
        }

        try {

            geometry_msgs::Point min_intersection_point;
            double min_distance = std::numeric_limits<double>::max();

            visualization_msgs::Marker marker_back;
            bool intersected= getMarkerForDistanceSensor(marker_back, min_distance, "/distance_sensor_back_left_link");
            if (intersected) {
                double sensor_value = distance_sensor_->sample(min_distance);
                adc_msg.ch2 = (unsigned int)(sensor_value*(1023/5.0));
                marker_array.markers.push_back(marker_back);
            }

            visualization_msgs::Marker marker_front;
            min_distance = std::numeric_limits<double>::max();
            intersected= getMarkerForDistanceSensor(marker_front, min_distance, "/distance_sensor_front_left_link");
            if (intersected) {
                double sensor_value = distance_sensor_->sample(min_distance);
                adc_msg.ch1 = (unsigned int)(sensor_value*(1023/5.0));
                marker_array.markers.push_back(marker_front);
            }

            visualization_msgs::Marker marker_back_right;
            intersected= getMarkerForDistanceSensor(marker_back_right, min_distance, "/distance_sensor_back_right_link");
            if (intersected) {
                double sensor_value = distance_sensor_->sample(min_distance);
                adc_msg.ch4 = (unsigned int)(sensor_value*(1023/5.0));
                marker_array.markers.push_back(marker_back_right);
            }

            visualization_msgs::Marker marker_front_right;
            min_distance = std::numeric_limits<double>::max();
            intersected= getMarkerForDistanceSensor(marker_front_right, min_distance, "/distance_sensor_front_right_link");
            if (intersected) {
                double sensor_value = distance_sensor_->sample(min_distance);
                adc_msg.ch3 = (unsigned int)(sensor_value*(1023/5.0));
                marker_array.markers.push_back(marker_front_right);
            }

            visualization_msgs::Marker marker_forward_right;
            intersected= getMarkerForDistanceSensor(marker_forward_right, min_distance, "/distance_sensor_forward_right_link");
            if (intersected) {
                double sensor_value = distance_sensor_->sample(min_distance);
                adc_msg.ch5 = (unsigned int)(sensor_value*(1023/5.0));
                marker_array.markers.push_back(marker_forward_right);
            }

            visualization_msgs::Marker marker_forward_left;
            intersected= getMarkerForDistanceSensor(marker_forward_left, min_distance, "/distance_sensor_forward_left_link");
            if (intersected) {
                double sensor_value = distance_sensor_->sample(min_distance);
                adc_msg.ch6 = (unsigned int)(sensor_value*(1023/5.0));
                marker_array.markers.push_back(marker_forward_left);
            }


            adc_pub_.publish(adc_msg);

            if (marker_array.markers.size() > 0)
            {
                dist_sensor_back_vis_pub_.publish(marker_array);
            }
        }catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
        }
    }



    static double round(double input){
        //        return floorf(input * 100) / 100;
        return input;
    }

    static bool different(double x1, double x2){
        double epsilon = 0.001;
        if (fabs(x1-x2)>epsilon) return true;
        else return false;
    }

    bool getMarkerForDistanceSensor(visualization_msgs::Marker& marker, double& min_distance, std::string sensor_frame_name){
            tf::StampedTransform sensor_transform;
            transform_listener_.waitForTransform("/map", sensor_frame_name,ros::Time(0), ros::Duration(5.0) );
            transform_listener_.lookupTransform("/map", sensor_frame_name,ros::Time(0), sensor_transform);

            // convert range to odom frame of ref and check for intersection
            geometry_msgs::PointStamped end_point;
            geometry_msgs::PointStamped end_point_out;
            end_point.point.x = 0;
            end_point.point.y = -max_sensor_range_;
            end_point.header.stamp = ros::Time::now();
            end_point.header.frame_id = sensor_frame_name;
            transform_listener_.transformPoint("/map",sensor_transform.stamp_,end_point,sensor_frame_name,end_point_out);

            geometry_msgs::PointStamped start_point;
            geometry_msgs::PointStamped start_point_out;
            start_point.point.x = 0;
            start_point.point.y = 0;
            start_point.header.stamp = ros::Time::now();
            start_point.header.frame_id = sensor_frame_name;
            transform_listener_.transformPoint("/map",sensor_transform.stamp_,start_point,sensor_frame_name,start_point_out);

            end_point_out.point.x = round(end_point_out.point.x);
            end_point_out.point.y = round(end_point_out.point.y);
            start_point_out.point.x = round(start_point_out.point.x);
            start_point_out.point.y = round(start_point_out.point.y);

            geometry_msgs::Point min_intersection_point;
            min_distance = std::numeric_limits<double>::max();

            for (size_t i=0; i<maze_map_.size();i++)
            {
                geometry_msgs::Point intersection_point;
                bool intersected = segment_intersection(maze_map_[i].first,maze_map_[i].second,start_point_out.point,end_point_out.point,intersection_point);

                if (intersected)
                {
                    intersection_point.z = sensor_height_;
                    double distance = point_distance(start_point_out.point,intersection_point);
                    if ((distance > min_sensor_range_) && (distance < max_sensor_range_))
                    {
                        if (distance < min_distance){
                            min_distance = distance;
                            min_intersection_point = intersection_point;
                        }
                    } else{
			if (distance < min_sensor_range_) {
				min_distance=std::numeric_limits<double>::max();
				break;
			}
		    }
                }
            }

            if (min_distance != std::numeric_limits<double>::max()){

                // marker
                marker.header.frame_id = "/map";
                marker.header.stamp = ros::Time::now();
                marker.lifetime = ros::Duration(1/30.0);
                marker.ns = sensor_frame_name;
                marker.id = 0;
                marker.type = visualization_msgs::Marker::LINE_STRIP;
                marker.action = visualization_msgs::Marker::ADD;
                marker.scale.x = 0.01;
                marker.scale.y = 0.1;
                marker.scale.z = 0.1;
                marker.color.a = 1.0;
                marker.color.r = 0.0;
                marker.color.g = 1.0;
                marker.color.b = 1.0;
                marker.points.push_back(start_point_out.point);
                marker.points.push_back(min_intersection_point);

                return true;
            } else{
                return false;

            }
        }



    static  bool segment_intersection(geometry_msgs::Point p1, geometry_msgs::Point p2, geometry_msgs::Point p3, geometry_msgs::Point p4, geometry_msgs::Point& intersection) {



        double x1 = p1.x, x2 = p2.x, x3 = p3.x, x4 = p4.x;
        double y1 = p1.y, y2 = p2.y, y3 = p3.y, y4 = p4.y;

        double d = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
        // If d is zero, there is no intersection
        if (d == 0) {
            return false;
        }

        // Get the x and y
        double pre = (x1*y2 - y1*x2), post = (x3*y4 - y3*x4);
        double x = ( pre * (x3 - x4) - (x1 - x2) * post ) / d;
        double y = ( pre * (y3 - y4) - (y1 - y2) * post ) / d;

        // Check if the x and y coordinates are within both lines

        if ( x < std::min(x1, x2) || x > std::max(x1, x2) ||
             x < std::min(x3, x4) || x > std::max(x3, x4) ) {
            if (different(x,x1) && different(x,x2)){
                return false;
            } else {
            }
        } else {
        }

        if ( y < std::min(y1, y2) || y > std::max(y1, y2) ||
             y < std::min(y3, y4) || y > std::max(y3, y4) ) {
            if (different(y,y1) && different(y,y2)){
                return false;
            }
        }

        // Return the point of intersection
        intersection.x = x;
        intersection.y = y;
        return true;
    }

    static double point_distance(geometry_msgs::Point p1, geometry_msgs::Point p2)
    {
        return sqrt((p1.x - p2.x)*(p1.x - p2.x) + (p1.y - p2.y)*(p1.y - p2.y));
    }

private:

    double distance_;
    DistanceSensor *distance_sensor_;
    std::string map_topic_, map_frame_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "generate_distance_node");
    GenerateDistanceNode distance_node;
    distance_node.init();


    ros::Rate loop_rate(30.0);

    while(distance_node.n_.ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
        distance_node.generateDistance();
    }

    return 0;
}
