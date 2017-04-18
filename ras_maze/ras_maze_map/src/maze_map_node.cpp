/*
 *  world_node.cpp
 *
 *
 *  Created on: Sept 18, 2014
 *  Authors:   Rares Ambrus
 *            raambrus <at> kth.se
 */

/* Copyright (c) 2015, Rares Ambrus, CVAP, KTH
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


// ROS includes.
#include <ros/ros.h>
#include <ros/time.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <tf/tf.h>

// Boost includes
#include <stdio.h>
#include <stdlib.h>



// std includes
#include <limits>
#include <iostream>
#include <fstream>

using namespace std;

int main(int argc, char **argv)
{
    // Set up ROS.
    ros::init(argc, argv, "maze_map_node");
    ros::NodeHandle n("~");
    ros::Rate r(10);


    string _map_file;
    string _map_frame = "/map";
    string _map_topic = "/maze_map";
    n.param<string>("map_file", _map_file, "maze_map.txt");
//    n.param<string>("map_frame", _map_frame, "/map");
//    n.param<string>("map_topic", _map_topic, "/maze_map");

    ROS_INFO_STREAM("Loading the maze map from " << _map_file);
    ROS_INFO_STREAM("The maze map will be published in frame " << _map_frame);
    ROS_INFO_STREAM("The maze map will be published on topic " << _map_topic);

    ifstream map_fs; map_fs.open(_map_file.c_str());
    if (!map_fs.is_open()){
        ROS_ERROR_STREAM("Could not read maze map from "<<_map_file<<". Please double check that the file exists. Aborting.");
        return -1;
    }

    ros::Publisher vis_pub = n.advertise<visualization_msgs::MarkerArray>( _map_topic, 0 );
    visualization_msgs::MarkerArray all_markers;
    visualization_msgs::Marker wall_marker;
    wall_marker.header.frame_id = _map_frame;
    wall_marker.header.stamp = ros::Time();
    wall_marker.ns = "world";
    wall_marker.type = visualization_msgs::Marker::CUBE;
    wall_marker.action = visualization_msgs::Marker::ADD;
    wall_marker.scale.y = 0.01;
    wall_marker.scale.z = 0.2;
    wall_marker.color.a = 1.0;
    wall_marker.color.r = (255.0/255.0);
    wall_marker.color.g = (0.0/255.0);
    wall_marker.color.b = (0.0/255.0);
    wall_marker.pose.position.z = 0.2;




    string line;
    int wall_id = 0;
    while (getline(map_fs, line)){

        if (line[0] == '#') {
            // comment -> skip
            continue;
        }

        double max_num = std::numeric_limits<double>::max();
        double x1= max_num,
               x2= max_num,
               y1= max_num,
               y2= max_num;

        std::istringstream line_stream(line);

        line_stream >> x1 >> y1 >> x2 >> y2;

        if ((x1 == max_num) || ( x2 == max_num) || (y1 == max_num) || (y2 == max_num)){
            ROS_WARN("Segment error. Skipping line: %s",line.c_str());
        }
        // angle and distance
        double angle = atan2(y2-y1,x2-x1);
        double dist = sqrt(pow(x1-x2,2) + pow(y1-y2,2));

        // set pose
        wall_marker.scale.x = std::max(0.01,dist);
        wall_marker.pose.position.x = (x1+x2)/2;
        wall_marker.pose.position.y = (y1+y2)/2;
        wall_marker.text=line_stream.str();
        tf::Quaternion quat; quat.setRPY(0.0,0.0,angle);
        tf::quaternionTFToMsg(quat, wall_marker.pose.orientation);

        // add to array
        wall_marker.id = wall_id;
        all_markers.markers.push_back(wall_marker);
        wall_id++;
    }
    ROS_INFO_STREAM("Read "<<wall_id<<" walls from map file.");

    // Main loop.
    while (n.ok())
    {
        vis_pub.publish(all_markers);
        ros::spinOnce();
        r.sleep();
    }

    return 0;
} 
