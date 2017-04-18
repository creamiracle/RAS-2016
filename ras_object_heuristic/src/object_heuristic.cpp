#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <ras_object_heuristic/PopObject.h>

struct TargetObject {
    geometry_msgs::PoseStamped pose;
    double score;
};

std::vector<TargetObject> objects;

bool popObjectCallback(ras_object_heuristic::PopObject::Request &req, ras_object_heuristic::PopObject::Response &res) {
    res.object_position = (*objects.begin()).pose;
    objects.erase(objects.begin());
    return true;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "object_heuristic");
    ros::NodeHandle n;
    ros::ServiceServer pop_server = n.advertiseService("pop_object", popObjectCallback);

    // Load objects

    ros::spin();
}