#include <ros/ros.h>
#include <ras_path_plan/path_srv.h>
#include <ras_follower/moveTo.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Path.h>
#include <ras_brain/GoToLocationAction.h>
#include <ras_brain/GoToLocationFeedback.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/simple_client_goal_state.h>
#include <ras_follower/FollowLineAction.h>
#include <std_srvs/Empty.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <ras_color_detection/FPose.h>
#include <ras_uarm_controller/Put.h>
#include <ras_target_recognition/Line.h>
#include <ras_mapping/line.h>
#include <ras_object_list/PopObject.h>
#include <ras_follower/Rotate.h>

// Code for testing
/*#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>*/
// End of code for testing

#include <ctime>
#include <algorithm>

#define DEBUG 0
#define EXPLORE 1
#define RESCUE 2

struct PointXYD {
    double x;
    double y;
    int d;
};

typedef actionlib::SimpleActionServer<ras_brain::GoToLocationAction> GOTOServer;

ros::ServiceClient path_plan_client;
ros::ServiceClient move_to_client;
ros::ServiceClient abort_follow_client;
ros::ServiceClient booby_pickup_client;
ros::ServiceClient object_pickup_client;
ros::ServiceClient release_object_client;
ros::ServiceClient update_map_client;
ros::ServiceClient clean_map_client;
ros::ServiceClient update_loc_map_client;
ros::ServiceClient add_battery_client;
ros::ServiceClient pop_object_client;
ros::ServiceClient robot_rotate_client;
geometry_msgs::PoseStamped current_pose;
geometry_msgs::PoseStamped current_object_center;
nav_msgs::OccupancyGrid map;
nav_msgs::OccupancyGrid map_static;
nav_msgs::OccupancyGrid grid;
actionlib::SimpleActionClient<ras_brain::GoToLocationAction>* exp_client;
ros::Publisher go_pub;
geometry_msgs::PoseStamped current_booby_pose;
std::vector<bool> isROI;
geometry_msgs::Point current_goal;
ras_mapping::line latest_battery;

// Machine states
int STATE;

const int IDLE = 0;
const int PLAN_EXPLORE = 1;
const int FOLLOWING_LINE = 2;
const int LIFTING_BOOBY = 3;
const int LAYING_BOOBY = 4;
const int LIFTING_OBJECT = 5;
const int LAYING_OBJECT = 6;
const int RESUMING_LAST_GOAL = 7;
const int LOOKING_FOR_QR_OR_WALL = 8;
const int RETRIEVE_NEXT_OBJECT = 9;
const int SEARCHING_OBJECT = 10;
// End of machine states

// Follower states
const int FOLLOWER_IDLE = 0;
const int FOLLOWER_ACTIVE = 1;
const int FOLLOWER_ABORTED_BY_SELF = -1;
const int FOLLOWER_ABORTED_BY_USER = 2;
const int FOLLOWER_ROTATING = 8;
//End of follower states

double x_exit, y_exit;
int MODE;                       // MODE = 0: Debug. MODE = 1: Explore. MODE = 2: Rescue
int brush_radius;
double debug_tolerance;
int follower_state;
int arm_state;
bool mShouldExplore;
bool mCarryingBooby = false;
bool mCarryingObject = false;
bool mExiting = false;
double obj_looking_dist;
double booby_diam;
double booby_window_height;
double booby_window_width;
double robot_width;
double ROI_tolerance;
int current_goal_index = -1;
nav_msgs::Path path_to_follow;

void sendGOsignal() {
    std_msgs::String goSignal;
    goSignal.data = "go";
    go_pub.publish(goSignal);
}

bool goService(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
    if(MODE == EXPLORE)
        STATE = PLAN_EXPLORE;
    else if(MODE == RESCUE)
        STATE = RETRIEVE_NEXT_OBJECT;
    sendGOsignal();
    return true;
}

bool compareByDistance (const PointXYD &a, const PointXYD &b) {
    return a.d > b.d;
}

void map_callback(const nav_msgs::OccupancyGrid::ConstPtr &msg) {
    map_static = *msg;
}

void grid_callback(const nav_msgs::OccupancyGrid::ConstPtr &msg) {
    grid = *msg;
}

void abortPathFollower() {
    std_srvs::Empty e;
    abort_follow_client.call(e);
}

bool retrievePath(double goalx, double goaly, nav_msgs::Path &output) {
    ras_path_plan::path_srv plan_this;
    plan_this.request.startx = current_pose.pose.position.x;
    plan_this.request.starty = current_pose.pose.position.y;
    plan_this.request.goalx = goalx;
    plan_this.request.goaly = goaly;
    ros::Rate rate(2); // WTF is this
    bool outcome = path_plan_client.call(plan_this);
    output = plan_this.response.planned_path;
    return outcome;
}

bool followLine(geometry_msgs::PoseStamped goal, double tolerance, bool lastFlag) {
    ras_follower::moveTo srv_msg;
    std_msgs::Bool isLast;
    isLast.data = lastFlag;
    srv_msg.request.goal = goal;
    srv_msg.request.tolerance = tolerance;
    srv_msg.request.turnOffMotors = isLast;
    return move_to_client.call(srv_msg);
}

void paintCircle(int xp, int yp, int brush_radius) {
    for (int y = std::max(yp-brush_radius,0); y < std::min(yp+brush_radius+1,(int)map.info.height); ++y) {
        for (int x = std::max(xp-brush_radius,0); x < std::min(xp+brush_radius+1,(int)map.info.width); ++x) {
            double dist = std::sqrt(std::pow(y-yp,2) + std::pow(x-xp,2));
            if (dist < brush_radius)
                map.data[y*map.info.width + x] = 100;
        }
    }
}

void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    current_pose = *msg;
    if (MODE == EXPLORE) {
        // Draw circle with current position
        paintCircle(std::floor(current_pose.pose.position.x/(double)map.info.resolution),
                    std::floor(current_pose.pose.position.y/(double)map.info.resolution),
                    brush_radius);
    }
}

void goto_execute(const ras_brain::GoToLocationGoalConstPtr& goal, GOTOServer* as) {
    ROS_INFO("Going with tolerance = %f", goal->tolerance);
    ros::Rate loop_rate(3);
    path_to_follow.poses.clear();
    retrievePath(goal->goal.linear.x, goal->goal.linear.y, path_to_follow);
    if(path_to_follow.poses.size() == 0){
      ROS_ERROR("NO path to follow... :(");
      as->setAborted();
      return;
    }
    // TODO: Convert to xy + orientation

     for(int i = 0; i < path_to_follow.poses.size(); i++)
       isROI.push_back(false);

    *(isROI.end()-1) = true;  // Such a pretty instruction

    for (int next_pose = 1; next_pose < path_to_follow.poses.size(); ++next_pose) {
        ras_brain::GoToLocationFeedback fb;
        geometry_msgs::Twist next;
        next.linear.x = path_to_follow.poses[next_pose].pose.position.x;
        next.linear.y = path_to_follow.poses[next_pose].pose.position.y;
        fb.next = next;
        as->publishFeedback(fb);
        ROS_INFO("Going to point %d of %d", next_pose+1, (int)path_to_follow.poses.size());
        double tTolerance;
        if (!isROI[next_pose])
            tTolerance = goal->tolerance;
        else
            tTolerance = ROI_tolerance;
        if(!followLine(path_to_follow.poses[next_pose], tTolerance, isROI[next_pose])){
            ROS_ERROR("followLine = false. Aborting...");
            as->setAborted();
            STATE = LOOKING_FOR_QR_OR_WALL;
            current_goal_index = -1;
            return;
        } else {
            current_goal_index = next_pose;
            do {
                loop_rate.sleep();
                if(as->isPreemptRequested()) {
                    abortPathFollower();
                    as->setPreempted();
                    current_goal_index = -1;
                    return;
                }
                // ros::spinOnce(); not needed
            } while(ros::ok() && (follower_state == FOLLOWER_ACTIVE || follower_state == FOLLOWER_ROTATING));
        }
        if (follower_state == FOLLOWER_ABORTED_BY_SELF || follower_state == FOLLOWER_ABORTED_BY_USER) {
            as->setAborted();
            STATE = LOOKING_FOR_QR_OR_WALL;
            current_goal_index = -1;
            return;
        }
    }
    if (follower_state == FOLLOWER_IDLE) {
        ROS_INFO("Done.");
        as->setSucceeded();
    } else {
        ROS_WARN("Action interrupted by follower.");
        as->setAborted();
    }
    current_goal_index = -1;
}

void go_home() {
    ROS_INFO("Got goal (exit). Starting action");
    actionlib::SimpleActionClient<ras_brain::GoToLocationAction> own_client("/brain/goto_location", true);
    ras_brain::GoToLocationGoal goal;
    goal.goal.linear.x = x_exit;
    goal.goal.linear.y = y_exit;
    goal.tolerance = debug_tolerance;
    own_client.sendGoal(goal);
}

bool exitMaze (std_srvs::Empty::Request &request, std_srvs::Empty::Response &response) {
    go_home();
}

void findNearestFree(int &x, int &y) {
   
    int k = 0;
    int i = 0;
    while(1) {
        i = 0;
        while (i <= k) {
            if((x + i) < map.info.width && (y + k) < map.info.height)
            {
                if(map.data[(x + i)+map.info.width*(y + k)]!= 0) 
                {
                    x += i;
                    y += k;
                    return;
                }
            }
            if((x - i) > 0 && (y + k) < map.info.height)
            {
                if(map.data[(x - i)+map.info.width*(y + k)]!= 0) 
                {
                    x -= i;
                    y += k;
                    return;
                } 
            }
            if((x + k) < map.info.width && (y + i) < map.info.height)
            {
                if(map.data[(x + k)+map.info.width*(y + i)]!= 0) 
                {
                    x += k;
                    y += i;
                    return;
                }
            }
            if((x + k) < map.info.width && (y - i) > 0)
            {
                if(map.data[(x + k)+map.info.width*(y - i)]!= 0) 
                {
                    x += k;
                    y -= i;
                    return;
                }
            }
            if((x + i) < map.info.width && (y - k) > 0)
            {
                if(map.data[(x + i)+map.info.width*(y - k)]!= 0) 
                {
                    x += i;
                    y -= k;
                    return;
                }
            }
            if((x - i) > 0 && (y - k) > 0)
            {
                if(map.data[(x - i)+map.info.width*(y - k)]!= 0) {
                    x -= i;
                    y -= k;
                    return;
                }
            }
            if((x - k) > 0 && (y + i) < map.info.height)
            {
                if(map.data[(x - k)+map.info.width*(y + i)]!= 0) 
                {
                    x -= k;
                    y += i;
                    return;
                }
            }
            if((x - k) > 0 && (y - i) > 0)
            {
                if(map.data[(x - k)+map.info.width*(y - i)]!= 0) 
                {
                    x -= k;
                    y -= i;
                    return;
                }
            }
            i++;
        }
        k++;
    }
}   


void createExplorationPlan(std::vector<PointXYD> &output) {
    int x_init = std::floor(current_pose.pose.position.x/map_static.info.resolution);
    int y_init = std::floor(current_pose.pose.position.y/map_static.info.resolution);
    findNearestFree(x_init, y_init);
    if(map_static.data[y_init*map_static.info.width+x_init]>0)
        ROS_WARN("Duh!");    
    std::vector<int> distance_map;
    distance_map.resize(map_static.data.size());
    for (int i = 0; i < map_static.data.size(); ++i) {
        if(map_static.data[i] == 0)
            distance_map[i] = 0;
        else
            distance_map[i] = -1;
    }

    distance_map[y_init*map_static.info.width + x_init] = 1;
    int nrOfZeros = 0;
    int nrOfZerosLastTime = -1;
    while(nrOfZeros != nrOfZerosLastTime) {
        nrOfZerosLastTime = nrOfZeros;
        nrOfZeros = 0;
        //ROS_INFO("%d zeros remaining", nrOfZerosLastTime);
        //done = true;
        for (int y = 0; y < map_static.info.height; ++y) {
            for (int x = 0; x < map_static.info.width; ++x) {
                if(distance_map[y*map_static.info.width + x] == 0) {
                    nrOfZeros++;
                    //done = false;
                }
                else {
                    for (int j = std::max(y-1,0); j < std::min(y+2,(int)map_static.info.height); ++j) {
                        for (int i = std::max(x-1,0); i < std::min(x+2,(int)map_static.info.width); ++i) {
                            if (distance_map[j*map_static.info.width + i] == 0){
                                distance_map[j*map_static.info.width + i] = distance_map[y*map_static.info.width + x] + 1;

                            }
                        }
                    }
                }
            }
        }
    }
    for (int y = 0; y < map_static.info.height; ++y) {
        for (int x = 0; x < map_static.info.width; ++x) {
            if (distance_map[y*map_static.info.width + x] > 0)
                output.push_back((PointXYD){(double)x*map_static.info.resolution,
                                            (double)y*map_static.info.resolution,
                                            distance_map[y*map_static.info.width + x]});
        }
    }
    std::sort(output.begin(), output.end(), compareByDistance);
}

bool gotoLocation_block(double x_goal, double y_goal, double d) {
    ROS_INFO("Got goal (blocking). Starting action");
    actionlib::SimpleActionClient<ras_brain::GoToLocationAction> own_client("/brain/goto_location", true);
    ras_brain::GoToLocationGoal goal;
    goal.goal.linear.x = x_goal;
    goal.goal.linear.y = y_goal;
    own_client.sendGoal(goal);
    return own_client.waitForResult(ros::Duration(d));
}

actionlib::SimpleActionClient<ras_brain::GoToLocationAction>* gotoLocation_non_block(double x_goal, double y_goal, double tolerance) {
    ROS_INFO("Got goal (non-blocking). Starting action");
    actionlib::SimpleActionClient<ras_brain::GoToLocationAction>* own_client;
    // TODO: Possible memory leak?
    own_client = new actionlib::SimpleActionClient<ras_brain::GoToLocationAction>("/brain/goto_location", true);
    ras_brain::GoToLocationGoal goal;
    goal.goal.linear.x = x_goal;
    goal.goal.linear.y = y_goal;
    goal.tolerance = tolerance;

    own_client->sendGoal(goal);
    return own_client;
}

void rvizGoalCallback(const geometry_msgs::PoseStamped::ConstPtr& rviz_goal) {
    if (MODE == DEBUG) {
        ROS_INFO("Got goal. Starting action");
        current_goal.x = rviz_goal->pose.position.x;
        current_goal.y = rviz_goal->pose.position.y;
        if (exp_client != NULL)
            delete(exp_client);
        exp_client = gotoLocation_non_block(rviz_goal->pose.position.x, rviz_goal->pose.position.y, debug_tolerance);
        STATE = FOLLOWING_LINE;
    }
}

bool addPoseToPath(ras_color_detection::FPose::Request &req, ras_color_detection::FPose::Response &res) {
    // TODO: Check repeated addpose requests?
    double min_dist_double = 100;
    double min_dist_single = 100;
    int min_index = -1;
    if(path_to_follow.poses.size() == 0 || current_goal_index < 0) {
      return false; //NO Path planning done!!! :(
    }
    for (std::vector<geometry_msgs::PoseStamped>::iterator path_pose = path_to_follow.poses.begin() + current_goal_index; path_pose != path_to_follow.poses.end() - 1; ++path_pose) {
        int i = path_pose - path_to_follow.poses.begin();
        double dist1 = std::pow(req.x.data - (float)(path_pose->pose.position.x), 2) +
                       std::pow(req.y.data - (float)(path_pose->pose.position.y), 2);
        double dist2 = std::pow(req.x.data - (float)((path_pose+1)->pose.position.x), 2) + 
                       std::pow(req.y.data - (float)((path_pose+1)->pose.position.y), 2);
        double dist = dist1 + dist2;
        if (dist < min_dist_double) {
            min_dist_double = dist;
            min_index = i;
        }
    }
    if(min_index < 0)
        return false;
    // Create the new waypoint
    double theta = atan2(req.y.data - (float)path_to_follow.poses[min_index].pose.position.y,
                         req.x.data - (float)path_to_follow.poses[min_index].pose.position.x);
    double t_max = std::sqrt(std::pow(req.y.data - (float)path_to_follow.poses[min_index].pose.position.y, 2) + 
                             std::pow(req.x.data - (float)path_to_follow.poses[min_index].pose.position.x, 2));
    geometry_msgs::PoseStamped to_add;
    to_add.pose.position.x = path_to_follow.poses[min_index].pose.position.x + (t_max - obj_looking_dist) * cos(theta);
    to_add.pose.position.y = path_to_follow.poses[min_index].pose.position.y + (t_max - obj_looking_dist) * sin(theta);
    to_add.pose.orientation = tf::createQuaternionMsgFromYaw(theta);
    path_to_follow.poses.insert(path_to_follow.poses.begin() + min_index + 1, to_add);
    isROI.insert(isROI.begin() + min_index + 1, true);
    return true;
}

void followerStateCallback(const std_msgs::Int32::ConstPtr &msg) {
    follower_state = msg->data;
}

void armStateCallback(const std_msgs::Int32::ConstPtr &msg) {
    arm_state = msg->data;
}

bool isReachable(geometry_msgs::PoseStamped target) {
    return target.pose.position.x > 0 &&
           target.pose.position.x < 0.4 && 
           target.pose.position.y > -0.2 &&
           target.pose.position.y < 0.2;
}

void rubbleCallback(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    if (STATE != LOOKING_FOR_QR_OR_WALL)
        return;
    tf::TransformListener tListen;
    geometry_msgs::PoseStamped to_robot;
    geometry_msgs::PoseStamped to_change = *msg;
    to_change.header.stamp = ros::Time(0);
    try {
        tListen.waitForTransform("robot_center",
                                 to_change.header.frame_id,
                                 to_change.header.stamp,
                                 ros::Duration(1));
        tListen.transformPose("robot_center", to_change, to_robot);
    } catch (tf::TransformException ex) {
        ROS_ERROR("%s", ex.what());
        return;
    }
    if (isReachable(to_robot)) {
        // Pick it up
        if (exp_client != NULL)
            exp_client->cancelGoal(); // NULL!
        ROS_INFO("Booby in range, picking up...");
        std_srvs::Empty e;
        abortPathFollower();
        booby_pickup_client.call(e);
        STATE = LIFTING_BOOBY;
    } else {
        ROS_WARN("Booby out of range");
    } // TODO: Handle pickup fail
}

int toPixels(double meters) {
    return (int)(meters/grid.info.resolution);
}

bool releaseBooby() {
    if (arm_state == FOLLOWER_ROTATING)
        return false;
    ras_uarm_controller::Put put_obj;
    tf::Vector3 destination;
    // Check right
    bool flag;
    for (int y = grid.info.height/2; y >= grid.info.height/2 - toPixels(booby_window_width); --y) {
        flag = true;
        for (int x = grid.info.width/2; x < grid.info.width/2 + toPixels(booby_window_height); ++x) {
            if (grid.data[y*grid.info.width + x] > 0)
                flag = false;
        }
        if (!flag)
            break;
    }
    if (flag) {
        ROS_INFO("Placing to the right");
        put_obj.request.point.x = 23;
        put_obj.request.point.y = 5;
        put_obj.request.point.z = 4;
        ROS_INFO("Releasing booby in (%f,%f,%f)",
                 put_obj.request.point.x,
                 put_obj.request.point.y,
                 put_obj.request.point.z);
        if (exp_client->getState() == actionlib::SimpleClientGoalState::ACTIVE){
            ROS_INFO("Following is active. Cancelling goal");
            exp_client->cancelGoal();
            abortPathFollower();
        }
        return release_object_client.call(put_obj);
    } else {
        // Check left
        for (int y = grid.info.height/2; y <= grid.info.height/2 + toPixels(booby_window_width); ++y) {
            flag = true;
            for (int x = grid.info.width/2; x < grid.info.width/2 + toPixels(booby_window_height); ++x) {
                if (grid.data[y*grid.info.width + x] > 0)
                    flag = false;
            }
            if (!flag)
                break;
        }
        if (flag) {
            ROS_INFO("Placing to the left");
            put_obj.request.point.x = -23;
            put_obj.request.point.y = 5;
            put_obj.request.point.z = 4;
            ROS_INFO("Releasing booby in (%f,%f,%f)",
                     put_obj.request.point.x,
                     put_obj.request.point.y,
                     put_obj.request.point.z);
            if (exp_client->getState() == actionlib::SimpleClientGoalState::ACTIVE){
                ROS_INFO("Following is active. Cancelling goal");
                exp_client->cancelGoal();
                abortPathFollower();
            }
            return release_object_client.call(put_obj);
        } else {
            return false;
        }
    }
}

//Testing service please delete
bool testBooby(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
    releaseBooby();
}

bool isBatteryDuplicate(ras_mapping::line cmp) {
    double dist1 = std::sqrt(std::pow(cmp.request.start.x - latest_battery.request.start.x, 2) +
                             std::pow(cmp.request.start.y - latest_battery.request.start.y, 2));
    double dist2 = std::sqrt(std::pow(cmp.request.stop.x - latest_battery.request.stop.x, 2) +
                             std::pow(cmp.request.stop.y - latest_battery.request.stop.y, 2));
    if (dist1 < 0.075 && dist2 < 0.075)
        return true;
    dist1 = std::sqrt(std::pow(cmp.request.start.x - latest_battery.request.stop.x, 2) +
                      std::pow(cmp.request.start.y - latest_battery.request.stop.y, 2));
    dist2 = std::sqrt(std::pow(cmp.request.stop.x - latest_battery.request.start.x, 2) +
                      std::pow(cmp.request.stop.y - latest_battery.request.start.y, 2));
    if (dist1 < 0.075 && dist2 < 0.075)
        return true;
    return false;
}

void batteryCallback (const ras_target_recognition::Line::ConstPtr &msg) {
    tf::TransformListener tListen;
    geometry_msgs::PoseStamped p1;
    p1.pose.position.x = msg->start.x;
    p1.pose.position.y = msg->start.y;
    p1.pose.orientation.w = 1;
    p1.header.stamp = ros::Time(0);
    p1.header.frame_id = "robot_center";
    geometry_msgs::PoseStamped p2;
    p2.pose.position.x = msg->stop.x;
    p2.pose.position.y = msg->stop.y;
    p2.pose.orientation.w = 1;
    p2.header.stamp = ros::Time(0);
    p2.header.frame_id = "robot_center";
    if (isReachable(p1) || isReachable(p2)) { // Well shit
        ROS_INFO("Battery in range.");
        if(exp_client!=NULL) {
            exp_client->cancelGoal();
            abortPathFollower();
            ROS_INFO("Cancelled goal");
        }
        geometry_msgs::PoseStamped p1_world, p2_world;
        try {
            tListen.waitForTransform("world",
                                     "robot_center",
                                     p1.header.stamp,
                                     ros::Duration(1));
            tListen.transformPose("world", p1, p1_world);
            tListen.transformPose("world", p2, p2_world);
        } catch (tf::TransformException ex) {
            ROS_ERROR("%s", ex.what());
            return;
        }
        ras_mapping::line to_add;
        to_add.request.start.x = p1_world.pose.position.x;
        to_add.request.start.y = p1_world.pose.position.y;
        to_add.request.stop.x = p2_world.pose.position.x;
        to_add.request.stop.y = p2_world.pose.position.y;
        if(!isBatteryDuplicate(to_add)) {
            if(add_battery_client.call(to_add)) {
                ROS_INFO("Added battery");
                if (MODE == EXPLORE || MODE == DEBUG)
                    STATE = RESUMING_LAST_GOAL;
            }
            latest_battery = to_add;
        }
    } else {
        ROS_INFO("Battery out of range");
    }
}

bool layDownObject() {
    ras_uarm_controller::Put put_obj;
    put_obj.request.point.x = 0;
    put_obj.request.point.y = 17;
    put_obj.request.point.z = 1;
    return release_object_client.call(put_obj);
}

void objectCenterCallback(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    tf::TransformListener tListen;
    geometry_msgs::PoseStamped tf_output;
    try {
        tListen.waitForTransform("robot_center",
                                 msg->header.frame_id,
                                 msg->header.stamp,
                                 ros::Duration(1));
        tListen.transformPose("robot_center", *msg, tf_output);
    } catch (tf::TransformException ex) {
        ROS_ERROR("%s", ex.what());
        return;
    }
    current_object_center = tf_output;
}

int main(int argc, char **argv) {
    double explore_time, rescue_time, exit_time;

    ros::init(argc, argv, "brain");
    ros::NodeHandle n("~");

    n.getParam("x_exit", x_exit);
    n.getParam("y_exit", y_exit);
    n.getParam("MODE", MODE);
    n.getParam("explore_time", explore_time);
    n.getParam("rescue_time", rescue_time);
    n.getParam("exit_time", exit_time);
    n.getParam("brush_radius", brush_radius);
    n.getParam("debug_tolerance", debug_tolerance);
    n.getParam("obj_looking_dist", obj_looking_dist);
    n.getParam("booby_diam", booby_diam);
    n.getParam("booby_window_height", booby_window_height);
    n.getParam("booby_window_width", booby_window_width);
    n.getParam("robot_width", robot_width);
    n.getParam("ROI_tolerance", ROI_tolerance);

    path_plan_client = n.serviceClient<ras_path_plan::path_srv>("/path/plan");
    move_to_client = n.serviceClient<ras_follower::moveTo>("/robot/goTo");
    abort_follow_client = n.serviceClient<std_srvs::Empty>("/robot/abort_follow_service");
    booby_pickup_client = n.serviceClient<std_srvs::Empty>("/uarm_controller/grab_rubble");
    object_pickup_client = n.serviceClient<std_srvs::Empty>("/uarm_controller/grab_object");
    update_map_client = n.serviceClient<std_srvs::Empty>("/map/updateMap");
    clean_map_client = n.serviceClient<std_srvs::Empty>("/map/cleanMap");
    update_loc_map_client = n.serviceClient<std_srvs::Empty>("/localization/add_walls");
    add_battery_client = n.serviceClient<ras_mapping::line>("/map/addBattery");
    pop_object_client = n.serviceClient<ras_object_list::PopObject>("/pop_object");
    robot_rotate_client = n.serviceClient<ras_follower::Rotate>("/robot/relative_rotate_to_target");
    release_object_client = n.serviceClient<ras_uarm_controller::Put>("/uarm_controller/release_object");
    ros::Subscriber pose_sub = n.subscribe("/localization/pose", 1, poseCallback);
    ros::Subscriber map_sub = n.subscribe("/mapping/map", 1, map_callback);
    ros::Subscriber grid_sub = n.subscribe("/mapping/grid", 1, grid_callback);
    ros::Subscriber goto_sub = n.subscribe("/move_base_simple/goal", 10, rvizGoalCallback);
    ros::Subscriber follower_state_sub = n.subscribe("/path_follower/state", 1, followerStateCallback);
    ros::Subscriber uarm_state_sub = n.subscribe("/uarm_controller/state", 1, armStateCallback);
    ros::Subscriber booby_sub = n.subscribe("/rubble/pose", 1, rubbleCallback);
    ros::Subscriber battery_sub = n.subscribe("/camera/battery", 1, batteryCallback);
    ros::Subscriber obj_center_sub = n.subscribe("/camera/ObjectPose", 1, objectCenterCallback);
    GOTOServer goto_server(n, "goto_location", boost::bind(&goto_execute, _1, &goto_server), false);
    goto_server.start();
    ros::ServiceServer go_home_server = n.advertiseService("exit_maze", exitMaze);
    ros::ServiceServer test_server = n.advertiseService("test_booby_release", testBooby);
    ros::ServiceServer add_pose_server = n.advertiseService("further_object_points", addPoseToPath);
    ros::ServiceServer go_server = n.advertiseService("go", goService);
    go_pub = n.advertise<std_msgs::String>("/robot/talk", 10);
    path_to_follow.header.frame_id = "world";

    // Visualization
    ros::Publisher map_pub = n.advertise<nav_msgs::OccupancyGrid>("map", 10);
    ros::Publisher path_pub = n.advertise<nav_msgs::Path>("planned_path", 10);

    std::time_t deadline = std::time(NULL);
    ros::Rate rate(5);
    std::vector<PointXYD> exp_plan;
    if(MODE == DEBUG) {
        deadline += 60 * 60;
        STATE = IDLE;
    } else {
        while(map_static.data.size() == 0) // Wait for map to be published
            ros::spinOnce();
        map = map_static;
        if (MODE == EXPLORE) {
            deadline += (explore_time - exit_time) * 60;
            mShouldExplore = true;
            STATE = IDLE;
        }
        else if (MODE == RESCUE) {
            deadline += (rescue_time - exit_time) * 60;
            STATE = IDLE;
        }
    }
    int tTimesWaitedForQRCode = 0;
    int tObjectSearchIteration = 0;
    // sendGOsignal();
    while(n.ok()) {
        rate.sleep();
        path_pub.publish(path_to_follow);
        map_pub.publish(map);
        //ROS_INFO("State %d", STATE);
        if (std::time(NULL) > deadline) {
            //TODO: Kill all and go home
            if(!mExiting) {
                if (STATE != LIFTING_OBJECT &&
                    STATE != LAYING_OBJECT &&
                    STATE != LOOKING_FOR_QR_OR_WALL) {
                    if (exp_client->getState() == actionlib::SimpleClientGoalState::ACTIVE){
                        exp_client->cancelGoal();
                        abortPathFollower();
                    }
                    delete(exp_client);
                    current_goal.x = x_exit;
                    current_goal.y = y_exit;
                    exp_client = gotoLocation_non_block(x_exit, y_exit, 0.15);
                    mExiting = true;
                    STATE = FOLLOWING_LINE;
                }
            }
        }
        // points_pub.publish(points);
        //map_pub.publish(map);
        int i; // Gotta declare it here
        switch (STATE) {
            case IDLE:
                ROS_INFO("IDLE");
                ros::spinOnce();
                break;
            case PLAN_EXPLORE:
                ROS_INFO("PLAN_EXPLORE");
                exp_plan.clear();
                createExplorationPlan(exp_plan);
                i = 0;
                while(map.data[std::floor(exp_plan[i].x/map.info.resolution) * map.info.width + std::floor(exp_plan[i].y/map.info.resolution)] > 0)
                    i++;
                try {
                    current_goal.x = exp_plan.at(i).x;
                    current_goal.y = exp_plan.at(i).y;
                    if (exp_client != NULL)
                        delete(exp_client);
                    exp_client = gotoLocation_non_block(current_goal.x,
                                                        current_goal.y,
                                                        debug_tolerance);
                    STATE = FOLLOWING_LINE;
                } catch (std::out_of_range &e) {
                    ROS_INFO("All map explored. Cleaning and restarting.");
                    map = map_static;
                }
                break;
            case RESUMING_LAST_GOAL:
                ROS_INFO("RESUMING_LAST_GOAL");
                if(exp_client!=NULL && exp_client->getState() == actionlib::SimpleClientGoalState::ACTIVE)
                    exp_client->cancelGoal();
                path_to_follow.poses.clear();
                if(!retrievePath(current_goal.x, current_goal.y, path_to_follow)){
                    if (MODE == EXPLORE) {
                        std_srvs::Empty e;
                        clean_map_client.call(e);
                        STATE = PLAN_EXPLORE;
                    }
                    else if (MODE == RESCUE) {
                        std_srvs::Empty e;
                        clean_map_client.call(e);
                        if (exp_client->getState() == actionlib::SimpleClientGoalState::ACTIVE){
                            exp_client->cancelGoal();
                            abortPathFollower();
                        }
                        delete(exp_client);
                        current_goal.x = x_exit;
                        current_goal.y = y_exit;
                        exp_client = gotoLocation_non_block(x_exit, y_exit, 0.15);
                        STATE = FOLLOWING_LINE;
                        mCarryingObject = true;
                    }
                    else if (MODE == DEBUG)
                            STATE = IDLE;
                } else {
                    if (exp_client != NULL)
                        delete(exp_client);
                    exp_client = gotoLocation_non_block(current_goal.x,
                                                        current_goal.y,
                                                        debug_tolerance);
                    STATE = FOLLOWING_LINE;
                }
                break;
            case FOLLOWING_LINE:
                ROS_INFO("FOLLOWING_LINE");
                // TODO: Switch through goal states
                if (mCarryingBooby) {
                    if (releaseBooby()) {
                        mCarryingBooby = false;
                        STATE = LAYING_BOOBY;
                    }
                }
                if((!mCarryingObject) && (MODE == RESCUE)) {
                    if (current_goal_index + 3 >= path_to_follow.poses.size())
                        if(isReachable(current_object_center)) {
                            // Trigger arm pickup
                            exp_client->cancelGoal();
                            abortPathFollower();
                            STATE = SEARCHING_OBJECT;
                            ros::Duration(1.0).sleep();
                    }
                }
                if(exp_client->getState() == actionlib::SimpleClientGoalState::PREEMPTED || exp_client->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
                    if (mExiting) {
                        STATE = IDLE;
                        // Save shit to file or something
                    }
                    else if (MODE == DEBUG)
                        STATE = IDLE;
                    else if (MODE == EXPLORE)
                        STATE = PLAN_EXPLORE;
                    else if (MODE == RESCUE) {
                        // if carrying an object, lay it down
                        // else search for object
                        if (!mCarryingObject)
                            STATE = SEARCHING_OBJECT;
                        else {
                            STATE = LAYING_OBJECT;
                            layDownObject();
                        }
                    }
                }
                else if (exp_client->getState() == actionlib::SimpleClientGoalState::ABORTED) {
                    // Probably wait for map update
                }
                break;
            case LIFTING_BOOBY:
                tTimesWaitedForQRCode = 0;
                ROS_INFO("LIFTING_BOOBY");
                if (arm_state == 0) { // Grabbing finished
                    if(!releaseBooby()) {
                        // TODO: Move a little or something
                        mCarryingBooby = true;
                        STATE = RESUMING_LAST_GOAL;
                    } else {
                        STATE = LAYING_BOOBY;
                    }
                }
                break;
            case LAYING_BOOBY:
                tTimesWaitedForQRCode = 0;
                ROS_INFO("LAYING_BOOBY");
                if (arm_state == -1) {// Or actually some "action completed" message
                    STATE = RESUMING_LAST_GOAL;
                }
                break;
            case LOOKING_FOR_QR_OR_WALL:
                ROS_INFO("LOOKING_FOR_QR_OR_WALL");
                if(tTimesWaitedForQRCode > 3){
                    tTimesWaitedForQRCode = 0;
                    std_srvs::Empty empty_srv_msg;
                    // Update static map
                    update_loc_map_client.call(empty_srv_msg);
                    // And update particle filter
                    bool updateResult = update_map_client.call(empty_srv_msg);
                    ROS_INFO_STREAM("Update map: " << updateResult);
                    STATE = RESUMING_LAST_GOAL;
                } else {
                    tTimesWaitedForQRCode++;
                }
                break;     
            case SEARCHING_OBJECT:
            {
                ROS_INFO("SEARCHING_OBJECT");
                std_srvs::Empty e;
                bool success = false;
                for (int i = 0; i < 3; i++) { // Try to pickup the object 3 times
                    if(object_pickup_client.call(e)) {
                        success = true;
                        STATE = LIFTING_OBJECT;
                        break;
                    } else {
                        ROS_WARN("Couldn't pickup. Retrying...");
                    }
                    ros::Duration(0.5).sleep();
                }
                if (success) {
                    tObjectSearchIteration = 0;
                    continue;
                }
                // Do some rotation or whatever
                ras_follower::Rotate r;
	        switch (tObjectSearchIteration++) {
                    case 0:
                        // Angle += 40°
                        r.request.target.angular.z = 40 / 180 * M_PI;
                        break;
                    case 1:
                    case 2:
                    case 3:
                        // Angle -= 20°
                        r.request.target.angular.z = -20 / 180 * M_PI;
                        break;
                    default:
                        STATE = RETRIEVE_NEXT_OBJECT;
                }
                break;
            }
            case RETRIEVE_NEXT_OBJECT:
            { // Brackets for hiding next_object from other cases. C++ amirite?
                ROS_INFO("RETRIEVE_NEXT_OBJECT");
                ras_object_list::PopObject next_object;
                if(pop_object_client.call(next_object))
                    ROS_INFO("Cool");
                ROS_INFO("Going to %f, %f", next_object.response.object_position.pose.position.x, next_object.response.object_position.pose.position.y);
                if (exp_client != NULL)
                    delete(exp_client);
                exp_client = gotoLocation_non_block(next_object.response.object_position.pose.position.x,
                                                    next_object.response.object_position.pose.position.y,
                                                    debug_tolerance);
                STATE = FOLLOWING_LINE;
                break; 
            }
            case LIFTING_OBJECT:
                ROS_INFO("LIFTING_OBJECT");
                if (arm_state == 0) { // Grabbing finished
                    mCarryingObject = true;
                    // Go home
                    if (exp_client != NULL)
                        delete(exp_client);
                    exp_client = gotoLocation_non_block(x_exit,
                                                        y_exit,
                                                        debug_tolerance);
                    STATE = FOLLOWING_LINE;
                }
                break;
            case LAYING_OBJECT:
                ROS_INFO("LAYING_OBJECT");
                if (arm_state == -1)
                    STATE = RETRIEVE_NEXT_OBJECT;
                break;
            default:
                ROS_ERROR("Something's fucky");
                break;
        }
        ros::spinOnce();
        // TODO: Maybe destroy exp_client?
    }
}
