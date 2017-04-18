#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <uarm/MoveTo.h>
#include <uarm/Pump.h>
#include <uarm/MoveToJoints.h>
#include <std_srvs/Empty.h>
#include <std_msgs/String.h>
#include <ras_uarm_controller/Put.h>
#include <std_msgs/Int32.h>

class uarmController
{
public:
  ros::Subscriber poseSubscriber;
  ros::Subscriber rubbleSubscriber;
  ros::NodeHandle nodeHandle;
  tf::TransformListener my_tf_listener;
  geometry_msgs::Point objectPoint;
  geometry_msgs::Point rubblePoint;
  ros::Time timeWeSawObject;
  ros::Time timeWeSawRubble;
  ros::ServiceServer grabServer;
  ros::ServiceServer rubbleServer;
  ros::ServiceServer homeServer;
  ros::ServiceServer releaseObjectServer;
  ros::ServiceClient uarmClient;
  ros::ServiceClient homeClient;
  ros::ServiceClient uarm_pump;
  ros::Publisher statePublisher;

  int mState;

  uarmController():nodeHandle("~") 
  {
    poseSubscriber = nodeHandle.subscribe<geometry_msgs::PoseStamped>("/camera/ObjectPose",
      1,
      &uarmController::poseCallback,
      this);
    rubbleSubscriber = nodeHandle.subscribe<geometry_msgs::PoseStamped>("/rubble/pose",
      1,
      &uarmController::rubbleCallback,
      this);
    statePublisher = nodeHandle.advertise<std_msgs::Int32>("state", 1);
    grabServer = nodeHandle.advertiseService("grab_object",
     &uarmController::grabObjectCallback,
     this);
    rubbleServer = nodeHandle.advertiseService("grab_rubble",
      &uarmController::grabRubbleCallback,
      this);

    releaseObjectServer = nodeHandle.advertiseService("release_object",
     &uarmController::putObjectCallback,
     this);
    homeServer = nodeHandle.advertiseService("go_home",
     &uarmController::goHomeCallback,
     this);
    uarmClient = nodeHandle.serviceClient<uarm::MoveTo>("/uarm/move_to");
    homeClient = nodeHandle.serviceClient<uarm::MoveTo>("/uarm/move_to");
    uarm_pump = nodeHandle.serviceClient<uarm::Pump>("/uarm/pump");

    mState = -1;
  }

  void poseCallback(const geometry_msgs::PoseStampedConstPtr &msg)
  {
    std::string frameName = msg->header.frame_id;
    ros::Time scanTime = msg->header.stamp;
    tf::StampedTransform transform;
    try {
      my_tf_listener.waitForTransform("uarm", frameName, scanTime, ros::Duration(0.2));
      geometry_msgs::PoseStamped objectPose;
      my_tf_listener.transformPose("uarm",scanTime, *msg,"robot_center", objectPose);
      //Transform to arm coordinats.
      objectPoint.x = objectPose.pose.position.x*100;

      objectPoint.y = objectPose.pose.position.y*100+2;
      objectPoint.z = objectPose.pose.position.z*100;
      if(objectPoint.x > 0){
              objectPoint.x += 1;  
              objectPoint.y -= 1 + 0.1*objectPoint.x;    
      }
  
      timeWeSawObject = objectPose.header.stamp;
    } catch (tf::TransformException ex){
      ROS_ERROR("%s", ex.what());
      return;
    }
  }

  void rubbleCallback(const geometry_msgs::PoseStampedConstPtr &msg) 
  {
    std::string frameName = msg->header.frame_id;
    ros::Time scanTime = msg->header.stamp;
    tf::StampedTransform transform;
    try {
      my_tf_listener.waitForTransform("uarm", frameName, scanTime, ros::Duration(0.2));
      geometry_msgs::PoseStamped objectPose;
      my_tf_listener.transformPose("uarm",scanTime, *msg,"robot_center", objectPose);
      //Transform to arm coordinats.
      rubblePoint.x = objectPose.pose.position.x*100;

      rubblePoint.y = objectPose.pose.position.y*100;
      rubblePoint.z = objectPose.pose.position.z*100;
      if(rubblePoint.x > 0){ // TODO: check this: Hardcoded offset?
              rubblePoint.x += 1;  
              rubblePoint.y -= 1 + 0.1*rubblePoint.x;    
      }
      else
      {
              rubblePoint.x += 2;  
              rubblePoint.y -= 1 + 0.1*rubblePoint.x; 
      }
  
      timeWeSawRubble = objectPose.header.stamp;
    } catch (tf::TransformException ex){
      ROS_ERROR("%s", ex.what());
      return;
    }
  }

  // Home at 78.22999999999999, 127.82999999999998, 16.03, 6.599999999999994
  // Create service
  bool goHome(float d) 
  {
    uarm::MoveTo home;
    home.request.position.x = 9.83;
    home.request.position.y = 5.29;
    home.request.position.z = 14.02;
    home.request.eef_orientation = -90;
    home.request.move_mode = 0; // Move mode: Absolute
    home.request.movement_duration = ros::Duration(d,0);
    home.request.ignore_orientation = true; // Irrelevant to know what this is. It should just be 0
    home.request.interpolation_type = 1; // Cubic
    home.request.check_limits = false;
    homeClient.call(home);
    return true;
  }

  bool moveAbs(double x, double y, double z, double d, uarm::MoveTo& toPos) 
  {
    ROS_INFO("Moving to: %f, %f, %f", x,y,z);    
    toPos.request.position.x = x;
    toPos.request.position.y = y;
    toPos.request.position.z = z;
    toPos.request.eef_orientation = -90;
    toPos.request.move_mode = 0; // Move mode: Absolute
    toPos.request.movement_duration = ros::Duration(d,0);
    toPos.request.ignore_orientation = true; // Irrelevant to know what this is. It should just be 0
    toPos.request.interpolation_type = 1; // Cubic
    toPos.request.check_limits = false;

    if (uarmClient.call(toPos)) {
      ROS_INFO("Call successful");
      return true;
    } else {
      ROS_ERROR("Couldn\'t call the /uarm/move_to");
      return false;
    }
  }

  bool moveRelative(double x, double y, double z, double d, uarm::MoveTo& correction) 
  {
    correction.request.position.x = x;
    correction.request.position.y = y;
    correction.request.position.z = z;
    correction.request.eef_orientation = -90;
    correction.request.move_mode = 1; // Move mode: Relative
    correction.request.movement_duration = ros::Duration(d,0);
    correction.request.ignore_orientation = true; // Irrelevant to know what this is. It should just be 0
    correction.request.interpolation_type = 2; // Linear
    correction.request.check_limits = false;

    if (uarmClient.call(correction)) {
      ROS_INFO("Success! Errors: dx = %f, dy = %f, dz = %f",
        objectPoint.x - correction.response.position.x,
        objectPoint.y - correction.response.position.y,
        objectPoint.z - correction.response.position.z);
      return true;
    } else {
      ROS_ERROR("Couldn\'t call the /uarm/move_to");
      return false;
    }

  }
  void turnOnPump()
  {
    uarm::Pump pmsg;
    pmsg.request.pump_status = true;
    uarm_pump.call(pmsg);
    ROS_INFO("Pump on");
  }
  void turnOffPump()
  {
    uarm::Pump pmsg;
    pmsg.request.pump_status = false;
    uarm_pump.call(pmsg);
    ROS_INFO("Pump off");
  }

  bool grabObjectCallback (std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) 
  {
    if ((ros::Time::now() - timeWeSawObject).toSec() > 3)
      return false;
    mState = 3;
    return true;
  }

  bool grabRubbleCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) 
  {
    if ((ros::Time::now() - timeWeSawRubble).toSec() > 3)
      return false;
    mState = 2;
    return true;
  }

  bool checkIfObjectIsAt(geometry_msgs::Point &objectPoint, geometry_msgs::Point &pointToCheck, ros::Time &timeWeSawObject)
  {
    if(timeWeSawObject < ros::Time::now() - ros::Duration(1.0,0))
      return false;
    if(std::abs(objectPoint.x - pointToCheck.x) > 1){
      return false;
    }
    if(std::abs(objectPoint.y - pointToCheck.y) > 1){
      return false;
    }
    if(std::abs(objectPoint.z - pointToCheck.z) > 1){
      return false;
    }
    return true;
  }

  bool goHomeCallback (std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) 
  {
    return goHome(1.5);
  }

  bool putObjectCallback (ras_uarm_controller::Put::Request& request, ras_uarm_controller::Put::Response& response) 
  {
    // Go grab the object!
    uarm::MoveTo toPos;
    bool resp = moveAbs(request.point.x, request.point.y, request.point.z, 2.0, toPos);
    if(resp) { 
      turnOffPump();
      goHome(2);
      mState = -1;
      return true;
    } else {
      mState = -1;
    	return false;
    }
  }

  void mainLoop()
  {
    ros::Rate rate(10);
    while(nodeHandle.ok()){
      int count = 0;
      uarm::MoveTo toPos;
      geometry_msgs::Point oldRubblePose = rubblePoint;
      geometry_msgs::Point oldObjectPose = objectPoint;
      switch (mState) {
      case 0:  //Having object.
        break;
      case 1: //Going home.
        break;
      case 2:  // Go grab the rubble!    
        ROS_INFO("Rubble at %f, %f, %f",
         rubblePoint.x,
         rubblePoint.y,
         rubblePoint.z);
        while(checkIfObjectIsAt(rubblePoint, oldRubblePose, timeWeSawRubble) && count < 3)
        {
          turnOffPump();
          if(moveAbs(rubblePoint.x, rubblePoint.y, rubblePoint.z+2.0, 2.0, toPos))
          {
            uarm::MoveTo downMove;
            moveRelative(0.0,0.0,-3.5,1.0,downMove);
            turnOnPump();
            ros::spinOnce();
            //goHome(4); Don't really go home
            uarm::MoveTo mt;
            moveRelative(0, 0, 15, 2, mt);
            std_msgs::Int32 tState;
            tState.data = mState;
            statePublisher.publish(tState);
            ros::spinOnce();
            count++;
          }
          else
            continue;
        }
        if (checkIfObjectIsAt(rubblePoint, oldRubblePose, timeWeSawRubble) && count == 3 )
        {
          mState = -1;
          turnOffPump();
        } else {
          mState= 0;
        }
        break;
      case 3:  // Go grab the object!
        ROS_INFO("Object at %f, %f, %f",
         objectPoint.x,
         objectPoint.y,
         objectPoint.z);
        while(checkIfObjectIsAt(objectPoint, oldObjectPose, timeWeSawObject) && count < 3)
        {
          turnOffPump();
          if(moveAbs(objectPoint.x, objectPoint.y, objectPoint.z+2.0, 2.0, toPos))
          {
            uarm::MoveTo downMove;
            moveRelative(0.0,0.0,-3.5,1.0,downMove);
            turnOnPump();
            ros::spinOnce();
            goHome(4);
            std_msgs::Int32 tState;
            tState.data = mState;
            statePublisher.publish(tState);
            ros::spinOnce();
            count++;
          }
          else
            continue;
        }
        if (checkIfObjectIsAt(objectPoint, oldObjectPose, timeWeSawObject) && count == 3 )
        {
          mState = -1;
          turnOffPump();
        } else{
          mState = 0;
        }
      default: //No object on arm.
        break;
      }
      std_msgs::Int32 tState;
      tState.data = mState;
      statePublisher.publish(tState);
      rate.sleep();
      ros::spinOnce();
    }
  }
};

int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "uarm_controller");
  uarmController uarm_controller;
  ros::ServiceServer grabServer;
  uarm_controller.mainLoop();
}
