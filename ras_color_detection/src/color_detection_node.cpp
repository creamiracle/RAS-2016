#include <cmath>
#include <ros/ros.h>
#include <iostream>
#include <pcl/filters/passthrough.h>
#include <sensor_msgs/PointCloud2.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <image_transport/image_transport.h>
#include <pcl/filters/filter.h>
#include <pcl_ros/point_cloud.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <pcl/features/normal_3d.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/features/vfh.h>
#include <std_msgs/String.h>
#include "ras_msgs/RAS_Evidence.h"
#include "ras_color_detection/object.hpp"
#include <stdlib.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <ras_color_detection/FPose.h>
#include <tf/transform_listener.h>

ros::Publisher point_pub, speak_pub, pose_pub, evidence_pub;
std::vector<object> target;
ros::ServiceClient further_call;
ras_msgs::RAS_Evidence objectEvidence;
pcl::PointCloud<pcl::PointXYZ>::Ptr pclData_filtered1 (new pcl::PointCloud<pcl::PointXYZ> ());
pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclData (new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZ>::Ptr model_keypoints (new pcl::PointCloud<pcl::PointXYZ> ());
std::vector<bool> object_ ;
pcl::PointCloud<pcl::VFHSignature308>::Ptr vfhs (new pcl::PointCloud<pcl::VFHSignature308> ());
std::vector<std::vector<double> > further_xyzpos;
std::vector<geometry_msgs::Point> ObjectList;
std::vector<geometry_msgs::Point> FurtherList;


bool check(double x, double y);
bool check_further(double x, double y);
int check_safeArea(double x, double y, double z);

void imageCallback(const sensor_msgs::ImageConstPtr& image) {
  if (!pclData->isOrganized()){ return;}
  pcl::PointCloud<pcl::PointXYZ>::Ptr pclData_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  pclData_filtered->header.frame_id = "camera_rgb_optical_frame";
  pclData_filtered->height = pclData->height;
  pclData_filtered->width = pclData->width;
  pclData_filtered->points.resize(pclData_filtered->height*pclData_filtered->width);
  tf::Transformer transForm;
  geometry_msgs::PointStamped map_points;
  tf::TransformListener listener; 
  ras_color_detection::FPose F_srv;
  geometry_msgs::PoseStamped pub_Objectpose;
  std_msgs::String msgs;
  // do the cv bridge
  cv_bridge::CvImageConstPtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvShare(image, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  cv::Mat img_bgr = cv_ptr->image.clone();
  cv::Mat img_hsv,img_output,erodeElemant,dilateElemant;
  pcl::PointCloud<pcl::PointXYZ>::Ptr pclData_filtered1 (new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::PassThrough<pcl::PointXYZ> pass;
  cv::cvtColor(img_bgr,img_hsv,CV_BGR2HSV);
  for (int color = 0; color < target.size(); ++color){
    //separate the object based on the color!
    cv::inRange(img_hsv,target[color].getHSVlower(),target[color].getHSVhigher(),img_output);
    erodeElemant = cv::getStructuringElement(cv::MORPH_RECT,cv::Size(2*target[color].getEsize()+1,2*target[color].getEsize()+1));
    dilateElemant = cv::getStructuringElement(cv::MORPH_RECT,cv::Size(2*target[color].getDsize()+1,2*target[color].getDsize()+1));
    cv::erode(img_output,img_output, erodeElemant);
    cv::dilate(img_output,img_output, dilateElemant);
    //find contours!
    std::vector< std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(img_output,contours,hierarchy,CV_RETR_CCOMP,CV_CHAIN_APPROX_SIMPLE );
    cv::cvtColor(img_output,img_output,CV_GRAY2BGR);
    // find moments
    if (hierarchy.size() > 0) {
      for (int index = 0; index >= 0; index = hierarchy[index][0]) {
        cv::Moments moment = cv::moments((cv::Mat)contours[index]);
        double area = moment.m00;
        //if the area is less than 30 px by 40px then it is probably just noise or too far away
        if(area > 1200){
          //save object center
          int a = (int) moment.m10/area;
          int b = (int) moment.m01/area;
          double temp_x =pclData->at(a,b).x;
          double temp_y =pclData->at(a,b).y;
          double temp_z =pclData->at(a,b).z;
          //check for the safe area!
          if(check_safeArea(temp_x,temp_y,temp_z) == 0) {
            //safe area 
          }
          else if(check_safeArea(temp_x,temp_y,temp_z) == 1){
            // NaN
            continue;
          }
          else if(check_safeArea(temp_x,temp_y,temp_z) == 2){
            // further eploration
            std::vector<double> temp_further;
            temp_further.push_back(temp_x);
            temp_further.push_back(temp_y);
            temp_further.push_back(temp_z);
            further_xyzpos.push_back(temp_further);
            continue;
          }
          target[color].setXpos(temp_x);
          target[color].setYpos(temp_y);
          target[color].setZpos(temp_z);
          std::vector<double> xyzpos;
          xyzpos.push_back(target[color].getXpos());
          xyzpos.push_back(target[color].getYpos());
          xyzpos.push_back(target[color].getZpos());
          target[color].Objectpos.push_back(xyzpos);
          object_[color] = true;
          //save specific object image with center!
          cv::Mat img_result = img_output.clone();
          target[color].image.push_back(img_result);
          cv::drawContours(target[color].image.back(),contours,index,cv::Scalar(0,0,255),CV_FILLED,8,hierarchy);             
        }
      }
    }
    if(object_[color] == true){
      for(int same_color = 0; same_color < target[color].image.size(); same_color++){
        for (int i = 0; i < image->width; ++i){
          for (int j = 0; j < image->height; ++j){
            if (target[color].image[same_color].at<cv::Vec3b>(j, i)[2] == 255 &&
                target[color].image[same_color].at<cv::Vec3b>(j, i)[1] == 0 &&
                target[color].image[same_color].at<cv::Vec3b>(j, i)[0] == 0){
              pcl::PointXYZRGB point_new = pclData->at(i,j);
              if(std::isfinite(point_new.x) && std::isfinite(point_new.y) && std::isfinite(point_new.z)){
                pclData_filtered->at(i,j).x = point_new.x;
                pclData_filtered->at(i,j).y = point_new.y;
                pclData_filtered->at(i,j).z = point_new.z;
              }
            }
          }
        }
        pass.setInputCloud (pclData_filtered);
        pass.setFilterFieldName ("z");
        pass.setFilterLimits (0.05,0.27);
        pass.filter (*pclData_filtered1);
        //check for point 
        if(pclData_filtered1->size() < 100){continue;}
        //remove outliers
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        sor.setInputCloud (pclData_filtered1);
        sor.setMeanK (50);
        sor.setStddevMulThresh (0.8);
        sor.filter (*model_keypoints);

        //estimating the normal
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal> ());
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
        ne.setInputCloud(model_keypoints);
        ne.setRadiusSearch (0.01);
        ne.compute (*cloud_normals);
        //extracting the vfh features
        pcl::VFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::VFHSignature308> vfh;
        vfh.setInputCloud (model_keypoints);
        vfh.setInputNormals (cloud_normals);
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree2 (new pcl::search::KdTree<pcl::PointXYZ> ());
        vfh.setSearchMethod (tree2);
        vfh.compute (*vfhs);
        // get the object center !
        double x = target[color].Objectpos[same_color][0];
        double y = target[color].Objectpos[same_color][1];
        double z = target[color].Objectpos[same_color][2];
        // publish for the arm...
        pub_Objectpose.pose.position.x = x;
        pub_Objectpose.pose.position.y = y;
        pub_Objectpose.pose.position.z = z;
        pub_Objectpose.pose.orientation.x = 0;
        pub_Objectpose.pose.orientation.y = 0;
        pub_Objectpose.pose.orientation.z = 0;
        pub_Objectpose.pose.orientation.w = 1;
        pub_Objectpose.header.frame_id = image->header.frame_id;
        //tf transformer
        try{
          geometry_msgs::PointStamped tf_points;
          tf_points.header.frame_id = "camera_rgb_optical_frame";
          tf_points.header.stamp = ros::Time(0) ;
          listener.waitForTransform("world",tf_points.header.frame_id,tf_points.header.stamp,ros::Duration(1));
          tf_points.point.x = x;
          tf_points.point.y = y;
          tf_points.point.z = z;
          listener.transformPoint("world",tf_points,map_points);
        }
        catch (tf::TransformException e){
          ROS_ERROR("%s",e.what());
        }
        //classify
        double sum = 0;
        double value; 
        std::vector<double> w;
        std::ifstream infile;
        std::ofstream fs;
        //clear the pclData_filtered!
        pclData_filtered->clear();
        pclData_filtered->height = pclData->height;
        pclData_filtered->width = pclData->width;
        pclData_filtered->points.resize(pclData_filtered->height*pclData_filtered->width);
        //red
        if (color == 0){
          // svm classify
          infile.open("/home/ras28/catkin_ws/src/ras_color_detection/result/ball.txt",std::ios::app);
          if(infile.is_open()){
            while(infile >> value){
              w.push_back(value);
            }
            infile.close();
          }
          for (int i = 0; i < 308; i++){
            sum += vfhs->points[0].histogram[i]* w[i];
          }
          sum = sum - 0.864084296806196;
          w.clear();
          // judeg the result
          if(sum > 0){
            if (x != 0.0 || y != 0.0 || z != 0.0){
              pub_Objectpose.pose.position.y -= 0.01;
              pub_Objectpose.pose.position.z += 0.01;
              pose_pub.publish(pub_Objectpose); 
              if (check(map_points.point.x,map_points.point.y)){
                //ros evidence
                objectEvidence.object_id = objectEvidence.red_ball;
                objectEvidence.image_evidence = *image;
                objectEvidence.object_location.transform.translation.x = x;
                objectEvidence.object_location.transform.translation.y = y;
                objectEvidence.object_location.transform.translation.z = z;
                evidence_pub.publish(objectEvidence);
                //ros speak
                msgs.data = target[color].getmsg_size1();
                speak_pub.publish(msgs);
                ROS_INFO("red ball");
                //brain object list
                geometry_msgs::Point check_points;
                check_points.x = map_points.point.x;
                check_points.y = map_points.point.y;
                check_points.z = map_points.point.z;
                ObjectList.push_back(check_points);
                //
                fs.open("/home/ras28/catkin_ws/src/ras_color_detection/result/objectlist.txt",std::ofstream::out | std::ofstream::app);
                if(!fs)
                {
                    std::cerr<<"Cannot open the output file."<<std::endl;
                }
                fs<<"red_ball"<<" "<<map_points.point.x<<" "<<map_points.point.y+0.015<<" "<<z<<"\n";
                fs.close();
              }
            }
          }
          else{
            infile.open("/home/ras28/catkin_ws/src/ras_color_detection/result/hollow.txt",std::ios::app);
            if(infile.is_open()){
            while(infile >> value){
              w.push_back(value);
            }
            infile.close();
            }
            for (int i = 0; i < 308; i++){
              sum += vfhs->points[0].histogram[i]* w[i];
            }
            sum = sum - 0.0210586197967321;
            w.clear();
            if(sum > 0){
              //tf transform
              if (x != 0.0 || y != 0.0 || z != 0.0){
                pub_Objectpose.pose.position.y += 0.01;
                pub_Objectpose.pose.position.z += 0.02;
                pub_Objectpose.pose.position.x -= 0.01;
                pose_pub.publish(pub_Objectpose); 
                if (check(map_points.point.x,map_points.point.y)){
                  //ros evidence
                  objectEvidence.object_id = objectEvidence.red_hollow_cube;
                  objectEvidence.image_evidence = *image;
                  objectEvidence.object_location.transform.translation.x = x;
                  objectEvidence.object_location.transform.translation.y = y;
                  objectEvidence.object_location.transform.translation.z = z;
                  evidence_pub.publish(objectEvidence);
                  //ros speak
                  msgs.data = target[color].getmsg_size2();
                  speak_pub.publish(msgs);
                  ROS_INFO("red cube hollow");
                  //brain object list
                  geometry_msgs::Point check_points;
                  check_points.x = map_points.point.x;
                  check_points.y = map_points.point.y;
                  check_points.z = map_points.point.z;
                  ObjectList.push_back(check_points);
                  //
                  fs.open("/home/ras28/catkin_ws/src/ras_color_detection/result/objectlist.txt",std::ofstream::out | std::ofstream::app); 
                  if(!fs)
                  {
                      std::cerr<<"Cannot open the output file."<<std::endl;
                  }
                  fs<<"red_hollow_cube"<<" "<<map_points.point.x<<" "<<map_points.point.y+0.015<<" "<<z-0.015<<"\n";
                  fs.close();
                }
              }
            }
            else{
              //tf transform
              if (x != 0.0 || y != 0.0 || z != 0.0){
                pub_Objectpose.pose.position.y -= 0.01;
                pub_Objectpose.pose.position.x -= 0.01;
                pub_Objectpose.pose.position.z += 0.01;
                pose_pub.publish(pub_Objectpose);
                if (check(map_points.point.x,map_points.point.y)){
                  //ros evidence
                  objectEvidence.object_id = objectEvidence.red_cube;
                  objectEvidence.image_evidence = *image;
                  objectEvidence.object_location.transform.translation.x = x;
                  objectEvidence.object_location.transform.translation.y = y;
                  objectEvidence.object_location.transform.translation.z = z;
                  evidence_pub.publish(objectEvidence);
                  //ros speak
                  msgs.data = target[color].getmsg_size3();
                  speak_pub.publish(msgs);
                  ROS_INFO("red cube");
                  //brain object list
                  geometry_msgs::Point check_points;
                  check_points.x = map_points.point.x;
                  check_points.y = map_points.point.y;
                  check_points.z = map_points.point.z;
                  ObjectList.push_back(check_points);
                  //
                  fs.open("/home/ras28/catkin_ws/src/ras_color_detection/result/objectlist.txt",std::ofstream::out | std::ofstream::app); 
                  if(!fs)
                  {
                      std::cerr<<"Cannot open the output file."<<std::endl;
                  }
                  fs<<"red_cube"<<" "<<map_points.point.x<<" "<<map_points.point.y+0.015<<" "<<z<<"\n";
                  fs.close();
                }
              }  
            }
          }
        }
        //green
        else if (color == 1){
          infile.open("/home/ras28/catkin_ws/src/ras_color_detection/result/cube.txt",std::ios::app);
          if(infile.is_open()){
            while(infile >> value){
              w.push_back(value);
            }
            infile.close();
          }
          for (int i = 0; i < 308; i++){
            sum += vfhs->points[0].histogram[i]* w[i];
          }
          sum = sum + 0.0728443557356126;
          w.clear();
          if(sum < 0){
            if (x != 0.0 || y != 0.0 || z != 0.0){
              pub_Objectpose.pose.position.y += 0.01;
              pub_Objectpose.pose.position.z += 0.02;
              pub_Objectpose.pose.position.x -= 0.01;
              pose_pub.publish(pub_Objectpose);               
              if (check(map_points.point.x,map_points.point.y)){
                // ros evidence
                objectEvidence.object_id = objectEvidence.green_cylinder;
                objectEvidence.image_evidence = *image;
                objectEvidence.object_location.transform.translation.x = x;
                objectEvidence.object_location.transform.translation.y = y;
                objectEvidence.object_location.transform.translation.z = z;
                evidence_pub.publish(objectEvidence);
                // ros speak
                msgs.data = target[color].getmsg_size1();
                speak_pub.publish(msgs);
                ROS_INFO("green cylinder");
                //brain object list
                geometry_msgs::Point check_points;
                check_points.x = map_points.point.x;
                check_points.y = map_points.point.y;
                check_points.z = map_points.point.z;
                ObjectList.push_back(check_points);
                //
                fs.open("/home/ras28/catkin_ws/src/ras_color_detection/result/objectlist.txt",std::ofstream::out | std::ofstream::app);
                if(!fs)
                {
                    std::cerr<<"Cannot open the output file."<<std::endl;
                }
                fs<<"green_cylinder"<<" "<<map_points.point.x<<" "<<map_points.point.y+0.015<<" "<<z-0.015<<"\n";
                fs.close();
              }
            }
          }
          else{
            if (x != 0.0 || y != 0.0 || z != 0.0){
              pub_Objectpose.pose.position.y -= 0.01;
              pub_Objectpose.pose.position.z += 0.01;
              pub_Objectpose.pose.position.x -= 0.01;
              pose_pub.publish(pub_Objectpose); 
              if (check(map_points.point.x,map_points.point.y)){
                //ros evidence
                objectEvidence.object_id = objectEvidence.green_cube;
                objectEvidence.image_evidence = *image;
                objectEvidence.object_location.transform.translation.x = x;
                objectEvidence.object_location.transform.translation.y = y;
                objectEvidence.object_location.transform.translation.z = z;
                evidence_pub.publish(objectEvidence);
                //ros speak
                msgs.data = target[color].getmsg_size2();
                speak_pub.publish(msgs);
                ROS_INFO("green cube");
                //brain object list
                geometry_msgs::Point check_points;
                check_points.x = map_points.point.x;
                check_points.y = map_points.point.y;
                check_points.z = map_points.point.z;
                ObjectList.push_back(check_points);
                //
                fs.open("/home/ras28/catkin_ws/src/ras_color_detection/result/objectlist.txt",std::ofstream::out | std::ofstream::app);
                if(!fs)
                {
                    std::cerr<<"Cannot open the output file."<<std::endl;
                }
                fs<<"green_cube"<<" "<<map_points.point.x<<" "<<map_points.point.y+0.015<<" "<<z<<"\n";
                fs.close();
              }
            }
          }
        }
        //blue
        else if (color == 2){
          infile.open("/home/ras28/catkin_ws/src/ras_color_detection/result/triangle.txt",std::ios::app);
          if(infile.is_open()){
            while(infile >> value){
              w.push_back(value);
            }
            infile.close();
          }
          for (int i = 0; i < 308; i++){
            sum += vfhs->points[0].histogram[i]* w[i];
          }
          sum = sum + 0.00558779801385191;
          w.clear();
          if(sum > 0){
            if (x != 0.0 || y != 0.0 || z != 0.0){
              pub_Objectpose.pose.position.y += 0.01;
              pub_Objectpose.pose.position.x -= 0.01;
              pub_Objectpose.pose.position.z += 0.02;
              pose_pub.publish(pub_Objectpose);
              if (check(map_points.point.x,map_points.point.y)){
                //ros evidence
                objectEvidence.object_id = objectEvidence.blue_triangle;
                objectEvidence.image_evidence = *image;
                objectEvidence.object_location.transform.translation.x = x;
                objectEvidence.object_location.transform.translation.y = y;
                objectEvidence.object_location.transform.translation.z = z;
                evidence_pub.publish(objectEvidence);
                //ros speak
                msgs.data = target[color].getmsg_size1();
                speak_pub.publish(msgs);
                ROS_INFO("blue triangle");
                //brain object list
                geometry_msgs::Point check_points;
                check_points.x = map_points.point.x;
                check_points.y = map_points.point.y;
                check_points.z = map_points.point.z;
                ObjectList.push_back(check_points);
                //
                fs.open("/home/ras28/catkin_ws/src/ras_color_detection/result/objectlist.txt",std::ofstream::out | std::ofstream::app); 
                if(!fs)
                {
                    std::cerr<<"Cannot open the output file."<<std::endl;
                }
                fs<<"blue_triangle"<<" "<<map_points.point.x<<" "<<map_points.point.y+0.015<<" "<<z-0.015<<"\n";
                fs.close();
              }
            }
          }
          else{
            if (x != 0.0 || y != 0.0 || z != 0.0){
              pub_Objectpose.pose.position.y -= 0.01;
              pub_Objectpose.pose.position.x -= 0.01;
              pub_Objectpose.pose.position.z += 0.02;
              pose_pub.publish(pub_Objectpose);
              if (check(map_points.point.x,map_points.point.y)){
                //ros evidence
                objectEvidence.object_id = objectEvidence.blue_cube;
                objectEvidence.image_evidence = *image;
                objectEvidence.object_location.transform.translation.x = x;
                objectEvidence.object_location.transform.translation.y = y;
                objectEvidence.object_location.transform.translation.z = z;
                evidence_pub.publish(objectEvidence);
                //ros speak
                msgs.data = target[color].getmsg_size2();
                speak_pub.publish(msgs);
                ROS_INFO("blue cube");
                //brain object list
                geometry_msgs::Point check_points;
                check_points.x = map_points.point.x;
                check_points.y = map_points.point.y;
                check_points.z = map_points.point.z;
                ObjectList.push_back(check_points);
                //
                fs.open("/home/ras28/catkin_ws/src/ras_color_detection/result/objectlist.txt",std::ofstream::out | std::ofstream::app);
                if(!fs)
                {
                    std::cerr<<"Cannot open the output file."<<std::endl;
                }
                fs<<"blue_cube"<<" "<<map_points.point.x<<" "<<map_points.point.y+0.015<<" "<<z<<"\n";
                fs.close();
              }
            }
          }
        }
        //purple
        else if (color == 3){
          infile.open("/home/ras28/catkin_ws/src/ras_color_detection/result/Star.txt",std::ios::app);
          if(infile.is_open()){
            while(infile >> value){
              w.push_back(value);
            }
            infile.close();
          }
          for (int i = 0; i < 308; i++){
            sum += vfhs->points[0].histogram[i]* w[i];
          }
          sum = sum + 0.0684464637696485;
          w.clear();
          if(sum > 0){
            if (x != 0.0 || y != 0.0 || z != 0.0){
              pub_Objectpose.pose.position.y += 0.01;
              pub_Objectpose.pose.position.x -= 0.01;
              pub_Objectpose.pose.position.z += 0.02;
              pose_pub.publish(pub_Objectpose);
              if (check(map_points.point.x,map_points.point.y)){
                //ros evidence
                objectEvidence.object_id = objectEvidence.purple_star;
                objectEvidence.image_evidence = *image;
                objectEvidence.object_location.transform.translation.x = x;
                objectEvidence.object_location.transform.translation.y = y;
                objectEvidence.object_location.transform.translation.z = z;
                evidence_pub.publish(objectEvidence);
                //ros speak
                msgs.data = target[color].getmsg_size2();
                speak_pub.publish(msgs);
                ROS_INFO("purple star");
                //brain object list
                geometry_msgs::Point check_points;
                check_points.x = map_points.point.x;
                check_points.y = map_points.point.y;
                check_points.z = map_points.point.z;
                ObjectList.push_back(check_points);
                //
                fs.open("/home/ras28/catkin_ws/src/ras_color_detection/result/objectlist.txt",std::ofstream::out | std::ofstream::app);
                if(!fs)
                {
                    std::cerr<<"Cannot open the output file."<<std::endl;
                }
                fs<<"purple_star"<<" "<<map_points.point.x<<" "<<map_points.point.y+0.015<<" "<<z-0.015<<"\n";
                fs.close();
              }
            }
          }
          else{
            if (x != 0.0 || y != 0.0 || z != 0.0){
              pub_Objectpose.pose.position.y -= 0.01;
              pub_Objectpose.pose.position.x -= 0.01;
              pub_Objectpose.pose.position.z += 0.02;
              pose_pub.publish(pub_Objectpose);
              if (check(map_points.point.x,map_points.point.y)){
                //ros evidence
                objectEvidence.object_id = objectEvidence.purple_cross;
                objectEvidence.image_evidence = *image;
                objectEvidence.object_location.transform.translation.x = x;
                objectEvidence.object_location.transform.translation.y = y;
                objectEvidence.object_location.transform.translation.z = z;
                evidence_pub.publish(objectEvidence);
                //ros speak
                msgs.data = target[color].getmsg_size2();
                speak_pub.publish(msgs);
                ROS_INFO("purple cross");
                //brain object list
                geometry_msgs::Point check_points;
                check_points.x = map_points.point.x;
                check_points.y = map_points.point.y;
                check_points.z = map_points.point.z;
                ObjectList.push_back(check_points);
                //
                fs.open("/home/ras28/catkin_ws/src/ras_color_detection/result/objectlist.txt",std::ofstream::out | std::ofstream::app); 
                if(!fs)
                {
                    std::cerr<<"Cannot open the output file."<<std::endl;
                }
                fs<<"purple_cross"<<" "<<map_points.point.x<<" "<<map_points.point.y+0.015<<" "<<z-0.015<<"\n";
                fs.close();
              }
            }
          }
        }
        //orange
        else if (color == 4){
          if (x != 0.0 || y != 0.0 || z != 0.0){
            pub_Objectpose.pose.position.y += 0.01;
            pub_Objectpose.pose.position.x -= 0.01;
            pub_Objectpose.pose.position.z += 0.02;
            pose_pub.publish(pub_Objectpose);
            if (check(map_points.point.x,map_points.point.y)){
              //ros evidence
              objectEvidence.object_id = objectEvidence.patric;
              objectEvidence.image_evidence = *image;
              objectEvidence.object_location.transform.translation.x = x;
              objectEvidence.object_location.transform.translation.y = y;
              objectEvidence.object_location.transform.translation.z = z;
              evidence_pub.publish(objectEvidence);
              //ros speak
              msgs.data = target[color].getmsg_size1();
              speak_pub.publish(msgs);
              ROS_INFO("orange star");
              //brain object list
              geometry_msgs::Point check_points;
              check_points.x = map_points.point.x;
              check_points.y = map_points.point.y;
              check_points.z = map_points.point.z;
              ObjectList.push_back(check_points);
              //
              fs.open("/home/ras28/catkin_ws/src/ras_color_detection/result/objectlist.txt",std::ofstream::out | std::ofstream::app);
              if(!fs)
              {
                  std::cerr<<"Cannot open the output file."<<std::endl;
              }
              fs<<"orange_star"<<" "<<map_points.point.x<<" "<<map_points.point.y+0.015<<" "<<z-0.015<<"\n";
              fs.close();
            }
          }
        }
        //yellow
        else if (color == 5){
          infile.open("/home/ras28/catkin_ws/src/ras_color_detection/result/ball.txt",std::ios::app);
          if(infile.is_open()){
            while(infile >> value){
              w.push_back(value);
            }
            infile.close();
          }
          for (int i = 0; i < 308; i++){
            sum += vfhs->points[0].histogram[i]* w[i];
          }
          sum = sum - 0.864084296806196;
          w.clear();
          if(sum > 0){
            if (x != 0.0 || y != 0.0 || z != 0.0){
              pub_Objectpose.pose.position.y -= 0.01;
              pub_Objectpose.pose.position.z += 0.01;
              pose_pub.publish(pub_Objectpose); 
              if (check(map_points.point.x,map_points.point.y)){
                //ros evidence
                objectEvidence.object_id = objectEvidence.yellow_ball;
                objectEvidence.image_evidence = *image;
                objectEvidence.object_location.transform.translation.x = x;
                objectEvidence.object_location.transform.translation.y = y;
                objectEvidence.object_location.transform.translation.z = z;
                evidence_pub.publish(objectEvidence);
                //ros speak
                msgs.data = target[color].getmsg_size1();
                speak_pub.publish(msgs);
                ROS_INFO("yellow ball");
                //brain object list
                geometry_msgs::Point check_points;
                check_points.x = map_points.point.x;
                check_points.y = map_points.point.y;
                check_points.z = map_points.point.z;
                ObjectList.push_back(check_points);
                //
                fs.open("/home/ras28/catkin_ws/src/ras_color_detection/result/objectlist.txt",std::ofstream::out | std::ofstream::app);
                if(!fs)
                {
                    std::cerr<<"Cannot open the output file."<<std::endl;
                }
                fs<<"yellow_ball"<<" "<<map_points.point.x<<" "<<map_points.point.y+0.015<<" "<<z<<"\n";
                fs.close();
              }
            }
          }
          else{
            if (x != 0.0 || y != 0.0 || z != 0.0){
              pub_Objectpose.pose.position.y -= 0.01;
              pub_Objectpose.pose.position.x -= 0.01;
              pub_Objectpose.pose.position.z += 0.01;
              pose_pub.publish(pub_Objectpose); 
              if (check(map_points.point.x,map_points.point.y)){
                //ros evidence
                objectEvidence.object_id = objectEvidence.yellow_cube;
                objectEvidence.image_evidence = *image;
                objectEvidence.object_location.transform.translation.x = x;
                objectEvidence.object_location.transform.translation.y = y;
                objectEvidence.object_location.transform.translation.z = z;
                evidence_pub.publish(objectEvidence);
                //ros speak
                msgs.data = target[color].getmsg_size2();
                speak_pub.publish(msgs);
                ROS_INFO("yellow cube");
                //brain object list
                geometry_msgs::Point check_points;
                check_points.x = map_points.point.x;
                check_points.y = map_points.point.y;
                check_points.z = map_points.point.z;
                ObjectList.push_back(check_points);
                //
                fs.open("/home/ras28/catkin_ws/src/ras_color_detection/result/objectlist.txt",std::ofstream::out | std::ofstream::app); 
                if(!fs)
                {
                    std::cerr<<"Cannot open the output file."<<std::endl;
                }
                fs<<"yellow_cube"<<" "<<map_points.point.x<<" "<<map_points.point.y+0.015<<" "<<z<<"\n";
                fs.close();
              }
            }
          }
        }
        //Publish the filtered pointcloud!
        point_pub.publish(model_keypoints);
      }
    }
  }
  for(int color = 0; color < target.size(); color++ ){
    target[color].Objectpos.clear();
    target[color].image.clear();
    object_[color] = false;
  }

  // further detection point!
  double min = 1000;
  double dis = 0;
  int further_index;
  if(further_xyzpos.size() != 0){
    for(int further_number = 0; further_number < further_xyzpos.size(); further_number++){
      dis = sqrt(pow(further_xyzpos[further_number][0],2)+pow(further_xyzpos[further_number][1],2));
      if (dis < min){
        min = dis;
        further_index = further_number;
      }
    }
    // WARNING!!!!  closest object methods have bugs!!!
    //tf to world frame
    try{
      listener.waitForTransform("world","camera_rgb_optical_frame",ros::Time(0),ros::Duration(1));
      geometry_msgs::PointStamped tf_points;
      tf_points.header.frame_id = "camera_rgb_optical_frame";
      tf_points.point.x = further_xyzpos[further_index][0];
      tf_points.point.y = further_xyzpos[further_index][1];
      tf_points.point.z = further_xyzpos[further_index][2];
      listener.transformPoint("world",tf_points,map_points);
    }
    catch (tf::TransformException e) {
      ROS_ERROR("%s",e.what());
    }
    //check for repeatation
    if (check(map_points.point.x,map_points.point.y)){
      if(check_further(map_points.point.x,map_points.point.y)){
        std::cout<<"further_map_point "<<map_points.point.x<<" "<<map_points.point.y<<std::endl;
        geometry_msgs::Point check_further_points;
        check_further_points.x = map_points.point.x;
        check_further_points.y = map_points.point.y;
        check_further_points.z = further_xyzpos[further_index][2];
        FurtherList.push_back(check_further_points);
        ROS_INFO("here!");
        F_srv.request.x.data = map_points.point.x;
        F_srv.request.y.data = map_points.point.y;
        ROS_INFO("here!");
        further_call.call(F_srv);
        ROS_INFO("here!");
      }
    }
  }
  further_xyzpos.clear();
  std::cout<<"finish"<<std::endl;
}

void pointDataCallback(const sensor_msgs::PointCloud2::ConstPtr& point_color_data) {
  pcl::fromROSMsg(*point_color_data, *pclData);  
}
        
int main(int argc, char **argv)
{
  ros::init(argc, argv, "color_detection_node");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber image_sub;
  ros::Subscriber sub_position;
  object red("red"), green("green"), blue("blue"), purple("purple"), yellow("yellow"), orange("orange");
  target.push_back(red);
  target.push_back(green);
  target.push_back(blue);
  target.push_back(purple);
  target.push_back(orange);
  target.push_back(yellow);

  object_.push_back(false);
  object_.push_back(false);
  object_.push_back(false);
  object_.push_back(false);
  object_.push_back(false);
  object_.push_back(false);

  objectEvidence.group_number = 8;
  objectEvidence.object_location.header.frame_id = "camera_rgb_optical_frame";
  objectEvidence.object_location.child_frame_id = "/world";
  objectEvidence.object_location.transform.rotation.x = 0;
  objectEvidence.object_location.transform.rotation.y = 0;
  objectEvidence.object_location.transform.rotation.z = 0;
  objectEvidence.object_location.transform.rotation.w = 1;

  sub_position = nh.subscribe<sensor_msgs::PointCloud2>("camera/depth_registered/points", 1, pointDataCallback);
  image_sub = it.subscribe("/camera/rgb/image_rect_color",1,imageCallback);
  point_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ> >("/camera/Object", 1);
  speak_pub = nh.advertise<std_msgs::String>("/robot/talk", 1);
  pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/camera/ObjectPose",1);
  further_call = nh.serviceClient<ras_color_detection::FPose>("brain/further_object_points");
  evidence_pub = nh.advertise<ras_msgs::RAS_Evidence>("/evidence",1);
  ros::spin();
  return 0;
}

bool check(double x, double y){
  if(x <= 0.0 && y <= 0.0){return false;}
  double distance;
  if(ObjectList.size() == 0)
    {return true;}
  for (int i = ObjectList.size()-1; i >= 0; i--){
    distance = std::sqrt(std::pow((x - ObjectList[i].x),2) + std::pow((y - ObjectList[i].y),2));
    if (distance < 0.075) {return false;}
  }
  return true;
}
bool check_further(double x, double y){
  if(x <= 0.0 && y <= 0.0){return false;}
  if(FurtherList.size() == 0){return true;}
  double distance;
  for (int i = FurtherList.size()-1; i >= 0; i--){
    distance = std::sqrt(std::pow((x - FurtherList[i].x),2) + std::pow((y - FurtherList[i].y),2));
    if (distance < 0.075) {return false;}
  }
  return true;
}
int check_safeArea(double x, double y, double z){
  if (-0.06 < x && x < 0.11 && 0.07< z && z < 0.22) 
    return 0;
  else if (!std::isfinite(x)) 
    return 1;
  else 
    return 2;
}

