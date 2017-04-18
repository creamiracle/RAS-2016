#include <cmath>
#include <ros/ros.h>
#include <iostream>
#include <pcl/filters/passthrough.h>
#include <sensor_msgs/PointCloud2.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pcl/filters/filter.h>
#include <pcl_ros/point_cloud.h>
#include <geometry_msgs/PoseStamped.h>
#include <pcl/features/normal_3d.h>
#include <pcl/impl/point_types.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <pcl/features/vfh.h>
#include <std_msgs/String.h>
#include "ras_msgs/RAS_Evidence.h"
#include "ras_object_detection/object.hpp"
#include <stdlib.h>
#include <pcl/filters/statistical_outlier_removal.h>


ros::Publisher point_pub, speak_pub, pose_pub;
std::vector<object> target;
geometry_msgs::PoseStamped objectPoint;
pcl::PointCloud<pcl::PointXYZ>::Ptr model_keypoints (new pcl::PointCloud<pcl::PointXYZ> ());
pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclData (new pcl::PointCloud<pcl::PointXYZRGB>);

std::vector<bool> object_ ;
pcl::PointCloud<pcl::VFHSignature308>::Ptr vfhs (new pcl::PointCloud<pcl::VFHSignature308> ());

int check_safeArea(double x, double y, double z);
void pointDataCallback(const sensor_msgs::PointCloud2::ConstPtr& point_color_data) {
  pcl::fromROSMsg(*point_color_data, *pclData);
}
void imageCallback(const sensor_msgs::ImageConstPtr& image) {
      if (!pclData->isOrganized()){ return;}
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
      pcl::PointCloud<pcl::PointXYZ>::Ptr pclData_filtered (new pcl::PointCloud<pcl::PointXYZ>);
      pcl::PointCloud<pcl::PointXYZ>::Ptr pclData_filtered2 (new pcl::PointCloud<pcl::PointXYZ>);
      pclData_filtered->height = pclData->height;
      pclData_filtered->width = pclData->width;
      pclData_filtered->header.frame_id = "camera_rgb_optical_frame";
      pclData_filtered->points.resize(pclData_filtered->height*pclData_filtered->width);
      cv::Mat img_bgr = cv_ptr->image.clone();
      cv::Mat img_hsv,img_output,erodeElemant,dilateElemant;
      pcl::PointCloud<pcl::PointXYZ>::Ptr pclData_filtered1 (new pcl::PointCloud<pcl::PointXYZ> ());
      pcl::PassThrough<pcl::PointXYZ> pass;
      cv::cvtColor(img_bgr,img_hsv,CV_BGR2HSV);

      for (int color = 0; color < target.size(); color++){
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
            int numObjects = hierarchy.size();
            for (int index = 0; index >= 0; index = hierarchy[index][0]) {
                cv::Moments moment = cv::moments((cv::Mat)contours[index]);
                double area = moment.m00;
                //if the area is less than 100 px by 50px then it is probably just noise or too far away
                if(area > 4000){
                  int a = (int)moment.m10/area;
                  int b = (int)moment.m01/area;
                  //ROS_INFO_STREAM("moment " << moment.m10 << " " << moment.m01);
                  double temp_x =pclData->at(a,b).x;
                  double temp_y =pclData->at(a,b).y;
                  double temp_z =pclData->at(a,b).z;
                  //ROS_INFO("object %f,%f,%f",temp_x,temp_y,temp_z);
                  if(check_safeArea(temp_x,temp_y,temp_z) == 0) {
                    //safe area 
                  }
                  else{
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
                  //cv::imshow("img_result",img_result);
                  //cv::waitKey(3);
                }
            }
        }

        if(object_[color] == true ){
          for(int same_color = 0; same_color < target[color].image.size(); same_color++){
            for (int i = 0; i < image->width; ++i)
            {
              for (int j = 0; j < image->height; ++j)
              {
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
            // Build a passthrough filter 
            pass.setInputCloud (pclData_filtered);
            pass.setFilterFieldName ("z");
            pass.setFilterLimits (0.05,0.35);
            pass.filter (*pclData_filtered1);
            //check for point 
            if(pclData_filtered1->size() < 100){break;}
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
            // initialize the object center !
            objectPoint.pose.position.x = target[color].Objectpos[same_color][0];
            objectPoint.pose.position.y = target[color].Objectpos[same_color][1];
            objectPoint.pose.position.z = target[color].Objectpos[same_color][2];
            //ROS_INFO("objectcenter %f,%f,%f",objectPoint.pose.position.x,objectPoint.pose.position.y,objectPoint.pose.position.z);
            objectPoint.pose.orientation.w = 1;
            objectPoint.pose.orientation.x = 0;
            objectPoint.pose.orientation.y = 0;
            objectPoint.pose.orientation.z = 0;
            objectPoint.header.frame_id = image->header.frame_id; 
            //clear the pclData_filtered!
            pclData_filtered->clear();
            pclData_filtered->height = pclData->height;
            pclData_filtered->width = pclData->width;
            pclData_filtered->points.resize(pclData_filtered->height*pclData_filtered->width);
            // separate
            double sum;
            double value; 
            std::vector<double> w;
            std::ifstream infile;
            //red
            //ROS_INFO("color %d", color);
            if (color == 0){
              sum = 0;
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
                ROS_INFO("red ball");
                objectPoint.pose.position.y -= 0.01;
                objectPoint.pose.position.z += 0.01;
                pose_pub.publish(objectPoint);
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
                  ROS_INFO("red cube hollow");
                  objectPoint.pose.position.x -= 0.01;
                  objectPoint.pose.position.y += 0.01;
                  objectPoint.pose.position.z += 0.02;
                  pose_pub.publish(objectPoint);
                }
                else{
                  ROS_INFO("red cube");
                  objectPoint.pose.position.x -= 0.01;
                  objectPoint.pose.position.y -= 0.01;
                  objectPoint.pose.position.z += 0.01;
                  pose_pub.publish(objectPoint);
                }
              }
            }
            //green
            if (color == 1){
              sum = 0;
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
                ROS_INFO("green cylinder");
                objectPoint.pose.position.x -= 0.01;
                objectPoint.pose.position.y += 0.01;
                objectPoint.pose.position.z += 0.02;
                pose_pub.publish(objectPoint);
              }
              else{
                ROS_INFO("green cube");
                objectPoint.pose.position.x -= 0.01;
                objectPoint.pose.position.y -= 0.01;
                objectPoint.pose.position.z += 0.01;
                pose_pub.publish(objectPoint);
              }
            }
            //blue
            if (color == 2){
              sum = 0;
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
                ROS_INFO("blue triangle");
                objectPoint.pose.position.x -= 0.01;
                objectPoint.pose.position.y += 0.01;
                objectPoint.pose.position.z += 0.02;
                pose_pub.publish(objectPoint);
              }
              else{
                ROS_INFO("blue cube");
                objectPoint.pose.position.x -= 0.01;
                objectPoint.pose.position.y -= 0.01;
                objectPoint.pose.position.z += 0.01;
                pose_pub.publish(objectPoint);
              }
            }
            //purple
            if (color == 3){
              sum = 0;
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
                ROS_INFO("purple star");
                objectPoint.pose.position.x -= 0.01;
                objectPoint.pose.position.y += 0.01;
                objectPoint.pose.position.z += 0.02;
                pose_pub.publish(objectPoint);
              }
              else{
                ROS_INFO("purple cross");
                objectPoint.pose.position.x -= 0.01;
                objectPoint.pose.position.y += 0.01;
                objectPoint.pose.position.z += 0.02;
                pose_pub.publish(objectPoint);
              }
            }
            //orange
            if (color == 4){
              objectPoint.pose.position.x -= 0.01;
              objectPoint.pose.position.y += 0.01;
              objectPoint.pose.position.z += 0.02;
              pose_pub.publish(objectPoint);
              ROS_INFO("orange star");
            }
            //yellow
            if (color == 5){
              sum = 0;
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
                ROS_INFO("yellow ball");
                objectPoint.pose.position.y -= 0.01;
                objectPoint.pose.position.z += 0.01;
                pose_pub.publish(objectPoint);
              }
              else{
                ROS_INFO("yellow cube");
                objectPoint.pose.position.x -= 0.01;
                objectPoint.pose.position.y -= 0.01;
                objectPoint.pose.position.z += 0.01;
                pose_pub.publish(objectPoint);
              }
            }          
            //Publish the filtered pointcloud!
            point_pub.publish(model_keypoints);
            //clear all the data!
            pclData_filtered1->clear();
            model_keypoints->clear();
            vfhs->clear();
            cloud_normals->clear();
          }
        }
      }
    for(int color = 0; color < target.size(); color++ ){
      target[color].Objectpos.clear();
      target[color].image.clear();
      object_[color] = false;
    }
    //ROS_INFO("finish");
    std::cout<<"finish"<<std::endl;
    }
        
int main(int argc, char **argv)
{
  ros::init(argc, argv, "ras_object_detection_node");
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

  sub_position = nh.subscribe<sensor_msgs::PointCloud2>("camera/depth_registered/points", 1, pointDataCallback);
  point_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ> >("/camera/Object", 1);
  image_sub = it.subscribe("/camera/rgb/image_rect_color",1,imageCallback);
  pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/camera/ObjectPose", 1);

  //cv::namedWindow("img_result",CV_WINDOW_AUTOSIZE);
  ros::spin();
  //cv::destroyWindow("img_result");
  return 0;
}
int check_safeArea(double x, double y, double z){
  if (-0.06 < x && x < 0.11 && 0.07< z && z < 0.32) 
    return 0;
  else if (!std::isfinite(x)) 
    return 1;
  else 
    return 2;
}
