#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/common/transforms.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
#include <vector>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <ras_target_recognition/Line.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

pcl::PassThrough<pcl::PointXYZ> pass;
ros::Publisher pcl_pub, battery_pub;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclColor (new pcl::PointCloud<pcl::PointXYZRGB>);
void pointDataCallback(const sensor_msgs::PointCloud2::ConstPtr& point_color_data) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl1 (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*point_color_data, *pcl1);
  pcl::fromROSMsg(*point_color_data, *pclColor);
  //rotate the point
  Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
  float theta = -M_PI/180*13.5;
  transform_2.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitX()));
  pcl::PointCloud<pcl::PointXYZ>::Ptr pclData (new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::transformPointCloud (*pcl1, *pclData, transform_2);

  pcl::PointCloud<pcl::PointXYZ>::Ptr pclData_filtered1 (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr pclData_filtered2 (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr model_keypoints (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  // Build a passthrough filter to remove spurious NaNs
  pass.setInputCloud (pclData);
  pass.setFilterFieldName ("y");
  pass.setFilterLimits (0.048, 0.05);
  pass.filter (*pclData_filtered2);

  pass.setInputCloud (pclData_filtered2);
  pass.setFilterFieldName ("x");
  pass.setFilterLimits (-0.15, 0.15);
  pass.filter (*pclData_filtered1);       

  if(pclData_filtered1->size() < 100){return ;}
  //rotate back   
  Eigen::Affine3f transform_1 = Eigen::Affine3f::Identity();
  float theta1 = M_PI/180*13.5;
  transform_1.rotate (Eigen::AngleAxisf (theta1, Eigen::Vector3f::UnitX()));
  pcl::PointCloud<pcl::PointXYZ>::Ptr pclback (new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::transformPointCloud (*pclData_filtered1, *pclback, transform_1);  

  //remove noise
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud (pclback);
  sor.setMeanK (50);
  sor.setStddevMulThresh (0.8);
  sor.filter (*model_keypoints);

  //downsampling
  pcl::VoxelGrid<pcl::PointXYZ> downsampling;
  downsampling.setInputCloud (model_keypoints);
  downsampling.setLeafSize (0.005f, 0.005f, 0.005f);
  downsampling.filter (*cloud_filtered);
  ROS_INFO("cloud size %ld",cloud_filtered->size());
  pcl_pub.publish(cloud_filtered);
}
void imageCallback(const sensor_msgs::ImageConstPtr& image) {
  if (!pclColor->isOrganized()){ return;}
  //cv bridge
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
  tf::TransformListener listener;
  ras_target_recognition::Line battery_point;
  geometry_msgs::PointStamped map_points, map_points1;
  //found black area!
  cv::Mat img_bgr = cv_ptr->image.clone();
  cv::Mat img_gray,img_output;
  cv::inRange(img_bgr,cv::Scalar(0,0,0),cv::Scalar(20,20,20),img_output);
  cv::Mat erodeElemant = cv::getStructuringElement(cv::MORPH_RECT,cv::Size(2*5+1,2*5+1));
  cv::Mat dilateElemant = cv::getStructuringElement(cv::MORPH_RECT,cv::Size(2*5+1,2*5+1));
  cv::erode(img_output,img_output, erodeElemant);
  cv::dilate(img_output,img_output, dilateElemant);

  std::vector<std::vector<double> > corner;
  std::vector<std::vector<cv::Point> > contours;
  std::vector<cv::Vec4i> hierarchy;
  cv::findContours( img_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );
  if (hierarchy.size() > 0) {
    int numObjects = hierarchy.size();
     for (int index = 0; index >= 0; index = hierarchy[index][0]) {
      cv::Moments moment = cv::moments((cv::Mat)contours[index]);
      double area = moment.m00;
      //if the area is less than 30 px by 40px then it is probably just noise or too far away
      if(area > 8000){
        ROS_INFO("STOP!!!");
        cv::Rect br = cv::boundingRect(contours[index]);
        int m0_y_left = (int)moment.m01/area;
        int m0_y_right = (int)moment.m01/area;
        int left_x = (int)br.x;
        int right_x = (int)(br.x + br.width);
        while(std::isnan(pclColor->at(left_x, m0_y_left).x)){
          m0_y_left++;
          if(m0_y_left > 479){
            break;
          }
        }
        while(std::isnan(pclColor->at(right_x, m0_y_right).x)){
          m0_y_right++;
          if(m0_y_right > 479){
            break;
          }
        }
        if(m0_y_right > 479 || m0_y_left > 479){
          continue;
        }
        //ROS_INFO("m0_y_left %d,m0_y_right %d",m0_y_left,m0_y_right);
        std::vector<double> position,position1;
        position.push_back(pclColor->at(left_x,m0_y_left).x);
        position.push_back(pclColor->at(left_x,m0_y_left).y);
        position.push_back(pclColor->at(left_x,m0_y_left).z);
        position1.push_back(pclColor->at(right_x,m0_y_right).x);
        position1.push_back(pclColor->at(right_x,m0_y_right).y);
        position1.push_back(pclColor->at(right_x,m0_y_right).z);
        corner.push_back(position);
        corner.push_back(position1);
        break; 
      }
    }
  }
  //seg fault?
  if(corner.size() == 0){return;}
  //tf transform
  try{
    geometry_msgs::PointStamped tf_points;
    tf_points.header.frame_id = "camera_rgb_optical_frame";
    tf_points.header.stamp = ros::Time(0) ;
    listener.waitForTransform("robot_center",tf_points.header.frame_id,tf_points.header.stamp,ros::Duration(1));
    tf_points.point.x = corner[0][0];
    tf_points.point.y = corner[0][1];
    tf_points.point.z = corner[0][2];
    listener.transformPoint("robot_center",tf_points,map_points);
    tf_points.point.x = corner[1][0];
    tf_points.point.y = corner[1][1];
    tf_points.point.z = corner[1][2];
    listener.transformPoint("robot_center",tf_points,map_points1);
  }
  catch (tf::TransformException e) {
    ROS_ERROR("%s",e.what());
  }
  //calculate the distance!
  map_points.point.x += 0.05;
  map_points1.point.x += 0.05;
  //call the service!      
  battery_point.start.x = map_points.point.x;
  battery_point.start.y = map_points.point.y;
  battery_point.start.z = map_points.point.z;
  battery_point.stop.x = map_points1.point.x;
  battery_point.stop.y = map_points1.point.y;
  battery_point.stop.z = map_points1.point.z;
  battery_pub.publish(battery_point);
}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "target_recognition");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber image_sub;
  ros::Subscriber sub_pointcloud;
  sub_pointcloud = nh.subscribe<sensor_msgs::PointCloud2>("camera/depth_registered/points", 1, pointDataCallback);
  pcl_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ> >("camera/obstacle", 1);
  battery_pub = nh.advertise<ras_target_recognition::Line>("/camera/battery",1);
  image_sub = it.subscribe("/camera/rgb/image_rect_color",1,imageCallback);
  ros::spin();
}
