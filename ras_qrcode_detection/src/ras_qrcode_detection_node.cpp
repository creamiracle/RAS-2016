#include <cmath>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/PointCloud2.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/features/normal_3d.h>
#include <geometry_msgs/PoseStamped.h>
#include <cmath>
#include <sensor_msgs/image_encodings.h>

geometry_msgs::PoseStamped rubble_top;
ros::Publisher rubble_pub;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclData;


void findClosestNotNAN(cv::Point2f &start, pcl::PointCloud<pcl::PointXYZRGB> &cloud) {
  int k;
  int i;
  start.x = round(start.x);
  start.y = round(start.y);

  k = 0;
  i = 0;
  while(1) {
    i = 0;
    while (i <= k) {
      if(!std::isnan(cloud((int)start.x + i, (int)start.y + k).x)) {
        start.x += i;
        start.y += k;
        return;
      } else if(!std::isnan(cloud((int)start.x - i, (int)start.y + k).x)) {
        start.x -= i;
        start.y += k;
        return;
      } else if(!std::isnan(cloud((int)start.x + k, (int)start.y + i).x)) {
        start.x += k;
        start.y += i;
        return;
      } else if(!std::isnan(cloud((int)start.x + k, (int)start.y - i).x)) {
        start.x += k;
        start.y -= i;
        return;
      } else if(!std::isnan(cloud((int)start.x + i, (int)start.y - k).x)) {
        start.x += i;
        start.y -= k;
        return;
      } else if(!std::isnan(cloud((int)start.x - i, (int)start.y - k).x)) {
        start.x -= i;
        start.y -= k;
        return;
      } else if(!std::isnan(cloud((int)start.x - k, (int)start.y + i).x)) {
        start.x -= k;
        start.y += i;
        return;
      } else if(!std::isnan(cloud((int)start.x - k, (int)start.y - i).x)) {
        start.x -= k;
        start.y -= i;
        return;
      }
      i++;
    }
    k++;
  }
}

bool areSimilar(double d1, double d2) {
  return std::abs(d1 - d2) / std::min(d1, d2) < 0.8;
}

bool getRubbleTop(cv::Point2f mk1, cv::Point2f mk2, pcl::PointCloud<pcl::PointXYZRGB> &cloud, cv::Mat &img_bgr) {
  if (mk1.x > cloud.width ||
      mk2.x > cloud.width ||
      mk1.y > cloud.height ||
      mk2.y > cloud.height)
    return false;
  bool go_left;
  cv::Rect bound(mk1, mk2);
  double ratio = ((double)bound.width)/((double)bound.height);
  pcl::PointXYZRGB points[3];
  findClosestNotNAN(mk1, cloud);
  findClosestNotNAN(mk2, cloud);
  cv::Point2f marks[3];

  points[0] = cloud((int)round(mk1.x), (int)round(mk1.y));
  points[1] = cloud((int)round(mk2.x), (int)round(mk2.y));
  double real_world_distance = std::sqrt(
    std::pow(points[0].x - points[1].x, 2) +
    std::pow(points[0].y - points[1].y, 2) +
    std::pow(points[0].z - points[1].z, 2)
  );
  if (real_world_distance > 0.04)
    return false;

  if (ratio > 2) { // Horizontal points
    go_left = false;
    if(mk1.x < mk2.x) {
      cv::Point2f mk3 = mk1;
      mk3.y += (mk2.x - mk1.x) / 2;
      findClosestNotNAN(mk3, cloud);
      points[0] = cloud((int)round(mk1.x), (int)round(mk1.y));
      points[1] = cloud((int)round(mk3.x), (int)round(mk3.y));
      points[2] = cloud((int)round(mk2.x), (int)round(mk2.y));
      marks[0] = mk1;
      marks[1] = mk3;
      marks[2] = mk2;
    } else {
      cv::Point2f mk3 = mk2;
      mk3.y += (mk1.x - mk2.x) / 2;
      findClosestNotNAN(mk3, cloud);
      points[0] = cloud((int)round(mk2.x), (int)round(mk2.y));
      points[1] = cloud((int)round(mk3.x), (int)round(mk3.y));
      points[2] = cloud((int)round(mk1.x), (int)round(mk1.y));
      marks[0] = mk2;
      marks[1] = mk3;
      marks[2] = mk1;
    }
  } else if (ratio < 0.5) { // Vertical points
    go_left = false;
    if(mk1.y < mk2.y) {
      points[0] = cloud((int)round(mk1.x), (int)round(mk1.y));
      points[1] = cloud((int)round(mk2.x), (int)round(mk2.y));
      cv::Point2f mk3 = mk1;
      mk3.x += (mk2.y - mk1.y) / 2;
      findClosestNotNAN(mk3, cloud);
      points[2] = cloud((int)round(mk3.x), (int)round(mk3.y));
      marks[0] = mk1;
      marks[1] = mk2;
      marks[2] = mk3;
    } else {
      points[0] = cloud((int)round(mk2.x), (int)round(mk2.y));
      points[1] = cloud((int)round(mk1.x), (int)round(mk1.y));
      cv::Point2f mk3 = mk2;
      mk3.x += (mk1.y - mk2.y) / 2;
      findClosestNotNAN(mk3, cloud);
      points[2] = cloud((int)round(mk3.x), (int)round(mk3.y));
      marks[0] = mk2;
      marks[1] = mk1;
      marks[2] = mk3;
    }
  } else { // Diagnonal points
    go_left = true;
    if (mk1.x < mk2.x) {
      cv::Point2f mk3 = mk1;
      mk3.y -= (mk2.x - mk1.x);
      findClosestNotNAN(mk3, cloud);
      points[0] = cloud((int)round(mk3.x), (int)round(mk3.y));
      points[1] = cloud((int)round(mk1.x), (int)round(mk1.y));
      points[2] = cloud((int)round(mk2.x), (int)round(mk2.y));
      marks[0] = mk3;
      marks[1] = mk1;
      marks[2] = mk2;
    } else {
      cv::Point2f mk3 = mk2;
      mk3.y -= (mk1.x - mk2.x);
      findClosestNotNAN(mk3, cloud);
      points[0] = cloud((int)round(mk3.x), (int)round(mk3.y));
      points[1] = cloud((int)round(mk2.x), (int)round(mk2.y));
      points[2] = cloud((int)round(mk1.x), (int)round(mk1.y));
      marks[0] = mk3;
      marks[1] = mk2;
      marks[2] = mk1;
    }
  }
  // Calculate QR system of reference. x -> diff1, y -> diff2, z -> norm
  cv::Point3d diff1, diff2;
  diff1.x = points[1].x - points[0].x;
  diff1.y = points[1].y - points[0].y;
  diff1.z = points[1].z - points[0].z;
  diff2.x = points[2].x - points[0].x;
  diff2.y = points[2].y - points[0].y;
  diff2.z = points[2].z - points[0].z;
  cv::Point3d norm = diff1.cross(diff2);
  diff2 = norm.cross(diff1); // Squaring up the system of reference
  // Normalize to obtain unit vectors
  double nn = std::sqrt(std::pow(norm.x, 2) + std::pow(norm.y, 2) + std::pow(norm.z, 2));
  double d1n = std::sqrt(std::pow(diff1.x, 2) + std::pow(diff1.y, 2) + std::pow(diff1.z, 2));
  double d2n = std::sqrt(std::pow(diff2.x, 2) + std::pow(diff2.y, 2) + std::pow(diff2.z, 2));
  norm = norm * (1 / nn);
  diff1 = diff1 * (1 / d1n);
  diff2 = diff2 * (1 / d2n);
  // Apply offsets and publish pose
  if (go_left) {
    cv::Point3d rubble_top_delta = - 0.047*norm - 0.05*diff1 - 0.01*diff2;
    rubble_top.pose.position.x = points[2].x + rubble_top_delta.x;
    rubble_top.pose.position.y = points[2].y + rubble_top_delta.y;
    rubble_top.pose.position.z = points[2].z + rubble_top_delta.z;
  } else {
    cv::Point3d rubble_top_delta = - 0.047*norm - 0.05*diff1 + 0.01*diff2;
    rubble_top.pose.position.x = points[0].x + rubble_top_delta.x;
    rubble_top.pose.position.y = points[0].y + rubble_top_delta.y;
    rubble_top.pose.position.z = points[0].z + rubble_top_delta.z;
  }
  rubble_top.header.stamp = ros::Time::now();
  rubble_pub.publish(rubble_top);
  // Visualize output
  cv::line(img_bgr, cv::Point(marks[0].x,marks[0].y), cv::Point(marks[1].x,marks[1].y), cv::Scalar(0,0,255), 5);
  cv::line(img_bgr, cv::Point(marks[0].x,marks[0].y), cv::Point(marks[2].x,marks[2].y), cv::Scalar(0,255,0), 5);
  cv::circle(img_bgr, cv::Point(marks[0].x, marks[0].y), 2, cv::Scalar(255, 0, 0), 5);
  // Success
  return true;
}

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
  //find contours 
  cv::Mat img_bgr,img_gray,edges;
  img_bgr = cv_ptr->image.clone();
  cv::cvtColor(img_bgr,img_gray,CV_BGR2GRAY);
  cv::Canny(img_gray, edges, 100 , 200, 3);
  std::vector< std::vector<cv::Point> > contours;
  std::vector<cv::Vec4i> hierarchy;
  cv::findContours(edges,contours,hierarchy,CV_RETR_TREE,CV_CHAIN_APPROX_SIMPLE );
  //get moments
  std::vector<cv::Moments> mu(contours.size());
  std::vector<cv::Point2f> mc(contours.size());
  //int mark = 0,A,B,C;
  std::vector<int> marker_indices;

  // Start processing the contour data
  for( int i = 0; i < contours.size(); i++ ) {
    mu[i] = cv::moments( contours[i], false ); 
    mc[i] = cv::Point2f( mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00 );
    int k=i;
    int c=0;
    while(hierarchy[k][2] != -1){
      k = hierarchy[k][2] ;
      c = c+1;
    }
    if(hierarchy[k][2] != -1) c = c+1;
    if (c >= 5){ 
      cv::Rect br = cv::boundingRect(contours[i]);
      double cont_area = cv::contourArea(contours[i]);
      double extent = cont_area / br.width / br.height;
      double a_ratio = ((double)std::max(br.width, br.height)) / ((double)std::min(br.width, br.height));
      if(a_ratio < 2 && extent > 0.5) {
        marker_indices.push_back(i);
        cv::drawContours( img_bgr, contours, i, cv::Scalar(0,0,255), 2, 8, hierarchy, 0, cv::Point() );
      }
    }
  } 
  if(marker_indices.size() > 2) {
    double min_dist = 100000000;
    int p1, p2;
    for (int i = 0; i < marker_indices.size() - 1; ++i) {
      for (int j = i + 1; j < marker_indices.size(); ++j) {
        double dist = std::pow(mc[marker_indices[i]].x - mc[marker_indices[j]].x, 2) + std::pow(mc[marker_indices[i]].y - mc[marker_indices[j]].y, 2);
        if (dist < min_dist) {
          min_dist = dist;
          p1 = i;
          p2 = j;
        }
      }
    }
    getRubbleTop(mc[marker_indices[p1]], mc[marker_indices[p2]], *pclData, img_bgr);
  } else if (marker_indices.size() == 2){
    getRubbleTop(mc[marker_indices[0]], mc[marker_indices[1]], *pclData, img_bgr);
  }
  //cv::imshow("qrcode",img_bgr);
  //cv::waitKey(3);
}

int main(int argc,char **argv)
{
	ros::init(argc, argv, "ras_qrcode_detection_node");
	ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber image_sub;
  rubble_top.header.frame_id = "camera_rgb_optical_frame";
  rubble_top.pose.orientation.z = 1;
	ros::Subscriber pc_sub;
  rubble_pub = nh.advertise<geometry_msgs::PoseStamped>("/rubble/pose", 10);
	//cv::namedWindow("qrcode",CV_WINDOW_AUTOSIZE);
	pc_sub = nh.subscribe<sensor_msgs::PointCloud2>("camera/depth_registered/points",1,pointDataCallback);
  image_sub = it.subscribe("/camera/rgb/image_rect_color",1,imageCallback);

  pclData = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

	ros::spin();
	//cv::destroyWindow("qrcode");
	return 0;
}
