#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <image_transport/image_transport.h>
#include <opencv/cv.h>
#include <vector>
#include <pcl/filters/passthrough.h>

//ros::Publisher pub,pub1;
//void shapefind(cv::Mat img, cv::Mat img_origin);
image_transport::Publisher pub_image,pub_image1;
void pointDataCallback(const sensor_msgs::PointCloud2::ConstPtr& point_color_data) {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclData (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*point_color_data, *pclData);

    //rotate the point cloud
    Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
    float theta = -M_PI/180*17.5;
    transform_2.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitX()));

    // Executing the transformation
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud1 (new pcl::PointCloud<pcl::PointXYZRGB> ());
    pcl::transformPointCloud (*pclData, *transformed_cloud1, transform_2);

    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    // Build a passthrough filter to remove spurious NaNs
    pass.setInputCloud (transformed_cloud1);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0, 0.4);
    pass.filter (*transformed_cloud);
    /*
    //pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_transformed (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_projected (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_projected1 (new pcl::PointCloud<pcl::PointXYZRGB>);

    // Create a set of planar coefficients with Z=X=0,Y=1
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    coefficients->values.resize (4);
    coefficients->values[0] = 0;
    coefficients->values[1] = 1;
    coefficients->values[2] = 0;
    coefficients->values[3] = 0;


    // Create a set of planar coefficients with Y=X=0,Z=1
    pcl::ModelCoefficients::Ptr coefficients1 (new pcl::ModelCoefficients ());
    coefficients1->values.resize (4);
    coefficients1->values[0] = 0;
    coefficients1->values[1] = 0;
    coefficients1->values[2] = 1;
    coefficients1->values[3] = 0;

    // Create the filtering object
    pcl::ProjectInliers<pcl::PointXYZRGB> proj;
    proj.setModelType (pcl::SACMODEL_PLANE);
    proj.setInputCloud (transformed_cloud);
    proj.setModelCoefficients (coefficients);
    proj.filter (*cloud_projected);


    // Create the filtering object
    pcl::ProjectInliers<pcl::PointXYZRGB> proj1;
    proj1.setModelType (pcl::SACMODEL_PLANE);
    proj1.setInputCloud (transformed_cloud);
    proj1.setModelCoefficients (coefficients1);
    proj1.filter (*cloud_projected1);*/

   /*pcl::PointXYZRGB min_pt;
    pcl::PointXYZRGB max_pt;
    pcl::getMinMax3D(*transformed_cloud, min_pt, max_pt);
    ROS_INFO("min_pt  %f, %f ,%f",min_pt.x,min_pt.y,min_pt.z);
    ROS_INFO("max_pt  %f, %f ,%f",max_pt.x,max_pt.y,max_pt.z);*/

    //project the image into xy-plane and xz-plane
    cv::Mat output = cv::Mat(transformed_cloud->height,transformed_cloud->width, CV_8UC3);
    cv::Mat output1 = cv::Mat(transformed_cloud->height,transformed_cloud->width, CV_8UC3);
    output.setTo(0);
    output1.setTo(0);
    if (!transformed_cloud->empty()){
        for(int h = 0; h < transformed_cloud->height; h++){
        for(int w = 0; w < transformed_cloud->width; w++)
        {
            pcl::PointXYZRGB point = transformed_cloud->at(w, h);
            Eigen::Vector3i rgb = point.getRGBVector3i();
            //ROS_INFO("%f,%f",point.z,point.x );
            if(std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z)){
                output.at<cv::Vec3b>((int)round((point.z)*480/1.2),(int)round((point.x+0.2)*640/0.7))[0] = rgb[2];
                output.at<cv::Vec3b>((int)round((point.z)*480/1.2),(int)round((point.x+0.2)*640/0.7))[1] = rgb[1];
                output.at<cv::Vec3b>((int)round((point.z)*480/1.2),(int)round((point.x+0.2)*640/0.7))[2] = rgb[0];

                output1.at<cv::Vec3b>((int)round((point.y+0.1)*480/0.3),(int)round((point.x+0.2)*640/0.7))[0] = rgb[2];
                output1.at<cv::Vec3b>((int)round((point.y+0.1)*480/0.3),(int)round((point.x+0.2)*640/0.7))[1] = rgb[1];
                output1.at<cv::Vec3b>((int)round((point.y+0.1)*480/0.3),(int)round((point.x+0.2)*640/0.7))[2] = rgb[0];
           }
        }
       }
    }
/*
    //process the image
    cv::Mat img_hsv,img_hsv1,img_output,img_output1;
    cv::cvtColor(output, img_hsv, CV_BGR2HSV);
    cv::cvtColor(output1, img_hsv1, CV_BGR2HSV);
    cv::inRange(img_hsv,cv::Scalar(0,200,146),cv::Scalar(11,255,255),img_output);
    cv::inRange(img_hsv,cv::Scalar(0,200,146),cv::Scalar(11,255,255),img_output1);
    //Erosion and Dilation
    cv::Mat erodeElemant = cv::getStructuringElement(cv::MORPH_RECT,cv::Size(2*2+1,2*2+1));
    cv::Mat dilateElemant = cv::getStructuringElement(cv::MORPH_RECT,cv::Size(2*2+1,2*2+1));
    cv::erode(img_output,img_output, erodeElemant);
    cv::dilate(img_output,img_output, dilateElemant);

    cv::erode(img_output1,img_output1, erodeElemant);
    cv::dilate(img_output1,img_output1, dilateElemant);
    //find the contours
    shapefind(img_output,output);
    shapefind(img_output1,output1);*/

    cv::imshow("Image project",output);
    cv::imshow("Image project1",output1);
    cv::waitKey(1);

    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", output).toImageMsg();
    sensor_msgs::ImagePtr msg1 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", output1).toImageMsg();
    //pub1.publish(cloud_projected1);
    //pub.publish(cloud_projected);
    pub_image.publish(msg);
    pub_image.publish(msg1);
}

int main (int argc, char** argv)
{
    ros::init(argc, argv, "ras_point_rotation_node");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe<sensor_msgs::PointCloud2>("camera/Object", 1, pointDataCallback);
    //pub =n.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("/camera/project", 1);
    //pub1 =n.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("/camera/project1", 1);
    image_transport::ImageTransport it(n);
    pub_image = it.advertise("camera/project_image_xzPlane", 1);
    pub_image1 = it.advertise("camera/project_image_xyPlane", 1);
    cv::namedWindow("Image project", CV_WINDOW_AUTOSIZE);
    cv::namedWindow("Image project1", CV_WINDOW_AUTOSIZE);
    ros::spin();
    cv::destroyWindow("Image processed");
    cv::destroyWindow("Image processed1");
    return 0;
}

/*void shapefind(cv::Mat img, cv::Mat img_origin)
{
    cv::Mat canny_output;
    std::vector<std::vector<cv::Point> > contours,model;
    std::vector<cv::Vec4i> hierarchy;
    std::vector<std::vector<cv::Point> > approx;
    Canny( img, canny_output, 20, 20*2, 3 );
    cv::findContours( canny_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );
    approx.resize(contours.size());
    for( size_t k = 0; k < contours.size(); k++ )
           cv::approxPolyDP(cv::Mat(contours[k]),approx[k], 3, true);
    for( int i = 0; i< approx.size(); i++ )
    {
        cv::Scalar color = cv::Scalar(0,0,0);
        drawContours( img_origin, approx, i, color, 1, 8, hierarchy, 0, cv::Point() );
    }
    //cv::imshow("Image",img_origin);
    //cv::waitKey(3);
    //ROS_INFO("approx.size %d   hierarchy[0]%d",approx.size(),hierarchy[0][2]);
}
*/
