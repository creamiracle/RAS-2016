#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <cv.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>
#include <fstream>
#include <iterator>
#include <string>
#include <opencv2/features2d/features2d.hpp>

int Hlow = 0;
int Hhigh = 360;
//int Hhigh = 255;
int Slow = 0;
int Shigh = 255;
int Vlow = 0;
int Vhigh = 255;
int minarea = 0;
//int mincircularity = 0;
//int minconvexity = 0;
//int mininteria = 0;
cv::Mat img_output;
int type1,type2;
int Erosionsize=0,Erosiontype=0,Dilationsize=0,Dilationtype=0;
std::vector<std::vector<cv::Point> > approx;
void on_trackbar( int, void* ) {}
//find shape
void shapefind(cv::Mat img, cv::Mat img_origin);

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    // Update GUI Window
    cv::Mat img_hsv;
    cv::cvtColor(cv_ptr->image, img_hsv, CV_BGR2HSV);
    //cv::imshow("Image raw", cv_ptr->image);
    cv::inRange(img_hsv, cv::Scalar(Hlow,Slow,Vlow), cv::Scalar(Hhigh,Shigh,Vhigh), img_output);
    //cv::inRange(cv_ptr->image, cv::Scalar(Hlow,Slow,Vlow), cv::Scalar(Hhigh,Shigh,Vhigh), img_output);
    //morphOps(img_output);
    if(Erosiontype == 0){type1 = cv::MORPH_RECT;}
    else if(Erosiontype == 1){type1 = cv::MORPH_CROSS;}
    else{type1 = cv::MORPH_ELLIPSE;}
    if(Dilationtype == 0){type2 = cv::MORPH_RECT;}
    else if(Dilationtype == 1){type2 = cv::MORPH_CROSS;}
    else{type2 = cv::MORPH_ELLIPSE;}
    cv::Mat erodeElemant = cv::getStructuringElement(type1,cv::Size(2*Erosionsize+1,2*Erosionsize+1));
    cv::Mat dilateElemant = cv::getStructuringElement(type2,cv::Size(2*Dilationsize+1,2*Dilationsize+1));
    cv::erode(img_output,img_output, erodeElemant);
    cv::dilate(img_output,img_output, dilateElemant);
    cv::imshow("Image", img_output);
    shapefind(img_output,cv_ptr->image);
    /*
    cv::SimpleBlobDetector::Params params;
    // Filter by Area.
    params.filterByArea = true;
    params.minArea = minarea;

    // Filter by Circularity
    params.filterByCircularity = true;
    params.minCircularity = mincircularity/10;

    // Filter by Convexity
    params.filterByConvexity = true;
    params.minConvexity = minconvexity/10;

    // Filter by Inertia
    params.filterByInertia = true;
    params.minInertiaRatio = mininteria/1000;

    //cv::SimpleBlobDetector detector(params);
    cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);

    // Detect blobs.
    std::vector<cv::KeyPoint> keypoints;
    detector->detect( img_output, keypoints);

    // Draw detected blobs as blue circles.
    // DrawMatchesFlags::DRAW_RICH_KEYPOINTS flag ensures the size of the circle corresponds to the size of blob
    cv::drawKeypoints( cv_ptr->image, keypoints, cv_ptr->image, cv::Scalar(0,0,0), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
    cv::imshow("Image raw", cv_ptr->image);
    cv::waitKey(3);
    */
}

int main (int argc, char** argv) {
    ros::init(argc, argv, "color_calibration");
    ros::NodeHandle n;
    cv::namedWindow("Image", CV_WINDOW_AUTOSIZE);
    cv::namedWindow("Image raw", CV_WINDOW_AUTOSIZE);
    cv::namedWindow("HSV Controls", CV_WINDOW_AUTOSIZE);
    cv::createTrackbar("H_low", "HSV Controls", &Hlow, 360, on_trackbar);
    cv::createTrackbar("H_high", "HSV Controls", &Hhigh, 360, on_trackbar);
    cv::createTrackbar("S_low", "HSV Controls", &Slow, 255, on_trackbar);
    cv::createTrackbar("S_high", "HSV Controls", &Shigh, 255, on_trackbar);
    cv::createTrackbar("V_low", "HSV Controls", &Vlow, 255, on_trackbar);
    cv::createTrackbar("V_high", "HSV Controls", &Vhigh, 255, on_trackbar);
    cv::createTrackbar("Erosion_type", "HSV Controls", &Erosiontype, 2,on_trackbar);
    cv::createTrackbar("Erosion_size", "HSV Controls", &Erosionsize, 10,on_trackbar);
    cv::createTrackbar("Dilation_type", "HSV Controls", &Dilationtype,2,on_trackbar);
    cv::createTrackbar("Dilation_size", "HSV Controls", &Dilationsize, 10,on_trackbar);
    //cv::createTrackbar("minArea", "HSV Controls", &minarea, 1000,on_trackbar);
    //cv::createTrackbar("minCircle", "HSV Controls", &mincircularity, 10,on_trackbar);
    //cv::createTrackbar("minConvexity", "HSV Controls", &minconvexity, 10,on_trackbar);
    //cv::createTrackbar("minInteria", "HSV Controls", &mininteria, 1000,on_trackbar);

    cv::startWindowThread();
    image_transport::ImageTransport it(n);
    image_transport::Subscriber sub = it.subscribe("camera/rgb/image_rect_color", 1, imageCallback);
    ros::spin();
    cv::destroyWindow("Image");
    cv::destroyWindow("Image raw");
    cv::destroyWindow("HSV Controls");
    //std::ofstream output_file("cube_contours.txt");
    //std::ostream_iterator<std::vector<cv::Point> > output_iterator(output_file, "\n");
    //std::copy(approx.begin(), approx.end(), output_iterator);

}
void shapefind(cv::Mat img, cv::Mat img_origin)
{
    cv::Mat canny_output;
    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;
    //std::vector<std::vector<cv::Point> > approx;
    Canny( img, canny_output, 20, 20*2, 3 );
    cv::findContours( canny_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );
    approx.resize(contours.size());
    for( size_t k = 0; k < contours.size(); k++ )
           cv::approxPolyDP(cv::Mat(contours[k]),approx[k], 1, true);
    for( int i = 0; i< approx.size(); i++ )
    {
        cv::Scalar color = cv::Scalar(0,0,0);
        drawContours( img_origin, approx, i, color, 1, 8, hierarchy, 0, cv::Point() );
    }
    ROS_INFO("approx size %ld", approx.size());
    cv::imshow("Image raw",img_origin);
    cv::waitKey(3);
}
