#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
//#include <geometry_msgs/Marker.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <std_srvs/Empty.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <laser_geometry/laser_geometry.h>
#include <visualization_msgs/Marker.h>

#include <iostream>
#include <fstream>
#include <vector>
#include <random>
#include <cmath>

// This structure purpose is to contain lines that will represent walls.
// The first four variables are for a first point, second point representation
// The last two varibles are for a vector representation
struct Line
{
    double x1;
    double y1;
    double x2;
    double y2;
    double theta;
    double t_max;
    double sin_t;
    double cos_t;
};

struct Particle {
    geometry_msgs::Twist position;
    double weight;
};
class Localiser {
std::string map_frame;                          // Map frame for TF purposes
std::vector<Line> walls;                        // Stores the map info
std::vector<Particle> particles;                // Stores the particles for the filter
std::default_random_engine generator;           // Random nuber generator
std::uniform_real_distribution<double> uniform; // Distribution for resampling
std::uniform_real_distribution<double> x_init_dist; // Distribution for initializing x
std::uniform_real_distribution<double> y_init_dist; // Distribution for initializing y
std::uniform_real_distribution<double> z_init_dist; // Distribution for initializing z
std::normal_distribution<double> noise_v;       // Normal distribution object: linear velocity;
std::normal_distribution<double> noise_w;       // Normal distribution object: angular velocity;
std::normal_distribution<double> noise_xy;       // Normal distribution object: position;
std::normal_distribution<double> noise_z;       // Normal distribution object: rotation;

tf::Vector3 shift;                              // Shift between robot center and laser scanner.
tf::Quaternion rotation;                        // Rotation between robot center and laser scanner.
tf::TransformListener my_tf_listener;

double wall_thickness = 0.018;      // Parameter - Thickness of maze walls.
double motion_variance_v = 0.001;   // Parameter - Variance of odometry noise: linear velocity. Default: STDDEV=1cm
double pose_variance_z = 0.001;   // Parameter - Variance of odometry noise: linear velocity. Default: STDDEV=1cm
double pose_variance_xy = 0.001;   // Parameter - Variance of odometry noise: linear velocity. Default: STDDEV=1cm
double motion_variance_w = 0.001;  // Parameter - Variance of odometry noise: angular velocity. Default: STDDEV=1°
double laser_variance = 0.0025;     // Parameter - Variance of laser reading noise. Default: STDVAR=5cm
double laser_min_range = 0.10;      // Parameter - Range below which the laser reading is discarded

int particle_quantity = 100;        // Parameter - Number of particles
bool tracking = true;               // Parameter - Tracking or global localization
double x_init = 2.24;               // Initial x coordinate. Tracking only
double y_init = 0.2;                // Initial y coordinate. Tracking only
double z_init = 1.571;              // Initial rotation. Tracking only
bool interlaced = true;             // true: use 180 laser values each time, alternating between odd and even. false: use all laser readings every time
int weight_combo_method = 0;        // 0: product of gaussian samples. 1: mean of gaussian samples. 2: inverse of sum of distances
bool laser_used = true;
bool noiseincallback = true;
double gaussian_premultiplier = 1;
bool resample = true;
bool addnoise;
int laser_skip, laser_start_index;
double rejected_fraction;
int to_reject;
double max_x, max_y;
double hough_resolution;

pcl::PointCloud<pcl::PointXYZI>::Ptr particle_cloud; // For display purposes
//pcl::PointCloud<pcl::PointXYZ>::Ptr particle_cloud_nw; // For display purposes
ros::Publisher cloud_pub;
ros::Publisher sim_scan_pub;
ros::Publisher new_walls_pub;
ros::Publisher new_walls_r_pub;
visualization_msgs::Marker new_walls;
visualization_msgs::Marker new_walls_r;
sensor_msgs::LaserScan current_laser_scan;
geometry_msgs::PoseStamped ps;
double accumulated_delta_v = 0;
double accumulated_delta_w = 0;
public:
double getExpectedReading (double x0, double y0, double theta) {
    std::vector<double> t_s;
    double sin_t = sin(theta);
    double cos_t = cos(theta);
    for (std::vector<Line>::iterator i = walls.begin(); i != walls.end(); ++i)
    {
        double det_inv = 1 / (sin_t*(i->cos_t)-cos_t*(i->sin_t));
        if (std::isfinite(det_inv)) {
            double t_line = ((x0 - i->x1)*(i->sin_t) - (y0 - i->y1)*(i->cos_t)) * det_inv;
            if (t_line > 0) {
                double t_wall = ((x0 - i->x1)*sin_t - (y0 - i->y1)*cos_t) * det_inv;
                if ((0 < t_wall) && (t_wall < i->t_max))
                    t_s.push_back(t_line);
            }
        }
    }
    std::sort(t_s.begin(), t_s.end());
    if (!t_s.empty())
        return t_s[0];
    else
        return -1;
}

void toLaser (Particle &p, geometry_msgs::Twist &output) {
    output.linear.x = shift.x() * cos(p.position.angular.z) - shift.y() * sin(p.position.angular.z);
    output.linear.y = shift.y() * cos(p.position.angular.z) + shift.x() * sin(p.position.angular.z);
    output.angular.z = rotation.getAngle() * rotation.getAxis().getZ();
}

void particleWeight (Particle &particle, sensor_msgs::LaserScan currentLaser) {
    double weight = 0;
    std::vector<double> weights;
    // int used = 0;
    for (int i = laser_start_index%laser_skip; i < currentLaser.ranges.size(); i += laser_skip) {
        while(!std::isfinite(currentLaser.ranges[i])) 
            i++;
        // used++;
        geometry_msgs::Twist delta;
        toLaser(particle, delta);
        double expected = getExpectedReading(particle.position.linear.x + delta.linear.x,
                                             particle.position.linear.y + delta.linear.y,
                                             particle.position.angular.z + delta.angular.z + currentLaser.angle_min + i * currentLaser.angle_increment);
            switch(weight_combo_method) {
                case 1:
                    // weight += gaussian_premultiplier * exp(-std::pow(expected - currentLaser.ranges[i], 2)/laser_variance);
                    weights.push_back(gaussian_premultiplier * exp(-std::pow(expected - currentLaser.ranges[i], 2)/laser_variance));
                break;
                case 2:
                    // weight += std::pow(expected - currentLaser.ranges[i], 2);
                    weights.push_back(std::pow(expected - currentLaser.ranges[i], 2));
                break;
                default:
                    ROS_ERROR("Wrong parameter value: weight_combo_method = %d. Admitted values are 1 and 2.", weight_combo_method);
                break;
        }
    }
    std::sort(weights.begin(), weights.end(), std::greater<double>());
    int to_reject = (int)(rejected_fraction*weights.size());
    for (std::vector<double>::iterator w = weights.begin(); w != weights.end() - to_reject; ++w) {
        weight += *w;
    }
    switch(weight_combo_method) {
        case 1:
            particle.weight = weight/(double)(weights.size() - to_reject);
            break;
        case 2:
            particle.weight = 1/weight/(double)(weights.size() - to_reject);
            break;
        default:
            ROS_ERROR("Wrong parameter value: weight_combo_method = %d. Admitted values are 1 and 2.", weight_combo_method);
            break;
    }
}

void laserScanCallback (const sensor_msgs::LaserScanConstPtr& scan) {
    current_laser_scan = *scan;
    laser_used = false;
}

void odometryCallback (const geometry_msgs::Twist::ConstPtr& odoPoseDiff) {
    double particle_vel = odoPoseDiff->linear.x;
    double particle_ang = odoPoseDiff->angular.z;
    for (int i = 0; i < particle_quantity; ++i) {
        if(addnoise && noiseincallback) {
            particle_vel += noise_v(generator);
            particle_ang += noise_w(generator);
        }
        particles[i].position.angular.z += particle_ang;
        particles[i].position.linear.x += cos(particles[i].position.angular.z) * particle_vel;
        particles[i].position.linear.y += sin(particles[i].position.angular.z) * particle_vel;
    }
}

void scatterParticles() {
    x_init_dist = std::uniform_real_distribution<double>(0.0, max_x);
    y_init_dist = std::uniform_real_distribution<double>(0.0, max_y);
    z_init_dist = std::uniform_real_distribution<double>(-M_PI, M_PI);
    
    for (int i = 0; i < particles.size(); ++i) {
        particles[i].position.linear.x = x_init_dist(generator);
        particles[i].position.linear.y = y_init_dist(generator);
        particles[i].position.angular.z = z_init_dist(generator);
        particles[i].weight = 1 / particle_quantity;
    }
}

bool scatterCallback (std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
    scatterParticles();
    return true;
}

bool isExistingWall(cv::Vec4i &line) {
    double d_line[4];
    d_line[0] = (double)line[0] * hough_resolution;
    d_line[1] = (double)line[1] * hough_resolution;
    d_line[2] = (double)line[2] * hough_resolution;
    d_line[3] = (double)line[3] * hough_resolution;
    for (std::vector<Line>::iterator wall = walls.begin(); wall != walls.end(); ++wall) {
        double dist1 = std::abs(wall->sin_t*(d_line[0] - wall->x1) -
                                wall->cos_t*(d_line[1] - wall->y1));
        double dist2 = std::abs(wall->sin_t*(d_line[2] - wall->x1) -
                                wall->cos_t*(d_line[3] - wall->y1));
        double euclid1 = std::sqrt(std::pow(d_line[0] - wall->x1, 2) +
                                   std::pow(d_line[1] - wall->y1, 2));
        double euclid2 = std::sqrt(std::pow(d_line[2] - wall->x1, 2) +
                                   std::pow(d_line[3] - wall->y1, 2));
        double euclid3 = std::sqrt(std::pow(d_line[0] - wall->x2, 2) +
                                   std::pow(d_line[1] - wall->y2, 2));
        double euclid4 = std::sqrt(std::pow(d_line[2] - wall->x2, 2) +
                                   std::pow(d_line[3] - wall->y2, 2));
        double min_1 = std::min(euclid1, euclid2);
        double min_2 = std::min(euclid3, euclid4);
        if(dist1 < 0.05 && dist2 < 0.05) {
          double p1 = wall->cos_t * (d_line[0] - wall->x1) + wall->sin_t * (d_line[1] - wall->y1);
          double p2 = wall->cos_t * (d_line[2] - wall->x1) + wall->sin_t * (d_line[3] - wall->y1);
          if (p1 * p2 > 0 && p1 > 0) {
              p1 = wall->cos_t * (d_line[0] - wall->x2) + wall->sin_t * (d_line[1] - wall->y2);
              p2 = wall->cos_t * (d_line[2] - wall->x2) + wall->sin_t * (d_line[3] - wall->y2);
              if(p1 * p2 > 0 && p1 < 0) {
                return true;
              } else if (min_2 < 0.1) { // Changing file
                return true;
              }
          } else {
            if (min_1 < 0.1)
              return true;
          }
        }
    }
    return false;
}

void addLinesToMap() {
    cv::Mat laser_img;
    int l_width = (int)(max_x/hough_resolution);
    int l_height = (int)(max_y/hough_resolution);
    laser_img = cv::Mat(l_width, l_height, CV_8UC1, cv::Scalar(0));
    laser_geometry::LaserProjection projector;
    pcl::PointCloud<pcl::PointXYZ> cloud;
    try {
        my_tf_listener.waitForTransform("world",
                                      current_laser_scan.header.frame_id,
                                      current_laser_scan.header.stamp,
                                      ros::Duration(0.2));
        sensor_msgs::PointCloud2 transCloud;
        projector.transformLaserScanToPointCloud("world",
                                                 current_laser_scan,
                                                 transCloud,
                                                 my_tf_listener);
        pcl::fromROSMsg(transCloud, cloud);
    } catch (tf::TransformException ex){
        ROS_ERROR("%s", ex.what());
    }
    for (int i = 0; i < cloud.points.size(); ++i) {
        int x = (int)(cloud.points[i].x/hough_resolution);
        int y = (int)(cloud.points[i].y/hough_resolution);
        if(x > 0 && y > 0 &&
           x < laser_img.cols &&
           y < laser_img.rows)
            laser_img.at<uchar>(y, x) = 225;
    }
    std::vector<cv::Vec4i> lines;
    cv::HoughLinesP(laser_img, lines, 1, M_PI/180, 10, 10, 10 );
    // Visualization
    new_walls.points.clear();
    for( size_t i = 0; i < lines.size(); i++ )
    {
        cv::Vec4i l = lines[i];
        if(!isExistingWall(l)) {
            geometry_msgs::Point p;
            p.x = l[0]*hough_resolution;
            p.y = l[1]*hough_resolution;
            new_walls.points.push_back(p);
            p.x = l[2]*hough_resolution;
            p.y = l[3]*hough_resolution;
            new_walls.points.push_back(p);
            // TODO: Add wall to list of walls
            double theta = atan2(hough_resolution*l[3]-hough_resolution*l[1],hough_resolution*l[2]-hough_resolution*l[0]);
            double t_max = std::sqrt(std::pow(hough_resolution*l[1]-hough_resolution*l[3], 2)+std::pow(hough_resolution*l[2]-hough_resolution*l[4], 2));
            walls.push_back((Line){hough_resolution*l[0], hough_resolution*l[1], hough_resolution*l[2], hough_resolution*l[3], theta, t_max, sin(theta), cos(theta)});
        } else {
           geometry_msgs::Point p;
            p.x = l[0]*hough_resolution;
            p.y = l[1]*hough_resolution;
            new_walls_r.points.push_back(p);
            p.x = l[2]*hough_resolution;
            p.y = l[3]*hough_resolution;
            new_walls_r.points.push_back(p);
        }
    }
}

bool newallCallback (std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
    addLinesToMap();
    return true;
}


int mainLoop(){
  // Visualization
  
  new_walls.header.frame_id = "world";
  new_walls.header.stamp = ros::Time::now();
  new_walls.ns = "newalls";
  new_walls.action = visualization_msgs::Marker::ADD;
  new_walls.pose.orientation.w = 1.0;
  new_walls.id = 0;
  new_walls.type = visualization_msgs::Marker::LINE_LIST;
  new_walls.scale.x = 0.05;
  new_walls.color.a = 1.0;
  new_walls.color.b = 1.0;

  new_walls_r.header.frame_id = "world";
  new_walls_r.header.stamp = ros::Time::now();
  new_walls_r.ns = "newalls_r";
  new_walls_r.action = visualization_msgs::Marker::ADD;
  new_walls_r.pose.orientation.w = 1.0;
  new_walls_r.id = 1;
  new_walls_r.type = visualization_msgs::Marker::LINE_LIST;
  new_walls_r.scale.x = 0.05;
  new_walls_r.color.a = 1.0;
  new_walls_r.color.r = 1.0;
  // End of visualization
  double x_est = 0, y_est = 0, z_est = 0;
  ps.header.frame_id = "world";
  ps.pose.orientation.z = 1;


  ros::NodeHandle n("~");
  ros::Publisher pose_pub = n.advertise<geometry_msgs::PoseStamped>("pose", 10);
  ros::Subscriber laser_sub = n.subscribe ("/scan",
                                           1,
                                           &Localiser::laserScanCallback, this);
  ros::Subscriber pose_sub = n.subscribe ("/robot8/pose_diff",
                                          1000,
                                          &Localiser::odometryCallback, this);
  new_walls_pub = n.advertise<visualization_msgs::Marker>("new_walls", 12);
  new_walls_r_pub = n.advertise<visualization_msgs::Marker>("new_walls_r", 12);
  ros::ServiceServer scatter_server = n.advertiseService("scatter_particles", &Localiser::scatterCallback, this);
  ros::ServiceServer newall_server = n.advertiseService("add_walls", &Localiser::newallCallback, this);
  ros::Rate rate(100);
  cloud_pub = n.advertise<pcl::PointCloud<pcl::PointXYZI>>("particles", 100); //Wut
  std::string _map_file = "/home/ras28/contest_maze.txt";
  // Get parameters
  n.getParam("map_frame", map_frame);
  n.getParam("map_file", _map_file);
  n.getParam("wall_thickness", wall_thickness);
  n.getParam("motion_variance_w", motion_variance_w);
  n.getParam("motion_variance_v", motion_variance_v);
  n.getParam("pose_variance_xy", pose_variance_xy);
  n.getParam("pose_variance_z", pose_variance_z);
  n.getParam("laser_variance", laser_variance);
  n.getParam("particle_quantity", particle_quantity);
  n.getParam("tracking", tracking);
  n.getParam("x_init", x_init);
  n.getParam("y_init", y_init);
  n.getParam("z_init", z_init);
  n.getParam("gaussian_premultiplier", gaussian_premultiplier);
  n.getParam("interlaced", interlaced);
  n.getParam("resample", resample);
  n.getParam("addnoise", addnoise);
  n.getParam("laser_min_range", laser_min_range);
  n.getParam("weight_combo_method", weight_combo_method);
  n.getParam("noiseincallback", noiseincallback);
  n.getParam("laser_skip", laser_skip);
  n.getParam("rejected_fraction", rejected_fraction);
  n.getParam("hough_resolution", hough_resolution);

  uniform = std::uniform_real_distribution<double>(0.0, 1/particle_quantity);

  ROS_INFO_STREAM("Loading map from " << _map_file);
  std::ifstream map_fs; map_fs.open(_map_file.c_str());
  if (!map_fs.is_open()){
      ROS_ERROR_STREAM("Could not read maze map from "<<_map_file<<
                       ". Please double check that the file exists. Aborting.");
      return -1;
  }
  std::string line;
  max_x = max_y = 0;
  while (std::getline(map_fs, line)){
      if (line[0] == '#') {
          // comment -> skip
          continue;
      }
      double x1, x2, y1, y2;

      std::istringstream line_stream(line);
      line_stream >> x1 >> y1 >> x2 >> y2;
      double theta = atan2(y2 - y1, x2 - x1);
      double t_max = sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));

      walls.push_back((Line){x1, y1, x2, y2, theta, t_max, sin(theta), cos(theta)});

      max_x = std::max(max_x, x1);
      max_x = std::max(max_x, x2);
      max_y = std::max(max_y, y1);
      max_y = std::max(max_y, y2);
  }


  tf::StampedTransform theTransform;
  int maxAttempts = 15;
  while (maxAttempts-- && ros::ok()) {
      try{
          my_tf_listener.waitForTransform("robot_center",
                                    "laser",
                                    ros::Time(0),
                                    ros::Duration(1));
          my_tf_listener.lookupTransform("robot_center",
                                   "laser",
                                   ros::Time(0),
                                   theTransform);
          ROS_INFO("Found transform. Continuing...");
          break;
      } catch (tf::TransformException e) {
          ROS_ERROR("Could not find transform... :(");
          continue;
      }
  }
  if (maxAttempts == 0 || !ros::ok()) {
      return 1;
  }
  shift = theTransform.getOrigin();
  rotation = theTransform.getRotation();

  particles = std::vector<Particle>(particle_quantity);
  pcl::PointCloud<pcl::PointXYZI> some_cloud;
  particle_cloud = some_cloud.makeShared();
  particle_cloud->width = particle_quantity;
  particle_cloud->height = 1;
  particle_cloud->points.resize(particle_quantity);
  for (int i = 0; i < particle_quantity; ++i) {
      pcl::PointXYZI* aPoint = new pcl::PointXYZI;
      aPoint->intensity = 1;
      particle_cloud->points[i] = *aPoint;
  }
  particle_cloud->header.frame_id = "world";
  if (tracking) {
      geometry_msgs::Twist init;
      init.linear.x = x_init;
      init.linear.y = y_init;
      init.linear.z = 0;
      init.angular.x = 0;
      init.angular.y = 0;
      init.angular.z = z_init;
      for (std::vector<Particle>::iterator i = particles.begin(); i != particles.end(); ++i) {
          i->position = init;
          i->weight = 1 / particle_quantity;
      }
  } else {
      scatterParticles();
  }

  noise_v = std::normal_distribution<double>(0.0, motion_variance_v);
  noise_w = std::normal_distribution<double>(0.0, motion_variance_w);
  noise_xy = std::normal_distribution<double>(0.0, pose_variance_xy);
  noise_z = std::normal_distribution<double>(0.0, pose_variance_z);

  laser_start_index = 0;

  while(ros::ok()) {
      new_walls_pub.publish(new_walls);
      new_walls_pub.publish(new_walls_r);
      if (!laser_used) {
          if(addnoise){
              for (int i = 0; i < particle_quantity; i += 10) {
                double x_shift = noise_xy(generator);
                double y_shift = noise_xy(generator);
                double z_shift = noise_z(generator);
                particles[i].position.angular.z += z_shift;
                particles[i].position.linear.x += x_shift;
                particles[i].position.linear.y += y_shift;
              }
          }

          laser_used = true;
          double totalWeight = 0;
          laser_start_index++;
          for (int i = 0; i < particles.size(); ++i) {
              particleWeight(particles[i], current_laser_scan);
              totalWeight += particles[i].weight;
          }
          std::vector<double> cumulative_weights;
          double cumulative_sum = 0;
          for (int i = 0; i < particles.size(); ++i) {
              particles[i].weight /= totalWeight;
              cumulative_sum += particles[i].weight;
              cumulative_weights.push_back(cumulative_sum);
              particle_cloud->points[i].x = particles[i].position.linear.x;
              particle_cloud->points[i].y = particles[i].position.linear.y;
              particle_cloud->points[i].intensity = particles[i].weight;
              x_est += particles[i].position.linear.x;
              y_est += particles[i].position.linear.y;
              z_est += particles[i].position.angular.z;
          }
          x_est /= particle_quantity;
          y_est /= particle_quantity;
          z_est /= particle_quantity;

          ps.header.stamp = ros::Time::now();
          ps.pose.position.x = x_est;
          ps.pose.position.y = y_est;
          tf::Quaternion q= tf::createQuaternionFromYaw(z_est);
          tf::quaternionTFToMsg(q, ps.pose.orientation);
          pose_pub.publish(ps);

          cloud_pub.publish(particle_cloud);
          if (resample) {
              double resample_index = uniform(generator);
              int j = 0;
              std::vector<Particle> temp(particles.size());
              x_est = y_est = z_est = 0;
              for (int i = 0; i < particle_quantity; ++i)
              {
                  while(cumulative_weights[j] <= resample_index)
                      j++;
                  /*if (j >= particle_quantity)
                      break;*/
                  resample_index += 1 / (double) particle_quantity;
                  temp[i] = particles[j];
                  particle_cloud->points[i].x = temp[i].position.linear.x;
                  particle_cloud->points[i].y = temp[i].position.linear.y;
              }
              particles = temp;
          }
      }
      rate.sleep();
      ros::spinOnce();
  }
}
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "localization");
    Localiser lc;
    return lc.mainLoop();
}
