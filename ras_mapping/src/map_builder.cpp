#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <ras_mapping/line.h>
#include <ras_mapping/point.h>
#include <laser_geometry/laser_geometry.h>
#include <list>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PointStamped.h>
#include <std_srvs/Empty.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/impl/transforms.hpp>
#include <Eigen/Geometry>
#include <pcl/common/transforms.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <math.h>
#include <cmath>
//#include <tf.h>

#define LASER_GARBAGE_THRESHOLD 0.24

struct Line
{
  double x1;
  double y1;
  double x2;
  double y2;
  double theta;
  double t_max;
};

typedef struct node_struct{
  int coords[2];
}node;

tf::Vector3 calculatePose(float angle, float r) {

  tf::Vector3 vec;
  vec.setX(r*cos(angle));
  vec.setY(r*sin(angle));
  vec.setZ(0);
  return vec;
}

class MapBuilder
{
public:
  ros::NodeHandle nodeHandler;
  ros::Subscriber scanSub;
  ros::Subscriber odometry_subscriber;
  ros::Subscriber battery_subscriber;
  ros::Publisher cloudPub;
  ros::Publisher mapPub;
  ros::ServiceServer service;
  ros::ServiceServer addBatteryService;
  ros::ServiceServer addPointService;
  tf::TransformListener my_tf_listener;
  laser_geometry::LaserProjection projector;
  std::vector<pcl::PointCloud<pcl::PointXYZ> > scans;
  std::vector<geometry_msgs::PointStamped> updatePoint;
  std::vector<geometry_msgs::PointStamped> updatePoints;
  ros::ServiceServer updateService;
  tf::StampedTransform transform;

  std::vector<int> updateCoords;

  nav_msgs::OccupancyGrid grid;
  nav_msgs::OccupancyGrid map;

  int activeScans;
  int scansToSkip;
  int currentIndex;
  int nrOfScans;
  int mMaxCellValue;
  double map_resolution;
  double grid_resolution;
  double robot_width;

  std::string mapFrame;
  std::string map_file;

  MapBuilder() 
  {
    nodeHandler = ros::NodeHandle("~");
    nodeHandler.getParam("map_frame", mapFrame);
    nodeHandler.getParam("map_file",map_file);
    mMaxCellValue = 100;
    nrOfScans = 10;
    nodeHandler.getParam("map_resolution", map_resolution);
    nodeHandler.getParam("grid_resolution", grid_resolution);
    nodeHandler.getParam("max_cell_value", mMaxCellValue);
    nodeHandler.getParam("robot_width", robot_width);
    nodeHandler.getParam("scans_to_save", nrOfScans);
    
    scans.resize(nrOfScans);

    currentIndex = 0;

    grid.info.origin.position.x = 0;
    grid.info.origin.position.y = 0;
    grid.info.origin.position.z = 0;
    grid.info.resolution = grid_resolution;
    /*grid.info.width = 100;
    grid.info.height = 100;*/

    grid.header.frame_id = "live_map";
    grid.header.stamp = ros::Time(0);

    map.info.origin.position.x = 0;
    map.info.origin.position.y = 0;
    map.info.origin.position.z = 0;
    /*map.info.resolution = 0.05;
    map.info.width = 100;
    map.info.height = 100;*/
    map.info.resolution = map_resolution;

    map.header.frame_id = mapFrame;
    map.header.stamp = ros::Time(0);

    //LEt's fill the map with 0;
    grid.data.resize(grid.info.width * grid.info.height, 0);
    fillMapFromFile();

    service = nodeHandler.advertiseService("/map/cleanMap", &MapBuilder::cleanMap, this);
    updateService = nodeHandler.advertiseService("/map/updateMap",&MapBuilder::updateMap, this);
    addBatteryService = nodeHandler.advertiseService("/map/addBattery",&MapBuilder::addBattery, this);
    addPointService = nodeHandler.advertiseService("/map/addPoint",&MapBuilder::addPoint, this);
    scanSub = nodeHandler.subscribe("/scan", 1, &MapBuilder::scanCallback, this);
    cloudPub = nodeHandler.advertise<nav_msgs::OccupancyGrid>("/mapping/grid",10);
    mapPub = nodeHandler.advertise<nav_msgs::OccupancyGrid>("/mapping/map",10);
    odometry_subscriber = nodeHandler.subscribe("/robot8/pose_diff",1000, &MapBuilder::diffCallback, this);
    battery_subscriber = nodeHandler.subscribe("/camera/obstacle",1, &MapBuilder::batteryCallback, this);


  }
  
  void batteryCallback(const  pcl::PointCloud<pcl::PointXYZ>::ConstPtr &objPointCloud)
  {
    //tf::StampedTransform transform;
      //ROS_INFO_STREAM("StampedTransform" << objPointCloud->header.stamp);    
      try{
         pcl::PointCloud<pcl::PointXYZ> cloud;
        // ROS_INFO_STREAM("stamp1" << objPointCloud->header.stamp);
         my_tf_listener.waitForTransform("/robot_center",objPointCloud->header.frame_id,ros::Time(objPointCloud->header.stamp),ros::Duration(0.2));
         pcl_ros::transformPointCloud("/robot_center", *objPointCloud,cloud, my_tf_listener);
         addCloudToList(cloud);
      }
    catch (tf::TransformException e) {
      //ROS_INFO_STREAM("stamp" << objPointCloud->header.stamp);
      ROS_ERROR("%s",e.what());
      ROS_ERROR("Could not find transform... :(");
      return;
    }
  }

  bool cleanMap(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
  {
    this->grid.data.clear();
    this->map.data.clear();
    fillMapFromFile();
    return true;
  }


  void diffCallback(const geometry_msgs::TwistConstPtr &msg)
  {
    float distance = msg->linear.x;
    float ang = msg->angular.z;
    float x = distance*std::cos(ang);
    float y = distance*std::sin(ang);
    float z = 0;
    Eigen::Vector3f offSet(-x,-y,-z);
    Eigen::Quaternion<float> q;
    q = Eigen::AngleAxisf(-ang, Eigen::Vector3f(0,0,1));
    for (int i = 0; i < scans.size(); ++i) {
      pcl::PointCloud<pcl::PointXYZ> newCloud;
      pcl::transformPointCloud(scans[i],newCloud,offSet, q);
      scans[i] = newCloud;
    }
  }

  void addCloudToList(pcl::PointCloud<pcl::PointXYZ> cloud)
  {
    if(currentIndex == nrOfScans){
      currentIndex = 0;
    }
    scans[currentIndex] = cloud;
    currentIndex++;
  }

  void scanCallback(const sensor_msgs::LaserScanConstPtr &msg)
  {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    sensor_msgs::LaserScan lstemp = *msg;
    for (int i = 200; i < 300; ++i) {
      if (lstemp.ranges[i] < LASER_GARBAGE_THRESHOLD)
        lstemp.ranges[i] = std::numeric_limits<double>::infinity();
    }

    try {
      my_tf_listener.waitForTransform("robot_center",
                                      msg->header.frame_id,
                                      msg->header.stamp,
                                      ros::Duration(0.2));
      sensor_msgs::PointCloud2 transCloud;
      projector.transformLaserScanToPointCloud("robot_center",lstemp,transCloud, my_tf_listener);
      pcl::fromROSMsg(transCloud, cloud);

    } catch (tf::TransformException ex){
      ROS_ERROR("In scanCallBack!");
      ROS_ERROR("%s", ex.what());
    }
    
    addCloudToList(cloud);
  }



   bool addBattery(ras_mapping::line::Request &req, ras_mapping::line::Response &rsp)
   {
    if(req.start.x == req.stop.x && req.start.y == req.stop.y)
      return false;

    int startx = (int) (req.start.x / map.info.resolution);
    int starty = (int) (req.start.y / map.info.resolution);
    int stopx = (int) (req.stop.x / map.info.resolution);
    int stopy = (int) (req.stop.y / map.info.resolution);
    node start = create_node(startx, starty);
    node stop = create_node(stopx, stopy);
    std::vector<node> list;
    list = drawLine(start,stop);
    for(int i = 0; i < list.size(); i++)
    {
      paintCircle(list[i].coords[0],list[i].coords[1],(int) (robot_width/(3*map.info.resolution)));
    }
    return true;
  }

  bool addPoint(ras_mapping::point::Request &req, ras_mapping::point::Response &rsp)
  {
    paintCircle(req.point.x/map.info.resolution, req.point.y/map.info.resolution,(int) (robot_width/(6*map.info.resolution)));
    return true;
  }

  node create_node(int a,int b)
  {
    node n;
    n.coords[0]=a;
    n.coords[1]=b;

    return n;
  }

  int compare(int x, int y)
  {
    if (x > y)
      return x;
    else
      return y;
  }

  std::vector<node> drawLine(node node1, node node2)
  {
    std::vector<node> list;
    list.push_back(node1);

    double slope = ((double)node2.coords[1] - (double)node1.coords[1]) / ((double)node2.coords[0] - (double)node1.coords[0]);
    double b = (double)node1.coords[1] - ((double)node1.coords[0] * slope);

    double y = 0.0;
    double x = 0.0;
    double dx1 = 0.0;
    double dx2 = 0.0;
    double dy1 = 0.0;
    double dy2 = 0.0;

    int bigX = 0;
    int littleX = 0;
    int bigY = 0;
    int littleY = 0;

    if(node1.coords[0] == node2.coords[0])
    {
      int biggerY = compare(node1.coords[1] , node2.coords[1]);
      int smallerY = 0;
      if(biggerY == node1.coords[1])
        smallerY = node2.coords[1];
      else
        smallerY = node1.coords[1];

      for (int i = smallerY ; i < biggerY; ++i)
      {
        node n = create_node(node1.coords[0],i);
        list.push_back(n);
      }
      node n = create_node(node1.coords[0],biggerY);
      list.push_back(n);
      return list;
    }

    if(node1.coords[1] == node2.coords[1])
    {
      int biggerX = compare(node1.coords[0] , node2.coords[0]);
      int smallerX = 0;
      if(biggerX == node1.coords[0])
        smallerX = node2.coords[0];
      else
        smallerX = node1.coords[1];

      for (int i = smallerX ; i < biggerX; ++i)
      {
        node n = create_node(node1.coords[0],i);
        list.push_back(n);
      }
      node n = create_node(biggerX,node1.coords[1]);
      list.push_back(n);
      return list;
    }

    bigX = compare(node1.coords[0],node2.coords[0]);

    if(bigX == node1.coords[0]){
      littleY = node2.coords[1];      
      littleX = node2.coords[0];
      bigY = node1.coords[1];
    }
    else{     
      littleY = node1.coords[1];
      littleX = node1.coords[0];
      bigY = node2.coords[1];
    }

    if(slope < 0 && slope > -1)
    {
      for (int i = littleX + 1; i <= bigX; i++ )
      {
        if(i == bigX)
        {
          list.push_back(node2);
          break;
        }
        y = slope * i + b;
        dy1 = std::abs(y - littleY);
        dy2 = std::abs(y - (littleY - 1));

        if(dy1 < dy2)
        {
          node n = create_node(i,littleY);
          list.push_back(n);
        }
        else
        {
          node n = create_node(i,littleY);
          list.push_back(n);
          littleY = littleY - 1;
        }
      }
      return list;
    }

    if(slope < -1)
    { 
      for (int i = littleY; i >= bigY; i--)
      {
        if(i == bigY)
        {
          list.push_back(node2);
          break;
        }

        x = (i - 1 - b) / slope;
        dx1 = std::abs(x - littleX);
        dx2 = std::abs(x - (littleX + 1));

        if(dx1 < dx2)
        {
          node n = create_node(littleX, i);
          list.push_back(n);
        }
        else
        {
          node n = create_node(littleX + 1, i);
          list.push_back(n);
          littleX = littleX + 1;
        }
      }
      return list;
    }

    if(slope >= 1)
    {   
      for (int i = littleY; i <= bigY; ++i)
      {
        if(i == bigY)
        {
          list.push_back(node2);
          break;
        }
        x = (i + 1 - b) / slope;
        dx1 = std::abs(x - littleX);
        dx2 = std::abs(x - (littleX + 1));

        if(dx1 < dx2)
        {
          node n = create_node(littleX, i);
          list.push_back(n);
        }
        else
        {
          node n = create_node(littleX + 1, i);
          list.push_back(n);
          littleX = littleX + 1;
        }
      }
      return list;
    }

    if(slope < 1 && slope > 0)
    {
      for (int i = littleX ; i <= bigX; i++ )
      {
        if(i == bigX)
        {
          list.push_back(node2);
          break;
        }

        y = slope * (i + 1) + b;
        dy1 = std::abs(y - littleY);
        dy2 = std::abs(y - (littleY + 1));

        if(dy1 < dy2)
        {
          node n = create_node(i, littleY);
          list.push_back(n);
        }
        else
        {
          node n = create_node(i, littleY + 1);
          list.push_back(n);
          littleY = littleY + 1;
        }
      }
      return list;
    }
    return list;
  }



  void putCloudIntoGrid(pcl::PointCloud<pcl::PointXYZ> &cloud) 
  {
    for(pcl::PointCloud<pcl::PointXYZ>::iterator point = cloud.begin(); point != cloud.end(); point++)
    {
      int XCoord = (int) ((point->x + grid.info.resolution*grid.info.width/2 )* (1/grid.info.resolution));
      int YCoord = (int) ((point->y + grid.info.resolution*grid.info.height/2 )* (1/grid.info.resolution));
      if(XCoord >0 && YCoord >0 && YCoord < grid.info.height && XCoord < grid.info.width ) 
      {
        if(grid.data[YCoord*grid.info.width + XCoord]<mMaxCellValue)
        { //We increase up to 10;
          grid.data[YCoord*grid.info.width + XCoord] += 1;
        }
      }
    }
  }

  bool updateMap(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
  {
    tf::StampedTransform toWorldTransform;
    try{
      my_tf_listener.waitForTransform("/world","/robot_center",ros::Time(0),ros::Duration(1));
      my_tf_listener.lookupTransform("/world","/robot_center", ros::Time(0), toWorldTransform);
    }
    catch (tf::TransformException e) {
      ROS_ERROR("%s",e.what());
      ROS_ERROR("Could not find transform... :(");
      return false;
    }
    for(int i = 0;i<scans.size();i++)
    {
      for(pcl::PointCloud<pcl::PointXYZ>::iterator point = scans[i].begin(); point != scans[i].end(); point++)
      {
        float x = (point->x);
        float y = (point->y );
        int XCoord = (int) (x* (1/grid.info.resolution) + grid.info.width/2);
        int YCoord = (int) (y* (1/grid.info.resolution) + grid.info.height/2);
        if ((XCoord >0 && YCoord >0 && YCoord < grid.info.height && XCoord < grid.info.width) 
             && grid.data[YCoord * grid.info.width + XCoord] > 2){
          tf::Vector3 pointInWorld = toWorldTransform(tf::Vector3(x,y,0));
          int yCoordInWorld = (int) (pointInWorld.getY()/map.info.resolution);
          int xCoordInWorld = (int) (pointInWorld.getX()/map.info.resolution);

          //ROS_INFO_STREAM("X = " << xCoordInWorld << ", Y = " << yCoordInWorld);
          paintCircle(xCoordInWorld, yCoordInWorld, robot_width/(3*map.info.resolution));
        }
      }
    }
    return true;
  }


  void broadcastMapTF()
  {
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3( -grid.info.resolution*grid.info.width/2 , -grid.info.resolution*grid.info.height/2 ,0.0));
    tf::Quaternion q;
    q.setRPY( 0, 0, 0);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "robot_center", "live_map"));
  }

  void broadcastMap()
  {
    grid.header.stamp = ros::Time::now();
    map.header.stamp = ros::Time::now();
    grid.data.clear();
    grid.data.resize(grid.info.width * grid.info.height, 0);
    for (int i = 0; i < scans.size(); ++i)
    {
      putCloudIntoGrid(scans[i]);
    }
    cloudPub.publish(grid);
    mapPub.publish(map);
  }

  void fillMapFromFile() 
  {
    std::vector<Line> walls;
    ROS_INFO_STREAM("Loading map from " << map_file);
    std::ifstream map_fs; map_fs.open(map_file.c_str());
    if (!map_fs.is_open())
    {
      ROS_ERROR_STREAM("Could not read maze map from "<<map_file<<
                       ". Please double check that the file exists. Aborting.");
      return;
    }
    std::string line;
    double max_x = 0, max_y = 0;
    while (std::getline(map_fs, line)){
      if (line[0] == '#') 
      {
            // comment -> skip
        continue;
      }
      double x1, x2, y1, y2;

      std::istringstream line_stream(line);
      line_stream >> x1 >> y1 >> x2 >> y2;
      double theta = atan2(y2 - y1, x2 - x1);
      double t_max = sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));

      walls.push_back((Line){x1, y1, x2, y2, theta, t_max});

      max_x = std::max(max_x, x1);
      max_x = std::max(max_x, x2);
      max_y = std::max(max_y, y1);
      max_y = std::max(max_y, y2);
    }
    map.info.width = round(max_x / map_resolution);
    map.info.height = round(max_y / map_resolution);
    map.data.resize(map.info.width * map.info.height, 0);
    grid.info.width = round(max_x / grid_resolution)+10;
    grid.info.height = round(max_y / grid_resolution)+10;
    grid.data.resize(grid.info.width * grid.info.height, 0);

    double resulution = map.info.resolution;
    int i = 1;
    int maxNr = walls.size();
    for (std::vector<Line>::iterator tLine = walls.begin(); tLine != walls.end(); ++tLine) 
    {
      ROS_INFO("On line nr %d of %d", i++, maxNr);
      double x = tLine->x1;
      double y = tLine->y1;
      while(((x<= tLine->x1 || x <= tLine-> x2) && (x >= tLine->x1 || x >= tLine-> x2)) && ((y>= tLine->y1 || y >= tLine-> y2) && (y<= tLine->y1 || y <= tLine-> y2))){
        int xCoord = (int) (floor(x/resulution));
        int yCoord = (int) (floor(y/resulution));
        if(yCoord * map.info.width + xCoord < map.data.size()){
          map.data[yCoord * map.info.width + xCoord] = 100;
        }
        x += cos(tLine->theta)*map_resolution/2;
        y += sin(tLine->theta)*map_resolution/2;
      }
    }
    for (int i = 0; i < map.info.width; ++i) 
    {
      map.data[(map.info.height-1)*map.info.width + i] = 100;
      map.data[i*map.info.width] = 100;
    }
    nav_msgs::OccupancyGrid temp_map = map;
    for (int y = 0; y < map.info.height; ++y) 
    {
      for (int x = 0; x < map.info.width; ++x) 
      {
        if (map.data[y * map.info.width + x] > 50) 
        {
          // Fill the circle
          for (int i = 0; i < map.info.height; ++i) 
          {
            for (int j = 0; j < map.info.width; ++j) 
            {
              double dist = sqrt(std::pow((double)(y-i)*map_resolution, 2) 
                                 + std::pow((double)(x-j)*map_resolution, 2));
              if (dist < (robot_width/2))
                temp_map.data[i * map.info.width + j] = 100;
            }
          }
        }
      }
    }
    map = temp_map;
  }

  void paintCircle(int xp, int yp, int brush_radius) 
  {
      for (int y = std::max(yp-brush_radius,0); y < std::min(yp+brush_radius+1,(int)map.info.height); ++y) 
      {
          for (int x = std::max(xp-brush_radius,0); x < std::min(xp+brush_radius+1,(int)map.info.width); ++x) 
          {
              double dist = std::sqrt(std::pow(y-yp,2) + std::pow(x-xp,2));
              if (dist < brush_radius && (xp <  map.info.width && yp < map.info.height) && xp > 0 && yp > 0)
                  map.data[y*map.info.width + x] = 100;
          }
      }
  }
};




int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "map_builder");
  MapBuilder mB;
  ros::Rate rate(10); //Hz

  while(mB.nodeHandler.ok()){
    ros::spinOnce();
    mB.broadcastMap();
    mB.broadcastMapTF();
    rate.sleep();
  }

}
