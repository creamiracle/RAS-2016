#include <ros/ros.h>
#include <cmath>
#include <stdio.h>
#include <std_msgs/String.h>
#include <iostream>
#include <nav_msgs/OccupancyGrid.h>
#include <vector>
#include <nav_msgs/MapMetaData.h>
#include <fstream>
#include <algorithm>
#include <geometry_msgs/PoseStamped.h>
#include <ras_object_list/PopObject.h>

typedef struct objectlists
{
	geometry_msgs::PoseStamped p;
	int weight;
	double score;
	std::string color;

}object;


nav_msgs::OccupancyGrid map;
std::vector<object> object_list;
std::vector<object> weight_list;
std::string object_file = "/home/ras28/catkin_ws/src/ras_color_detection/result/objectlist.txt";



void mapCallBack(const nav_msgs::OccupancyGridConstPtr &msg)
{
	map = *msg;
}

object create_object(double x, double y, int weight, std::string color, double score)
{
	object n;
	n.p.pose.position.x = x;
	n.p.pose.position.y = y;
	n.p.header.frame_id = "/world";
	n.weight = weight;
	n.score = score;
	n.color = color;
	return n;
}


void findNearest(int &x, int &y, std::vector<int> distance_map)
{
	ROS_INFO("find near");
	if(x - 1 < 0 && y - 1 < 0)
	{
		ROS_INFO("enter 1");
	    int k = 0;
	    int i = 0;
	    while(1) {
	        i = 0;
	        while (i <= k) {
	            if(distance_map[(x + i)+map.info.width*(y + k)]!= -1) {
	                x += i;
	                y += k;
	                return;
	            } 	            
	            else if(distance_map[(x + k)+map.info.width*(y + i)]!= -1) {
	                x += k;
	                y += i;
	                return;
	            }
	            i++;
	        }
	        k++;
	    }
	}
	else if(x - 1 < 0 && y + 1 > map.info.width)
	{
		ROS_INFO("enter 2");
		int k = 0;
	    int i = 0;
	    while(1) {
	        i = 0;
	        while (i <= k) {
	            if(distance_map[(x + k)+map.info.width*(y - i)]!= -1) {
	                x += k;
	                y -= i;
	                return;
	            } else if(distance_map[(x + i)+map.info.width*(y - k)]!= -1) {
	                x += i;
	                y -= k;
	                return;
	            } 
	            i++;
	        }
	        k++;
	    }
	}
	else if(x + 1 > map.info.height && y + 1 > map.info.width)
	{
		ROS_INFO("enter 3");
		int k = 0;
	    int i = 0;
	    while(1) {
	        i = 0;
	        while (i <= k) {
	            if(distance_map[(x - i)+map.info.width*(y - k)]!= -1) {
	                x -= i;
	                y -= k;
	                return;
	            } else if(distance_map[(x - k)+map.info.width*(y - i)]!= -1) {
	                x -= k;
	                y -= i;
	                return;
	            }
	            i++;
	        }
	        k++;
	    }
	}
	else if( y - 1 < 0 && x + 1 > map.info.height)
	{
		ROS_INFO("enter 4");
		int k = 0;
	    int i = 0;
	    while(1) {
	        i = 0;
	        while (i <= k) {
	            if(distance_map[(x - i)+map.info.width*(y + k)]!= -1) {
	                x -= i;
	                y += k;
	                return;
	            } 
	            else if(distance_map[(x - k)+map.info.width*(y + i)]!= -1) {
	                x -= k;
	                y += i;
	                return;
	            } 
	            i++;
	        }
	        k++;
	    }
	}
	else if(x - 1 < 0)
	{
		ROS_INFO("enter 5");
		int k = 0;
	    int i = 0;
	    while(1) {
	        i = 0;
	        while (i <= k) {
	            if( (y + i) < map.info.height) {
	            	if(distance_map[(x + i)+map.info.width*(y + k)]!= -1 )
	            	{
	            		x += i;
	                	y += k;
	                	return;
	            	}
	            } 
	            if((y + i) < map.info.height) {
	            	if(distance_map[(x + k)+map.info.width*(y + i)]!= -1 )
	            	{
	            		x += k;
	                	y += i;
	                	return;
	            	}
	            } 
	            if( (y - i) > 0) {
	            	if(distance_map[(x + k)+map.info.width*(y - i)]!= -1 )
	            	{
	            		x += k;
	                	y -= i;
	                	return;
	            	}
	            } 
	            if((y - k) > 0) {
	            	if(distance_map[(x + i)+map.info.width*(y - k)]!= -1 )
	            	{
	            		x += i;
	                	y -= k;
	                	return;
	            	}
	            }
	            i++;
	        }
	        k++;
	    }
	}
	else if(x + 1 > map.info.width)
	{
		ROS_INFO("enter 6");
		int k = 0;
	    int i = 0;
	    while(1) {
	        i = 0;
	        while (i <= k) {
	            if( (y + k) < map.info.height) {
	            	if(distance_map[(x - i)+map.info.width*(y + k)]!= -1 )
	            	{
 						x -= i;
		                y += k;
		                return;
	            	}
	            }
	            if( (y - k) > 0) {
	            	if(distance_map[(x - i)+map.info.width*(y - k)]!= -1 )
	            	{
	            		x -= i;
		                y -= k;
		                return;
	            	}
	            }
	            if( (y + i) < map.info.height) {
	            	if(distance_map[(x - k)+map.info.width*(y + i)]!= -1 )
	            	{
	            		x -= k;
		                y += i;
		                return;
	            	}
	            }
	            if( (y - i) > 0) {
	            	if(distance_map[(x - k)+map.info.width*(y - i)]!= -1 )
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
	else if(y - 1 < 0)
	{
		ROS_INFO("enter 7");
		int k = 0;
	    int i = 0;
	    while(1) {
	        i = 0;
	        while (i <= k) {
	            if((x + i) < map.info.width) 
	            {
	            	if(distance_map[(x + i)+map.info.width*(y + k)]!= -1  )
	               {
		               	x += i;
		                y += k;
		                return;
	               }
	            } 
	            if( (x - i) > 0) {
	            	if(distance_map[(x - i)+map.info.width*(y + k)]!= -1)
	            	{
	            		x -= i;
	                	y += k;
	                	return;
	            	}
	                
	            } 
	            if ((x + k) < map.info.width) {
					if(distance_map[(x + k)+map.info.width*(y + i)]!= -1)
					{
						x += k;
	               		y += i;
	                	return;
					}
	               
	            } 
	            if( (x - k) > 0) {
	            	if(distance_map[(x - k)+map.info.width*(y + i)]!= -1)
	            	{
	            		x -= k;
	                	y += i;
	                	return;
	            	}
	                
	            } 
	            i++;
	        }
	        k++;
	    }
	}
	else if(y + 1 > map.info.height)
	{	ROS_INFO("enter 8");
		int k = 0;
	    int i = 0;
	    while(1) {
	        i = 0;
	        while (i <= k) {
	            if( (x + k) < map.info.width) 
	            {
	            	if(distance_map[(x + k)+map.info.width*(y - i)]!= -1 )
	            	{
	            		x += k;
		                y -= i;
		                return;
	            	}   
	            }
	            if( (x + i) < map.info.width) {
	            	if(distance_map[(x + i)+map.info.width*(y - k)]!= -1 )
	            	{
	            		x += i;
		                y -= k;
		                return;
	            	}
	            }
	            if( (x - i) > 0) {
	            	if(distance_map[(x - i)+map.info.width*(y - k)]!= -1 )
	            	{
						x -= i;
	               		y -= k;
	                	return;
	            	}
	            }
	            if( (x - k) > 0) {
	            	if(distance_map[(x - k)+map.info.width*(y - i)]!= -1 )
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
	else
	{
		ROS_INFO("enter 9");
	    int k = 0;
	    int i = 0;
	    while(1) {
	        i = 0;
	        while (i <= k) {
	        	if((x + i) < map.info.width && (y + k) < map.info.height)
	        	{
	        		if(distance_map[(x + i)+map.info.width*(y + k)]!= -1) 
	        		{
		                x += i;
		                y += k;
		                return;
	            	}
	        	}
	            if((x - i) > 0 && (y + k) < map.info.height)
	        	{
	              	if(distance_map[(x - i)+map.info.width*(y + k)]!= -1) 
	              	{
		                x -= i;
		                y += k;
		                return;
	            	} 
	        	}
	        	if((x + k) < map.info.width && (y + i) < map.info.height)
	        	{
	        		if(distance_map[(x + k)+map.info.width*(y + i)]!= -1) 
	        		{
		                x += k;
		                y += i;
		                return;
	            	}
	        	}
	        	if((x + k) < map.info.width && (y - i) > 0)
	        	{
	        		if(distance_map[(x + k)+map.info.width*(y - i)]!= -1) 
	        		{
		                x += k;
		                y -= i;
		                return;
		            }
	        	}
	        	if((x + i) < map.info.width && (y - k) > 0)
	        	{
	        		if(distance_map[(x + i)+map.info.width*(y - k)]!= -1) 
	        		{
		                x += i;
		                y -= k;
		                return;
		            }
	        	}
	        	if((x - i) > 0 && (y - k) > 0)
	        	{
	        		if(distance_map[(x - i)+map.info.width*(y - k)]!= -1) {
		                x -= i;
		                y -= k;
		                return;
		            }
	        	}
	        	if((x - k) > 0 && (y + i) < map.info.height)
	        	{
					if(distance_map[(x - k)+map.info.width*(y + i)]!= -1) 
	        		{
		                x -= k;
		                y += i;
		                return;
		            }
	        	}
	        	if((x - k) > 0 && (y - i) > 0)
	        	{
	        		if(distance_map[(x - k)+map.info.width*(y - i)]!= -1) 
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
}

void calObjectScore(object &output) 
{
	ROS_INFO("cal score");
	int x_init = 0.3/map.info.resolution;
	int y_init = 0.3/map.info.resolution;

	int cox = 0;
	int coy = 0;

	cox = (int) (output.p.pose.position.x/map.info.resolution);
	coy = (int) (output.p.pose.position.y/map.info.resolution);

    bool done = false;
    std::vector<int> distance_map;
    distance_map.resize(map.data.size());

    for (int i = 0; i < map.data.size(); ++i) {
        if(map.data[i] == 0)
            distance_map[i] = 0;
        else
            distance_map[i] = -1;
    }

    distance_map[y_init*map.info.width + x_init] = 1;

    int nrOfZeros = 0;
    int nrOfZerosLastTime = -1;

    while(!done) {
        nrOfZerosLastTime = nrOfZeros;
        nrOfZeros = 0;
        //ROS_INFO("%d zeros remaining", nrOfZerosLastTime);
        done = true;
        for (int y = 0; y < map.info.height; ++y) {
            for (int x = 0; x < map.info.width; ++x) {
                if(distance_map[y*map.info.width + x] == 0) {
                    nrOfZeros++;
                    done = false;
                }
                else {
                    for (int j = std::max(y-1,0); j < std::min(y+2,(int)map.info.height); ++j) {
                        for (int i = std::max(x-1,0); i < std::min(x+2,(int)map.info.width); ++i) {
                            if (distance_map[j*map.info.width + i] == 0){
                                distance_map[j*map.info.width + i] = distance_map[y*map.info.width + x] + 1;

                            }
                        }
                    }
                }
            }
        }
    }
    if(distance_map[coy * map.info.width + cox] == -1 || distance_map[coy * map.info.width + cox] == 0)
    {
    	ROS_INFO_STREAM("current x y "<<cox<<" "<<coy <<" dist " <<distance_map[coy * map.info.width + cox]);
    	findNearest(cox, coy, distance_map);
    	ROS_INFO_STREAM("after x y "<<cox <<" "<<coy<<" dist " <<distance_map[coy * map.info.width + cox]);

    }
    	

	output.score = ((double)output.weight) / distance_map[coy * map.info.width + cox] ;	

}

bool compareByDistance (const object &a, const object &b) 
{
	return a.score > b.score;
}

int checkWeight(std::string color)
{
	for(int i=0;i<weight_list.size();i++)
		if(weight_list[i].color == color)
			return weight_list[i].weight;
}

double checkDist(double x1,double y1,double x2,double y2)
{
	double distance = 0.0;
	distance = std::sqrt(std::pow((x1 - x2),2) + std::pow((y1 - y2),2));
	return distance;
}	

int weightList(std::string color)
{
	for(int i = 0; i < weight_list.size();i++)
	{
		if(weight_list[i].color == color)
			return weight_list[i].weight;
	}
	return 0;
}

void initWeightList()
{
	if(weight_list.size())
		weight_list.clear();

//cube weight 100
	object blue_cube = create_object(0.0,0.0,100,"blue_cube" ,0);
	weight_list.push_back(blue_cube);
	object red_cube = create_object(0.0,0.0,100,"red_cube" ,0);
	weight_list.push_back(red_cube);
	object green_cube = create_object(0.0,0.0,100,"green_cube" ,0);
	weight_list.push_back(green_cube);
	object yellow_cube = create_object(0.0,0.0,100,"yellow_cube" ,0);
	weight_list.push_back(yellow_cube);

//cross star 1000
	object purple_star = create_object(0.0,0.0,1000,"purple_star" ,0);
	weight_list.push_back(purple_star);
	object orange_star = create_object(0.0,0.0,1000,"orange_star" ,0);
	weight_list.push_back(orange_star);
	object purple_cross = create_object(0.0,0.0,1000,"purple_cross" ,0);
	weight_list.push_back(purple_cross);

//hollow 5000
	object red_hollow = create_object(0.0,0.0,5000,"red_hollow" ,0);
	weight_list.push_back(red_hollow);

//triangle 100
	object blue_triangle = create_object(0.0,0.0,100,"blue_triangle" ,0);
	weight_list.push_back(blue_triangle);
	object green_cylinder = create_object(0.0,0.0,100,"green_cylinder" ,0);
	weight_list.push_back(green_cylinder);

//sphere 10000
	object red_ball = create_object(0.0,0.0,10000,"red_ball" ,0);
	weight_list.push_back(red_ball);
	object yellow_ball = create_object(0.0,0.0,10000,"yellow_ball" ,0);
	weight_list.push_back(yellow_ball);

}

void fillData()
{
	double threshold = 0.1;
	int index = 0;

	if(object_list.size() != 0)
		object_list.clear();

	initWeightList();

	ROS_INFO_STREAM("Loading data from " << object_file);
	std::ifstream map_fs; 
	map_fs.open(object_file.c_str());
	ROS_INFO_STREAM("finish open");
	if (!map_fs.is_open()){
		ROS_ERROR_STREAM("Could not read data from "<<object_file<<
			". Please double check that the file exists. Aborting.");
		return;
	}
	//ROS_INFO_STREAM("here");
	std::string line;

	while (std::getline(map_fs, line))
	{
		if (line[0] == '#')
		{
			continue;
		}
		std::string color;
		double x, y, z;
		std::istringstream line_stream(line);
		line_stream >> color >> x >> y >> z;	
		int weight = weightList(color);
		if(x < 0 || y < 0  )
			continue;
		if(object_list.size() == 0)
		{
			object o = create_object(x,y,weight,color,0);
			//ROS_INFO_STREAM("start score");
			calObjectScore(o);
			//ROS_INFO_STREAM("finish score");
			object_list.push_back(o);
			index += 1;
		}

		object o = create_object(x,y,weight,color,0);
		calObjectScore(o);

		if(o.color != object_list[index - 1].color || checkDist(o.p.pose.position.x, o.p.pose.position.y, object_list[index - 1].p.pose.position.x, object_list[index - 1].p.pose.position.y) > threshold )
		{
			object_list.push_back(o);
			index += 1;
		}		
	}

	std::sort(object_list.begin(), object_list.end(), compareByDistance);

	for (int i = 0; i < object_list.size(); i++)
	{
		if(object_list[i].score < 0)
		{
			std::vector<object>::iterator it = object_list.begin() + i;
			object_list.erase(it);
			i -= 1;
		}
	}
	for (int i = 0; i < object_list.size(); ++i)
	{
		ROS_INFO_STREAM("score is "<< object_list[i].score << " color " << object_list[i].color);
	}

}

bool popObjectCallback(ras_object_list::PopObject::Request &req, ras_object_list::PopObject::Response &res) 
{
	if(object_list.size() == 0)
		return false;
	
    res.object_position = (*object_list.begin()).p;
    ROS_INFO_STREAM("get pose "<< res.object_position.pose.position.x << " " << res.object_position.pose.position.y << " size " <<object_list.size());
    object_list.erase(object_list.begin());
    return true;
}

int main(int argc, char *argv[])
{
	ros::init(argc,argv,"object_list");	

	ros::NodeHandle n;
	ros::Subscriber map_sub = n.subscribe("/mapping/map",1000, mapCallBack);
	ros::ServiceServer pop_server = n.advertiseService("pop_object", popObjectCallback);

	ros::Rate loop_rate(10);


	while(ros::ok() && map.data.size() == 0) 
	{
			ros::spinOnce();	
	}
	fillData();
	ros::spin();


	return 0;
}


