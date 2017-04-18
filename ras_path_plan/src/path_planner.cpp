#include <ros/ros.h>
#include <std_msgs/String.h>
#include <stdio.h>
#include <cmath>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>
#include <vector>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PointStamped.h>
#include <ras_path_plan/sets.h>
#include <ras_path_plan/path_srv.h>
#include <nav_msgs/Path.h>

int map_width = 0;

//ros::Publisher path_pub;
nav_msgs::Path pos_list;
nav_msgs::OccupancyGrid map;
std::vector<signed char> Occupancy_Grid;

node create_node(int coords[2], double t_f, double t_g, int came_from[2])
{
	node n;
	n.coords[0]=coords[0];
	n.coords[1]=coords[1];
	n.t_f=t_f;
	n.t_g=t_g;
	n.came_from[0]=NO_VAL;
	n.came_from[1]=NO_VAL;

	return n;
}

//heurestic function for a*
double heur(int coords[2], int goal_coords[2])
{
	double t_h=0;

	if(goal_coords[0]!=NO_VAL)
	{
		t_h = std::abs(coords[0]-goal_coords[0])+std::abs(coords[1]-goal_coords[1]);
		return 4.0*t_h;
	}
	else
	{ // no goal, no heuristic
		return 0.0;
	}
}

//cost function for a*
double g_cost(int coords[2], int prev_coords[2], int prev_from_coords[2]){

	if(prev_coords[0]-prev_from_coords[0]!=0)
	{ // movement along x

		if(coords[0]-prev_coords[0]!=0)
		{ // keep the same direction of movement
			return 1;
		}
		else
		{ // implies rotation

			return 500;
		}
	}
	else
	{ // movement along y

		if(coords[1]-prev_coords[1]!=0){

			return 1;
		}
		else
		{
			return 500;
		}
	}
	return 1; // just in case
}

std::vector<node> retrieve_path(node goal, search_set * closed)
{
	node current = goal;
	std::vector<node> return_list(1,goal);

	while(current.came_from[0]!=NO_VAL)
	{
		current=(*closed).pop_requested(current.came_from);
		return_list.push_back(current);
	}
	ROS_INFO("end of finding");
	return return_list;
}

std::vector<node> findPath(node start, node goal, std::vector<int8_t> map)
{	


	int coords[2],from[2],cost = 0;
	int lateral_size = map_width;
	node current, n;
	std::vector<node> return_error;
	double t_g = 0, t_f = 0, t_h = 0;
	
	search_set closedset;
	search_set openset(current);
	search_set came_from;
	search_set nodes_set(current);

	coords[0]=start.coords[0];
	coords[1]=start.coords[1];
	from[0]=NO_VAL;
	from[1]=NO_VAL;

	t_g=0;
	t_h=heur(coords,goal.coords);
	//ROS_INFO("doing heur function with  %d %d ", coords[0], coords[1]);
	t_f = t_g + t_h;

	current=create_node(coords,t_f,t_g,from);
	//ROS_INFO("current coords %d %d",current.coords[0],current.coords[1]);
	while(!openset.isempty())
	{
		
		//ROS_INFO("CURRENT COORDS %d %d", current.coords[0],current.coords[1]);
		//check if find the goal
		if(current.coords[0] == goal.coords[0] && current.coords[1] == goal.coords[1])
		{
			//ROS_INFO("Found solution at %d %d!",current.coords[0],current.coords[1]);
			return retrieve_path(current, &closedset);

		}
		closedset.push_node(current);

		//ROS_INFO("start check neighbor");
		//check the neighbor
		//ROS_INFO("now the coords is %d %d", current.coords[0],current.coords[1]);
		if(current.coords[0] - 1 >= 0)
		{
			//ROS_INFO("enter 1");
			coords[0]=current.coords[0]-1;
			coords[1]=current.coords[1];
			//ROS_INFO("check the point  %d %d! ",coords[0],coords[1]);
			
			if( map[map_width * coords[1] + coords[0]] < 70 )
			{	
				//ROS_INFO("obstacle??? %d",map[map_width * coords[1] + coords[0]]);
				t_h = heur(coords,goal.coords);
				t_g = current.t_g + g_cost(coords,current.coords,current.came_from);
				t_f = t_g + t_h;

				if(openset.check_if_in_set(coords))
				{
					n = openset.read_node(coords); // only doing this to get the f value, if present				
				}
				else
				{
					n = create_node(coords,t_f,t_g,current.coords);
				}
				if(!closedset.check_if_in_set(n.coords) || t_f < n.t_f )
				{
					n.t_g=t_g;
					n.t_f=t_f;
					n.came_from[0]=current.coords[0];
					n.came_from[1]=current.coords[1];
					if(!openset.check_if_in_set(n.coords))
						openset.push_node(n);
					else
					{ // update the value of the node in openset
						openset.pop_requested(n.coords);
						openset.push_node(n);
					}
				}
			}
		}

		if(current.coords[0] + 1 <= lateral_size)
		{
			//ROS_INFO("enter 2");
			coords[0]=current.coords[0]+1;
			coords[1]=current.coords[1];
			//ROS_INFO("check the point  %d %d! ",coords[0],coords[1]);
			if( map[map_width * coords[1] + coords[0]] < 70 )
			{
				//ROS_INFO("obstacle??? %d",map[map_width * coords[1] + coords[0]]);
				t_h=heur(coords,goal.coords);
				t_g = current.t_g + g_cost(coords,current.coords,current.came_from);
				t_f = t_g + t_h;

				if(openset.check_if_in_set(coords))
				{
					n = openset.read_node(coords); 
				}
				else
				{
					n = create_node(coords,t_f,t_g,current.coords);
				}

				if(!closedset.check_if_in_set(n.coords) || t_f < n.t_f )
				{
					n.t_g=t_g;
					n.t_f=t_f;
					n.came_from[0]=current.coords[0];
					n.came_from[1]=current.coords[1];
					if(!openset.check_if_in_set(n.coords))
						openset.push_node(n);
					else
					{ 
						openset.pop_requested(n.coords);
						openset.push_node(n);
					}
				}
			}
		}

		if(current.coords[1] - 1 >= 0)
		{
			//ROS_INFO("enter 3");
			coords[0]=current.coords[0];
			coords[1]=current.coords[1]-1;

			//ROS_INFO("check the point  %d %d! ",coords[0],coords[1]);
			if( map[map_width * coords[1] + coords[0]] < 70  )
			{
				//ROS_INFO("obstacle??? %d ",map[map_width * coords[1] + coords[0]]);
				t_h=heur(coords,goal.coords);
				t_g = current.t_g + g_cost(coords,current.coords,current.came_from);
				t_f = t_g + t_h;

				if(openset.check_if_in_set(coords))
				{
					n = openset.read_node(coords); 
				}
				else
				{
					n = create_node(coords,t_f,t_g,current.coords);
				}

				if(!closedset.check_if_in_set(n.coords) || t_f < n.t_f )
				{

					n.t_g=t_g;
					n.t_f=t_f;
					n.came_from[0]=current.coords[0];
					n.came_from[1]=current.coords[1];
					if(!openset.check_if_in_set(n.coords))
						openset.push_node(n);
					else
					{ 
						openset.pop_requested(n.coords);
						openset.push_node(n);
					}
				}
			}
		}

		if(current.coords[1] + 1 <= lateral_size)
		{
			//ROS_INFO("enter 4");
			coords[0]=current.coords[0];
			coords[1]=current.coords[1]+1;
			//ROS_INFO("check the point  %d %d! ", coords[0], coords[1]);
			if( map[map_width * coords[1] + coords[0]] < 70 )
			{
				//ROS_INFO("obstacle??? %d",map[map_width * coords[1] + coords[0]]);
				t_h=heur(coords,goal.coords);
				t_g = current.t_g + g_cost(coords,current.coords,current.came_from);
				t_f = t_g + t_h;

				if(openset.check_if_in_set(coords))
				{
					n = openset.read_node(coords); 
				}
				else
				{
					n = create_node(coords,t_f,t_g,current.coords);
				}

				if(!closedset.check_if_in_set(n.coords) || t_f < n.t_f )
				{

					n.t_g=t_g;
					n.t_f=t_f;
					n.came_from[0]=current.coords[0];
					n.came_from[1]=current.coords[1];
					if(!openset.check_if_in_set(n.coords))
						openset.push_node(n);
					else
					{ 
						openset.pop_requested(n.coords);
						openset.push_node(n);
					}
				}
			}
			current=openset.pop_best();
		}

		if(current.coords[0] - 1 >= 0 && current.coords[1] + 1 <= lateral_size)
		{
			//ROS_INFO("enter 1");
			coords[0]=current.coords[0]-1;
			coords[1]=current.coords[1]+1;
			//ROS_INFO("check the point  %d %d! ",coords[0],coords[1]);
			
			if( map[map_width * coords[1] + coords[0]] < 70 )
			{	
				//ROS_INFO("obstacle??? %d",map[map_width * coords[1] + coords[0]]);
				t_h = heur(coords,goal.coords);
				t_g = current.t_g + g_cost(coords,current.coords,current.came_from);
				t_f = t_g + t_h;

				if(openset.check_if_in_set(coords))
				{
					n = openset.read_node(coords); // only doing this to get the f value, if present				
				}
				else
				{
					n = create_node(coords,t_f,t_g,current.coords);
				}
				if(!closedset.check_if_in_set(n.coords) || t_f < n.t_f )
				{
					n.t_g=t_g;
					n.t_f=t_f;
					n.came_from[0]=current.coords[0];
					n.came_from[1]=current.coords[1];
					if(!openset.check_if_in_set(n.coords))
						openset.push_node(n);
					else
					{ // update the value of the node in openset
						openset.pop_requested(n.coords);
						openset.push_node(n);
					}
				}
			}
		}

		if(current.coords[0] + 1 <=lateral_size && current.coords[1] + 1 <= lateral_size)
		{
			//ROS_INFO("enter 1");
			coords[0]=current.coords[0]+1;
			coords[1]=current.coords[1]+1;
			//ROS_INFO("check the point  %d %d! ",coords[0],coords[1]);
			
			if( map[map_width * coords[1] + coords[0]] < 70 )
			{	
				//ROS_INFO("obstacle??? %d",map[map_width * coords[1] + coords[0]]);
				t_h = heur(coords,goal.coords);
				t_g = current.t_g + g_cost(coords,current.coords,current.came_from);
				t_f = t_g + t_h;

				if(openset.check_if_in_set(coords))
				{
					n = openset.read_node(coords); // only doing this to get the f value, if present				
				}
				else
				{
					n = create_node(coords,t_f,t_g,current.coords);
				}
				if(!closedset.check_if_in_set(n.coords) || t_f < n.t_f )
				{
					n.t_g=t_g;
					n.t_f=t_f;
					n.came_from[0]=current.coords[0];
					n.came_from[1]=current.coords[1];
					if(!openset.check_if_in_set(n.coords))
						openset.push_node(n);
					else
					{ // update the value of the node in openset
						openset.pop_requested(n.coords);
						openset.push_node(n);
					}
				}
			}
		}

		if(current.coords[0] - 1 >= 0 && current.coords[1] - 1 >= 0)
		{
			//ROS_INFO("enter 1");
			coords[0]=current.coords[0]-1;
			coords[1]=current.coords[1]-1;
			//ROS_INFO("check the point  %d %d! ",coords[0],coords[1]);
			
			if( map[map_width * coords[1] + coords[0]] < 70 )
			{	
				//ROS_INFO("obstacle??? %d",map[map_width * coords[1] + coords[0]]);
				t_h = heur(coords,goal.coords);
				t_g = current.t_g + g_cost(coords,current.coords,current.came_from);
				t_f = t_g + t_h;

				if(openset.check_if_in_set(coords))
				{
					n = openset.read_node(coords); // only doing this to get the f value, if present				
				}
				else
				{
					n = create_node(coords,t_f,t_g,current.coords);
				}
				if(!closedset.check_if_in_set(n.coords) || t_f < n.t_f )
				{
					n.t_g=t_g;
					n.t_f=t_f;
					n.came_from[0]=current.coords[0];
					n.came_from[1]=current.coords[1];
					if(!openset.check_if_in_set(n.coords))
						openset.push_node(n);
					else
					{ // update the value of the node in openset
						openset.pop_requested(n.coords);
						openset.push_node(n);
					}
				}
			}
		}

		if(current.coords[0] + 1 <=lateral_size && current.coords[1] - 1 >= 0)
		{
			//ROS_INFO("enter 1");
			coords[0]=current.coords[0]+1;
			coords[1]=current.coords[1]-1;
			//ROS_INFO("check the point  %d %d! ",coords[0],coords[1]);
			
			if( map[map_width * coords[1] + coords[0]] < 70 )
			{	
				//ROS_INFO("obstacle??? %d",map[map_width * coords[1] + coords[0]]);
				t_h = heur(coords,goal.coords);
				t_g = current.t_g + g_cost(coords,current.coords,current.came_from);
				t_f = t_g + t_h;

				if(openset.check_if_in_set(coords))
				{
					n = openset.read_node(coords); // only doing this to get the f value, if present				
				}
				else
				{
					n = create_node(coords,t_f,t_g,current.coords);
				}
				if(!closedset.check_if_in_set(n.coords) || t_f < n.t_f )
				{
					n.t_g=t_g;
					n.t_f=t_f;
					n.came_from[0]=current.coords[0];
					n.came_from[1]=current.coords[1];
					if(!openset.check_if_in_set(n.coords))
						openset.push_node(n);
					else
					{ // update the value of the node in openset
						openset.pop_requested(n.coords);
						openset.push_node(n);
					}
				}
			}
		}
	}
	ROS_INFO("nothing");
	return return_error;
}

void pathCallBack(const nav_msgs::OccupancyGridConstPtr &msg)
{

	map = *msg;
	map_width = map.info.width;	
	//ROS_INFO("get the map");

	
}

int compare(int x, int y)
{
	if (x > y)
		return x;
	else
		return y;
}

bool collisionCheck(node node1, node node2)
{
	if(node1.coords[0] == node2.coords[0])
	{
		ROS_INFO("a straight line");
		return true;
	}

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

	if(slope == 0)
	{
		ROS_INFO("a flat line");
		return true;
	}

	if(slope < 0 && slope > -1)
	{
		ROS_INFO("current slope is %.2f",slope);
		for (int i = littleX + 1; i < bigX; i++ )
		{
			y = slope * i + b;
			dy1 = std::abs(y - littleY);
			dy2 = std::abs(y - (littleY - 1));
			
			if(dy1 < dy2)
			{

				if(map.data[map_width * littleY + i] > 70)
					return false;
			}
			else
			{
				if(map.data[map_width * (littleY - 1) + i] > 70)
					return false;
				//not an obstacle move to next grid
				littleY = littleY - 1;
			}
		}
		ROS_INFO("finish finding obstacle");

		return true;
	}

	if(slope < -1)
	{	
		ROS_INFO("current slope is %.2f",slope);
		for (int i = littleY; i > bigY; i--)
		{
			x = (i - 1 - b) / slope;
			dx1 = std::abs(x - littleX);
			dx2 = std::abs(x - (littleX + 1));

			if(dx1 < dx2)
			{
				if(map.data[map_width * (i - 1) + littleX] > 70)
					return false;
			}
			else
			{
				if(map.data[map_width * (i - 1) + (littleX + 1)] > 70)
					return false;

				littleX = littleX + 1;
			}
		}
		ROS_INFO("finish finding obstacle");

		return true;
	}


	if(slope >= 1)
	{		
		ROS_INFO("current slope is %.2f",slope);
		for (int i = littleY; i < bigY; ++i)
		{
			x = (i + 1 - b) / slope;
			dx1 = std::abs(x - littleX);
			dx2 = std::abs(x - (littleX + 1));

			if(dx1 < dx2)
			{
				if(map.data[map_width * (i + 1) + littleX] > 70)
					return false;
			}
			else
			{
				if(map.data[map_width * (i + 1) + (littleX + 1)] > 70)
					return false;

				littleX = littleX + 1;
			}
			
		}
		ROS_INFO("finish finding obstacle");

		return true;
	}

	if(slope < 1 && slope > 0)
	{
		ROS_INFO("current slope is %.2f",slope);
		for (int i = littleX ; i < bigX; i++ )
		{
			y = slope * (i + 1) + b;
			dy1 = std::abs(y - littleY);
			dy2 = std::abs(y - (littleY + 1));
			
			if(dy1 < dy2)
			{

				if(map.data[map_width * littleY + i + 1] > 70)
					return false;
			}
			else
			{
				if(map.data[map_width * (littleY + 1) + i + 1] > 70)
					return false;

				littleY = littleY + 1;
			}
		}
		ROS_INFO("finish finding obstacle");

		return true;
	}

	
	ROS_INFO("finish all obstacle finding");
	return true;
}

std::vector<node> smoothing(std::vector<node> solution)
{
	int size = solution.size();
	std::vector<node> changeNode;
	std::vector<node> newSolution;
	changeNode.resize(size);
	std::vector<node> changeNodes;
	int count = 0;
	int finishFlag = 0;
	
	ROS_INFO("start checking obstacle");

	newSolution.push_back(solution[size-1]);
	//do collision detection
	int currentIndex = 0;
	int index = solution.size() - 2;
	int times = 0;
	for (int i = solution.size() - 1; i >= 0; )
	{

		if(i == 0 )
		{
			//push back the goal
			newSolution.push_back(solution[i]); 
			break;
		}
		for (int j = index ; j >=0 ; j--)
		{
			//ROS_INFO("checking the change node NO %d to %d",i,j);
			if (collisionCheck(solution[i],solution[j]) == false)
			{	
				ROS_INFO("cant pass");
				//ROS_INFO("current size of newSolution is %d",newSolution.size());
				if( i == j + 1)
				{
					i = j;
					break;
				}
				newSolution.push_back(solution[j+1]);
				i = j + 1;
				index = j;
				break;
			}
			if(j == 0 )
			{	
				currentIndex = j;
				i = 0; 
				break;
			}
		}
	}
	return newSolution;
}

bool findNearestPoint(node &p, int amount) {
	int min_x = -1, min_y;
	double min_dist = (double)amount * 4;
	for (int y = std::max(p.coords[1] - amount, 0); y < std::min(p.coords[1] + amount + 1, map_width); ++y) {
		for (int x = std::max(p.coords[0] - amount, 0); x < std::min(p.coords[0] + amount + 1, map_width); ++x) {
			if(map.data[y * map_width + x] < 70) {
				double d = std::sqrt(std::pow(x - p.coords[0],2) + std::pow(y - p.coords[1],2));
				if (d < min_dist){
					min_dist = d;
					min_x = x;
					min_y = y;
				}
			}
		}
	}
	if (min_x > 0) {
		p.coords[0] = min_x;
		p.coords[1] = min_y;
		return true;
	}
	else
		return false;
}

bool planning_service(ras_path_plan::path_srv::Request &req, ras_path_plan::path_srv::Response &res)
{
	if(map.data.size() == 0)
		return false;

	std::vector<node> solution;
	std::vector<node> newSolution;
	pos_list.header.frame_id = "/world";
	node start, goal;
	int amount = round(0.18 / map.info.resolution);

	start.coords[0] = (int)(req.startx / map.info.resolution);
	start.coords[1] = (int)(req.starty / map.info.resolution);
	goal.coords[0] = (int)(req.goalx / map.info.resolution);
	goal.coords[1] = (int)(req.goaly / map.info.resolution);

	if (start.coords[0] < 0 )
		start.coords[0] = 0;
	if (start.coords[1] < 0 )
		start.coords[1] = 0;

	if (start.coords[1] > map.info.width)
		start.coords[1] = map.info.width;
	if (start.coords[0] > map.info.width)
		start.coords[0] = map.info.width;

	if (goal.coords[0] < 0 )
		goal.coords[0] = 0;
	if (goal.coords[1] < 0 )
		goal.coords[1] = 0;

	if (goal.coords[1] > map.info.width)
		goal.coords[1] = map.info.width;
	if (goal.coords[0] > map.info.width)
		goal.coords[0] = map.info.width;
	
	
	if(map.data[map_width * start.coords[1] + start.coords[0]] > 0)
		if(!findNearestPoint(start,amount))
			return false;

	if(map.data[map_width * goal.coords[1] + goal.coords[0]] > 0)
		if(!findNearestPoint(goal,amount))
			return false;

	ROS_INFO("GET MAP RESOLUTION %.2f", map.info.resolution);

	ROS_INFO("get the start and goal,start is %d %d, goal is %d %d",start.coords[0],start.coords[1],goal.coords[0],goal.coords[1]);

	ROS_INFO("START DOING PATH PLAN");

	solution = findPath(start, goal, map.data);	

	if(solution.size()==0)
		return false;
	//ROS_INFO("length of path is %d",solution.size());
	
	ROS_INFO("FINISH FINDING THE PATH");



	newSolution = smoothing(solution);

	ROS_INFO("FINISH SMOOTHING THE PATH");
	

	pos_list.poses.resize(newSolution.size());

	for (int i = 0; i < newSolution.size(); ++i)
	{
		//ROS_INFO("enter pos_list");
		pos_list.poses[i].pose.position.x = newSolution[i].coords[0] * map.info.resolution;
		pos_list.poses[i].pose.position.y = newSolution[i].coords[1] * map.info.resolution;
		pos_list.poses[i].pose.position.z = 0;
		pos_list.poses[i].header.frame_id = "/world";
		ROS_INFO("solution NO %d is %d %d",i,newSolution[i].coords[0],newSolution[i].coords[1]);

	}	
	
		
	res.planned_path = pos_list;
	return true;
	// }	
}


int main(int argc, char *argv[])
{
	ros::init(argc, argv, "path_planner");
	ros::NodeHandle n;
	ros::Time init_time=ros::Time::now();
	ros::ServiceServer path_service;

	
	//path_pub = n.advertise<nav_msgs::Path>("/path/plan",1);
	ros::Subscriber path_sub = n.subscribe("/mapping/map",1,pathCallBack);
	path_service = n.advertiseService("/path/plan",planning_service);

	ros::Rate loop_rate(10);

	while(n.ok()){
		loop_rate.sleep();
		ros::spinOnce();
		//path_pub.publish(pos_list);
	}

	return 0;
}
