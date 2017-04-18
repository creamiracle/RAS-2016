#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>

double robot_width = 0.19;

// This structure purpose is to contain lines that will represent walls.
// The first four variables are for a first point, second point representation
// The last two varibles are for a vector representation
struct Line {
    double x1;
    double y1;
    double x2;
    double y2;
    double theta;
    double t_max;
};

struct PointXY {
    double x;
    double y;
};

struct Cell
{
    std::vector<Line> borders;
    int owner;
};

Line getAxis(PointXY p1, PointXY p2) {
    double x = (p1.x + p2.x) / 2;
    double y = (p1.y + p2.y) / 2;
    double theta = atan2(p1.y - p2.y, p1.x - p2.x);
    theta += M_PI/2;
    return (Line){x, y, 0, 0, theta, 0};
}

bool isDuplicate(double x, double y, std::vector<PointXY> &points) {
    for (int i = 0; i < points.size(); ++i)
        if (std::abs(points[i].x - x) < 0.00001)
            if (std::abs(points[i].y - y) < 0.00001)
                return true;
    return false;
}

void split_cell(Cell &split_cell, Cell &second_output, Line bisector) {
    second_output.borders.clear();
    second_output.owner = 0;
    double x1 = -1, y1, x2, y2;
    int split_border_index_1, split_border_index_2;
    Cell temp;
    for (int i = 0; i < split_cell.borders.size(); i++) {
        // Check intersection between split_cell.borders[i] and bisector
        if (std::abs(split_cell.borders[i].theta - bisector.theta) > 0.000001) {
            double t_border = ((bisector.x1 - split_cell.borders[i].x1)*sin(bisector.theta)
                                - (bisector.y1 - split_cell.borders[i].y1)*cos(bisector.theta))
                                   /sin(bisector.theta - split_cell.borders[i].theta);
            if ((0 < t_border) && (t_border < split_cell.borders[i].t_max)) {
                // Intersection found
                if (x1 < 0) {
                    x1 = split_cell.borders[i].x1 + t_border*cos(split_cell.borders[i].theta);
                    y1 = split_cell.borders[i].y1 + t_border*sin(split_cell.borders[i].theta);
                    split_border_index_1 = i;
                } else {
                    x2 = split_cell.borders[i].x1 + t_border*cos(split_cell.borders[i].theta);
                    y2 = split_cell.borders[i].y1 + t_border*sin(split_cell.borders[i].theta);
                    split_border_index_2 = i;
                    break;
                }
            }
        }
    }
    if (x1 < 0) {
        second_output.owner = -100;
        return;
    }
    // New border contained in (x1,y1) -> (x2,y2)
    for (int i = 0; i < split_cell.borders.size(); i++) {
        if (i != split_border_index_1) {
            if(i != split_border_index_2)
                temp.borders.push_back(split_cell.borders[i]);
            else { // i == split_border_index_2
                double theta = atan2(y2 - split_cell.borders[i].y1, x2 - split_cell.borders[i].x1);
                double t_max = sqrt(pow(y2 - split_cell.borders[i].y1, 2) + pow(x2 - split_cell.borders[i].x1, 2));
                temp.borders.push_back((Line){split_cell.borders[i].x1,
                                              split_cell.borders[i].y1,
                                              x2, y2, theta, t_max});
                theta = atan2(y1 - y2, x1 - x2);
                t_max = sqrt(pow(y2 - y1, 2) + pow(x2 - x1, 2));
                temp.borders.push_back((Line){x2,y2,x1,y1,theta,t_max});
                i = split_border_index_1;
                theta = atan2(split_cell.borders[i].y2 - y1, split_cell.borders[i].x2 - x1);
                t_max = sqrt(pow(y1 - split_cell.borders[i].y2, 2) + pow(x1 - split_cell.borders[i].x2, 2));
                temp.borders.push_back((Line){x1,y1,
                                              split_cell.borders[i].x2,
                                              split_cell.borders[i].y2,
                                              theta, t_max});
            }
        }
        else { // i == split_border_index_1
            double theta = atan2(y1 - split_cell.borders[i].y1, x1 - split_cell.borders[i].x1);
            double t_max = sqrt(pow(y1 - split_cell.borders[i].y1, 2) + pow(x1 - split_cell.borders[i].x1, 2));
            temp.borders.push_back((Line){split_cell.borders[i].x1,
                                          split_cell.borders[i].y1,
                                          x1, y1, theta, t_max});
            theta = atan2(y2 - y1, x2 - x1);
            t_max = sqrt(pow(y2 - y1, 2) + pow(x2 - x1, 2));
            temp.borders.push_back((Line){x1,y1,x2,y2,theta,t_max});
            i = split_border_index_2;
            theta = atan2(split_cell.borders[i].y2 - y2, split_cell.borders[i].x2 - x2);
            t_max = sqrt(pow(y2 - split_cell.borders[i].y2, 2) + pow(x2 - split_cell.borders[i].x2, 2));
            temp.borders.push_back((Line){x2,y2,
                                          split_cell.borders[i].x2,
                                          split_cell.borders[i].y2,
                                          theta, t_max});
        }
    }
    int i = std::min(split_border_index_1, split_border_index_2) +1;
    int i_max = std::max(split_border_index_1, split_border_index_2);
    for (i; i < i_max; i++)
        second_output.borders.push_back(split_cell.borders[i]);
    if (split_border_index_1 < split_border_index_2) {
        double theta = atan2(y2 - split_cell.borders[split_border_index_2].y1, x2 - split_cell.borders[split_border_index_2].x1);
        double t_max = sqrt(pow(y2 - split_cell.borders[split_border_index_2].y1, 2) + pow(x2 - split_cell.borders[split_border_index_2].x1, 2));
        second_output.borders.push_back((Line){split_cell.borders[split_border_index_2].x1,
                                               split_cell.borders[split_border_index_2].y1,
                                               x2, y2, theta, t_max});
        theta = atan2(y1 - y2, x1 - x2);
        t_max = sqrt(pow(y1 - y2, 2) + pow(x1 - x2, 2));
        second_output.borders.push_back((Line){x2,y2,x1,y1,theta,t_max});
        theta = atan2(split_cell.borders[split_border_index_2].y1 - y1, split_cell.borders[split_border_index_2].x1 - x1);
        t_max = sqrt(pow(y1 - split_cell.borders[split_border_index_2].y1, 2) + pow(x1 - split_cell.borders[split_border_index_2].x1, 2));
        second_output.borders.push_back((Line){x1,y1,
                                               split_cell.borders[split_border_index_1].x2,
                                               split_cell.borders[split_border_index_1].y2,
                                               theta, t_max});
    } else {
        double theta = atan2(y1 - split_cell.borders[split_border_index_2].y1, x1 - split_cell.borders[split_border_index_2].x1);
        double t_max = sqrt(pow(y1 - split_cell.borders[split_border_index_2].y1, 2) + pow(x1 - split_cell.borders[split_border_index_2].x1, 2));
        second_output.borders.push_back((Line){split_cell.borders[split_border_index_1].x1,
                                               split_cell.borders[split_border_index_1].y1,
                                               x2, y2, theta, t_max});
        theta = atan2(y2 - y1, x2 - x1);
        t_max = sqrt(pow(y1 - y2, 2) + pow(x1 - x2, 2));
        second_output.borders.push_back((Line){x1,y1,x2,y2,theta,t_max});
        theta = atan2(split_cell.borders[split_border_index_2].y1 - y2, split_cell.borders[split_border_index_2].x1 - x2);
        t_max = sqrt(pow(y2 - split_cell.borders[split_border_index_2].y1, 2) + pow(x2 - split_cell.borders[split_border_index_2].x1, 2));
        second_output.borders.push_back((Line){x2,y2,
                                               split_cell.borders[split_border_index_2].x2,
                                               split_cell.borders[split_border_index_2].y2,
                                               theta, t_max});
    }

    split_cell = temp;
}

void assign_owner(Cell &stray_cell, std::vector<PointXY> &vertices) {
    Line axis1 = getAxis((PointXY){stray_cell.borders[0].x1,stray_cell.borders[0].y1},
                         (PointXY){stray_cell.borders[0].x2,stray_cell.borders[0].y2});
    Line axis2 = getAxis((PointXY){stray_cell.borders[1].x1,stray_cell.borders[1].y1},
                         (PointXY){stray_cell.borders[1].x2,stray_cell.borders[1].y2});
    double t_1 = (axis1.x1 - axis2.x1)*sin(axis2.theta) - (axis1.y1 - axis2.y1)*cos(axis2.theta);
    t_1 /= sin(axis1.theta - axis2.theta);
    double x = axis1.x1 + t_1 * cos(axis1.theta);
    double y = axis1.y1 + t_1 * sin(axis1.theta);
    double min_d = 100;
    double min_i = -1;
    for (int i = 0; i < vertices.size(); ++i) {
        double d = sqrt(pow(x - vertices[i].x, 2) + pow(y - vertices[i].y, 2));
        if (d < min_d) {
            min_d = d;
            min_i = i;
        }
    }
    stray_cell.owner = min_i;
}

bool is_same_double(double d1, double d2) {
    return std::abs(d1-d2) < 0.000001;
}

bool is_same_line(Line l1, Line l2) {
    return (is_same_double(l1.x1, l2.x1)
            && is_same_double(l1.x2, l2.x2)
            && is_same_double(l1.y1, l2.y1)
            && is_same_double(l1.y2, l2.y2))
            || (is_same_double(l1.x1, l2.x2)
            && is_same_double(l1.x2, l2.x1)
            && is_same_double(l1.y1, l2.y2)
            && is_same_double(l1.y1, l2.y2));
}

void order_cell_edges(Cell &toOrder) {
    Cell ordered;
    ordered.borders.push_back(toOrder.borders[0]);
    toOrder.borders.erase(toOrder.borders.begin());
    while (toOrder.borders.size() > 0) {
        for (int i = 0; i < toOrder.borders.size(); ++i) {
            if (is_same_double(toOrder.borders[i].x1, ordered.borders.back().x2)
                && is_same_double(toOrder.borders[i].y1, ordered.borders.back().y2)) {

                ordered.borders.push_back(toOrder.borders[i]);
                toOrder.borders.erase(toOrder.borders.begin() + i);
                break;
            }
        }
    }
    toOrder = ordered;
}

void merge_cells(std::vector<Cell> &toMerge, std::vector<PointXY> &vertices) {
    for (int i = 0; i < toMerge.size(); ++i)
        assign_owner(toMerge[i], vertices);
    std::vector<Cell> temp_group;
    std::vector<Cell> merged;
    for (int i = 0; i < vertices.size(); ++i) {
        temp_group.clear();
        for (int j = 0; j < toMerge.size(); ++j){
            if(toMerge[j].owner == i)
                temp_group.push_back(toMerge[j]);
        }
        // Temp group contains all cells with same owner
        // Have to remove double lines
        for (int k = 0; k < temp_group.size(); ++k) {
            for (int z = 0; z < temp_group[k].borders.size(); ++z) {
                for (int m = 0; m < temp_group.size(); ++m) {
                    for (int n = 0; n < temp_group[m].borders.size(); ++n) {
                        if (k == m && z == n)
                            continue;
                        if(is_same_line(temp_group[k].borders[z], temp_group[m].borders[n])) {
                            // Remove
                            temp_group[k].borders.erase(temp_group[k].borders.begin() + z);
                            temp_group[m].borders.erase(temp_group[m].borders.begin() + n);
                        }
                    }
                }
            }
        }
        // Push all edges to a new cell and push that in merged
        Cell new_cell;
        for (int k = 0; k < temp_group.size(); ++k)
            for (int z = 0; z < temp_group[k].borders.size(); ++z)
                new_cell.borders.push_back(temp_group[k].borders[z]);
        order_cell_edges(new_cell);
        merged.push_back(new_cell);
    }
    toMerge = merged;
}

int main(int argc, char** argv) {
    std::vector<Line> walls;
    std::vector<PointXY> walls_vertices;
    std::vector<PointXY> processed_vertices;
    std::vector<Line> vornoi_edges;

    std::vector<Cell> cells;

    std::string _map_file = "/home/ras28/catkin_ws/src/ras_maze/ras_maze_map/maps/lab_maze_2016.txt";

    double max_x = 0, max_y = 0, min_x = 100, min_y = 100;

    ros::init(argc, argv, "vornoi_builder");
    ros::NodeHandle n;
    ros::Rate rate(1);
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("/vornoi_nodes", 1);
    ros::Publisher marker_pub_lines = n.advertise<visualization_msgs::Marker>("/vornoi_lines", 1);
    ros::Publisher marker_pub_borders = n.advertise<visualization_msgs::Marker>("/vornoi_borders", 1);
    visualization_msgs::Marker rviz_points, rviz_lines, rviz_borders;
    rviz_points.header.frame_id = rviz_borders.header.frame_id = rviz_lines.header.frame_id = "/world";
    rviz_points.header.stamp = rviz_borders.header.stamp = rviz_lines.header.stamp = ros::Time::now();
    rviz_points.ns = rviz_borders.ns = rviz_lines.ns = "vornoi_nodes";
    rviz_points.type = visualization_msgs::Marker::POINTS;
    rviz_lines.type = rviz_borders.type = visualization_msgs::Marker::LINE_LIST;
    rviz_points.action = rviz_lines.action = rviz_borders.action = visualization_msgs::Marker::ADD;
    rviz_points.pose.orientation.w = rviz_borders.pose.orientation.w = rviz_lines.pose.orientation.w =1.0;
    rviz_points.scale.x = 0.02;
    rviz_points.scale.y = 0.02;
    rviz_lines.scale.x = rviz_borders.scale.x = 0.02;
    rviz_points.color.g = 1.0f;
    rviz_lines.color.b = 1.0;
    rviz_borders.color.r = 1.0;
    rviz_points.color.a = rviz_borders.color.a = rviz_lines.color.a = 1.0;

    ROS_INFO_STREAM("Loading map from " << _map_file);
    std::ifstream map_fs; map_fs.open(_map_file.c_str());
    if (!map_fs.is_open()){
        ROS_ERROR_STREAM("Could not read maze map from "<<_map_file<<
                         ". Please double check that the file exists. Aborting.");
        return -1;
    }
    std::string line;
    while (std::getline(map_fs, line)){
        if (line[0] == '#') {
            // comment -> skip
            continue;
        }
        double x1, x2, y1, y2;

        std::istringstream line_stream(line);
        line_stream >> x1 >> y1 >> x2 >> y2;

        if (x1 > max_x)
            max_x = x1;
        if (y1 > max_y)
            max_y = y1;
        if (x2 > max_x)
            max_x = x2;
        if (y2 > max_y)
            max_y = y2;

        if(x1 < min_x)
            min_x = x1;
        if(x2 < min_x)
            min_x = x2;
        if(y1 < min_y)
            min_y = y1;
        if(y2 < min_y)
            min_y = y2;

        double theta = atan2(y2 - y1, x2 - x1);
        double t_max = sqrt(pow(y2-y1, 2) + pow(x2-x1, 2));

        walls.push_back((Line){x1, y1, x2, y2, theta, t_max});
        if (!isDuplicate(x1, y1, walls_vertices))
            walls_vertices.push_back((PointXY){x1, y1});
        if (!isDuplicate(x2, y2, walls_vertices))
            walls_vertices.push_back((PointXY){x2, y2});
    }

    std::vector<Line> first_cell;
    first_cell.push_back((Line){min_x, min_y, min_x, max_y, M_PI/2, max_y - min_y});
    first_cell.push_back((Line){min_x, max_y, max_x, max_y, 0, max_x - min_x});
    first_cell.push_back((Line){max_x, max_y, max_x, min_y, -M_PI/2, max_y - min_y});
    first_cell.push_back((Line){max_x, min_y, min_x, min_y, M_PI, max_x - min_x});
    cells.push_back((Cell){first_cell, -1});
    
    processed_vertices.push_back(walls_vertices[0]);
    geometry_msgs::Point p;
    p.x = processed_vertices[0].x;
    p.y = processed_vertices[0].y;
    rviz_points.points.push_back(p);
    for (int i = 1; i < walls_vertices.size(); ++i) {
        ROS_INFO_STREAM("Process step " << i << " of " << walls_vertices.size());
        processed_vertices.push_back(walls_vertices[i]);
        p.x = processed_vertices[i].x;
        p.y = processed_vertices[i].y;
        rviz_points.points.push_back(p);
        std::vector<Cell> cells_temp;
        for (int j = 0; j < cells.size(); ++j) {
            Line bisector = getAxis(processed_vertices[i], processed_vertices[j]);
            Cell second_output;
            split_cell(cells[j], second_output, bisector);
            cells_temp.push_back(cells[j]);
            if (second_output.owner != -100)
                cells_temp.push_back(second_output);
        }
        merge_cells(cells_temp, processed_vertices);
        cells = cells_temp;
    }
    

    for (int i = 0; i < cells.size(); ++i) {
        for (int j = 0; j < cells[i].borders.size(); ++j) {
            p.x = cells[i].borders[j].x1;
            p.y = cells[i].borders[j].y1;
            rviz_lines.points.push_back(p);
            p.x = cells[i].borders[j].x2;
            p.y = cells[i].borders[j].y2;
            rviz_lines.points.push_back(p);
        }
    }

    /*Line splitter = {max_x/2, max_y/2, 0, 0, M_PI/3, 0};
    split_cell(cells[0], cella, splitter);
    split_cell(cells[0], cella, (Line){max_x/2, max_y/2, 0, 0, M_PI/6, 0});
    for (int i = 0; i < cells[0].borders.size(); ++i) {
        geometry_msgs::Point p;
        p.x = cella.borders[i].x1;
        p.y = cella.borders[i].y1;
        rviz_borders.points.push_back(p);
        p.x = cella.borders[i].x2;
        p.y = cella.borders[i].y2;
        rviz_borders.points.push_back(p);
    }*/

    // Publish
    while(ros::ok()) {
        rate.sleep();
        /*rviz_borders.points.clear();
        for (int i = 0; i < cells[0].borders.size(); ++i) {
            geometry_msgs::Point p;
            p.x = cells[0].borders[i].x1;
            p.y = cells[0].borders[i].y1;
            rviz_borders.points.push_back(p);
            p.x = cells[0].borders[i].x2;
            p.y = cells[0].borders[i].y2;
            rviz_borders.points.push_back(p);
        }*/
        marker_pub_borders.publish(rviz_borders);
        marker_pub.publish(rviz_points);
        /*rate.sleep();
        rviz_borders.points.clear();
        for (int i = 0; i < cells[0].borders.size(); ++i) {
            geometry_msgs::Point p;
            p.x = cella.borders[i].x1;
            p.y = cella.borders[i].y1;
            rviz_borders.points.push_back(p);
            p.x = cella.borders[i].x2;
            p.y = cella.borders[i].y2;
            rviz_borders.points.push_back(p);
        }
        marker_pub.publish(rviz_points);
        marker_pub_lines.publish(rviz_lines);
        marker_pub_borders.publish(rviz_borders);*/
    }
}