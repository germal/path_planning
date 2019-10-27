#ifndef A_STAR_H
#define A_STAR_H

#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include <vector>
#include <queue>
#include <unordered_set>

class Node{
};

bool operator==(const Node& lhs, const Node& rhs);

class Compare_f_cost{

};

class Compare_g_cost{

};

class A_star{
    public:
    A_star();
    void backtracker(std::vector<Node*>& path);
    int min_cost(const Node& node);
    void gpsCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void costMapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);

    private:
    Node start;
    Node target;
    geometry_msgs::Pose current_pose;
    std::priority_queue<Node, Compare_f_cost> open_set;
    std::unordered_set<Node> closed_set;
    std::vector<std::vector<int>> cost_map;

};

#endif