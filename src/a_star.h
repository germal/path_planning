#ifndef A_STAR_H
#define A_STAR_H

#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include <vector>
#include <queue>
#include <unordered_set>

class Node{
    public:
    Node(int x, int y, Node* parent);
    private:
    geometry_msgs::Point position;
    Node* parent;
    int f;
    int g;
    int h;
};

bool operator==(const Node& lhs, const Node& rhs);

class Compare_f_cost{

};

class Compare_g_cost{

};

class A_star{
    public:
    A_star();
    void Search();
    void backtracker();
    int min_cost(const Node& node);
    bool valid_node(const Node& node);
    void gpsCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void costMapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);

    protected:
    friend class Node;

    private:
    Node start;
    Node target;
    geometry_msgs::Pose current_pose;
    std::priority_queue<Node, Compare_f_cost> open_set;
    std::unordered_set<Node> closed_set;
    std::vector<std::vector<int>> cost_map;
    std::vector<Node*>& path;

};

#endif