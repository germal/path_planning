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
    const int x;
    const int y;
    const Node* parent;
    int f;
    int g;
    const int h;
};

class Node_hash{
    size_t operator()(const Node& node) const;
};

class Compare_cord{
    bool operator()(const Node& lhs, const Node& rhs);
}

class Compare_f_cost{
    bool operator()(const Node& node1, const Node& node2);
};

class Compare_g_cost{
    bool operator()(const Node& node1, const Node& node2);
};

class A_star{
    public:
    A_star();
    void Search();
    void Backtracker(std::vector<Node>& path);
    bool ValidNode(const Node& node);
    void gpsCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void costMapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);

    protected:
    friend class Node;

    private:
    const Node start;
    const Node target;
    geometry_msgs::Pose current_pose;
    std::priority_queue<Node, Compare_f_cost> open_set;
    std::unordered_set<Node, node_hash> closed_set;
    std::vector<std::vector<int>> cost_map;

};

#endif