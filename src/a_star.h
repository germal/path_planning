#ifndef A_STAR_H
#define A_STAR_H

#include <vector>
#include <queue>
#include <unordered_set>
#include <cmath>

class Node{
    public:
        Node(const int x_in, const int y_in, const Node* parent_in);
        int f() const;
    private:
        const int x;
        const int y;
        const Node* parent;
        int g;
        const int h;
};

class Node_hash{
    size_t operator()(const Node& node) const;
};

class Compare_coord{
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
        void search();
        void backtracker(std::vector<Node>& path);
        bool validNode(const Node& node);
        bool processNode(cost int x, const int y, const Node* parent);
        void gpsCallback(const nav_msgs::Odometry::ConstPtr& msg);
        void costMapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
        double calculateEuclideanDistance(const Node& node1, const Node& node2);

    private:
        const Node start;
        const Node target;
        std::vector<Node> path;
        std::priority_queue<Node, Compare_f_cost> open_set;
        std::unordered_set<Node, node_hash, Compare_cord> closed_set;
        std::vector<std::vector<int>> cost_map;
};

#endif
