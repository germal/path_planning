#include "a_star.h"

bool operator==(const Node& lhs, const Node& rhs){
    return rhs.x == lhs.x && rhs.y == rhs.y;
}

bool Compare_f_cost::operator()(const Node& node1, const Node& node2){
    return node1.f < node2.f;
}

bool Compare_g_cost::operator()(const Node& node1, const Node& node2){
    return node1.g < node2.g;
}

A_star::A_star(){}

void A_star::backtracker(){}
int A_star::min_cost(const Node& node){}
bool A_star::valid_node(const Node& node) {
    if(cost_map.empty()){
        // TODO we have a problem
    }
    int cost_map_height = cost_map.size();
    if(cost_map[0].empty()){
        // TODO we have a problem
    }
    int cost_map_width = cost_map[0].size();
    // check if position is 'in-bounds'
    if(node.x < 0 || node.y < 0 || node.x >= cost_map_width || node.y >= cost_map_height){
        return false;
    }
    // check if we have already visited node
    auto it = closed_set.find(node);
    if(!it){
        it = find(open_set.begin(), open_set.end(), node);
    }
    // if we have already found node with lower cost don't add node
    if(it && it->g <= node.g){
            return false;
    }
    // else we want to explore node
    return true;
}
void A_star::gpsCallback(const nav_msgs::Odometry::ConstPtr& msg){}
void A_star::costMapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg){}

void A_star::Search(){
    open_set.insert(start);
    while(!open_set.empty()){
        Node cur = open_set.top();
        open_set.pop();
        // Using 4 connected for now
        Node north = Node(cur.x, cur.y+1, &cur);
        if(north == target){

        }
        if(valid_node(north)){
            open_set.push(north);
        }
        Node east = Node(cur.x+1, cur.y, &cur);
        if(east == target){

        }
        if(valid_node(east)){
            open_set.push(east);
        }
        Node south = Node(cur.x, cur.y-1, &cur);
        if(south == target){

        }
        if(valid_node(south)){
            open_set.push(south);
        }
        Node west = Node(cur.x-1, cur.y, &cur);
        if(west == target){

        }
        if(valid_node(west)){
            open_set.push(west);
        }
        closed_set.insert(cur);
    }
}