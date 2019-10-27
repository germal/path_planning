#include "a_star.h"
bool operator==(const Node& lhs, const Node& rhs){}

A_star::A_star(){}

void A_star::backtracker(){}
int A_star::min_cost(const Node& node){}
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