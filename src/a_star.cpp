#include "a_star.h"
#include <algorithm>
#include <iostream>

Node::Node(int x_in, int y_in, const Node* parent_in) : x(x_in), y(y_in){
	// x = x_in;
	// y = y_in;
	parent = parent_in;
	g = parent->g + cost_map[x][y];
	h = calculateEuclideanDistance(*this, target);
}

int Node::f() const {
	return g + h;
}

size_t Node_hash::operator()(const Node& node) const {
	const size_t hashx = std::hash<int>() (node.x);
	const size_t hashy = std::hash<int>() (node.y);
	// XOR to avoid hash collision
	return hashx ^ hashy;
}

bool Compare_cord::operator()(const Node& lhs, const Node& rhs) {
	return rhs.x == lhs.x && rhs.y == rhs.y;
}

bool Compare_f_cost::operator()(const Node& node1, const Node& node2) {
	return node1.f() < node2.f();
}

bool Compare_g_cost::operator()(const Node& node1, const Node& node2) {
	return node1.g < node2.g;
}

A_star::A_star() {}

void A_star::backtracker(){
    Node cur = closed_set.find(target);
    while(cur.parent){
        path.insert(cur);
        cur = cur.parent;
    }
    path.insert(start);
    std::reverse(path.begin(), path.end());
}

bool A_star::validNode(const Node& node) {
    const int cost_map_height = cost_map.size();
    const int cost_map_width = cost_map[0].size();
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

bool A_star::processNode(const int x, const int y, const Node* parent) {
	Node node = Node(x, y, parent);
	if (Compare_cord(node, target)) {
		closed_set.insert(node);
		return true;
	}
	if (valid_node(node)) {
		open_set.push(node);
	}
	return false;
}

bool A_star::search(){
    open_set.insert(start);
    while(!open_set.empty()){
        Node cur = open_set.top();
        open_set.pop();
        // Using 4 connected for now
        //TODO make 8 connected
        if(processNode(cur.x, cur.y+1, &cur) // north
        || processNode(cur.x+1, cur.y, &cur) // east
        || processNode(cur.x, cur.y-1, &cur) // south
        || processNode(cur.x-1, cur.y, &cur) // west
        || processNode(cur.x + 1, cur.y + 1, &cur) // north-east
        || processNode(cur.x + 1, cur.y - 1, &cur) // south-east
        || processNode(cur.x - 1, cur.y + 1, &cur) // north-west
        || processNode(cur.x - 1, cur.y - 1, &cur)) { // south-west
            break;
        }
        closed_set.insert(cur);
    }
    backtracker();
    return true;
}

double A_star::calculateEuclideanDistance(const Node& node1, const Node& node2){
    return pow(node1.x - node2.x, 2) - pow(node1.y - node2.y, 2); // can compare dist^2
}

void A_star::gpsCallback(const nav_msgs::Odometry::ConstPtr& msg){
    // Fill out the needed "target" position member variable using info from the odom message
    target = Node(msg->pose.pose.position.x, msg->pose.pose.position.y, nullptr);
}

void A_star::costmapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
	// Fill out the costmap width and height from the occupancy grid info message
    int costmap_width = msg->info.width;
    int costmap_height = msg->info.height;

    // Fill out the needed "start" position member variable using info from the costmap origin message
    start = Node(msg->info.origin.orientation.x, msg->info.origin.orientation.y, nullptr);

    // Fill out the costmap member variable using info from the occupancy grid costmap message
    for(int i = 0; i < costmap_width; i++){
        for(int j = 0; j < costmap_height; j++){
            cost_map[i][j] = msg->data[i + (j * costmap_width)];
        }
    }
}
