#include "a_star.h"

A_Star::Node::Node(const int x, const int y, const Node* parent, const int h){
    x = x;
    y = y;
    parent = parent;
    g = parent->g + cost_map[x][y];
    h = calculateEuclideanDistance(*this, target);
}

int A_Star::Node::f() const {
    return g+h;
}

size_t A_Star::Node_hash::operator()(const Node& node) const{
    const size_t hashx = std::hash<int>() (node.x);
    const size_t hashy = std::hash<int>() (node.y);
    // XOR to avoid hash collision
    return hashx ^ hashy;
}

bool A_Star::Compare_cord::operator()(const Node& lhs, const Node& rhs){
    return rhs.x == lhs.x && rhs.y == rhs.y;
}

bool A_Star::Compare_f_cost::operator()(const Node& node1, const Node& node2){
    return node1.f() < node2.f();
}

bool A_Star::Compare_g_cost::operator()(const Node& node1, const Node& node2){
    return node1.g < node2.g;
}

A_star::A_star(){
    //TODO
}

void A_star::Backtracker(std::vector<Node>& path){
    Node cur = closed_set.find(target);
    while(cur.parent){
        path.insert(cur);
        cur = cur.parent;
    }
    path.insert(start);
    std::reverse(path.begin(), path.end());
}

bool A_star::ValidNode(const Node& node) {
    if(cost_map.empty()){
        // TODO we have a problem
    }
    const int cost_map_height = cost_map.size();
    if(cost_map[0].empty()){
        // TODO we have a problem
    }
    cobst int cost_map_width = cost_map[0].size();
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

bool A_star::processNode(const int x, const int y, const Node* parent){
    Node node = Node(x, y, parent);
    if(Compare_cord(node, target)){
        closed_set.insert(node);
        return true;
    }
    if(valid_node(node)){
        open_set.push(node);
    }
    return false;
}

void A_star::Search(){
    open_set.insert(start);
    while(!open_set.empty()){
        Node cur = open_set.top();
        open_set.pop();
        // Using 4 connected for now
        //TODO make 8 connected
        if(processNode(cur.x, cur.y+1, &cur) // north
        || processNode(cur.x+1, cur.y, &cur) // east
        || processNode(cur.x, cur.y-1, &cur) // south
        || processNode(cur.x-1, cur.y, &cur) ){ // west
            break;
        }
        closed_set.insert(cur);
    }
    std::vector<Node> path;
    backtracker(path);
}

double A_star::calculateEuclideanDistance(const Node& node1, const Node& node2)
{
    return pow(pow(node1.x - node2.x, 2) - pow(node1.y - node2.y, 2),0.5);
}