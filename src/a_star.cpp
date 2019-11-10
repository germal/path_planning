#include "a_star.h"

A_Star::Node::Node(const int x_in, const int y_in, const Node* parent){
    x = x_in;
    y = y_in;
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

bool A_Star::Compare_coord::operator()(const Node& lhs, const Node& rhs){
    return rhs.x == lhs.x && rhs.y == rhs.y;
}

bool A_Star::Compare_f_cost::operator()(const Node& node1, const Node& node2){
    return node1.f() < node2.f();
}

bool A_Star::Compare_g_cost::operator()(const Node& node1, const Node& node2){
    return node1.g < node2.g;
}

A_star::A_star(){}

void A_star::Backtracker(){
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

bool A_star::processNode(const int x, const int y, const Node* parent){
    Node node = Node(x, y, parent);
    if(Compare_coord(node, target)){
        closed_set.insert(node);
        return true;
    }
    if(valid_node(node)){
        open_set.push(node);
    }
    return false;
}

bool A_star::Search(){
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
    Backtracker();
    return true;
}

double A_star::calculateEuclideanDistance(const Node& node1, const Node& node2){
    return pow(pow(node1.x - node2.x, 2) - pow(node1.y - node2.y, 2),0.5);
}

void A_star::gpsCallback(const nav_msgs::Odometry::ConstPtr& msg){
    // Fill out the needed "target" position member variable using info from the odom message
    target = Node(msg->pose.pose.position.x, msg->pose.pose.position.y, nullptr);
	
	// Fill out the member variable that stores the current pose
	currentPose = msg->pose.pose;
}

void A_star::costmapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg){
	// Fill out the costmap width and height from the occupancy grid info message
    int costmap_width = msg->info.width;
    int costmap_height = msg->info.height;

    // Fill out the needed "start" position member variable using info from the costmap origin message
    start = Node(msg->info.origin.orientation.x, msg->info.origin.orientation.y, nullptr); // why is this here and in gps?

    // Fill out the costmap member variable using info from the occupancy grid costmap message
    for(int i = 0; i < costmap_width; i++){
        for(int j = 0; j < costmap_height; j++){
            cost_map[i][j] = msg->data[i + (j * costmap_width)];
        }
    }
}
