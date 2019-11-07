#include "a_star.h"

const std::string kGPS_topic = "/odometry/filtered";
const std::string kCostmap_topic = "move_base/local_costmap/costmap";

using namespace std;

class calculate_path{
private:
	ros::Subscriber &gps;
	ros::Subscriber &costmap;
	GoatControl &listener;
public:
	vector<position> solution_path;
	calculate_path(ros::Subscriber &gps_in, ros::Subscriber &costmap_in, A_star &a_star)
		: gps(gps_in), costmap(costmap_in), listener(a_star) { }
	void operator()(const ros::TimerEvent&);
};