#include "a_star.h"

const std::string kGPS_topic = "/odometry/filtered";
const std::string kCostmap_topic = "move_base/local_costmap/costmap";

using namespace std;

class calculate_path{
private:
	ros::Subscriber &gps;
	ros::Subscriber &costmap;
	A_star &listener;
public:
	vector<position> solution_path;
	calculate_path(ros::Subscriber &gps_in, ros::Subscriber &costmap_in, A_star &a_star)
		: gps(gps_in), costmap(costmap_in), listener(a_star) { }
	void operator()(const ros::TimerEvent&);
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "listener");

	ros::NodeHandle n;

	bool at_target = false;

	A_star listener = A_star{};

	ros::Subscriber gps_sub = n.subscribe(kGPS_topic, 1000, &GoatControl::gpsCallback, &listener);
	ros::Subscriber costmap_sub = n.subscribe(kCostmap_topic, 1000, &GoatControl::costmapCallback, &listener);

	// Calculated path vector can be accessed via path.solution_path
	calculate_path path(gps_sub, costmap_sub);
	
	// Update coordinates once before calculating path
	ros::spinOnce();

	// Start running A* at a frequency specified by period to keep updating path
	// as the area is traversed
	const double period = 0.05;
	ros::Timer timer = n.createTimer(ros::Duration(period), path);
	timer.start();

	// Keep spinning until you reach the target
    ros::Rate rate(10); // 10 hz
	while(!at_target)
	{
		ros::spinOnce();
        rate.sleep();
    }

	// Stop running timer that calculates path
	timer.stop();

	// TO DO: publish path once it is calculated

	// TO DO: look into how to determine whether target is reached (at_target
	// currently stays false)

	return 0;
}

void calculate_path::operator()(const ros::TimerEvent&) {
	// Placeholders for the coordinates of the starting position
	unsigned int start_x = listener.start.x;
	unsigned int start_y = listener.start.y;

	// Placeholders for gps coordinates for the target
	unsigned int gps_target_x = listener.target.x;
	unsigned int gps_target_y = listener.target.y;

	// Create a position object that holds the starting position
	position start(start_x, start_y);

	// Create a position object that holds the position of the target
	position target(gps_target_x, gps_target_y);

	// Attempt to find a solution
	bool foundtarget = listener.Search();

}