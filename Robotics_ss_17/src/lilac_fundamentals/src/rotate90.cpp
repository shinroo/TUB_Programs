#include <math.h>
#include <cstdlib>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "create_fundamentals/DiffDrive.h"

#define WHEEL_RAD 3.7
#define ROBOT_CIRC 83.25

int main(int argc, char **argv){

	double angle = M_PI/2;
	double velocity = 3;

	ros::init(argc, argv, "rotate90");
	ros::NodeHandle n;
	ros::ServiceClient diffDrive = n.serviceClient<create_fundamentals::DiffDrive>("diff_drive");
	create_fundamentals::DiffDrive srv;

	double seconds = (angle* ROBOT_CIRC) / (2 * M_PI * velocity * WHEEL_RAD);

	srv.request.left = -1 * velocity;
	srv.request.right = velocity;
	diffDrive.call(srv);

	ros::Duration(seconds).sleep();

	srv.request.left = 0;
	srv.request.right = 0;
	diffDrive.call(srv);

	return 0;
}