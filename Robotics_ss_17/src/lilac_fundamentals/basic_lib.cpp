#include <math.h>
#include <cstdlib>
#include <sstream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "create_fundamentals/DiffDrive.h"

#DEFINE WHEEL_RAD 1
#DEFINE ROBOT_CIRC 62.8

// distance in meters, velocity in m/s
int move(double distance, double velocity){
	// ros init
	ros::init(argc, argv, "move");
  	ros::NodeHandle n;
	ros::ServiceClient diffDrive = n.serviceClient<create_fundamentals::DiffDrive>("diff_drive");

	/*
		angular velocity * radius of wheel = velocity
		velocity = distance / time
	*/
	double seconds = distance / velocity;
	double angular_velocity = velocity / WHEEL_RAD;

	// construct ROS_INFO message
	std::ostringstream message;
	message << "diffDrive " << angular_velocity << " " << angular_velocity;
	ROS_INFO(message.str());

	// move
	srv.request.left = angular_velocity;
	srv.request.right = angular_velocity;
	diffDrive.vall(srv);
	ros::Duration(seconds).sleep();

	// stop
	ROS_INFO("diffDrive 0 0");
	srv.request.left = 0;
	srv.request.right = 0;
	diffDrive.call(srv);

	// call callbacks - we have none, but whatever
	ros::spin();

	return 0;
}

// angle in radians, velocity in rad/s
int rotate(double angle, double velocity){
	// ros init
        ros::init(argc, argv, "move");
        ros::NodeHandle n;
        ros::ServiceClient diffDrive = n.serviceClient<create_fundamentals::DiffDrive>("diff_drive");

	// time to rotate = (desired angle * robot circumference) / (2 * pi * velocity of wheel * radius of wheel)
	double seconds =  (angle * ROBOT_CIRC) / (2 * M_PI * velocity * WHEEL_RAD);

	// construct ROS_INFO message
        std::ostringstream message;
        message << "diffDrive -" << velocity << " " << angular_velocity;
        ROS_INFO(message.str());

	// move
        srv.request.left = -1 *velocity;
        srv.request.right = velocity;
        diffDrive.vall(srv);
        ros::Duration(seconds).sleep();

	// stop
        ROS_INFO("diffDrive 0 0");
        srv.request.left = 0;
        srv.request.right = 0;
        diffDrive.call(srv);

        // call callbacks - we have none, but whatever
        ros::spin();

	return 0
}
