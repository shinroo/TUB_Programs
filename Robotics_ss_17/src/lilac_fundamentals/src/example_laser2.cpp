#include "ros/ros.h"
#include <cstdlib>
#include <cmath>
#include "sensor_msgs/LaserScan.h"
#include "DiffDrive.h"
#include "SensorPacket.h"


void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  //ROS_INFO("SIZE>>>>>>>>>>>>  %d", msg->ranges.size());
	double laser_middle = msg->ranges[msg->ranges.size()/2];	//front of the robot
	double laser_right= msg->ranges[0];	//20° left of the robot
	double laser_left= msg->ranges[(msg->ranges.size())-1];	//20° right of the robot
	/**ROS_INFO("left:%f", laser_left);
	ROS_INFO("middle:%f", laser_middle);
	ROS_INFO("right:%f", laser_right);**/

}

void sensorCallback(const create_fundamentals::SensorPacket::ConstPtr& msg)
{
	ROS_INFO("left encoder: %f, right encoder: %f", msg->encoderLeft, msg->encoderRight);
	ROS_INFO("wallsignal: %d", msg->wallSignal);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "example");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("scan_filtered", 1, laserCallback);
  ros::Subscriber sub2 = n.subscribe("sensor_packet", 1, sensorCallback);
  ros::spin();
  return 0;
}
