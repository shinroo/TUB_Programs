#include "ros/ros.h"
#include <cstdlib>
#include <cmath>
#include "sensor_msgs/LaserScan.h"
#include "DiffDrive.h"
#include "SensorPacket.h"
#include <math.h>
#include <float.h>
#include <time.h>
#include <algorithm> 
#include <vector>
#include "lilac_fundamentals/Cell.h"
#include "lilac_fundamentals/Grid.h"
#include "lilac_fundamentals/Row.h"
#include "lilac_fundamentals/ExecutePlan.h"
#include "lilac_fundamentals/ActualLocalization.h"

#define WHEEL_RAD 3
#define ROBOT_DIAMETER 26.5

struct point    
  {
    double x;
    double y;
  };

struct model    
  {
    double m;
    double c;
  };

struct MapPoint    
{
	int x;
	int y;
};

struct RobotPerspective
{
	MapPoint m;
	int globalPerspective;

	bool Front;
	bool Left;
	bool Right;

	int visibleWalls;
};

struct Cell
{
	MapPoint m;
	
	bool Top;
	bool Right;
	bool Left;
	bool Bottom;
};

void localizationCallback(const lilac_fundamentals::ActualLocalization::ConstPtr&  msg)
{
   	ROS_INFO("row %d, column %d, orientation %d, localized %d", msg->row, msg->column, msg->orientation, msg->localized);

}





int main(int argc, char **argv)
{
	ros::init(argc, argv, "LocalizationSubscriber");
	ros::NodeHandle n;
	
	ros::Subscriber sub = n.subscribe("actual_localization", 1000, localizationCallback);
	
	ros::spin();
	
	return 0;
}


			
			
