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







int main(int argc, char **argv)
{
	ros::init(argc, argv, "LocalizationPublisher");
	ros::NodeHandle n;
	
	ros::Publisher localizationPublisher = n.advertise<lilac_fundamentals::ActualLocalization>("actual_localization", 1000);
	
	//ros::Subscriber sub2 = n.subscribe("sensor_packet", 1, &Localization::sensorCallback, &a1);
	
	RobotPerspective actual = RobotPerspective();
	actual.m.x = 0;
	actual.m.y = 1;
	actual.globalPerspective = 1;
	bool localized1 = false;
	
	int counter = 0;
	while (counter < 3){
		lilac_fundamentals::ActualLocalization msg;


		if(counter == 1){
			actual.m.x = 3;
			actual.m.y = 4;
			actual.globalPerspective = 3;
			localized1 = true;
		}

		msg.row = actual.m.x;
		msg.column = actual.m.y;
		msg.orientation = actual.globalPerspective;
		msg.localized = localized1;

		

		ROS_INFO("row %d, column %d, orientation %d, localized %d", msg.row, msg.column, msg.orientation, msg.localized);


		localizationPublisher.publish(msg);
		ros::spinOnce();
		ros::Duration(0.5).sleep();
		counter++;

	}
	
	
	
	return 0;
}


			
			
