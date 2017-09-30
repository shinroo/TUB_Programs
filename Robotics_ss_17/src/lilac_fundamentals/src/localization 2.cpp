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

	bool Top;
	bool Bot;
	bool Left;
	bool Right;

	int visibleWalls;
};

class Localization
{
	public:
		Localization();
		
		bool laser;
		double distanceFront;
		double distanceRight;
		double distanceLeft;
		int counter;

		std::vector<struct point> points;
		std::vector<struct model> walls;
		
		ros::NodeHandle *rosnode;

		ros::ServiceClient diffDrive;
		
		
		create_fundamentals::DiffDrive msg_forward;
		create_fundamentals::DiffDrive msg_backward;
		create_fundamentals::DiffDrive msg_stop;
		create_fundamentals::DiffDrive msg_turn_left;
		create_fundamentals::DiffDrive msg_turn_right;

		void laserCallbackRotate(const sensor_msgs::LaserScan::ConstPtr& msg);
		void laserCallbackDistance(const sensor_msgs::LaserScan::ConstPtr& msg);

	
		void align();
		
		
} l1;

Localization::Localization()
{	
	msg_turn_right.request.left = 3;
	msg_turn_right.request.right = -3;

	msg_forward.request.left = 3;
	msg_forward.request.right = 3;

	msg_backward.request.left = -3;
	msg_backward.request.right = -3;

	msg_stop.request.left = 0;
	msg_stop.request.right = 0;

	msg_turn_left.request.left = -3;
	msg_turn_left.request.right = 3;

	counter = 0;
	
}

void Localization::laserCallbackPerspective(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	for(int i = 0; i < 512; i++){
		double dist = msg->ranges[i];
				
		if(!isnan(dist) && dist != 0.0){
			points.push_back(point());
			points.at(counter).x = dist * cos(((M_PI / 511.0) * i) + (M_PI/2 * position));
			points.at(counter).y = dist * sin(((M_PI / 511.0) * i) + (M_PI/2 * position));
			
			counter++;
		}
	}
	laser = true;
	
}

RobotPerspective Localization::getPerspective(){
	points.clear();
	walls.clear();
	RobotPerspective rp = RobotPerspective();
	laser = false;
	ros::Subscriber sub2 = rosnode->subscribe("scan_filtered", 1, &Localization::laserCallbackPerspective, this);
	diffDrive.call(msg_turn_left);
	while(!laser){
		ros::spinOnce();
    }
	return rp;
}


void Localization::laserCallbackRotate(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	ROS_INFO("105 >>>>  %f   90 >>>>  %f   75 >>>>  %f", msg->ranges[296], msg->ranges[256], msg->ranges[216]);
	if(!isnan(msg->ranges[256]) && !isnan(msg->ranges[216]) && !isnan(msg->ranges[296])){
		if(std::abs(msg->ranges[216] - msg->ranges[296]) < 0.01 && 
			std::abs(msg->ranges[216] - msg->ranges[256] / cos(40/512 * M_PI)) < 0.015){
	ROS_INFO("abs(75 - 105) >>>>  %f   abs2() >>>>  %f", std::abs(msg->ranges[216] - msg->ranges[296]), std::abs(msg->ranges[216] - msg->ranges[256] / cos(40/512 * M_PI)));
			diffDrive.call(msg_stop);
			laser = true;
			distanceFront = msg->ranges[256];
		}
	}
}

void Localization::laserCallbackDistance(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	//ROS_INFO("105 >>>>  %f   90 >>>>  %f   75 >>>>  %f", msg->ranges[296], msg->ranges[256], msg->ranges[216]);
	if(!isnan(msg->ranges[256])){
		if(std::abs(msg->ranges[256] - 0.26) < 0.015){
	
			diffDrive.call(msg_stop);
			laser = true;
			distanceFront = msg->ranges[256];
			if(!isnan(msg->ranges[0])){
				distanceRight = msg->ranges[0];
			} else{
				distanceRight = -1.0;
			}
			if(!isnan(msg->ranges[msg->ranges.size()-1])){
				distanceLeft = msg->ranges[msg->ranges.size()-1];
			} else{
				distanceLeft = -1.0;
			}
			ROS_INFO("distanceFront >>>>  %f   distanceLeft >>>>  %f   distanceRight >>>>  %f", distanceFront, distanceLeft, distanceRight);
		}
	}
	
}



	
void Localization::align()
{
	laser = false;
 	
	ros::Subscriber sub2 = rosnode->subscribe("scan_filtered", 1, &Localization::laserCallbackRotate, this);
	diffDrive.call(msg_turn_left);
	while(!laser){
		ros::spinOnce();
    }
    ROS_INFO("Aligned to wall");
    laser = false;
	sub2 = rosnode->subscribe("scan_filtered", 1, &Localization::laserCallbackDistance, this);
    if(distanceFront > 0.25){
	ROS_INFO("distanceFront forward %f", distanceFront);
    	diffDrive.call(msg_forward);
    	while(!laser){
			ros::spinOnce();
    	}
    } else if(distanceFront < 0.25){
	ROS_INFO("distanceFront backward %f", distanceFront);
    	diffDrive.call(msg_backward);
    	while(!laser){
			ros::spinOnce();
    	}
    }

    laser = false;
		sub2 = rosnode->subscribe("scan_filtered", 1, &Localization::laserCallbackRotate, this);
		diffDrive.call(msg_turn_right);
		while(!laser){
			ros::spinOnce();
    	}

    	laser = false;
		sub2 = rosnode->subscribe("scan_filtered", 1, &Localization::laserCallbackDistance, this);
    	if(distanceFront > 0.25){
    		diffDrive.call(msg_forward);
    		while(!laser){
				ros::spinOnce();
    		}
    	} else if(distanceFront < 0.27){
    		diffDrive.call(msg_backward);
    		while(!laser){
				ros::spinOnce();
    		}
    	}



    if(distanceLeft > 0.0){
    	laser = false;
		sub2 = rosnode->subscribe("scan_filtered", 1, &Localization::laserCallbackRotate, this);
		diffDrive.call(msg_turn_left);
		ros::Duration(1.5).sleep();
		while(!laser){
			ros::spinOnce();
    	}

    	laser = false;
		sub2 = rosnode->subscribe("scan_filtered", 1, &Localization::laserCallbackDistance, this);
    	if(distanceFront > 0.25){
    		diffDrive.call(msg_forward);
    		while(!laser){
				ros::spinOnce();
    		}
    	} else if(distanceFront < 0.27){
    		diffDrive.call(msg_backward);
    		while(!laser){
				ros::spinOnce();
    		}
    	}

    	laser = false;
		sub2 = rosnode->subscribe("scan_filtered", 1, &Localization::laserCallbackRotate, this);
		diffDrive.call(msg_turn_right);
		while(!laser){
			ros::spinOnce();
    	}

    	laser = false;
		sub2 = rosnode->subscribe("scan_filtered", 1, &Localization::laserCallbackDistance, this);
    	if(distanceFront > 0.25){
    		diffDrive.call(msg_forward);
    		while(!laser){
				ros::spinOnce();
    		}
    	} else if(distanceFront < 0.27){
    		diffDrive.call(msg_backward);
    		while(!laser){
				ros::spinOnce();
    		}
    	}
    } else if(distanceRight > 0.0){
    	laser = false;
		sub2 = rosnode->subscribe("scan_filtered", 1, &Localization::laserCallbackRotate, this);
		ros::Duration(1.5).sleep();
		diffDrive.call(msg_turn_right);
		while(!laser){
			ros::spinOnce();
    	}

    	laser = false;
		sub2 = rosnode->subscribe("scan_filtered", 1, &Localization::laserCallbackDistance, this);
    	if(distanceFront > 0.25){
    		diffDrive.call(msg_forward);
    		while(!laser){
				ros::spinOnce();
    		}
    	} else if(distanceFront < 0.27){
    		diffDrive.call(msg_backward);
    		while(!laser){
				ros::spinOnce();
    		}
    	}

    	laser = false;
		sub2 = rosnode->subscribe("scan_filtered", 1, &Localization::laserCallbackRotate, this);
		diffDrive.call(msg_turn_left);
		while(!laser){
			ros::spinOnce();
    	}

    	laser = false;
		sub2 = rosnode->subscribe("scan_filtered", 1, &Localization::laserCallbackDistance, this);
    	if(distanceFront > 0.25){
    		diffDrive.call(msg_forward);
    		while(!laser){
				ros::spinOnce();
    		}
    	} else if(distanceFront < 0.27){
    		diffDrive.call(msg_backward);
    		while(!laser){
				ros::spinOnce();
    		}
    	}
    }
}






int main(int argc, char **argv)
{
	ros::init(argc, argv, "Localization");
	ros::NodeHandle n;
	Localization l1;
	l1.rosnode = &n;
	
	//ros::Subscriber sub2 = n.subscribe("sensor_packet", 1, &Localization::sensorCallback, &a1);
	l1.diffDrive = n.serviceClient<create_fundamentals::DiffDrive>("diff_drive");
	create_fundamentals::DiffDrive srv;
	
	l1.align();

	
	return 0;
}


			
			
