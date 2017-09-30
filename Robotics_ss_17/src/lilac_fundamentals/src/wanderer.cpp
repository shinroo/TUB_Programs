#include "ros/ros.h"
#include <cstdlib>
#include <cmath>
#include "sensor_msgs/LaserScan.h"
#include "DiffDrive.h"
#include "SensorPacket.h"

class Wanderer
{
	public:
		Wanderer();
		int counter;
		
		double laser[512];
		int collision;

		ros::ServiceClient diffDrive;
		
		
		create_fundamentals::DiffDrive msg_forward;
		create_fundamentals::DiffDrive msg_stop;
		create_fundamentals::DiffDrive msg_turn_left;
		create_fundamentals::DiffDrive msg_turn_right;
  	
		void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
		void sensorCallback(const create_fundamentals::SensorPacket::ConstPtr& msg);
} w1;

Wanderer::Wanderer()
{	
	msg_turn_right.request.left = -1;
	msg_turn_right.request.right = 1;

	msg_forward.request.left = 3;
	msg_forward.request.right = 3;

	msg_stop.request.left = 0;
	msg_stop.request.right = 0;

	msg_turn_left.request.left = 1;
	msg_turn_left.request.right = -1;
}


void Wanderer::laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	
	
	//ros::ServiceClient diffDrive = n.serviceClient<create_fundamentals::DiffDrive>("diff_drive");
	//std::copy(msg->ranges, msg->ranges + 512, laser);
	/**for(int j = 0; j < msg->ranges.size(); j++){
		laser[j]=msg->ranges[j];
	}
	laser_middle = msg->ranges[msg->ranges.size()/2];	//front of the robot
	laser_right= msg->ranges[(msg->ranges.size()/18)*2];	//20° left of the robot
	laser_left= msg->ranges[(msg->ranges.size()/18)*16];	//20° right of the robot
	/**ROS_INFO("<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<SIZE:%d", msg->ranges.size());
	ROS_INFO("left:%f", laser_left);
	ROS_INFO("middle:%f", laser_middle);
	ROS_INFO("right:%f", laser_right);

	// TODO:
	// 	* if sensor finds object (#include <cmath>) !isnan(double value)
	//	* stop -> turn right for some time -> drive forward
	//	* return (end this function)
	
		ROS_INFO("left:%f", w1.laser_left);
		ROS_INFO("middle:%f", w1.laser_middle);
		ROS_INFO("right:%f", w1.laser_right);**/
		int free = 1;
		
		for(int i = (512/180)*30; i < (512/180)*150; i++)	
		{
			double dist = msg->ranges[i];
			
			
			if(!isnan(dist) && dist != 0.0 && dist < 0.3){
				free = 0;
				if(collision == 0 && i <= 256)
				{
					
					diffDrive.call(msg_turn_right);
					//ros::Duration(1.0).sleep();
					//w1.diffDrive.call(w1.msg_forward);
					collision = 1;
					/**ROS_INFO("collision:%d", collision);
					ROS_INFO("dist:%f", dist);
					ROS_INFO("range:%d    angle:%d", i, (512/180)*i);**/
						
				}

				if(collision == 0 && i > 256)
				{
					
					diffDrive.call(msg_turn_left);
					//ros::Duration(1.0).sleep();
					//w1.diffDrive.call(w1.msg_forward);
					collision = 2;	
					/**ROS_INFO("collision:%d", collision);
					ROS_INFO("dist:%f", dist);
					ROS_INFO("range:%d    angle:%d", i, (512/180)*i);**/
					
				}
			}
			if(free == 0 || collision != 0){
				break;
			}
		} 
		if(collision != 0 && free == 1){
			
			diffDrive.call(w1.msg_forward);
			//ROS_INFO(">>>>>>>>>>>>>>>>>collision:%d", collision);
			collision = 0;
			//ROS_INFO("dist:%f", dist);
		}
	
	}

	


void Wanderer::sensorCallback(const create_fundamentals::SensorPacket::ConstPtr& msg)
{
	ROS_INFO("left encoder: %f, right encoder: %f", msg->encoderLeft, msg->encoderRight);
	ROS_INFO("wallsignal: %d", msg->wallSignal);
}


int main(int argc, char **argv)
{
	Wanderer w1;
	ros::init(argc, argv, "Wanderer");
	ros::NodeHandle n;
	w1.collision = 0;
	
	ros::Subscriber sub2 = n.subscribe("sensor_packet", 1, &Wanderer::sensorCallback, &w1);
	w1.diffDrive = n.serviceClient<create_fundamentals::DiffDrive>("diff_drive");
	
	create_fundamentals::DiffDrive srv;
	
	// TODO: start drive forward ...
	w1.diffDrive.call(w1.msg_forward);
	ros::Subscriber sub1 = n.subscribe("scan_filtered", 1, &Wanderer::laserCallback, &w1);
	
	//w1.diffDrive.call(w1.msg_stop);
	
	ros::spin();
	w1.diffDrive.call(w1.msg_stop);
	return 0;
}


			
			/**
			if(laser_middle < 0.3)
			{
				diffDrive.call(msg_stop);
				diffDrive.call(msg_turn_right);
				ros::Duration(1.0).sleep();
				diffDrive.call(msg_forward);	
			}	
	
			if(laser_right < 0.2)
			{	
				diffDrive.call(msg_stop);
				diffDrive.call(msg_turn_left);
				ros::Duration(1.0).sleep();
				diffDrive.call(msg_forward);
				
			}
		
			if(laser_left < 0.2)
			{
				diffDrive.call(msg_stop);
				diffDrive.call(msg_turn_right);
				ros::Duration(1.0).sleep();
				diffDrive.call(msg_forward);
					
			}
			**/
