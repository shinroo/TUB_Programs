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

#include "lilac_fundamentals/Pose.h"
#include "lilac_fundamentals/Position.h"



class PoseAmp {

	public: 
		PoseAmp();

		int actualRow;
		int actualColumn;
		
		bool valid;
		
		ros::NodeHandle *rosnode;
		ros::Publisher posePublisher;
		lilac_fundamentals::Position msgPosition;

		
		ros::Subscriber poseSubscriber;

		
		void poseCallback(const lilac_fundamentals::Pose::ConstPtr&  msg);

};

PoseAmp::PoseAmp(){
	valid = false;
}



void PoseAmp::poseCallback(const lilac_fundamentals::Pose::ConstPtr&  msg)
{

	ROS_INFO("Pose is (%d, %d)",msg->row , msg->column);
	if(msg->row >= 0 && msg->column >= 0){
		actualRow = msg->row;
		actualColumn = msg->column;
		valid = true;
	} else{
		actualRow = msg->row;
		actualColumn = msg->column;
		valid = true;
	}
		
		//listenToPose = false;
	
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "PoseAmp");
	ros::NodeHandle n;
	PoseAmp m1;
	m1.rosnode = &n;
	
	m1.poseSubscriber = n.subscribe("pose", 1, &PoseAmp::poseCallback, &m1);
	m1.posePublisher = n.advertise<lilac_fundamentals::Position>("position", 1000);
	
	while(ros::ok()){
		if(m1.valid){
			m1.msgPosition.row = m1.actualRow;
			m1.msgPosition.column = m1.actualColumn;
			m1.posePublisher.publish(m1.msgPosition);
		}
		ros::spinOnce();
	}
	
	
	return 0;
}

			
			
