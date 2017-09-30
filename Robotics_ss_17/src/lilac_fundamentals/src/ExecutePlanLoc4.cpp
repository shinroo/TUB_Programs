#include "ros/ros.h"
#include <cstdlib>
#include <cmath>
#include "sensor_msgs/LaserScan.h"
#include "DiffDrive.h"
#include <math.h>
#include <float.h>
#include <time.h>
#include "lilac_fundamentals/ExecutePlan.h"
#include <algorithm> 
#include <vector>
#include <time.h> 
#include "create_fundamentals/SensorPacket.h"
#include "create_fundamentals/ResetEncoders.h"
#include "lilac_fundamentals/ActualLocalization.h"
#include "lilac_fundamentals/Pose.h"
#include "lilac_fundamentals/Cell.h"
#include "lilac_fundamentals/Grid.h"
#include "lilac_fundamentals/Row.h"
#include <iostream>
#include <chrono>
#include <thread>

#define INVALID -1
#define WHEEL_RAD 0.03
#define ROBOT_DIAMETER 0.265
#define ROBOT_VELOCITY 10.0
#define ROBOT_TURN_VELOCITY 3.0
#define ROBOT_ERROR 9.15
#define ITERATIONS_PER_MOVEMENT 40

class timer
{
    // alias our types for simplicity
    using clock             = std::chrono::system_clock;
    using time_point_type   = std::chrono::time_point < clock, std::chrono::milliseconds > ;
public:
    // default constructor that stores the start time
    timer()
    {
        start = std::chrono::time_point_cast<std::chrono::milliseconds>(clock::now());
    }

    // gets the time elapsed from construction.
    long /*milliseconds*/ getTimePassed()
    {
        // get the new time
        auto end = clock::now();

        // return the difference of the times
        return (end - start).count();
    }

private:
    std::chrono::time_point < clock, std::chrono::milliseconds > start;
};

struct EncoderReading {

	double encoderLeft;
	double encoderRight;
	bool advance;

};

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

struct LaserReading {

	double left;
	double right;
	double front;
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

class Plan
{
	public:
		Plan();
	
		std::vector<int> plan;

		std::vector<RobotPerspective> perspectives;
		std::vector<struct point> points;
		std::vector<struct model> walls;
		
		int orientation;
		MapPoint actualLocation;

		int startrow;
		int startcolumn;
		int endrow;
		int endcolumn;
		double delay;

		bool mapCalculated;
		bool localized;
		bool laser;
		bool aboutToCrash;
		bool encounteredRobot;
		bool moving;

		double distanceFront;
		double distanceRight;
		double distanceLeft;
		int counter;

		bool emergencyStop;

		ros::NodeHandle *rosnode;
		ros::Publisher posePublisher;

		double velLeft;
		double velRight;

		ros::ServiceClient diffDrive;
		ros::ServiceClient resetEncoders;

		ros::Subscriber LaserSubscriber;
		ros::Subscriber EncoderSubscriber;
				
		create_fundamentals::DiffDrive msg_forward;
		create_fundamentals::DiffDrive msg_stop;
		create_fundamentals::DiffDrive msg_turn_left;
		create_fundamentals::DiffDrive msg_turn_right;

		void rotateLeft(double angle);
		void rotateRight(double angle);
		
		void move(double distance);
		void reverse(int n, double subseconds);

		void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
		void encoderCallback(const create_fundamentals::SensorPacket::ConstPtr& msg);
		
		bool executePlan();
		bool execute(lilac_fundamentals::ExecutePlan::Request &req, lilac_fundamentals::ExecutePlan::Response &res);

		std::vector<EncoderReading> encoderReadings;
		std::vector<LaserReading> laserReadings;

		LaserReading getLatestLaser();
		EncoderReading getLatestEncoder();

		void addLaserReading(LaserReading newReading);
		void addEncoderReading(EncoderReading newReading);

		void updateVelocities();

		void mapCallback(const lilac_fundamentals::Grid::ConstPtr& grid_msg);
		void mapToStructs();
		MapPoint calculateNextLocalization(int direction, MapPoint oldLocation);

		RobotPerspective getPerspective();
		void laserCallbackPerspective(const sensor_msgs::LaserScan::ConstPtr& msg);
		double distance(struct point p, struct model m1);
		void ransac();
		bool publishLocation(int direction);
		void localizationCallback(const lilac_fundamentals::ActualLocalization::ConstPtr&  msg);
} p1;

Plan::Plan()
{	
	msg_turn_right.request.left = ROBOT_TURN_VELOCITY;
	msg_turn_right.request.right = -ROBOT_TURN_VELOCITY;

	msg_forward.request.left = ROBOT_VELOCITY;
	msg_forward.request.right = ROBOT_VELOCITY;

	msg_stop.request.left = 0;
	msg_stop.request.right = 0;

	msg_turn_left.request.left = -ROBOT_TURN_VELOCITY;
	msg_turn_left.request.right = ROBOT_TURN_VELOCITY;
	
	orientation = 1;
	actualLocation.x = -1;
	actualLocation.y = -1;
	localized = false;
	mapCalculated = false;
	aboutToCrash = false;
	encounteredRobot = false;
	moving = false;
	delay = -1;
}


void Plan::mapToStructs(){
	ros::Subscriber sub = rosnode->subscribe("map", 1, &Plan::mapCallback, this);
	mapCalculated = false;
	while(!mapCalculated){
		ros::spinOnce();
    }
}

void Plan::mapCallback(const lilac_fundamentals::Grid::ConstPtr& grid_msg){

	if (mapCalculated == true){
		return;
	}

	int rowCount = grid_msg->rows.size();
	int cellCount = grid_msg->rows[0].cells.size();

	for(int row = 0; row < rowCount; row++){

		for (int cell = 0; cell < cellCount; cell++){

			int wallCount = grid_msg->rows[row].cells[cell].walls.size();

			Cell newCell;

			newCell.m.x = row;
			newCell.m.y = cell;

			newCell.Top = false;
			newCell.Right = false;
			newCell.Left = false;
			newCell.Bottom = false;

			for (int wall = 0; wall < wallCount; wall++){

				int walltype = grid_msg->rows[row].cells[cell].walls[wall];
				
				if (walltype == 0){

					//RIGHT
					newCell.Right = true;

				} else if (walltype == 1){

					//TOP
					newCell.Top = true;

				} else if (walltype == 2){

					//LEFT
					newCell.Left = true;

				} else {

					//BOTTOM
					newCell.Bottom = true;

				}

			}

			// Determine perspectives from cell

			RobotPerspective temp;
			temp.m.x = newCell.m.x;
			temp.m.y = newCell.m.y;

			// UP
			temp.globalPerspective = 1;
			
			temp.Front = false;
			temp.Left = false;
			temp.Right = false;

			temp.visibleWalls = 0;

			if (newCell.Top == true){
				temp.Front = true;
				temp.visibleWalls++;
			}	
			if (newCell.Left == true){
				temp.Left = true;
				temp.visibleWalls++;
			}
			if (newCell.Right == true){
				temp.Right = true;
				temp.visibleWalls++;
			}		

			perspectives.push_back(temp);

			// DOWN
			temp.globalPerspective = 3;
			
			temp.Front = false;
			temp.Left = false;
			temp.Right = false;

			temp.visibleWalls = 0;

			if (newCell.Bottom == true){
				temp.Front = true;
				temp.visibleWalls++;
			}	
			if (newCell.Left == true){
				temp.Right = true;
				temp.visibleWalls++;
			}
			if (newCell.Right == true){
				temp.Left = true;
				temp.visibleWalls++;
			}

			perspectives.push_back(temp);

			// LEFT
			temp.globalPerspective = 2;
			
			temp.Front = false;
			temp.Left = false;
			temp.Right = false;

			temp.visibleWalls = 0;

			if (newCell.Top == true){
				temp.Right = true;
				temp.visibleWalls++;
			}	
			if (newCell.Left == true){
				temp.Front = true;
				temp.visibleWalls++;
			}
			if (newCell.Bottom == true){
				temp.Left = true;
				temp.visibleWalls++;
			}

			perspectives.push_back(temp);

			// RIGHT
			temp.globalPerspective = 0;
			
			temp.Front = false;
			temp.Left = false;
			temp.Right = false;

			temp.visibleWalls = 0;

			if (newCell.Top == true){
				temp.Left = true;
				temp.visibleWalls++;
			}	
			if (newCell.Bottom == true){
				temp.Right = true;
				temp.visibleWalls++;
			}
			if (newCell.Right == true){
				temp.Front = true;
				temp.visibleWalls++;
			}

			perspectives.push_back(temp);
		}

	} 

	/*int size = perspectives.size();

	for(int i = 0; i < size; i++){
	
		ROS_INFO("POINT (%d,%d), Perspective %d, Visisble Walls %d, Front %d, Left %d, Right %d", 
				perspectives.at(i).m.x, 
				perspectives.at(i).m.y, 
				perspectives.at(i).globalPerspective,
				perspectives.at(i).visibleWalls,
				perspectives.at(i).Front,
				perspectives.at(i).Left,
				perspectives.at(i).Right);
	}*/

	mapCalculated = true;
}



// #######################################################################################

void Plan::reverse(int n, double subseconds){

	encounteredRobot = true;

	create_fundamentals::DiffDrive msg_reverse;

	for (int i = 0; i < n; i++){
		msg_reverse.request.left = -1 * velLeft;
		msg_reverse.request.right = -1 * velRight;

		diffDrive.call(msg_reverse);
		ros::Duration(subseconds).sleep();
		ros::spinOnce();
	}

	diffDrive.call(msg_stop);

}

// Movement Functions

void Plan::move(double distance){
	ROS_INFO("Move this distance %f",distance);
	moving = true;
	//double movementerror = 30.0;
	double seconds = distance / (ROBOT_VELOCITY * WHEEL_RAD);
	if(delay == -1){
		double movementerror = 25.0;
		delay = (movementerror * (seconds / 100));
	}
	ROS_INFO("\n\n\nDelay %f\n", delay);
	seconds = seconds - delay;

	double subseconds = seconds / ITERATIONS_PER_MOVEMENT;

	int counter = 0;
	int check_value = 0;

	for (int i = 0; i < ITERATIONS_PER_MOVEMENT; i++) {

		if (aboutToCrash){
			RobotPerspective rp = getPerspective();
			if(!rp.Front){
				ROS_INFO("The last one found a robot");
				diffDrive.call(msg_stop);
				reverse(counter, subseconds);
				break;
			}
			
		}

		create_fundamentals::DiffDrive msg_submove;

		msg_submove.request.left = velLeft;
		msg_submove.request.right = velRight;

		diffDrive.call(msg_submove);
		ros::Duration(subseconds).sleep();
		ros::spinOnce();

		counter++;

		if (aboutToCrash){
			diffDrive.call(msg_stop);
			reverse(counter, subseconds);
			break;
		}

	}

	msg_forward.request.left = ROBOT_VELOCITY;
	msg_forward.request.right = ROBOT_VELOCITY;
    timer t;
	diffDrive.call(msg_stop);
	EncoderReading er = getLatestEncoder();
	while(er.advance){
		ros::spinOnce();
		er = getLatestEncoder();
	}
	delay = (double) t.getTimePassed();
    delay = delay/1000000000;
	moving = false;
}

void Plan::rotateRight(double angle){
    	double seconds =  (angle * ROBOT_DIAMETER * M_PI) / (2 * M_PI * ROBOT_TURN_VELOCITY * WHEEL_RAD);
	seconds = seconds - (ROBOT_ERROR * (seconds / 100));

	diffDrive.call(msg_turn_right);
        ros::Duration(seconds).sleep();

        diffDrive.call(msg_stop);
}

void Plan::rotateLeft(double angle){
    	double seconds =  (angle * ROBOT_DIAMETER * M_PI) / (2 * M_PI * ROBOT_TURN_VELOCITY * WHEEL_RAD);
	seconds = seconds - (ROBOT_ERROR * (seconds / 100));
	
	diffDrive.call(msg_turn_left);
        ros::Duration(seconds).sleep();
        
	diffDrive.call(msg_stop);
}

// #######################################################################################

// Plan functions

void Plan::laserCallbackPerspective(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	distanceFront = -1;
	distanceRight = -1;
	distanceLeft = -1;
	counter = 0;
	for(int i = 0; i < 512; i++){
		double dist = msg->ranges[i];
				
		if(!isnan(dist) && dist <= 0.65){
			points.push_back(point());
			points.at(counter).x = dist * cos(((M_PI / 511.0) * i));
			points.at(counter).y = dist * sin(((M_PI / 511.0) * i));
			counter++;
			if(i == 256){
				distanceFront = dist;
			} 
			else if(i == 0){
				distanceRight = dist;
			}
			else if(i == 511){
				distanceLeft = dist;
			}
		}
	}
	laser = true;
	
}

double Plan::distance(struct point p, struct model m1){
	double temp = m1.m * p.x - p.y + m1.c ; 
	if (temp < 0.0 ) temp = - temp ; 
	double racine = sqrt (m1.m * m1.m + 1.0*1.0);
	double distance = temp / racine ; 
	return distance;
}

void Plan::ransac()
{
	//ROS_INFO("RANSAC TIME");
	srand (time(NULL));
	while(walls.size() < 3 && points.size() > 50){
		int iterations = 0;
		model bestfit;
		int besterr = 0; ///Besterror is the amount of points fitting the model 
		//ROS_INFO("RANSAC Nr %d", walls.size());
		while(iterations < 1000){
			point p1 = points.at(rand() % (points.size()-1) + 0);
			point p2 = points.at(rand() % (points.size()-1) + 0);
			model maybemodel;
			int thiserr = 0;
			maybemodel.m = (p2.y - p1.y)/(p2.x - p1.x);
			maybemodel.c = -maybemodel.m * p1.x + p1.y;
			for(int i = 0; i < points.size(); i++){
				if(distance(points.at(i), maybemodel) < 0.02){
					thiserr++;
				}
			}
			if((std::abs(maybemodel.m) <= 0.2 && thiserr > 90 && thiserr > besterr) || (std::abs(maybemodel.m) > 0.2 && thiserr > 20 && thiserr > besterr)){
				//ROS_INFO("New Bestmodel! %d    error: %d", iterations, thiserr);
				bestfit = maybemodel;
				besterr = thiserr;
			}
			iterations++;
			//ROS_INFO("Model worked %d    error: %d", iterations, thiserr);
		}
		if(besterr >= 20){
			walls.push_back(bestfit);
			counter = 0;
			std::vector<struct point> newPoints;
			for(int i = 0; i < points.size(); i++){			
				if(distance(points.at(i), bestfit) >= 0.02){
					newPoints.push_back(point());
					newPoints[counter].x = points.at(i).x;
					newPoints[counter].y =  points.at(i).y;
					counter++;
				}	
			}
			points = newPoints;
			ROS_INFO("bestfit: %f *x + %f  error: %d    >>>WALLS: %lu", bestfit.m, bestfit.c, besterr, walls.size());
			//ROS_INFO("New points size: %d", points.size());	
		}else{
			//ROS_INFO("No acceptable bestfit found");
			break;
		}
	}

	points.clear();	
	
	
}


RobotPerspective Plan::getPerspective(){
	points.clear();
	walls.clear();
	RobotPerspective rp = RobotPerspective();
	rp.visibleWalls = 0;
	rp.Front = false;
	rp.Left = false;
	rp.Right = false;

	laser = false;
	ros::Subscriber sub2 = rosnode->subscribe("scan_filtered", 1, &Plan::laserCallbackPerspective, this);
	
	while(!laser){
		ros::spinOnce();
    }

    ransac();
    rp.visibleWalls = walls.size();
    for (int i = 0; i < walls.size(); i++)
    {
    	if(std::abs(walls.at(i).c) < 0.65 && distanceFront != -1 && distanceFront < 0.65){
    		rp.Front = true;
    	}
    	double y0 = -walls.at(i).c/walls.at(i).m;
    	if(std::abs(y0) < 0.65){
    		if(y0 > 0.0){
    			if(distanceRight != -1 && distanceRight < 0.65){
    				rp.Right = true;
    			} else{
    				rp.visibleWalls -= 1;
    			}
    		} else if(y0 < 0.0){
    			if(distanceLeft != -1 && distanceLeft < 0.65){
    				rp.Left = true;
    			} else{
    				rp.visibleWalls -= 1;
    			}
    		}
    	}
    }


    walls.clear();
    ROS_INFO("\nGET PERSPECTIVE visibleWalls  >>>>  %d  front  >>>>  %d  left  >>>>  %d  right  >>>>  %d",rp.visibleWalls,
    	rp.Front, rp.Left, rp.Right);
    return rp;
}


bool Plan::publishLocation(int direction){
	MapPoint newCellLocation = calculateNextLocalization(direction, actualLocation);
	bool correct = false;
	RobotPerspective actualPerspective = getPerspective();

	for (int k = 0; k < perspectives.size(); k++)
	{
		if(perspectives.at(k).m.x == newCellLocation.x && perspectives.at(k).m.y == newCellLocation.y
			&& perspectives.at(k).globalPerspective == direction){
			if(actualPerspective.Front == perspectives.at(k).Front && 
				actualPerspective.Left == perspectives.at(k).Left &&
				actualPerspective.Right == perspectives.at(k).Right){
				ROS_INFO("BEFORE (%d,%d)   we are in the right cell (%d,%d)",actualLocation.x, actualLocation.y, newCellLocation.x, newCellLocation.y);
				correct = true;
			}
			k = perspectives.size()+1000;				
		}
	}
	if(!correct){
		move(0.07);
		actualPerspective = getPerspective();

	for (int k = 0; k < perspectives.size(); k++)
	{
		if(perspectives.at(k).m.x == newCellLocation.x && perspectives.at(k).m.y == newCellLocation.y
			&& perspectives.at(k).globalPerspective == direction){
			if(actualPerspective.Front == perspectives.at(k).Front && 
				actualPerspective.Left == perspectives.at(k).Left &&
				actualPerspective.Right == perspectives.at(k).Right){
				ROS_INFO("BEFORE (%d,%d)   we are in the right cell (%d,%d)",actualLocation.x, actualLocation.y, newCellLocation.x, newCellLocation.y);
				correct = true;
			}
			k = perspectives.size()+1000;				
		}
	}
	}

	
	if(correct){
		actualLocation.x = newCellLocation.x;
		actualLocation.y = newCellLocation.y;
		
		int count3 = 0;
		while (count3 < 1){
			

			lilac_fundamentals::Pose msgPose;

			
			msgPose.row = actualLocation.x;
			msgPose.column = actualLocation.y;
			msgPose.orientation = orientation;

			posePublisher.publish(msgPose);
			ros::spinOnce();
			ros::Duration(0.5).sleep();
			count3++;
		}
	}else{
		actualLocation.x = -2;
		actualLocation.y = -2;
		
		int count3 = 0;
		while (count3 < 1){
			

			lilac_fundamentals::Pose msgPose;

			
			msgPose.row = actualLocation.x;
			msgPose.column = actualLocation.y;
			msgPose.orientation = orientation;

			posePublisher.publish(msgPose);
			ros::spinOnce();
			ros::Duration(0.5).sleep();
			count3++;
			localized = false;
		}
	}
	return correct;			
}




bool Plan::executePlan()
{
	bool success = true;
	aboutToCrash = false;
    encounteredRobot = false;

	//RobotPerspective temp = getPerspective();
	//if (temp.Front){
	//	success = false;
	//	return success;
	//}

	for(int i = 0; i < plan.size(); i++){
		LaserReading current = getLatestLaser();
	ROS_INFO("NEXT MOVE %d CURRENT LASER front %f, left %f, right %f",plan.at(i), current.front, current.left, current.right);
		if(!encounteredRobot){
			if(plan.at(i) == orientation){
				move(0.8);
			}else if(plan.at(i) == orientation + 1 || (orientation == 3 && plan.at(i) == 0)){
				double distanceToMove = 0.8;
				if(current.right != INVALID && current.right > 0.45){
					distanceToMove = distanceToMove -  (current.right-0.4);
				} else if(current.right != INVALID && current.right < 0.35){
					distanceToMove = distanceToMove + (0.4-current.right);
				}
				rotateLeft(M_PI/2);
				orientation = plan.at(i);
				move(distanceToMove);
			}else if(plan.at(i) == orientation - 1 || (orientation == 0 && plan.at(i) == 3)){
				double distanceToMove = 0.8;
				if(current.left != INVALID && current.left > 0.45){
					distanceToMove = distanceToMove - (current.left-0.4);
				} else if(current.left != INVALID && current.left < 0.35){
					distanceToMove = distanceToMove + (0.4-current.left);
				}
				rotateRight(M_PI/2);
				orientation = plan.at(i);
				move(distanceToMove);
			}else if(abs(plan.at(i) - orientation) == 2){
				double distanceToMove = 0.8;
				if(current.front != INVALID && current.front > 0.30){
					distanceToMove = distanceToMove - (current.front-0.27);
				} else if(current.front != INVALID && current.front < 0.24){
					distanceToMove = distanceToMove + (0.27-current.front);
					
				}
				rotateRight(M_PI);
				orientation = plan.at(i);
				move(distanceToMove);
			}
			if(localized && !publishLocation(plan.at(i))){
				i = plan.size() + 100000;
				success = false;
				orientation = 1;
			}
		} else {
			success = false;
			ROS_INFO("Unanticipated obstacle encountered!");
			i = plan.size() + 1000000;
		}
	}

	diffDrive.call(msg_stop);
	plan.clear();
	return success;
}

bool Plan::execute(lilac_fundamentals::ExecutePlan::Request &req, lilac_fundamentals::ExecutePlan::Response &res){
	ROS_INFO("\n\nI will run this plan:");
	for(int i = 0; i < req.plan.size(); i++){
		plan.push_back(req.plan[i]);
		ROS_INFO("%d", plan.at(i));
			
	}
	
	res.success = executePlan();

	if (res.success == false){
		res.startrow = startrow;
		res.startcolumn = startcolumn;
		res.endrow = endrow;
		res.endcolumn = endcolumn;
	}

	return res.success;

}

// #######################################################################################

// Sensor Functions

void Plan::updateVelocities(){

	LaserReading current = getLatestLaser();

	//ROS_INFO("\nDistance Left: %f", current.left);
	//ROS_INFO("\nDistance Right: %f", current.right);

	if (current.left != INVALID && current.right != INVALID){  
		//ROS_INFO("\nVelocity Update Case: \n\tWe are in a corridor");
		
		double x = std::abs(current.left - current.right);
		double fx = std::pow(M_E, x) - 1;

		//ROS_INFO("\n\tX: %f \n\t f(X): %f",x,fx);

		if (current.left < current.right){

			//ROS_INFO("current.left > current.right");

			velLeft = ROBOT_VELOCITY + (fx * ROBOT_VELOCITY);
			velRight = ROBOT_VELOCITY;

		}
		else {

			//ROS_INFO("current.left < current.right");

			velLeft = ROBOT_VELOCITY;
			velRight = ROBOT_VELOCITY + (fx *  ROBOT_VELOCITY);
	
		}
		//ROS_INFO("\n\tVelocity Left: %f\n\tVelocity Right: %f", msg_forward.request.left, msg_forward.request.right);
	}
	else if (current.left != INVALID) {
		//ROS_INFO("\nVelocity Update Case: \n\tSomething to my left, nothing to my right");

		current.right = 0.8 - current.left;

		double x = std::abs(current.left - current.right);
		double fx = std::pow(M_E, x) - 1;

		//ROS_INFO("\n\tX: %f \n\t f(X): %f",x,fx);

		if (current.left < current.right){

			velLeft = ROBOT_VELOCITY + (fx * ROBOT_VELOCITY);
			velRight = ROBOT_VELOCITY;

		}
		else {

			velLeft = ROBOT_VELOCITY;
			velRight = ROBOT_VELOCITY + (fx * ROBOT_VELOCITY);
	
		}
		//ROS_INFO("\n\tVelocity Left: %f\n\tVelocity Right: %f", msg_forward.request.left, msg_forward.request.right);

	}
	else if (current.right != INVALID) {
		//ROS_INFO("\nVelocity Update Case: \n\tSomething to my right, nothing to my left");

		current.left = 0.8 - current.right;

		double x = std::abs(current.left - current.right);
		double fx = std::pow(M_E, x) - 1;

		//ROS_INFO("\n\tX: %f \n\t f(X): %f",x,fx);

		if (current.left < current.right){

			velLeft = ROBOT_VELOCITY + (fx * ROBOT_VELOCITY);
			velRight = ROBOT_VELOCITY;

		}
		else {

			velLeft = ROBOT_VELOCITY;
			velRight = ROBOT_VELOCITY + (fx * ROBOT_VELOCITY);
	
		}
		//ROS_INFO("\n\tVelocity Left: %f\n\tVelocity Right: %f", msg_forward.request.left, msg_forward.request.right);

	}
	else {
		//ROS_INFO("\nVelocity Update Case: \n\tFalling back to default values");

		velLeft = ROBOT_VELOCITY;
		velRight = ROBOT_VELOCITY;

		//ROS_INFO("\n\tVelocity Left: %f\n\tVelocity Right: %f", msg_forward.request.left, msg_forward.request.right);
	}
	if(velLeft > 12.0){
		velLeft = 12.0;
	}
	if(velRight > 12.0){
		velRight = 12.0;
	}
}

void Plan::laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	LaserReading newReading;
	aboutToCrash = false;

	double readingLeft = msg->ranges[511];
	double readingRight = msg->ranges[0];

	double readingFront = msg->ranges[255];
/*
	if(!isnan(readingFront) && readingFront <= 0.15){
		int robotPoints = 0;
		int wallPoints = 0;
		model lineModel = model();

		for (int i = 120; i < 390; i++)
		{
			point temp = point();
			double dist = msg->ranges[i];
			if(!isnan(msg->ranges[i])){
				temp.x = dist * cos(((M_PI / 511.0) * i));
				temp.y = dist * sin(((M_PI / 511.0) * i));
				
				robotPoints++;
			}
			if(robotPoints >= 136){
				i = 1000000000;
			}
		}
		if(robotPoints > 136){

		}
		aboutToCrash = true;
	}*/
	if(moving && !isnan(readingFront) && readingFront <= 0.10 && readingFront > 0.0 && ((msg->ranges[120] < 0.12 && msg->ranges[120] > 0.0) || (msg->ranges[390] < 0.12 && msg->ranges[390] > 0.0))){
		
			aboutToCrash = true;
		
	}

	newReading.left = INVALID;
	newReading.right = INVALID;
	newReading.front = INVALID;

	if(!isnan(readingLeft) && readingLeft <= 0.65){
		newReading.left = readingLeft;
	} 

	if(!isnan(readingRight) && readingRight <= 0.65){
		newReading.right = readingRight;
	}

	if(!isnan(readingFront) && readingRight <= 0.65){
		newReading.front = readingRight;
	}

	addLaserReading(newReading);

	updateVelocities();
}

void Plan::encoderCallback(const create_fundamentals::SensorPacket::ConstPtr& msg)
{
	EncoderReading newReading;

	newReading.encoderLeft = msg->encoderLeft;
	newReading.encoderRight = msg->encoderRight;
	newReading.advance = msg->advance;

	addEncoderReading(newReading);
}

void Plan::addLaserReading(LaserReading newReading){
	laserReadings.push_back(newReading);
}

void Plan::addEncoderReading(EncoderReading newReading){
	encoderReadings.push_back(newReading);
}

LaserReading Plan::getLatestLaser(){
	return laserReadings.back();
}

EncoderReading Plan::getLatestEncoder(){
	return encoderReadings.back();
}


void Plan::localizationCallback(const lilac_fundamentals::ActualLocalization::ConstPtr&  msg)
{
   	ROS_INFO("row %d, column %d, orientation %d, localized %d", msg->row, msg->column, msg->orientation, msg->localized);
   	if(msg->localized && !localized){
   		orientation = msg->orientation;
   		actualLocation.x = msg->row;
   		actualLocation.y = msg->column;
   		localized = true;
		
		int count3 = 0;
		while (count3 < 1){
			

			lilac_fundamentals::Pose msgPose;

			
			msgPose.row = actualLocation.x;
			msgPose.column = actualLocation.y;
			msgPose.orientation = orientation;

			posePublisher.publish(msgPose);
			ros::spinOnce();
			ros::Duration(0.5).sleep();
			count3++;
		}
   	}
}

MapPoint Plan::calculateNextLocalization(int direction, MapPoint oldLocation){
	MapPoint nextLocation = MapPoint();
	nextLocation.x = oldLocation.x;
	nextLocation.y = oldLocation.y;

	if(direction == 1){
		nextLocation.x -= 1;
	} else if(direction == 2){
		nextLocation.y -= 1;
	} else if(direction == 3){
		nextLocation.x += 1;
	} else if(direction == 0){
		nextLocation.y += 1;
	}

	startrow = oldLocation.x;
	startcolumn = oldLocation.y;
	endrow = nextLocation.x;
	endcolumn = nextLocation.y;
	
	return nextLocation;
}

// #######################################################################################

int main(int argc, char **argv)
{
	ros::init(argc, argv, "Plan");
	ros::NodeHandle n;
	Plan p1;
	p1.rosnode = &n;

	p1.velLeft = ROBOT_VELOCITY;
	p1.velRight = ROBOT_VELOCITY;

	p1.mapToStructs();
	p1.posePublisher = n.advertise<lilac_fundamentals::Pose>("pose", 1000);
	
	p1.diffDrive = n.serviceClient<create_fundamentals::DiffDrive>("diff_drive");
	p1.resetEncoders = n.serviceClient<create_fundamentals::ResetEncoders>("reset_encoders");

	p1.EncoderSubscriber = n.subscribe("sensor_packet", 1, &Plan::encoderCallback, &p1);
	p1.LaserSubscriber = n.subscribe("scan_filtered", 1, &Plan::laserCallback, &p1);
	ros::Subscriber sub = n.subscribe("actual_localization", 1, &Plan::localizationCallback, &p1);
	
	create_fundamentals::DiffDrive srv;
	
	ros::ServiceServer service = n.advertiseService("execute_plan", &Plan::execute, &p1);
	
	ROS_INFO("Ready to execute plan localize");
	
	ros::spin();
	return 0;
}
