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
#include "lilac_fundamentals/Pose.h"

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




class Localization
{
	public:
		Localization();
		
		bool laser;
		double distanceFront;
		double distanceRight;
		double distanceLeft;
		int counter;
		bool mapCalculated;
		bool boyesFailed;
		bool isLocalized;
	        ros::Subscriber sub2 ;
		MapPoint actualLocation;
		int actualOrientation;

		std::vector<MapPoint> visitedCells;
		std::vector<struct point> points;
		std::vector<struct model> walls;
		std::vector<RobotPerspective> perspectives;
		
		ros::NodeHandle *rosnode;
		
		ros::ServiceClient diffDrive;
		ros::ServiceClient planExecuter;
		ros::Publisher localizationPublisher;
		ros::Subscriber sub3;
		
		
		create_fundamentals::DiffDrive msg_forward;
		create_fundamentals::DiffDrive msg_backward;
		create_fundamentals::DiffDrive msg_stop;
		create_fundamentals::DiffDrive msg_turn_left;
		create_fundamentals::DiffDrive msg_turn_right;

		lilac_fundamentals::ExecutePlan srvExecute;

		void mapCallback(const lilac_fundamentals::Grid::ConstPtr& grid_msg);
		void laserCallbackRotate(const sensor_msgs::LaserScan::ConstPtr& msg);
		void laserCallbackDistance(const sensor_msgs::LaserScan::ConstPtr& msg);

		RobotPerspective getPerspective();
		void laserCallbackPerspective(const sensor_msgs::LaserScan::ConstPtr& msg);
		double distance(struct point p, struct model m1);
		void ransac();
		
		void correctDistance();
		void align();
		void rotateLeft(double angle);
		void mapToStructs();
		void printPerspectives(std::vector<RobotPerspective> perspectives);
		void solveBoyes();

		std::vector<RobotPerspective> findMovement(RobotPerspective actualPerspective, std::vector<RobotPerspective> oldPerspectives);
		std::vector<RobotPerspective> calculateNewPerspectives(int moveDicection, std::vector<RobotPerspective> oldPerspectives);
		bool isVisited(MapPoint newCell);
		MapPoint calculateNextLocalization(int direction, MapPoint oldLocation);

		void poseCallback(const lilac_fundamentals::Pose::ConstPtr&  msg);

		
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
	mapCalculated = false;

	actualOrientation = 1;
	actualLocation.x = 0;
	actualLocation.y = 0;
	visitedCells.push_back(actualLocation);
	isLocalized = false;
	
}


void Localization::mapCallback(const lilac_fundamentals::Grid::ConstPtr& grid_msg){

	if (mapCalculated == true){
		return;
	}
	
	int rowCount = grid_msg->rows.size();
	int cellCount = grid_msg->rows[0].cells.size();

	for(int row = 0; row < rowCount; row++){

		for (int cell = 0; cell < cellCount; cell++){

			int wallCount = grid_msg->rows[row].cells[cell].walls.size();

			Cell newCell;
			//ROS_INFO("cell %d",cell);
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


void Localization::rotateLeft(double angle){

        
    double seconds =  (angle * ROBOT_DIAMETER * M_PI) / (2 * M_PI * 3.0 * WHEEL_RAD);

	seconds = seconds - (9.15*(seconds / 100));

	//stop
	diffDrive.call(msg_turn_left);



        //sleep until at correct angle
        ros::Duration(seconds).sleep();

        //stop rotating
        diffDrive.call(msg_stop);
        //ros::Duration(3).sleep();
   
	//ROS_INFO("Rotate Position: %d", position);
}

void Localization::laserCallbackPerspective(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	if(laser == true){
		return;
	}
	distanceFront = -1;
	distanceRight = -1;
	distanceLeft = -1;
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

double Localization::distance(struct point p, struct model m1){
	double temp = m1.m * p.x - p.y + m1.c ; 
	if (temp < 0.0 ) temp = - temp ; 
	double racine = sqrt (m1.m * m1.m + 1.0*1.0);
	double distance = temp / racine ; 
	return distance;
}

void Localization::ransac()
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
			if(thiserr > 50 && thiserr > besterr){
				//ROS_INFO("New Bestmodel! %d    error: %d", iterations, thiserr);
				bestfit = maybemodel;
				besterr = thiserr;
			}
			iterations++;
			//ROS_INFO("Model worked %d    error: %d", iterations, thiserr);
		}
		if(besterr >= 50){
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
			//ROS_INFO("bestfit: %f *x + %f  error: %d    >>>WALLS: %d", bestfit.m, bestfit.c, besterr, walls.size());
			//ROS_INFO("New points size: %d", points.size());	
		}else{
			//ROS_INFO("No acceptable bestfit found");
			break;
		}
	}

	points.clear();	
	
	
}

RobotPerspective Localization::getPerspective(){
	points.clear();
	walls.clear();
	RobotPerspective rp = RobotPerspective();
	rp.visibleWalls = 0;
	rp.Front = false;
	rp.Left = false;
	rp.Right = false;

	laser = false;
	 sub2 = rosnode->subscribe("scan_filtered", 1, &Localization::laserCallbackPerspective, this);
	
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
    sub2.shutdown();
    return rp;
}


void Localization::laserCallbackRotate(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	//ROS_INFO("105 >>>>  %f   90 >>>>  %f   75 >>>>  %f", msg->ranges[296], msg->ranges[256], msg->ranges[216]);
	if(laser == true){
		return;
	}

	if(!isnan(msg->ranges[256]) && !isnan(msg->ranges[226]) && !isnan(msg->ranges[286])){
		if(std::abs(msg->ranges[226] - msg->ranges[286]) < 0.01 && 
			std::abs(msg->ranges[226] - msg->ranges[256] / cos(30/512 * M_PI)) < 0.015){
	//ROS_INFO("abs(75 - 105) >>>>  %f   abs2() >>>>  %f", std::abs(msg->ranges[216] - msg->ranges[296]), std::abs(msg->ranges[216] - msg->ranges[256] / cos(40/512 * M_PI)));
			diffDrive.call(msg_stop);
			laser = true;
			distanceFront = msg->ranges[256];
		}
	}
}

void Localization::laserCallbackDistance(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	//ROS_INFO("105 >>>>  %f   90 >>>>  %f   75 >>>>  %f", msg->ranges[296], msg->ranges[256], msg->ranges[216]);
	if(laser == true){
		return;
	}

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
			//ROS_INFO("distanceFront >>>>  %f   distanceLeft >>>>  %f   distanceRight >>>>  %f", distanceFront, distanceLeft, distanceRight);
		}
	}
	
}

void Localization::correctDistance(){
	laser = false;
	 sub2 = rosnode->subscribe("scan_filtered", 1, &Localization::laserCallbackDistance, this);
    if(distanceFront > 0.25){
	//ROS_INFO("distanceFront forward %f", distanceFront);
    	diffDrive.call(msg_forward);
    	while(!laser){
			ros::spinOnce();
    	}
    } else if(distanceFront < 0.27){
	//ROS_INFO("distanceFront backward %f", distanceFront);
    	diffDrive.call(msg_backward);
    	while(!laser){
			ros::spinOnce();
    	}
    }
    sub2.shutdown();
}

	
void Localization::align()
{
        laser = false;
        
        sub2 = rosnode->subscribe("scan_filtered", 1, &Localization::laserCallbackRotate, this);
        diffDrive.call(msg_turn_left);
        while(!laser){
                ros::spinOnce();
    }
    ROS_INFO("Aligned to wall");
    laser = false;
       sub2 = rosnode->subscribe("scan_filtered", 1, &Localization::laserCallbackDistance, this);
    if(distanceFront > 0.25){
        ROS_INFO("1   distanceFront forward %f", distanceFront);
        diffDrive.call(msg_forward);
        while(!laser){
                        ros::spinOnce();
        }
    } else if(distanceFront < 0.25){
        ROS_INFO("2     distanceFront backward %f", distanceFront);
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
ROS_INFO("3     distanceFront backward %f", distanceFront);


    if(distanceLeft > 0.0){
        laser = false;
                sub2 = rosnode->subscribe("scan_filtered", 1, &Localization::laserCallbackRotate, this);
                diffDrive.call(msg_turn_left);
                ros::Duration(1).sleep();
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
ROS_INFO("LEFT     distanceFront backward %f", distanceFront);
    } else if(distanceRight > 0.0){
        laser = false;
                sub2 = rosnode->subscribe("scan_filtered", 1, &Localization::laserCallbackRotate, this);
                
                diffDrive.call(msg_turn_right);
                ros::Duration(1).sleep();
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
ROS_INFO("RIGHT     distanceFront backward %f", distanceFront);
    }
    sub2.shutdown();
}

void Localization::mapToStructs(){

	mapCalculated = false;
	ros::Subscriber sub = rosnode->subscribe("map", 1, &Localization::mapCallback, this);

	while(!mapCalculated){
		ros::spinOnce();
		    
	}
	sub.shutdown();
}

void Localization::solveBoyes(){
	
	std::vector<RobotPerspective> boyesPerspectives;
	visitedCells.clear();
	actualLocation.x = 0;
	actualLocation.y = 0;
	actualOrientation = 1;
	visitedCells.push_back(actualLocation);
	//std::copy(perspectives.begin(), perspectives.end(), boyesPerspectives.begin() );
	boyesPerspectives = perspectives;
	
	int zz = 0;
	while(boyesPerspectives.size() > 1){
		ROS_INFO("\nboyes Perspectives %d\n", boyesPerspectives.size());
		RobotPerspective actualPerspective = getPerspective();
		std::vector<RobotPerspective> tempPerspectives;
		for (int i = 0; i < boyesPerspectives.size(); i++)
		{
			if(boyesPerspectives.at(i).visibleWalls == actualPerspective.visibleWalls){
				tempPerspectives.push_back(boyesPerspectives.at(i));
			}
		}
		boyesPerspectives = tempPerspectives;
		if(actualPerspective.visibleWalls != 0 && actualPerspective.visibleWalls != 3){
			tempPerspectives.clear();
			for (int i = 0; i < boyesPerspectives.size(); i++)
			{
				if(boyesPerspectives.at(i).Front == actualPerspective.Front && boyesPerspectives.at(i).Left == actualPerspective.Left &&
					boyesPerspectives.at(i).Right == actualPerspective.Right){
					tempPerspectives.push_back(boyesPerspectives.at(i));
				}
			}
			boyesPerspectives = tempPerspectives;
		}
		
		//std::copy(tempPerspectives.begin(), tempPerspectives.end(), boyesPerspectives.begin());
		
		printPerspectives(boyesPerspectives);
		if(actualPerspective.Front){
			correctDistance();
		}
		boyesPerspectives = findMovement(actualPerspective, boyesPerspectives);
		ROS_INFO("\n\nSolve Boyes Step %d\n", zz);
		zz++;
	}
	if(boyesPerspectives.size() == 1){
		ROS_INFO("\n\n\n\nI got it!!! i am here:\n");
		//std::system("rosrun lilac_fundamentals Indiana.py&");
		printPerspectives(boyesPerspectives);
		actualLocation.x = boyesPerspectives.front().m.x;
		actualLocation.y = boyesPerspectives.front().m.y;
		actualOrientation = boyesPerspectives.front().globalPerspective;
		isLocalized = true;
		
		localizationPublisher = rosnode->advertise<lilac_fundamentals::ActualLocalization>("actual_localization", 1000);

		int count2 = 0;
		while (count2 < 2){
			ROS_INFO("\n\nPublish for execute plan\n");
			lilac_fundamentals::ActualLocalization msg;

			

			msg.row = actualLocation.x;
			msg.column = actualLocation.y;
			msg.orientation = actualOrientation;
			msg.localized = isLocalized;

			

			localizationPublisher.publish(msg);
			
			ros::spinOnce();
			ros::Duration(0.5).sleep();
			count2++;
		}

		
	} else if(boyesPerspectives.size() == 0 && !boyesFailed){
		boyesFailed = true;
		solveBoyes();
	}

}



std::vector<RobotPerspective> Localization::findMovement(RobotPerspective actualPerspective, std::vector<RobotPerspective> oldPerspectives){
	std::vector<RobotPerspective> newPerspectives;

	MapPoint nextLocation;
	int moveOrientation;
	int moveDicection;
	bool moveNewCell = false;

	if(!actualPerspective.Front){
		moveDicection = 1;
		moveOrientation = actualOrientation;
		nextLocation = calculateNextLocalization(moveOrientation, actualLocation);
		if(!isVisited(nextLocation)){
			moveNewCell = true;
		}
	} 
	if(!actualPerspective.Left && !moveNewCell){
		moveDicection = 2;
		moveOrientation = actualOrientation + 1;
		if(moveOrientation == 4){
			moveOrientation = 0;
		}
		nextLocation = calculateNextLocalization(moveOrientation, actualLocation);
		if(!isVisited(nextLocation)){
			moveNewCell = true;
		}
	}
	if(!actualPerspective.Right && !moveNewCell){
		moveDicection = 0;
		moveOrientation = actualOrientation - 1;
		if(moveOrientation == -1){
			moveOrientation = 3;
		}
		nextLocation = calculateNextLocalization(moveOrientation, actualLocation);
		if(!isVisited(nextLocation)){
			moveNewCell = true;
		}
	} 
	if(!moveNewCell && actualPerspective.visibleWalls == 3){
		moveDicection = 3;
		moveOrientation = actualOrientation + 2;
		if(moveOrientation == 4){
			moveOrientation = 0;
		}
		if(moveOrientation == 5){
			moveOrientation = 1;
		}
		nextLocation = calculateNextLocalization(moveOrientation, actualLocation);
		if(!isVisited(nextLocation)){
			moveNewCell = true;
		}
	}
	ROS_INFO("moveOrientation %d isNewCell %d", moveOrientation, moveNewCell);


	std::vector<int> tempArr;
	tempArr.push_back(moveOrientation);

	srvExecute.request.plan = tempArr;
	bool success = planExecuter.call(srvExecute);
	ROS_INFO("ExecutePlan %d", success);

	/*//Maybe cause errors
	while(!srv.response){
		ROS_INFO("Wait for ExecutePlan");
		ros::Duration(1).sleep();
	}*/
	actualOrientation = moveOrientation;
	actualLocation = nextLocation;
	if(moveNewCell){
		visitedCells.push_back(actualLocation);
	}

	newPerspectives = calculateNewPerspectives(moveDicection, oldPerspectives);

	return newPerspectives;
}

std::vector<RobotPerspective> Localization::calculateNewPerspectives(int moveDicection, std::vector<RobotPerspective> oldPerspectives){
	std::vector<RobotPerspective> newPerspectives;

	for (int i = 0; i < oldPerspectives.size(); i++)
	{
		int newGlobalPerspective;
		if(moveDicection == 1){
			newGlobalPerspective = oldPerspectives.at(i).globalPerspective;
		} else if(moveDicection == 2){
			newGlobalPerspective = oldPerspectives.at(i).globalPerspective + 1;
			if(newGlobalPerspective == 4){
				newGlobalPerspective = 0;
			}
		} else if(moveDicection == 0){
			newGlobalPerspective = oldPerspectives.at(i).globalPerspective - 1;
			if(newGlobalPerspective == -1){
				newGlobalPerspective = 3;
			}
		}  else if(moveDicection == 3){
			newGlobalPerspective = oldPerspectives.at(i).globalPerspective + 2;
			if(newGlobalPerspective == 4){
				newGlobalPerspective = 0;
			}
			if(newGlobalPerspective == 5){
				newGlobalPerspective = 1;
			}
		} 
		MapPoint newCellLocation = calculateNextLocalization(newGlobalPerspective, oldPerspectives.at(i).m);
		for (int k = 0; k < perspectives.size(); k++)
		{
			if(perspectives.at(k).m.x == newCellLocation.x && perspectives.at(k).m.y == newCellLocation.y
				&& perspectives.at(k).globalPerspective == newGlobalPerspective){
				newPerspectives.push_back(perspectives.at(k));
				ROS_INFO("i %d k %d", i, k);
				k = perspectives.size()+1000;
				
			}
		}
	}
	ROS_INFO("\nUpdated Perspectives\n");
	//ROS_INFO("Size %d", newPerspectives.size());
	printPerspectives(newPerspectives);
	return newPerspectives;
}

bool Localization::isVisited(MapPoint newCell){
	for (int i = 0; i < visitedCells.size(); i++)
	{
		if(visitedCells.at(i).x == newCell.x && visitedCells.at(i).y == newCell.y){
			return true;
		}
	}
	return false;
}

MapPoint Localization::calculateNextLocalization(int direction, MapPoint oldLocation){
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

	return nextLocation;
}

void Localization::printPerspectives(std::vector<RobotPerspective> thisperspectives){
	int size = thisperspectives.size();

	for(int i = 0; i < size; i++){
	
		ROS_INFO("\nPOINT (%d,%d), Perspective %d, Visisble Walls %d, Front %d, Left %d, Right %d", 
				thisperspectives.at(i).m.x, 
				thisperspectives.at(i).m.y, 
				thisperspectives.at(i).globalPerspective,
				thisperspectives.at(i).visibleWalls,
				thisperspectives.at(i).Front,
				thisperspectives.at(i).Left,
				thisperspectives.at(i).Right);
	}
	ROS_INFO("\n\n\n\n");
}

void Localization::poseCallback(const lilac_fundamentals::Pose::ConstPtr&  msg)
{
	ROS_INFO("Pose is (%d, %d)",msg->row , msg->column);
	if(msg->row == -2 && msg->column == -2){

		ROS_INFO("I am lost! I will localize one more time");
		isLocalized = false;

	}
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "Localization");
	ros::NodeHandle n;
	Localization l1;
	l1.rosnode = &n;
	
	//ros::Subscriber sub2 = n.subscribe("sensor_packet", 1, &Localization::sensorCallback, &a1);
       //std::system("rosrun lilac_fundamentals publish_map.py&");
	//std::system("rosrun lilac_fundamentals execute_plan_loc&");
	l1.diffDrive = n.serviceClient<create_fundamentals::DiffDrive>("diff_drive");
	l1.planExecuter = n.serviceClient<lilac_fundamentals::ExecutePlan>("execute_plan");

	create_fundamentals::DiffDrive srv;
	
	bool alreadyLocalized = false;
	while(ros::ok()){
	
		l1.mapToStructs();
		ROS_INFO("maptostructs");
		//if(!alreadyLocalized){
			l1.align();
			ROS_INFO("align");
		//}
		l1.solveBoyes();
		ROS_INFO("solveBoyes");
		alreadyLocalized = true;

	
		l1.sub3 = n.subscribe("pose", 1, &Localization::poseCallback, &l1);
		while(l1.isLocalized){
			ros::spinOnce();
		}
		l1.sub3.shutdown();

	
		l1.boyesFailed = false;
		l1.visitedCells.clear();

		l1.actualOrientation = 1;
		l1.actualLocation.x = 0;
		l1.actualLocation.y = 0;
		l1.visitedCells.push_back(l1.actualLocation);

		l1.mapCalculated = false;
		l1.perspectives.clear();
}
	
	return 0;
}


			
			
