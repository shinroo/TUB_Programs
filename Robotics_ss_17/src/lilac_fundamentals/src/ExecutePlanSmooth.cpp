#include "ros/ros.h"
#include <cstdlib>
#include <cmath>
#include "sensor_msgs/LaserScan.h"
#include "DiffDrive.h"
#include "SensorPacket.h"
#include <math.h>
#include <float.h>
#include <time.h>
#include "lilac_fundamentals/ExecutePlan.h"

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

class Plan
{
	public:
		Plan();
		std::vector<struct point> points;
		std::vector<struct model> walls;
		std::vector<int> plan;
		int position;
		int ready;
		int counter;
		bool align;
		bool executing;
		bool laser;
		bool perfectangle;
		int orientation;
		double distanceRight;
		double distanceLeft;
		double expectedError;
		ros::NodeHandle *rosnode;

		ros::ServiceClient diffDrive;
		
		
		create_fundamentals::DiffDrive msg_forward;
		create_fundamentals::DiffDrive msg_stop;
		create_fundamentals::DiffDrive msg_turn_left;
		create_fundamentals::DiffDrive msg_turn_right;

		void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
		//void sensorCallback(const create_fundamentals::SensorPacket::ConstPtr& msg);
		void rotateLeft(double angle);
		void rotateRight(double angle);
		void ransac();
		double distance(struct point p, struct model m1);
		void solve();
		void move(double distance);
		point nearestPoint(struct point intersection, struct model line, double dist);
		double distancePoint(struct point p1, struct point p2);
		void alignToCell();
		void executePlan();
		//void rotatePolar(double angle);
		double findRadius(struct point p1, struct point p2, struct point p3);
		bool execute(lilac_fundamentals::ExecutePlan::Request &req, lilac_fundamentals::ExecutePlan::Response &res);

		void laserCallbackExecute(const sensor_msgs::LaserScan::ConstPtr& msg);
		void laserCallbackCurve(const sensor_msgs::LaserScan::ConstPtr& msg);
		void moveCentered(double distance);
		void curveRight();
		void curveLeft();
		
		
} p1;

Plan::Plan()
{	
	msg_turn_right.request.left = 3;
	msg_turn_right.request.right = -3;

	msg_forward.request.left = 3;
	msg_forward.request.right = 3;

	msg_stop.request.left = 0;
	msg_stop.request.right = 0;

	msg_turn_left.request.left = -3;
	msg_turn_left.request.right = 3;

	position = 0;
	ready = 1;
	counter = 0;
	align = false;
	executing = false;
	orientation = 1;
	perfectangle = false;
	expectedError = 0.39;
}

void Plan::move(double distance){
 
	double seconds = distance / (0.03 * 3.0);
	seconds = seconds - (9.15*(seconds / 100));
    diffDrive.call(msg_forward);

    ros::Duration(seconds).sleep();

    // stop
    diffDrive.call(msg_stop);
}


void Plan::rotateRight(double angle){

    //ROS_INFO("rotate Right %f", angle);
    double seconds =  (angle * ROBOT_DIAMETER * M_PI) / (2 * M_PI * 3.0 * WHEEL_RAD);

	seconds = seconds - (9.15*(seconds / 100));

	//stop
	diffDrive.call(msg_turn_right);

    //sleep until at correct angle
    ros::Duration(seconds).sleep();

    //stop rotating
    diffDrive.call(msg_stop);
    //ros::Duration(3).sleep();
    ready = 1;
	//ROS_INFO("Rotate Position: %d", position);
	//ROS_INFO("complete rotate Right %f", angle);
}

void Plan::rotateLeft(double angle){

    //ROS_INFO("rotate Left %f", angle);
    double seconds =  (angle * ROBOT_DIAMETER * M_PI) / (2 * M_PI * 3.0 * WHEEL_RAD);

	seconds = seconds - (9.15*(seconds / 100));

	//stop
	diffDrive.call(msg_turn_left);

    //sleep until at correct angle
    ros::Duration(seconds).sleep();

    //stop rotating
    diffDrive.call(msg_stop);
    //ros::Duration(3).sleep();
    ready = 1;
	//ROS_INFO("Rotate Position: %d", position);
	//ROS_INFO("complete rotate Left %f", angle);
}


double Plan::distancePoint(struct point p1, struct point p2){
	return sqrt((p2.x-p1.x)*(p2.x-p1.x)+(p2.y-p1.y)*(p2.y-p1.y));
}

point Plan::nearestPoint(struct point intersection, struct model line, double dist){
	point p1 = point();
	point p2 = point();

	p1.x = (1.0/(sqrt(1.0 + line.m * line.m))) * dist + intersection.x;
	p1.y = line.m/(sqrt(1.0 + line.m * line.m)) * dist + intersection.y;

	p2.x = -1.0/(sqrt(1.0 + line.m * line.m)) * dist + intersection.x;
	p2.y = -line.m/(sqrt(1.0 + line.m * line.m)) * dist + intersection.y;

	point origin = point();
	origin.x = 0.0;
	origin.y = 0.0;
	if(distancePoint(p1, origin) >= distancePoint(p2, origin)){
		return p2;
	}else{
		return p1;
	}
}

double Plan::findRadius(struct point p1, struct point p2, struct point p3){
	model line1;
	line1.m = (p2.y - p1.y)/(p2.x - p1.x);
    line1.c = -line1.m * p1.x + p1.y;
	model line2;
	line2.m = (p3.y - p1.y)/(p3.x - p1.x);
    line2.c = -line2.m * p1.x + p1.y;
	model line1p;
	line1p.m = -1/line1.m;
	line1p.c = -line1p.m * ((p2.x - p1.x)/2 + p1.x) + ((p2.y - p1.y)/2 + p1.y);
	model line2p;
	line2p.m = -1/line2.m;
	line2p.c = -line2p.m * ((p3.x - p1.x)/2 + p1.x) + ((p3.y - p1.y)/2 + p1.y);
	point intersection = point();
    intersection.x = (line1p.c - line2p.c)/(line2p.m - line1p.m);
	intersection.y = line2p.m * intersection.x + line2p.c;

	return distancePoint(intersection, p1);
}

void Plan::laserCallbackExecute(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	if(!isnan(msg->ranges[0]) && msg->ranges[0] != 0.0 && !isnan(msg->ranges[1]) && 
		msg->ranges[1] != 0.0 && abs(msg->ranges[0]-msg->ranges[1]) <= 0.0001){
	
	distanceRight = msg->ranges[0];
	laser = true;
	} else{
		distanceRight = -1.0;
		laser = true;
	}
	if(!isnan(msg->ranges[msg->ranges.size()-1]) && msg->ranges[msg->ranges.size()-1] != 0.0 &&
		!isnan(msg->ranges[msg->ranges.size()-2]) && msg->ranges[msg->ranges.size()-2] != 0.0	
		&& abs(msg->ranges[msg->ranges.size()-2]-msg->ranges[msg->ranges.size()-1]) <= 0.0001){
		
		distanceLeft = msg->ranges[msg->ranges.size()-1];
		laser = true;
	} else{
		distanceLeft = -1.0;
		laser = true;
	}
}

void Plan::moveCentered(double distance){
 	laser = false;
 	//Recover
	ros::Subscriber sub2 = rosnode->subscribe("scan_filtered", 1, &Plan::laserCallbackExecute, this);
	while(!laser){
		ros::spinOnce();
        }
	double longDistance;
	bool right = true;
	if(distanceRight >= 0.39){
		longDistance = distanceRight;
		right = false;
	} else if(distanceLeft >= 0.39){
		longDistance = distanceLeft;
		right = true;
	} else if(distanceLeft == -1.0 && distanceRight == -1.0){
		longDistance = 0.0;
	}else{
		longDistance = std::max(distanceLeft,distanceRight);
		if(longDistance == distanceLeft){
			right = false;
		} else{
			right = true;
		}
	}
	if(longDistance/0.39 > 1.05 || longDistance/0.39 < 0.95 || longDistance != 0.0){
		//ROS_INFO("distanceLeft %f    distanceRight %f", distanceLeft, distanceRight);

		

		point p1 = point();
		p1.x = 0.0;
		p1.y = 0.0;

		point p2 = point();
		p2.x = std::abs(longDistance - 0.39);
		p2.y = distance/2;

		point p3 = point();
		p3.x = std::abs(longDistance - 0.39);
		p3.y = -1 * distance/2;

		double radius = findRadius(p1,p2,p3);
		double angle = acos(1 - (pow(distancePoint(p1,p2),2.0) / (2 * pow(radius, 2.0))));

    	double arclengthOut = (radius + ROBOT_DIAMETER/200) * angle;
    	double arclengthIn = (radius - ROBOT_DIAMETER/200) * angle;
 		//ROS_INFO("longDistance %f arclengthOut %f    arclengthIn %f     radius %f     angle %f", longDistance, arclengthOut, arclengthIn, radius, angle);
    	create_fundamentals::DiffDrive srv;

		double velIn = 4.0;
	
		if(arclengthOut/arclengthIn > 1.5){
			velIn = 3.0;
		}
		/*if(arclengthOut/arclengthIn > 3 || arclengthOut/arclengthIn < 0.33333){
			velIn = 1.0;
		}*/
    	double velOut = (arclengthOut/arclengthIn * 0.78) * velIn;

		if (velOut > 6.0){
			velOut = 6.0;
			radius = 0.53;
			p2.x = radius - sqrt(pow(radius, 2.0) - pow(distance, 2.0));
			p2.y = distance/2;
			angle = acos(1 - (pow(distancePoint(p1,p2),2.0) / (2 * pow(radius, 2.0))));

    			arclengthOut = (radius + ROBOT_DIAMETER/200) * angle;
    			arclengthIn = (radius - ROBOT_DIAMETER/200) * angle;
		}
    
    	double seconds = ((arclengthIn / (0.03 * velIn))+(arclengthOut / (0.03 * velOut)))/2;
    	seconds = seconds - (10*(seconds / 100));

    	if(right){
    		srv.request.left = velIn;
    		srv.request.right = velOut;

    		//ROS_INFO("Diff drive left %f    right %f", srv.request.left, srv.request.right);
			diffDrive.call(srv);
    		ros::Duration(seconds).sleep();
    		srv.request.left = velOut;
    		srv.request.right = velIn;
    		//ROS_INFO("Diff drive left %f    right %f", srv.request.left, srv.request.right);
			diffDrive.call(srv);
    		ros::Duration(seconds).sleep();

    		
    	} else {
    		srv.request.left = velOut;
    		srv.request.right = velIn;
 			//ROS_INFO("Diff drive left %f    right %f", srv.request.left, srv.request.right);
			diffDrive.call(srv);
    		ros::Duration(seconds).sleep();
    		srv.request.left = velIn;
    		srv.request.right = velOut;
			//ROS_INFO("Diff drive left %f    right %f", srv.request.left, srv.request.right);
			diffDrive.call(srv);
    		ros::Duration(seconds).sleep();
    	
    	
    	}
    } else{
		move(0.8);
    }
}

void Plan::curveLeft(){
	laser = false;

	ros::Subscriber sub2 = rosnode->subscribe("scan_filtered", 1, &Plan::laserCallbackExecute, this);
	while(!laser){
		ros::spinOnce();
    }
    double radius = 0.39;
    if(distanceRight != -1.0){
    	radius = 0.80 - distanceRight;
    }
    create_fundamentals::DiffDrive srv;

		double ain = (radius-(ROBOT_DIAMETER/200)) * (M_PI/2);
		double aout = (radius+(ROBOT_DIAMETER/200)) * (M_PI/2);

		double vin = 3;
		double vout = vin * (aout/ain);

		double time = ain / (vin * 0.03);

		srv.request.left = vin;
		srv.request.right = vout;

		diffDrive.call(srv);

		ros::Duration(time).sleep();

	
}

void Plan::curveRight(){

	laser = false;

	ros::Subscriber sub2 = rosnode->subscribe("scan_filtered", 1, &Plan::laserCallbackExecute, this);
	while(!laser){
		ros::spinOnce();
    }
    double radius = 0.39;
    if(distanceRight != -1.0){
    	radius = 0.80 - distanceRight;
    } 
	create_fundamentals::DiffDrive srv;

	double ain = (radius-(ROBOT_DIAMETER/200)) * (M_PI/2);
	double aout = (radius+(ROBOT_DIAMETER/200)) * (M_PI/2);

	double vin = 3;
	double vout = vin * (aout/ain);

	double time = ain / (vin * 0.03);

	srv.request.left = vout;
	srv.request.right = vin;

	diffDrive.call(srv);

	ros::Duration(time).sleep();	
}

void Plan::laserCallbackCurve(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	
	if(!perfectangle && !isnan(msg->ranges[0]) && msg->ranges[0] != 0.0 && !isnan(msg->ranges[30]) && 
		msg->ranges[30] != 0.0){
		point p1 = point();
		p1.x = msg->ranges[0];
		p1.y = 0;

		point p2 = point();
		p2.x = msg->ranges[30] * cos((M_PI / 511.0) * 30);
		p2.y = msg->ranges[30] * sin((M_PI / 511.0) * 30);

		double correctionAngle = atan((p2.y)/(p2.x-p1.x))/M_PI * 180;
		if(correctionAngle > 0.0 && correctionAngle < 80.0){
			rotateRight((90 - correctionAngle)/180 * M_PI);
		}else if(correctionAngle < 0.0 && correctionAngle > -80.0){
			rotateLeft((correctionAngle + 90)/180 * M_PI);
		}
		ROS_INFO("Correction Angle1  %f", correctionAngle);			
	
	}
	else if(!perfectangle && !isnan(msg->ranges[msg->ranges.size()-1]) && msg->ranges[msg->ranges.size()-1] != 0.0 &&
		!isnan(msg->ranges[msg->ranges.size()-31]) && msg->ranges[msg->ranges.size()-31] != 0.0){
		point p1 = point();
		p1.x = msg->ranges[msg->ranges.size()-1];
		p1.y = 0;

		point p2 = point();
		p2.x = msg->ranges[msg->ranges.size()-31] * cos((M_PI / 511.0) * msg->ranges.size()-31);
		p2.y = msg->ranges[msg->ranges.size()-31] * sin((M_PI / 511.0) * msg->ranges.size()-31);

		double correctionAngle = atan((p2.y)/(p2.x-p1.x))/M_PI * 180;
		if(correctionAngle > 0.0 && correctionAngle < 80.0){
			rotateLeft((90 - correctionAngle)/180 * M_PI);
		}else if(correctionAngle < 0.0 && correctionAngle > -80.0){
			rotateRight((correctionAngle + 90)/180 * M_PI);
		}
		ROS_INFO("Correction Angle2  %f", correctionAngle);
	}


	perfectangle = true;
}

void Plan::executePlan()
{
	if(plan.at(0) == orientation + 1 || (orientation == 3 && plan.at(0) == 0)){
		rotateLeft(M_PI/2);
		orientation = plan.at(0);
	}else if(plan.at(0) == orientation - 1 || (orientation == 0 && plan.at(0) == 3)){
		rotateRight(M_PI/2);
		orientation = plan.at(0);
	}else if(abs(plan.at(0) - orientation) == 2){
		rotateRight(M_PI);
		orientation = plan.at(0);
	}

	std::vector<char> dirPlan;
	//f => moveCentered(0.8);
	//h => moveCentered(0.4);
	//r => curveRight();
	//l => curveLeft();
	//b => rotateRight(M_PI); moveCentered(0.8);

	dirPlan.push_back('f');
	for(int i = 0; i < plan.size()-1; i++){
		
			if(plan.at(i) == plan.at(i+1)){
				dirPlan.push_back('f');
			} else if(plan.at(i) - plan.at(i+1) == 1 || plan.at(i) - plan.at(i+1) == -3){
				if(dirPlan.back() == 'h'){
					dirPlan.pop_back();
					dirPlan.push_back('r');
					dirPlan.push_back('h');
				} else if(dirPlan.back() == 'f'){
					dirPlan.pop_back();
					dirPlan.push_back('h');
					dirPlan.push_back('r');
					dirPlan.push_back('h');
				} else{
					dirPlan.push_back('h');
					dirPlan.push_back('r');
					dirPlan.push_back('h');
				}
			} else if(plan.at(i+1) - plan.at(i) == 1 || plan.at(i+1) - plan.at(i)== -3){
				if(dirPlan.back() == 'h'){
					dirPlan.pop_back();
					dirPlan.push_back('l');
					dirPlan.push_back('h');
				} else if(dirPlan.back() == 'f'){
					dirPlan.pop_back();
					dirPlan.push_back('h');
					dirPlan.push_back('l');
					dirPlan.push_back('h');
				} else{
					dirPlan.push_back('h');
					dirPlan.push_back('l');
					dirPlan.push_back('h');
				}
			} else if(abs(plan.at(i) - plan.at(i+1)) == 2){
				dirPlan.push_back('b');
				dirPlan.push_back('f');
			}
		
		
	}

	//ros::Subscriber sub2 = rosnode->subscribe("scan_filtered", 1, &Plan::laserCallbackCurve, this);
	
	for(int i = 0; i < dirPlan.size(); i++){
		if(dirPlan.at(i) == 'f'){
			
				//ros::spinOnce();
    		
			moveCentered(0.8);
			perfectangle = false;
		} else if(dirPlan.at(i) == 'h'){
			
			
				//ros::spinOnce();
    		
			moveCentered(0.4);
			perfectangle = false;
		} else if(dirPlan.at(i) == 'r'){
			curveRight();
		} else if(dirPlan.at(i) == 'l'){
			curveLeft();
		} else if(dirPlan.at(i) == 'b'){
			rotateRight(M_PI);
		}
		ROS_INFO("Move >>>>>>   %c", dirPlan.at(i));
	}
	diffDrive.call(msg_stop);
	plan.clear();
	position = 0;
	ready = 1;
	counter = 0;
	align = false;
	executing = false;	

}


void Plan::solve()
{
	ROS_INFO("Calculating middle");
	point intersection = point();
	int w2 = 0;

	point origin = point();
	origin.x = 0.0;
	origin.y = 0.0;
	
	for(int i = 1; i < walls.size(); i++){
		intersection.x = (walls.at(i).c - walls.at(0).c)/(walls.at(0).m - walls.at(i).m);
		intersection.y = walls.at(0).m * intersection.x + walls.at(0).c;
		if(distancePoint(intersection, origin) <= 1.5){
			w2 = i;
			break;
		}
	}
	ROS_INFO("Intersection found X: %f Y: %f", intersection.x, intersection.y);

	point diag1 = nearestPoint(intersection, walls.at(0), 0.78);
	point diag2 = nearestPoint(intersection, walls.at(w2), 0.78);

	point result = point();
	ROS_INFO("Diag1 X: %f Y: %f", diag1.x, diag1.y);
	ROS_INFO("Diag2 X: %f Y: %f", diag2.x, diag2.y);

	result.x = diag1.x + (diag2.x - diag1.x)/2;
	result.y = diag1.y + (diag2.y - diag1.y)/2;

	ROS_INFO("RESULT X: %f Y: %f", result.x, result.y);
	
	
	
	double angle = atan2(result.y,result.x);
	if(angle < 0.0){
		angle = 2 * M_PI + angle;
	}
	/*if(angle > 0.06){
		angle = angle - 0.06;
	}*/

	ROS_INFO("ANGLE: %f DIST: %f", angle*180/M_PI, sqrt(result.x * result.x + result.y * result.y));
	if(angle < M_PI/2){
		rotateRight(M_PI/2 - angle);
		move(sqrt(result.x * result.x + result.y * result.y));
		rotateLeft(M_PI/2 - angle);
	} else if(angle < M_PI){
		rotateLeft(angle - M_PI/2);
		move(sqrt(result.x * result.x + result.y * result.y));
		rotateRight(angle - M_PI/2);
	} else if(angle < 2 * M_PI){
		rotateRight((2 * M_PI - angle) + M_PI/2);
		move(sqrt(result.x * result.x + result.y * result.y));
		rotateLeft((2 * M_PI - angle) + M_PI/2);
	}
	
	align = false;
	


	ROS_INFO("Align solved");
	

}


void Plan::ransac()
{
	ROS_INFO("RANSAC TIME");
	srand (time(NULL));
	while(walls.size() < 4 && points.size() > 200){
		int iterations = 0;
		model bestfit;
		int besterr = 0; ///Besterror is the amount of points fitting the model 
		ROS_INFO("RANSAC Nr %d", walls.size());
		while(iterations < 10000){
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
			if(thiserr > 200 && thiserr > besterr){
				ROS_INFO("New Bestmodel! %d    error: %d", iterations, thiserr);
				bestfit = maybemodel;
				besterr = thiserr;
			}
			iterations++;
			//ROS_INFO("Model worked %d    error: %d", iterations, thiserr);
		}
		if(besterr >= 200){
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
			ROS_INFO("bestfit: %f *x + %f  error: %d    >>>WALLS: %d", bestfit.m, bestfit.c, besterr, walls.size());
			ROS_INFO("New points size: %d", points.size());	
		}else{
			ROS_INFO("No acceptable bestfit found");
			break;
		}
		
		//ros::Duration(7).sleep();

	}	
	position = 5;
	if(walls.size() > 1){
		solve();
	}else{
		point reference = point();
		//point reference2 = point();
		reference.x = 100.0;
		reference.y = 100.0;

		point origin = point();
		origin.x = 0.0;
		origin.y = 0.0;
		counter = 0;

		for(int i = 0; i < points.size(); i++){			
				if(distancePoint(points.at(i), origin) < distancePoint(reference, origin)){
					reference.x = points.at(i).x;
					reference.y = points.at(i).y;
					counter++;
				}	
		}

		model newLine = model();
		newLine.m = -1/walls.at(0).m;
		newLine.c = reference.y - newLine.m * reference.x;
		walls.push_back(newLine);
		solve();
	}
	
}

double Plan::distance(struct point p, struct model m1){
	double temp = m1.m * p.x - p.y + m1.c ; 
	if (temp < 0.0 ) temp = - temp ; 
	double racine = sqrt (m1.m * m1.m + 1.0*1.0);
	double distance = temp / racine ; 
	return distance;
}

void Plan::laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	if(align){
	if(position == 4){
		ransac();
	}
		if(ready == 1 && position < 4){
		ROS_INFO("LaserCallBack Position: %d", position);
			ready = 0;
			for(int i = 0; i < 512; i++){
				double dist = msg->ranges[i];
				
				if(!isnan(dist) && dist != 0.0){
					points.push_back(point());
					points.at(counter).x = dist * cos(((M_PI / 511.0) * i) + (M_PI/2 * position));
					points.at(counter).y = dist * sin(((M_PI / 511.0) * i) + (M_PI/2 * position));
					switch(position){
						case 0:
						points[counter].y += 0.13;
						break;
						case 1:
						points[counter].x -= 0.13;
						break;
						case 2:
						points[counter].y -= 0.13;
						break;
						case 3:
						points[counter].x += 0.13;
						break;
						default: 
						break;
					}
					//if(i % 4 == 0){ 
					//ROS_INFO("PUSH i range: %d   dist: %f    x: %f  y: %f", i, dist, points.at(counter).x,points.at(counter).y);}
					counter++;

					//counter++;
				}
			}
			ROS_INFO("LaserCallBack points SIZE: %d", points.size());
			position++;
			rotateLeft(M_PI/2.0);
			

		}
		} else if(executing){
	
		}
		
	}

	
void Plan::alignToCell(){
	ROS_INFO("Align begins\n");
	align = true;
	ros::Subscriber sub1 = rosnode->subscribe("scan_filtered", 1, &Plan::laserCallback, this);
	ROS_INFO("Subscriber works1\n");
	while(align){
		ros::spinOnce();
	}
}



bool Plan::execute(lilac_fundamentals::ExecutePlan::Request &req, lilac_fundamentals::ExecutePlan::Response &res){
	
	/*rotateLeft(M_PI/2.0);
	ros::Duration(5).sleep();
	rotateRight(M_PI/2.0);
	*/
	

	for(int i = 0; i < req.plan.size(); i++){
		plan.push_back(req.plan[i]);	
	}
	alignToCell();
	executePlan();
	//ros::Duration(30).sleep();
	res.success = true;
	return true;

}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "Plan");
	ros::NodeHandle n;
	Plan p1;
	p1.rosnode = &n;
	
	//ros::Subscriber sub2 = n.subscribe("sensor_packet", 1, &Plan::sensorCallback, &a1);
	p1.diffDrive = n.serviceClient<create_fundamentals::DiffDrive>("diff_drive");
	create_fundamentals::DiffDrive srv;
	

	ros::ServiceServer service = n.advertiseService("execute_plan", &Plan::execute, &p1);
	
	ROS_INFO("Ready to execute plan 2");
	
	ros::spin();
	return 0;
}


			
			
