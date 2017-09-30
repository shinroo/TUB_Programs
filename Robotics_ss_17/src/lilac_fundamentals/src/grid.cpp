#include "ros/ros.h"
#include <cstdlib>
#include <cmath>
#include "sensor_msgs/LaserScan.h"
#include "DiffDrive.h"
#include "SensorPacket.h"
#include <math.h>
#include <float.h>
#include <time.h>


#define WHEEL_RAD 3
#define ROBOT_DIAMETER 26.5

struct point    
  {
    double x;
    double y;
  };

struct model    
  {
    double m; // coeff
    double c; // offset 
  };

struct dest 
{
	double road ; 
	double angle ; 
};
class Align
{
	public:
		Align();
		//int counter;
		
		std::vector<struct point> points;
		std::vector<struct model> walls;
		int position;
		int ready;
		int counter;

		ros::ServiceClient diffDrive;
		
		
		create_fundamentals::DiffDrive msg_forward;
		create_fundamentals::DiffDrive msg_stop;
		create_fundamentals::DiffDrive msg_turn_left;
		create_fundamentals::DiffDrive msg_turn_right;
  	
		void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
		void sensorCallback(const create_fundamentals::SensorPacket::ConstPtr& msg);
		void rotate(double angle);
		void ransac();
		double distance(struct point p, struct model m1);
		struct point Mitte(struct std::vector<struct model>)
} a1;

Align::Align()
{	
	msg_turn_right.request.left = -3;
	msg_turn_right.request.right = 3;

	msg_forward.request.left = 3;
	msg_forward.request.right = 3;

	msg_stop.request.left = 0;
	msg_stop.request.right = 0;

	msg_turn_left.request.left = 3;
	msg_turn_left.request.right = -3;

	position = 0;
	ready = 1;
	counter = 0;
}

//TODO


void Align::rotate(double angle){

        
    double seconds =  (angle * ROBOT_DIAMETER * M_PI) / (2 * M_PI * 3.0 * WHEEL_RAD);

	seconds = seconds - (9.15*(seconds / 100));

	//stop
	diffDrive.call(msg_turn_right);



        //sleep until at correct angle
        ros::Duration(seconds).sleep();

        //stop rotating
        diffDrive.call(msg_stop);
        ros::Duration(7).sleep();
        ready = 1;
	ROS_INFO("Rotate Position: %d", position);
}

void Align::ransac()
{
	ROS_INFO("RANSAC TIME");
	srand (time(NULL));
	while(walls.size() < 4 && points.size() > 50){
		int iterations = 0;
		model bestfit;
		int besterr = 0; ///Besterror is the amount of points fitting the model 
		ROS_INFO("RANSAC Nr %d", walls.size());
		while(iterations < 2000){
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
		
		ros::Duration(7).sleep();

	}	
	position = 5;
}

struct point Align::Mitte(struct std::vector<struct model> w)
	struct model m1 = w.begin() ;
	double x , y ;  
	//if (m1.m * m1.next.m + 1 * 1 == 0.0)
	//{
		x = (m1.next.c - m1.c) / (m1.m - m1.next.m);
		y = m1.m * x + m1.c ; 
		double d1,d2,d3,d4; 
		d1 = sqrt ((x-0.78)*(x-0.78));
		d2 = sqrt ((x+0.78)*(x-0.78));
		d3 = sqrt ((x-0.78)*(x+0.78));
		d4 = sqrt ((x+0.78)*(x+0.78));	
		
	//}	
}
double distanceBetween2Pts(struct point p1 , struct point p2)
{
	return sqrt((p2.x-p1.x)*(p2.x-p1.x)+(p2.y-p1.y)*(p2.y-p1.y));
}
struct dest disAndAng(struct point p)
{	struct dest s = new dest() ; 
	if (p == NULL | p.x.isnan() | p.y.isnan())
		return NULL ; 
	s.road = sqrt(p.x*p.x+p.y*p.y);
	s.angle = atan2(p.y,p.x) * 180/ M_PI ; 

	return s ; 

}

double Align::distance(struct point p, struct model m1){
	double temp = m1.m * p.x - p.y + m1.c ; 
	if (temp < 0.0 ) temp = - temp ; 
	double racine = sqrt (m1.m * m1.m + 1.0*1.0);
	double distance = temp / racine ; 
	return distance;
}


void Align::laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
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
			rotate(M_PI/2.0);
			

		}
		
	}

	


void Align::sensorCallback(const create_fundamentals::SensorPacket::ConstPtr& msg)
{
	//ROS_INFO("left encoder: %f, right encoder: %f", msg->encoderLeft, msg->encoderRight);
}




int main(int argc, char **argv)
{
	Align a1;
	ros::init(argc, argv, "Align");
	ros::NodeHandle n;
	
	//ros::Subscriber sub2 = n.subscribe("sensor_packet", 1, &Align::sensorCallback, &a1);
	a1.diffDrive = n.serviceClient<create_fundamentals::DiffDrive>("diff_drive");
	
	create_fundamentals::DiffDrive srv;
	
	
	
	ros::Subscriber sub1 = n.subscribe("scan_filtered", 1, &Align::laserCallback, &a1);
	


	
	
	ros::spin();
	return 0;
}


			
			
