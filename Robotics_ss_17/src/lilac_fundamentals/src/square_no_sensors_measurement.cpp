#include <math.h>
#include <cstdlib>
#include <sstream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "create_fundamentals/DiffDrive.h"
#include "create_fundamentals/SensorPacket.h"
#include "create_fundamentals/ResetEncoders.h"

#define WHEEL_RAD 3
#define ROBOT_DIAMETER 26.5

void sensorCallback(const create_fundamentals::SensorPacket::ConstPtr& msg){
	ROS_INFO("left encoder: %f right encoder: %f", msg->encoderLeft, msg->encoderRight);
}

void move(double distance, double velocity, ros::ServiceClient *diffDrive){
        create_fundamentals::DiffDrive srv;

        //double seconds = distance / velocity;
        //double angular_velocity = velocity / WHEEL_RAD;

	double seconds = 11.111111;
	double angular_velocity = 3;

	//stop
        srv.request.left = 0;
        srv.request.right = 0;
        diffDrive->call(srv);

	ros::Duration(0.1).sleep();

        // move
        srv.request.left = angular_velocity;
        srv.request.right = angular_velocity;
        diffDrive->call(srv);

        ros::Duration(seconds).sleep();

        // stop
        srv.request.left = 0;
        srv.request.right = 0;
        diffDrive->call(srv);

	ros::spinOnce();
}

void rotate(double angle, double rotational_velocity, ros::ServiceClient *diffDrive){

        create_fundamentals::DiffDrive srv;
        double seconds =  (angle * ROBOT_DIAMETER * M_PI) / (2 * M_PI * rotational_velocity * WHEEL_RAD);

	seconds = seconds - (9.15*(seconds / 100));

	//stop
	srv.request.left = 0;
	srv.request.right = 0;
	diffDrive->call(srv);

	ros::Duration(0.1).sleep();

        //rotate
        srv.request.left = -1 * rotational_velocity;
        srv.request.right = rotational_velocity;
        diffDrive->call(srv);

        //sleep until at correct angle
        ros::Duration(seconds).sleep();

        //stop rotating
        srv.request.left = 0;
        srv.request.right = 0;
        diffDrive->call(srv);

	ros::spinOnce();
}

int main(int argc, char **argv){
        ros::init(argc, argv, "rotate");
        ros::NodeHandle n;
        ros::ServiceClient diffDrive = n.serviceClient<create_fundamentals::DiffDrive>("diff_drive");
	ros::ServiceClient resetEncoders = n.serviceClient<create_fundamentals::ResetEncoders>("reset_encoders");
	ros::Subscriber sub = n.subscribe("sensor_packet", 1, sensorCallback);

	create_fundamentals::ResetEncoders encoderSrv;
	resetEncoders.call(encoderSrv);

        //make a square
        for (int i = 0; i < 4; i++){
		resetEncoders.call(encoderSrv);
                rotate((M_PI/2), 3, &diffDrive);
		resetEncoders.call(encoderSrv);
                move(100, 10, &diffDrive);
		resetEncoders.call(encoderSrv);
        }

        return 0;
}

