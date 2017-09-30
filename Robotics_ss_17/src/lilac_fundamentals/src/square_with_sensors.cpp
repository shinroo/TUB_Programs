// adapted version of square_no_sensors attempting to use the encoders as a feeback system to improve accuracy

#include <math.h>
#include <cstdlib>
#include <sstream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "create_fundamentals/DiffDrive.h"
#include "create_fundamentals/SensorPacket.h"
#include "create_fundamentals/ResetEncoders.h"

// These values were measured by us
#define WHEEL_RAD 3
#define ROBOT_DIAMETER 26.5

// We calculated these "desired" values to compare against for the feedback system
#define DESIRED_ROTATION 6.937683777
#define DESIRED_MOVEMENT 33.333333333

double encoderLeft = 0;
double encoderRight = 0;

void sensorCallback(const create_fundamentals::SensorPacket::ConstPtr& msg){
        //ROS_INFO("left encoder: %f, right encoder: %f", msg->encoderLeft, msg->encoderRight);

	encoderLeft = msg->encoderLeft;
	encoderRight = msg->encoderRight;

}

void move(ros::ServiceClient *diffDrive){

	ROS_INFO("---------------------MOVING--------------------------------");

        create_fundamentals::DiffDrive srv;

        // calculated values
	double seconds = 100/9;
	double angular_velocity_left = 3;
	double angular_velocity_right = 3;

	// calculate sleep value, encoder increment value for incremental movement
	int loop_index = 10;
	double loop_sleep = seconds / loop_index;
	double loop_encoder_increment = DESIRED_MOVEMENT / loop_index;

	// variables that will be incremented with ideal value after every incremental movement
	double loop_left_counter = 0;
	double loop_right_counter = 0;

	// penalty / reward value
	double penalty = -0.1;
	double reward = 0.1;

	// reset global variables
	encoderLeft = 0;
	encoderRight = 0;

	for (int i = 0; i < loop_index; i++){

		ROS_INFO("loop iteration %d encoderLeft %f encoderRight %f tLeft %f tRight %f", i, encoderLeft, encoderRight, loop_left_counter, loop_right_counter);

        	// move
		// check if left is moving too fast or too slow and adjust
		if (encoderLeft > loop_left_counter) {
        		angular_velocity_left += penalty;
		} else if (encoderLeft < loop_left_counter ) {
			angular_velocity_left += reward;
		}
		srv.request.left = angular_velocity_left;
		// check if right is moving too fast or too slow and adjust
		if (encoderRight > loop_right_counter) {
			angular_velocity_right += penalty;
		} else  if (encoderRight < loop_right_counter) {
			angular_velocity_right += reward;
		}
		srv.request.right = angular_velocity_right;
        	diffDrive->call(srv);

		ROS_INFO("loop iteration %d velocityLeft %f velocityRight %f", i, angular_velocity_left, angular_velocity_right);

        	ros::Duration(loop_sleep).sleep();

        	// stop
        	srv.request.left = 0;
        	srv.request.right = 0;
        	diffDrive->call(srv);

		ros::spinOnce();

		loop_left_counter += loop_encoder_increment;
		loop_right_counter += loop_encoder_increment;

		// reset velocities
		angular_velocity_left = 3;
        	angular_velocity_right = 3;
	}

	ROS_INFO("after movement left: %f right: %f expected: 150", encoderLeft, encoderRight);

	// reset global variables
	encoderLeft = 0;
	encoderRight = 0;
}

void rotate(ros::ServiceClient *diffDrive){

	ROS_INFO("---------------------ROTATING--------------------------------");

        create_fundamentals::DiffDrive srv;

	double rotational_velocity_left = -3;
	double rotational_velocity_right = 3;
	double angle = M_PI / 2;

	double seconds =  (angle * ROBOT_DIAMETER * M_PI) / (2 * M_PI * rotational_velocity_right * WHEEL_RAD);
	seconds = seconds - (9.15*(seconds / 100));

	//ROS_INFO("seconds: %f", seconds);

	// calculate sleep value, encoder increment value for incremental movement
        int loop_index = 10;
        double loop_sleep = seconds / loop_index;
        double loop_encoder_increment = DESIRED_ROTATION / loop_index;

        // variables that will be incremented with ideal value after every incremental movement
        double loop_left_counter = 0;
        double loop_right_counter = 0;

        // penalty / reward value
        double penalty = -0.1;
        double reward = 0.1;

        // reset global variables
        encoderLeft = 0;
        encoderRight = 0;

	for (int i = 0; i < loop_index; i++){

		ROS_INFO("loop iteration %d encoderLeft %f encoderRight %f tLeft %f tRight %f", i, encoderLeft, encoderRight, loop_left_counter, loop_right_counter);
		

        	// rotate
		// check left and right rotation encoders and compare to what they should be
		if (encoderLeft < loop_left_counter) {
                        rotational_velocity_left -= penalty;
                } else if (i != 1 && encoderLeft > loop_left_counter) {
                        rotational_velocity_left -= reward;
                }
        	srv.request.left = rotational_velocity_left;
		if (encoderRight > loop_right_counter) {
                        rotational_velocity_right += penalty;
                } else  if (i != 1 && encoderRight < loop_right_counter) {
                        rotational_velocity_right += reward;
                }
		srv.request.right = rotational_velocity_right;
        	diffDrive->call(srv);

		ROS_INFO("loop iteration %d velocityLeft %f velocityRight %f", i, rotational_velocity_left, rotational_velocity_right);

		//ROS_INFO("velLeft: %f velRight: %f", srv.request.left, srv.request.right);

        	// sleep until at correct angle
        	ros::Duration(loop_sleep).sleep();

        	// stop rotating
        	srv.request.left = 0;
        	srv.request.right = 0;
        	diffDrive->call(srv);

		ros::spinOnce();

		loop_left_counter -= loop_encoder_increment;
                loop_right_counter += loop_encoder_increment;

		rotational_velocity_left = -3;
        	rotational_velocity_right = 3;
	}

	ROS_INFO("after rotation left: %f right: %f expected: |34.5|", encoderLeft, encoderRight);

	// reset global variables
	encoderLeft = 0;
	encoderRight = 0;
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
                rotate(&diffDrive);
		resetEncoders.call(encoderSrv);
                move(&diffDrive);
		resetEncoders.call(encoderSrv);
        }
        return 0;
}

