#include "ros/ros.h"
#include "std_msgs/Bool.h"

#include <iostream>

#include "eve_main/EndEffectorPosition.h"
#include "eve_main/GetPosition.h"

float eeXPosition = 0;
float eeYPosition = 0;
float eeZPosition = 0;

bool liftingY = false;
bool harvesting = true; // only publishing harvesting to bypass servoing
bool calibrationDone = false;
bool liftingDone = false;

float currentY = 0;

void updateEndEffectorPosition(const eve_main::EndEffectorPosition::ConstPtr& msg) {
	eeXPosition = msg -> xPosition;
	eeYPosition = msg -> yPosition;
	eeZPosition = msg -> zPosition;
}

void updateCalibrationDone(const std_msgs::Bool::ConstPtr& msg) {
	calibrationDone = msg->data;
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "test_y");
	ros::NodeHandle nh;

	ros::Publisher liftingYPub = nh.advertise<std_msgs::Bool>("lifting_y", 10);
	ros::Publisher harvestingPub = nh.advertise<std_msgs::Bool>("harvesting", 10);

	ros::Subscriber endEffectorPositionSub = nh.subscribe("end_effector_position", 10, updateEndEffectorPosition);
	ros::Subscriber calibrationDoneSub = nh.subscribe("calibration_done", 10, updateCalibrationDone);

	ros::ServiceClient client = nh.serviceClient<eve_main::GetPosition>("get_position");
	eve_main::GetPosition srv;

	ros::Rate rate(1000);


	while(ros::ok()){
		std_msgs::Bool harvestingMsg;
		std_msgs::Bool liftingYMsg;
		
		if(calibrationDone && !liftingDone) {
			harvestingMsg.data = harvesting;
			harvestingPub.publish(harvestingMsg);
		
			client.call(srv);
			currentY = srv.response.yPosition;

			std::cout << "Y BEFORE MOVE: " << currentY << std::endl;		
			
			while(abs(eeYPosition - currentY) <= 1130) {
				std::cout << "yDiff: " << abs(eeYPosition - currentY) << std::endl;
				liftingY = true;
				liftingYMsg.data = liftingY;
				liftingYPub.publish(liftingYMsg);

				ros::spinOnce();
				rate.sleep();
			}

			liftingDone = true;

			usleep(5000);

			liftingY = false;
			liftingYMsg.data = liftingY;
			liftingYPub.publish(liftingYMsg);

			client.call(srv);
			currentY = srv.response.yPosition;
			std::cout << "Y AFTER MOVE: " << currentY << std::endl;
		}

		ros::spinOnce();
		rate.sleep();
	}

	return(0);
}
