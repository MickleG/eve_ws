#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int32.h"

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <wiringPi.h>

#include "eve_main/EndEffectorPosition.h"
#include "eve_main/GetPosition.h"
#include "eve_main/GoToPosition.h"
#include "eve_main/HomeY.h"
#include "eve_main/Cutter.h"
#include "eve_main/Grip.h"

const int totalPlantsPerVine = 3;
const int totalColumns = 3;

bool harvesting = false;
bool initialCenteringDone = false;
bool harvestZoneDetected = false;
bool liftingY = false;
bool columnDone = false;
bool haltServoing = false;
bool allColumnsDone = false;

bool dryRunMode = false;

float eeXPosition = 0;
float eeYPosition = 0;
float eeZPosition = 0;

float dropZoneX = 0;
float dropZoneZ = 150;
float bottomY = 0;

float goToPositionSpeed = 150;
float homingYSpeed = 100;


int harvestCount = 0;
int columnCount = 0;
int curLimit = 30;
int goalCur = 200;



using namespace std;

void updateEndEffectorPosition(const eve_main::EndEffectorPosition::ConstPtr& msg) {
	eeXPosition = msg -> xPosition;
	eeYPosition = msg -> yPosition;
	eeZPosition = msg -> zPosition;
}

void updateInitialCenteringDone(const std_msgs::Bool::ConstPtr& msg) {
	initialCenteringDone = msg -> data;
}

void updateHarvestZoneDetected(const std_msgs::Bool::ConstPtr& msg) {
	harvestZoneDetected = msg -> data;
}


int main(int argc, char **argv) {
	// Starting ROS
	ros::init(argc, argv, "eve_main_node");
	ros::NodeHandle nh;

	// Establishing ROS Publishers and Subscribers
	ros::Publisher liftingYPub = nh.advertise<std_msgs::Bool>("lifting_y", 10);
	ros::Publisher harvestingPub = nh.advertise<std_msgs::Bool>("harvesting", 10);
	ros::Publisher harvestZoneDetectedPub = nh.advertise<std_msgs::Bool>("harvest_zone_detected", 10);
	ros::Publisher columnDonePub = nh.advertise<std_msgs::Bool>("column_done", 10);
	ros::Publisher haltServoingPub = nh.advertise<std_msgs::Bool>("halt_servoing", 10);
	ros::Publisher initialCenteringDonePub = nh.advertise<std_msgs::Bool>("initial_centering_done", 10);
	ros::Publisher allColumnsDonePub = nh.advertise<std_msgs::Bool>("all_columns_done", 10);

	ros::Subscriber endEffectorPositionSub = nh.subscribe("end_effector_position", 10, updateEndEffectorPosition);
	ros::Subscriber initialCenteringDoneSub = nh.subscribe("initial_centering_done", 10, updateInitialCenteringDone);
	ros::Subscriber harvestZoneDetectedSub = nh.subscribe("harvest_zone_detected", 10, updateHarvestZoneDetected);

	// Establishing ROS Client for Service calls
	ros::ServiceClient getPositionClient = nh.serviceClient<eve_main::GetPosition>("get_position");
	ros::ServiceClient goToPositionClient = nh.serviceClient<eve_main::GoToPosition>("go_to_position");
	ros::ServiceClient homeYClient = nh.serviceClient<eve_main::HomeY>("home_y");

	eve_main::GetPosition getPositionService;
	eve_main::GoToPosition goToPositionService;
	eve_main::HomeY homeYService;

	ros::Rate rate(1000);

	// Instantiating cutter and gripper objects
	Cutter cutter(16);

	//Create a servo motor: servo*(dynamyxel ID, mode, currentlimit, goalcurrent, min angle, max angle)
    MotorXM430 servo1(1, 5, curLimit, goalCur, 225, 315);
    MotorXM430 servo2(2, 5, curLimit, goalCur, 315, 225);

    // servo1.PrintOperatingMode();
    // servo2.PrintOperatingMode();

    servo1.SetProfile(500, 400); // velocity=32000
    servo2.SetProfile(500, 400);

	cutter.resetCut();
	drop(servo1, servo2);

	usleep(100000); // add delay to prevent motor jerk
	

	while(ros::ok() && !allColumnsDone){
		std_msgs::Bool liftingYMsg;
		std_msgs::Bool harvestingMsg;
		std_msgs::Bool harvestZoneDetectedMsg;
		std_msgs::Bool columnDoneMsg;
		std_msgs::Bool haltServoingMsg;
		std_msgs::Bool initialCenteringDoneMsg;
		std_msgs::Bool allColumnsDoneMsg;

		if(initialCenteringDone && !harvestZoneDetected && !columnDone) {
			liftingY = true;
			liftingYMsg.data = true;
			liftingYPub.publish(liftingYMsg);
		} else if(harvestZoneDetected && !columnDone) {

			getPositionClient.call(getPositionService);

			float currentY = getPositionService.response.yPosition;

			std::cout << "cup in position, stopping servoing" << std::endl;

			harvesting = true;
			harvestingMsg.data = harvesting;
			harvestingPub.publish(harvestingMsg);
			ros::spinOnce();


			std::cout << "lifting in y to get to top of cup" << std::endl;


			while(abs(eeYPosition - currentY) <= 1) {
				liftingY = true;
				liftingYMsg.data = true;
				liftingYPub.publish(liftingYMsg);

				ros::spinOnce();
				rate.sleep();
			}

			
			liftingY = false;
			liftingYMsg.data = liftingY;
			liftingYPub.publish(liftingYMsg);

			ros::spinOnce();
			rate.sleep();

			std::cout << "y movement done, at top of cup" << std::endl;
			std::cout << "gripping and cutting" << std::endl;

			if(!dryRunMode) {
				grip(servo1, servo2);
				usleep(50000);
				cutter.cutPlant();
			}

			// Getting current position of end effector
			getPositionClient.call(getPositionService);

			float currentX = getPositionService.response.xPosition;
			currentY = getPositionService.response.yPosition;
			float currentZ = getPositionService.response.zPosition;


			// Returning to home
			goToPositionService.request.desiredXPosition = dropZoneX;
			goToPositionService.request.desiredZPosition = dropZoneZ;
			goToPositionService.request.speed = goToPositionSpeed;

			goToPositionClient.call(goToPositionService);

			std::cout << "dropping" << std::endl;
			drop(servo1, servo2);
			usleep(500000);

			harvestCount++;


			
			if(harvestCount == totalPlantsPerVine) {
				columnDone = true;
				columnCount++;
			} else {
				std::cout << "returning to harvest zone" << std::endl;

				// Returning to plant vine
				goToPositionService.request.desiredXPosition = currentX;
				goToPositionService.request.desiredZPosition = currentZ;
				goToPositionService.request.speed = goToPositionSpeed;

				goToPositionClient.call(goToPositionService);

				
				harvesting = false;
				harvestingMsg.data = harvesting;
				harvestingPub.publish(harvestingMsg);
				ros::spinOnce();

				currentY = getPositionService.response.yPosition;

				std::cout << "lifting to next servo zone" << std::endl;
				
				while(abs(eeYPosition - currentY) <= 120) {
					liftingY = true;
					liftingYMsg.data = true;
					liftingYPub.publish(liftingYMsg);

					haltServoing = true;
					haltServoingMsg.data = true;
					haltServoingPub.publish(haltServoingMsg);

					ros::spinOnce();
					rate.sleep();
				}

				std::cout << "starting to servo again" << std::endl;
			}

			liftingY = false;
			harvestZoneDetected = false;
			harvesting = false;
			haltServoing = false;
		
		}

		if(columnDone) {

			std::cout << "column done, going to new column" << std::endl;

			initialCenteringDone = false;
			initialCenteringDoneMsg.data = false;
			initialCenteringDonePub.publish(initialCenteringDoneMsg);

			ros::spinOnce();
			rate.sleep();

			homeYService.request.speed = homingYSpeed;
			homeYClient.call(homeYService);

			columnDone = false;
			

			harvestCount = 0;

			printf("homing done\n");

		}

		if(columnCount >= totalColumns) {
			allColumnsDone = true;
		} else {
			allColumnsDone = false;
		}

		

		liftingYMsg.data = liftingY;
		harvestingMsg.data = harvesting;
		harvestZoneDetectedMsg.data = harvestZoneDetected;
		columnDoneMsg.data = columnDone;
		haltServoingMsg.data = haltServoing;
		allColumnsDoneMsg.data = allColumnsDone;

		liftingYPub.publish(liftingYMsg);
		harvestingPub.publish(harvestingMsg);
		harvestZoneDetectedPub.publish(harvestZoneDetectedMsg);
		columnDonePub.publish(columnDoneMsg);
		haltServoingPub.publish(haltServoingMsg);
		allColumnsDonePub.publish(allColumnsDoneMsg);



		ros::spinOnce();
		rate.sleep();
	}
}
