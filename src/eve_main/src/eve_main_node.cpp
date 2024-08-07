#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Empty.h"

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

const int totalPlantsPerVine = 3; // number of plants to be harvested on each vine
const int totalColumns = 2; // number of vines on a greenhouse row

bool dryRunMode = false; // turn to true if you want to skip gripping and cutting to check if positioning and image processing works without actually harvesting the plants

const float dropZoneX = 0; // location in x (mm) at which to drop plants
const float dropZoneZ = 150; // location in z (mm) at which to drop plants
const float goToPositionSpeed = 150; // speed (mm/s) at which to move both towards and away from drop zone at
const float homingYSpeed = 100; // speed (mm/s) at which to move y-axis to home y

int harvestCount = 0; // variable tracking how many harvests have been done, to know when top of vine column is reached
int columnCount = 0; // variable tracking how many vines have been harvested, to know when the end of the greenhouse row is reached
int curLimit = 30; // current limit for the dynamixels in the Grip class
int goalCur = 200; // 


// local state variables for storing ROS message information
bool harvesting = false;
bool initialCenteringDone = false;
bool harvestZoneDetected = false;
bool liftingY = false;
bool columnDone = false;
bool haltZServoing = false;
bool allColumnsDone = false;

bool tugbotGripperAttached = false;
bool tugbotCenteredSuccess = false;

float eeXPosition = 0;
float eeYPosition = 0;
float eeZPosition = 0;





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

void updateTugbotGripperAttached(const std_msgs::Bool::ConstPtr& msg) {
	tugbotGripperAttached = msg -> data;
}

void updateTugbotCenteredSuccess(const std_msgs::Bool::ConstPtr& msg) {
	tugbotCenteredSuccess = msg -> data;
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
	ros::Publisher haltZServoingPub = nh.advertise<std_msgs::Bool>("halt_servoing", 10);
	ros::Publisher initialCenteringDonePub = nh.advertise<std_msgs::Bool>("initial_centering_done", 10);
	ros::Publisher allColumnsDonePub = nh.advertise<std_msgs::Bool>("all_columns_done", 10);
	ros::Publisher tugbotCenterGripperPub = nh.advertise<std_msgs::Empty>("gripper/center", 10);
	ros::Publisher tugbotGripperAttachPub = nh.advertise<std_msgs::Bool>("gripper/attach", 10);

	ros::Subscriber endEffectorPositionSub = nh.subscribe("end_effector_position", 10, updateEndEffectorPosition);
	ros::Subscriber initialCenteringDoneSub = nh.subscribe("initial_centering_done", 10, updateInitialCenteringDone);
	ros::Subscriber harvestZoneDetectedSub = nh.subscribe("harvest_zone_detected", 10, updateHarvestZoneDetected);
	ros::Subscriber tugbotGripperAttachedSub = nh.subscribe("gripper/attach_sucess", 10, updateTugbotGripperAttached); // the topic names have success misspelled because it is misspelled in the tugbot's software. lmao portugal moment
	ros::Subscriber tugbotCenteredSuccessSub = nh.subscribe("gripper/center_sucess", 10, updateTugbotCenteredSuccess);

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
	
	std_msgs::Empty tugbotCenterMsg;
	tugbotCenterGripperPub.publish(tugbotCenterMsg);
	ros::spinOnce();
	rate.sleep();



	//std::cout << "gripper centered" << std::endl;
	//usleep(5000000);

	//while(!tugbotGripperAttached) {
	//	std_msgs::Bool tugbotGripperAttachMsg;

	//	tugbotGripperAttachMsg.data = true;
	//	tugbotGripperAttachPub.publish(tugbotGripperAttachMsg);

	//	ros::spinOnce();
	//	rate.sleep();
	//}

	//std::cout << "gripper attached" << std::endl;
	//usleep(5000000);


	while(ros::ok() && !allColumnsDone){
		std_msgs::Bool liftingYMsg;
		std_msgs::Bool harvestingMsg;
		std_msgs::Bool harvestZoneDetectedMsg;
		std_msgs::Bool columnDoneMsg;
		std_msgs::Bool haltZServoingMsg;
		std_msgs::Bool initialCenteringDoneMsg;
		std_msgs::Bool allColumnsDoneMsg;

		if(initialCenteringDone && !harvestZoneDetected && !columnDone) { // checking for if upwards y movement should occur
			// publish true for liftingY to allow upwards y movement
			liftingY = true;
			liftingYMsg.data = true;
			liftingYPub.publish(liftingYMsg);
		} else if(harvestZoneDetected && !columnDone) { // checking if harvest zone has been reached and if the column still has plants to be harvested to start the harvesting process

			getPositionClient.call(getPositionService); // collect current position at this moment in time to use for later

			float currentY = getPositionService.response.yPosition; // extract y value from current position

			std::cout << "cup in position, stopping servoing" << std::endl;

			// publishing true for harvesting to stop servoing in both x and z axis
			harvesting = true;
			harvestingMsg.data = harvesting;
			harvestingPub.publish(harvestingMsg);
			ros::spinOnce();


			std::cout << "lifting in y to get to top of cup" << std::endl;

			// this portion is still being worked on, will be removed once image processing is updated
			while(abs(eeYPosition - currentY) <= 33) {
				liftingY = true;
				liftingYMsg.data = true;
				liftingYPub.publish(liftingYMsg);

				ros::spinOnce();
				rate.sleep();
			}

			// stopping lifting in y, as we are now at harvesting zone
			liftingY = false;
			liftingYMsg.data = liftingY;
			liftingYPub.publish(liftingYMsg);

			ros::spinOnce();
			rate.sleep();

			std::cout << "y movement done, at top of cup" << std::endl;
			std::cout << "gripping and cutting" << std::endl;

			if(!dryRunMode) { // performing the grip and cut as long as dryRunMode is off
				grip(servo1, servo2);
				usleep(50000);
				cutter.cutPlant();
			} else {
				usleep(500000);
			}

			// Getting current position of end effector at this moment in time. Will be stored so end effector can return back after the drop operation
			getPositionClient.call(getPositionService);
			float currentX = getPositionService.response.xPosition;
			currentY = getPositionService.response.yPosition;
			float currentZ = getPositionService.response.zPosition;


			// Moving end effector to drop zone to perform the drop
			goToPositionService.request.desiredXPosition = dropZoneX;
			goToPositionService.request.desiredZPosition = dropZoneZ;
			goToPositionService.request.speed = goToPositionSpeed;
			goToPositionClient.call(goToPositionService);

			// Dropping the plant
			std::cout << "dropping" << std::endl;
			drop(servo1, servo2);
			usleep(500000);

			harvestCount++; // incrementing the harvestCount now that plant is dropped


			// Check if the newly harvested plant is the topmost plant in the vine. If it is, mark column as done
			if(harvestCount == totalPlantsPerVine) {
				columnDone = true;
				columnCount++;
			} else { // if plant is not top plant, perform steps to stage end effector for next plant
				std::cout << "returning to harvest zone" << std::endl;

				// Returning to previously stored vine position
				goToPositionService.request.desiredXPosition = currentX;
				goToPositionService.request.desiredZPosition = currentZ;
				goToPositionService.request.speed = goToPositionSpeed;
				goToPositionClient.call(goToPositionService);

				// set harvesting to false to allow for servoing again
				harvesting = false;
				harvestingMsg.data = harvesting;
				harvestingPub.publish(harvestingMsg);
				ros::spinOnce();

				currentY = getPositionService.response.yPosition; // collect current yPosition at this moment in time in order to lift up a set amount. This lift will allow x-servoing but disable z-servoing to prevent the end effector from following the back vine rib

				std::cout << "lifting to next servo zone" << std::endl;
				
				// Move in y with z-servoing disabled for 120mm
				while(abs(eeYPosition - currentY) <= 120) {
					liftingY = true;
					liftingYMsg.data = true;
					liftingYPub.publish(liftingYMsg);

					haltZServoing = true;
					haltZServoingMsg.data = true;
					haltZServoingPub.publish(haltZServoingMsg);

					ros::spinOnce();
					rate.sleep();
				}

				std::cout << "starting to servo again" << std::endl;
			}

			// Updating local variables to publish as ROS messages to signify that harvesting is completely done
			liftingY = false;
			harvestZoneDetected = false;
			harvesting = false;
			haltZServoing = false;
		
		}

		// If column is done (last plant has been harvested), then reset state variables to initial values and call homeY service to reset robot back to bottom of the rail
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

		// Checking if all columns have been completed, update local variables for publishing to ROS message
		if(columnCount >= totalColumns) {
			allColumnsDone = true;
		} else {
			allColumnsDone = false;
		}

		

		// Setting and publishing ROS messages
		liftingYMsg.data = liftingY;
		harvestingMsg.data = harvesting;
		harvestZoneDetectedMsg.data = harvestZoneDetected;
		columnDoneMsg.data = columnDone;
		haltZServoingMsg.data = haltZServoing;
		allColumnsDoneMsg.data = allColumnsDone;

		liftingYPub.publish(liftingYMsg);
		harvestingPub.publish(harvestingMsg);
		harvestZoneDetectedPub.publish(harvestZoneDetectedMsg);
		columnDonePub.publish(columnDoneMsg);
		haltZServoingPub.publish(haltZServoingMsg);
		allColumnsDonePub.publish(allColumnsDoneMsg);



		ros::spinOnce();
		rate.sleep();
	}
}
