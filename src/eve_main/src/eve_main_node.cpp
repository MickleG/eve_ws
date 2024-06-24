#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int32.h"

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <wiringPi.h>

#include "eve_main/EndEffectorPosition.h"

bool harvesting = false;
bool initialCenteringDone = false;
bool harvestZoneDetected = false;
bool liftingY = false;

float eeXPosition = 0;
float eeYPosition = 0;
float eeZPosition = 0;

int vAvg = 0;
int harvestCount = 0;

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

void updateVAvg(const std_msgs::Int32::ConstPtr& msg) {
	vAvg = msg -> data;
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "eve_main_node");
	ros::NodeHandle nh;

	ros::Publisher liftingYPub = nh.advertise<std_msgs::Bool>("lifting_y", 10);
	ros::Publisher harvestingPub = nh.advertise<std_msgs::Bool>("harvesting", 10);
	ros::Publisher harvestZoneDetectedPub = nh.advertise<std_msgs::Bool>("harvest_zone_detected", 10);

	ros::Subscriber endEffectorPositionSub = nh.subscribe("end_effector_position", 10, updateEndEffectorPosition);
	ros::Subscriber initialCenteringDoneSub = nh.subscribe("initial_centering_done", 10, updateInitialCenteringDone);
	ros::Subscriber harvestZoneDetectedSub = nh.subscribe("harvest_zone_detected", 10, updateHarvestZoneDetected);
	ros::Subscriber vAvgSub = nh.subscribe("v_avg", 10, updateVAvg);

	ros::Rate rate(1000);

	while(ros::ok()){
		std_msgs::Bool liftingYMsg;
		std_msgs::Bool harvestingMsg;
		std_msgs::Bool harvestZoneDetectedMsg;

		if(initialCenteringDone && !harvestZoneDetected) {
			liftingY = true;
			liftingYMsg.data = true;
			liftingYPub.publish(liftingYMsg);
		} else if(harvestZoneDetected) {
			liftingY = false;
			liftingYMsg.data = false;
			liftingYPub.publish(liftingYMsg);

			harvesting = true;
			harvestingMsg.data = true;
			harvestingPub.publish(harvestingMsg);

			float currentY = eeYPosition;

			int yDistanceToHarvestingZone;
			if(harvestCount == 0) {
				yDistanceToHarvestingZone = 950;
			} else {
				yDistanceToHarvestingZone = 1130;
			}

			std::cout << "moving up in y to get to top of cup" << std::endl;

			int bottomPixel = vAvg;

			while(abs(eeYPosition - currentY) <= (yDistanceToHarvestingZone - bottomPixel)) {
				std::cout << "yDiff: " << abs(eeYPosition - currentY) << ", buffer: " << yDistanceToHarvestingZone - bottomPixel << std::endl;
				liftingY = true;
				liftingYMsg.data = true;
				liftingYPub.publish(liftingYMsg);
			}
			
			liftingY = false;
			liftingYMsg.data = false;
			liftingYPub.publish(liftingYMsg);

			std::cout << "y movement done, at top of cup" << std::endl;
			std::cout << "gripping and cutting" << std::endl;

			usleep(500000);

			float currentX = eeXPosition;
			float currentZ = eeZPosition;

			//dropping code here
			
			while(abs(eeYPosition - currentY) <= 650) {
				liftingY = true;
				liftingYMsg.data = true;
				liftingYPub.publish(liftingYMsg);
			}

			liftingY = false;
			harvestZoneDetected = false;
			harvesting = false;
			
			harvestCount++;

			if(harvestCount == 3) {
				break;
			}
		}

		liftingYMsg.data = liftingY;
		harvestingMsg.data = harvesting;
		harvestZoneDetectedMsg.data = harvestZoneDetected;

		liftingYPub.publish(liftingYMsg);
		harvestingPub.publish(harvestingMsg);
		harvestZoneDetectedPub.publish(harvestZoneDetectedMsg);

		ros::spinOnce();
		rate.sleep();
	}


}
