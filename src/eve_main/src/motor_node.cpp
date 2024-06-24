#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int32.h"

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <wiringPi.h>

#include "eve_main/EndEffectorConfig.h"
#include "eve_main/MotorConfig.h"
#include "eve_main/EndEffectorPosition.h"

bool harvesting = false;
bool findingZ = false;
bool blueDetected = false;
bool initialCenteringDone = false;
bool liftingY = false;

int hardwareBuffer = 45; // distance in mm from realsense camera to end of scissor sheath
int zDeadbandBuffer = 30; // target distance in mm away from vine with respect to realsense camera
int xDeadbandBuffer = 5; // target distance away from center of frame in pixels for x visual servoing

int xServoingSpeed = -1;
int zServoingSpeed = -1;
int xOffset = 0;
int zOffset = 0;

int centeredCounter = -1;
int goalZ = -1;

EndEffectorConfig mechanism(0, 0);

using namespace std;

template <typename T> int sign(T val) {
	return (T(0) < val) - (val < T(0));
}

void updateHarvesting(const std_msgs::Bool::ConstPtr& msg) {
	harvesting = msg -> data;
}

void updateGoalZ(const std_msgs::Int32::ConstPtr& msg) {
	goalZ = msg -> data;
}

void updateBlueDetected(const std_msgs::Bool::ConstPtr& msg) {
	blueDetected = msg -> data;
}

void updateXOffset(const std_msgs::Int32::ConstPtr& msg) {
	xOffset = msg -> data;
}

void updateLiftingY(const std_msgs::Bool::ConstPtr& msg) {
	liftingY = msg -> data;
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "motor_run_node");
	ros::NodeHandle nh;

	ros::Subscriber harvestingSub = nh.subscribe("harvesting", 10, updateHarvesting);
	ros::Subscriber goalZSub = nh.subscribe("goal_z", 10, updateGoalZ);
	ros::Subscriber blueDetectedSub = nh.subscribe("blue_detected", 10, updateBlueDetected);
	ros::Subscriber xOffsetSub = nh.subscribe("x_offset", 10, updateXOffset);
	ros::Subscriber liftingYSub = nh.subscribe("lifting_y", 10, updateLiftingY);

	ros::Publisher initialCenteringDonePub = nh.advertise<std_msgs::Bool>("initial_centering_done", 10);
	ros::Publisher endEffectorPositionPub = nh.advertise<eve_main::EndEffectorPosition>("end_effector_position", 10);

	ros::Rate rate(100000);

	printf("motors running\n");

	mechanism.calibrateZero(50);
	mechanism.updateCurrentPosition();

	cout << "current Z: " << mechanism.zPosition << endl;
	cout << "Moving in Z a little to make room in X" << endl;

	mechanism.goToPosition(0, 200, 50);
	mechanism.updateCurrentPosition();

	cout << "Ready to Harvest" << endl;

	mechanism.yMotor.setSpeed(50);
	mechanism.yMotor.setAcceleration(10);

	while(ros::ok()) {
		eve_main::EndEffectorPosition endEffectorPositionMsg;
		std_msgs::Bool initialCenteringDoneMsg;
		
		if(goalZ > 0 && !harvesting) {
			if(!findingZ && !blueDetected) {
				if(abs(xOffset) < xDeadbandBuffer) {
					xServoingSpeed = 0;
					findingZ = true;
				} else {
					xServoingSpeed = xOffset;
				}

				mechanism.moveInX(-xServoingSpeed);
				mechanism.updateCurrentPosition();
			}

			if(findingZ && !blueDetected) {
				zOffset = goalZ - hardwareBuffer;
				//cout << "zOffset: " << zOffset << endl;
				if(abs(zOffset) < zDeadbandBuffer) {
					zServoingSpeed = 0;
					findingZ = false;
				} else {
					zServoingSpeed = zOffset;
				}

				mechanism.moveInZ(zServoingSpeed);
				mechanism.updateCurrentPosition();
			}

			if(xServoingSpeed == 0 && zServoingSpeed == 0) {
				centeredCounter++;
				if(centeredCounter >= std::numeric_limits<int>::max()) {
					centeredCounter = 1;
				}
			}
		}

		if(liftingY) {
			mechanism.yMotor.motorDriveY();
			mechanism.updateCurrentPosition();
		}

		if(centeredCounter == 0) {
			initialCenteringDone = true;
		}

		endEffectorPositionMsg.xPosition = mechanism.xPosition;
		endEffectorPositionMsg.yPosition = mechanism.yPosition;
		endEffectorPositionMsg.zPosition = mechanism.zPosition;

		initialCenteringDoneMsg.data = initialCenteringDone;

		endEffectorPositionPub.publish(endEffectorPositionMsg);
		initialCenteringDonePub.publish(initialCenteringDoneMsg);

		ros::spinOnce();
		rate.sleep();
	}

	return(0);
}
