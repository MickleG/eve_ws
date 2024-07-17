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
#include "eve_main/GetPosition.h"
#include "eve_main/GoToPosition.h"
#include "eve_main/HomeY.h"

bool harvesting = false;
bool findingZ = false;
bool blueDetected = false;
bool harvestZoneDetected = false;
bool initialCenteringDone = false;
bool liftingY = false;
bool calibrationDone = false;
bool columnDone = false;
bool haltServoing = false;
bool allColumnsDone = false;
bool cameraRunning = false;

int hardwareBuffer = 57; // distance in mm from realsense camera to end of scissor sheath
int minZ = 30; // distance from realsense camera to start of camera sheath
int desiredZDistance = -5;
int zDeadbandBuffer = 5; // target distance in mm away from vine with respect to realsense camera
int xDeadbandBuffer = 15; // target distance away from center of frame in pixels for x visual servoing

int xOffset = 0;
int zOffset = 0;

EndEffectorConfig mechanism(0, 0);

using namespace std;

template <typename T> int sign(T val) {
	return (T(0) < val) - (val < T(0));
}

bool getPosition(eve_main::GetPosition::Request &req, eve_main::GetPosition::Response &res) {
	res.xPosition = mechanism.xPosition;
	res.yPosition = mechanism.yPosition;
	res.zPosition = mechanism.zPosition;
	return true;
}

bool goToPosition(eve_main::GoToPosition::Request &req, eve_main::GoToPosition::Response &res) {
	float targetX = req.desiredXPosition;
	float targetZ = req.desiredZPosition;
	float speed = req.speed;

	mechanism.goToPosition(targetX, targetZ, 100);
	mechanism.updateCurrentPosition();

	return true;
}

bool homeY(eve_main::HomeY::Request &req, eve_main::HomeY::Response &res) {
	mechanism.centeredCounter = -1;
    mechanism.goalZ = -1;
    mechanism.xServoingSpeed = -1;
    mechanism.zServoingSpeed = -1;
    initialCenteringDone = false;

    ros::spinOnce();

	while(mechanism.yMotor.driveState == 1) {
        mechanism.yMotor.setSpeed(-1 * req.speed); // speed limiting y stage due to lower speed cap than left and right motors
        mechanism.yMotor.controlLoopY();
        mechanism.updateCurrentPosition();
    }

    mechanism.yMotor.setStepPosition(0);
    mechanism.calibrateZero(req.speed);
    mechanism.updateCurrentPosition();

    mechanism.goToPosition(0, 200, 150);
    mechanism.updateCurrentPosition();

    mechanism.yMotor.setSpeed(100);
    
    return true;
}

void updateHarvesting(const std_msgs::Bool::ConstPtr& msg) {
	harvesting = msg -> data;
}

void updateGoalZ(const std_msgs::Int32::ConstPtr& msg) {
	mechanism.goalZ = msg -> data;
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

void updateColumnDone(const std_msgs::Bool::ConstPtr& msg) {
	columnDone = msg -> data;
}

void updateHarvestZoneDetected(const std_msgs::Bool::ConstPtr& msg) {
	harvestZoneDetected = msg -> data;
}

void updateHaltServoing(const std_msgs::Bool::ConstPtr& msg) {
	haltServoing = msg -> data;
}

void updateAllColumnsDone(const std_msgs::Bool::ConstPtr& msg) {
	allColumnsDone = msg -> data;
}

void updateCameraRunning(const std_msgs::Bool::ConstPtr& msg) {
	cameraRunning = msg -> data;
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "motor_run_node");
	ros::NodeHandle nh;

	ros::Publisher initialCenteringDonePub = nh.advertise<std_msgs::Bool>("initial_centering_done", 10);
	ros::Publisher endEffectorPositionPub = nh.advertise<eve_main::EndEffectorPosition>("end_effector_position", 10);
	
	ros::Subscriber harvestingSub = nh.subscribe("harvesting", 10, updateHarvesting);
	ros::Subscriber goalZSub = nh.subscribe("goal_z", 10, updateGoalZ);
	ros::Subscriber blueDetectedSub = nh.subscribe("blue_detected", 10, updateBlueDetected);
	ros::Subscriber harvestZoneDetectedSub = nh.subscribe("harvest_zone_detected", 10, updateHarvestZoneDetected);
	ros::Subscriber xOffsetSub = nh.subscribe("x_offset", 10, updateXOffset);
	ros::Subscriber liftingYSub = nh.subscribe("lifting_y", 10, updateLiftingY);
	ros::Subscriber columnDoneSub = nh.subscribe("column_done", 10, updateColumnDone);
	ros::Subscriber haltServoingSub = nh.subscribe("halt_servoing", 10, updateHaltServoing);
	ros::Subscriber allColumnsDoneSub = nh.subscribe("all_columns_done", 10, updateAllColumnsDone);
	ros::Subscriber cameraRunningSub = nh.subscribe("camera_running", 10, updateCameraRunning);


	ros::ServiceServer getPositionService = nh.advertiseService("get_position", getPosition);
	ros::ServiceServer goToPositionService = nh.advertiseService("go_to_position", goToPosition);
	ros::ServiceServer homeYService = nh.advertiseService("home_y", homeY);

	ros::Rate rate(10000);

	while(!cameraRunning) {
		ros::spinOnce();
		rate.sleep();
	}


	mechanism.calibrateZero(100);
	mechanism.updateCurrentPosition();

	// cout << "current Z: " << mechanism.zPosition << endl;
	// cout << "Moving in Z a little to make room in X" << endl;

	mechanism.goToPosition(0, 200, 150);
	mechanism.updateCurrentPosition();

	// cout << "Ready to Harvest" << endl;

	mechanism.yMotor.setSpeed(100);
	mechanism.yMotor.setAcceleration(50);

	// calibrationDone = true;

	while(ros::ok() && !allColumnsDone) {
		eve_main::EndEffectorPosition endEffectorPositionMsg;
		std_msgs::Bool initialCenteringDoneMsg;
		std_msgs::Bool calibrationDoneMsg;
		
		if(mechanism.goalZ > 0 && !harvesting) {
			if(!findingZ && !harvesting) {
				if(abs(xOffset) < xDeadbandBuffer) {
					mechanism.xServoingSpeed = 0;
					findingZ = true;
				} else {
					mechanism.xServoingSpeed = xOffset;
				}

				mechanism.moveInX(-mechanism.xServoingSpeed);
				mechanism.updateCurrentPosition();
			}

			if(findingZ && !blueDetected && !haltServoing) {
				zOffset = mechanism.goalZ - hardwareBuffer;

				if((abs(zOffset - desiredZDistance)) < zDeadbandBuffer) {
					mechanism.zServoingSpeed = 0;
					findingZ = false;
				} else {
					if(zOffset >= 130) {
						zOffset = 130;
					} else {
						mechanism.zServoingSpeed = zOffset - desiredZDistance;
					}
					
				}

				// std::cout << "z_speed: " << mechanism.zServoingSpeed << std::endl;

				mechanism.moveInZ(mechanism.zServoingSpeed);
				mechanism.updateCurrentPosition();
			}

			if(mechanism.xServoingSpeed == 0 && mechanism.zServoingSpeed == 0) {
				mechanism.centeredCounter++;
				if(mechanism.centeredCounter >= std::numeric_limits<int>::max()) {
					mechanism.centeredCounter = 1;
				}
			}
		}

		if(liftingY && !columnDone && initialCenteringDone) {
			mechanism.yMotor.motorDriveY();
			mechanism.updateCurrentPosition();
			mechanism.yMotor.controlLoopY();
		}

		if(mechanism.centeredCounter >= 0) {
			initialCenteringDone = true;
		}

		endEffectorPositionMsg.xPosition = mechanism.xPosition;
		endEffectorPositionMsg.yPosition = mechanism.yPosition;
		endEffectorPositionMsg.zPosition = mechanism.zPosition;

		initialCenteringDoneMsg.data = initialCenteringDone;
		// calibrationDoneMsg.data = calibrationDone;

		endEffectorPositionPub.publish(endEffectorPositionMsg);
		initialCenteringDonePub.publish(initialCenteringDoneMsg);
		// calibrationDonePub.publish(calibrationDoneMsg);
		

		ros::spinOnce();
		rate.sleep();
	}

	return(0);
}
