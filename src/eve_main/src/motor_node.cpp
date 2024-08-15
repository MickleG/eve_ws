#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Int8.h"
#include "geometry_msgs/Twist.h"

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

const int hardwareBuffer = 57; // distance in mm from front of Intel RealSense D405 camera to the end of the scissor sheath (the flat portion furthest from the realsense lens). This is driven from the hardware, so if this dimension changes, this variable needs to change accordingly
const int desiredZDistance = -10; // distance in mm that should be targeted for z-axis visual-servoing between the front of the scissor sheath and the front rib of the vine. This value is negative if the scissor sheath extends past the vine rib (which currently is the case)
const int zDeadbandBuffer = 5; // margin of error in mm allowed for z-axis visual servoing
const int xDeadbandBuffer = 15; // margin of error in pixels allowed for x-axis visual servoing

int ySpeedsMotorTesting[4] = {70, 0, -50, 0}; // Speeds that eve loops through when testing motor
int ySpeed = 250;


// local state variables for storing ROS message information
bool harvesting = false;
bool findingZ = false;
bool blueDetected = false;
bool harvestZoneDetected = false;
bool initialCenteringDone = false;
bool liftingY = false;
bool calibrationDone = false;
bool columnDone = false;
bool haltZServoing = false;
bool allColumnsDone = false;
bool cameraRunning = false;

bool testMotor = false; // set to true if you want to test motor movement without full autonomy


int xOffset = 0; // used for storing current xOffset in pixels that the vine rib is from the center of the camera frame. Used for x-axis visual servoing - this value should be within xDeadbandBUffer pixels of center of realsense image frame when x-axis visual servoing is complete
int zOffset = 0; // used for storing the distance in mm between the front of the vine rib and the end of the scissor sheath - this value should be within zDeadbandBuffer mm of desiredZDistance when z-axis visual servoing is complete 

EndEffectorConfig mechanism(0, 0); // initializing EndEffectorConfig object for the robot's end effector

using namespace std;


// ***Advertised ROS service functions*** //

// Retrieves instantaneous position of end effector
bool getPosition(eve_main::GetPosition::Request &req, eve_main::GetPosition::Response &res) {
	res.xPosition = mechanism.xPosition;
	res.yPosition = mechanism.yPosition;
	res.zPosition = mechanism.zPosition;
	return true;
}

// Goes to requested x and z position at speed
bool goToPosition(eve_main::GoToPosition::Request &req, eve_main::GoToPosition::Response &res) {
	float targetX = req.desiredXPosition;
	float targetZ = req.desiredZPosition;
	float speed = req.speed;

	mechanism.goToPosition(targetX, targetZ, 100);
	mechanism.updateCurrentPosition();

	return true;
}

// Called at the end of a column, returns all state variables to their initial values and drives Y down until limit switch is triggered. Additionally performs a calibration to reinitialize x and z position
bool homeY(eve_main::HomeY::Request &req, eve_main::HomeY::Response &res) {
	
	// reset state variables back to initial values
	mechanism.centeredCounter = -1;
    mechanism.goalZ = -1;
    mechanism.xServoingSpeed = -1;
    mechanism.zServoingSpeed = -1;
    initialCenteringDone = false;

    ros::spinOnce(); // spinOnce placed here to update variable values before the time consuming y-movement. This prevents the robot from trying to servo/harvest while it's moving back down to the bottom

    // Moving down in y until limit switch is pressed
	while(mechanism.yMotor.driveState == 1) {
        mechanism.yMotor.setSpeed(-1 * req.speed); // speed limiting y stage due to lower speed cap than left and right motors
        mechanism.yMotor.controlLoopY();
        mechanism.updateCurrentPosition();
    }

    // Calibrating and returning to start position of x = 0, z = 200
    mechanism.yMotor.setStepPosition(0);
    mechanism.calibrateZero(req.speed);
    mechanism.updateCurrentPosition();
    // mechanism.goToPosition(0, 180, 150);
    // mechanism.updateCurrentPosition();

    // Reinitializing yMotor properties
    mechanism.yMotor.setSpeed(100);
    mechanism.yMotor.setAcceleration(50);
    
    return true;
}

// ***ROS Subscriber callbacks*** //
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
void updatehaltZServoing(const std_msgs::Bool::ConstPtr& msg) {
	haltZServoing = msg -> data;
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

	// Initializaton of ROS publishers
	ros::Publisher initialCenteringDonePub = nh.advertise<std_msgs::Bool>("initial_centering_done", 10);
	ros::Publisher endEffectorPositionPub = nh.advertise<eve_main::EndEffectorPosition>("end_effector_position", 10);
	//ros::Publisher tugbotLidarSafetyAreaPub = nh.advertise<std_msgs::Int8>("wheels_controller/lidar_safety_area", 10);
	//ros::Publisher tugbotCmdVelPub = nh.advertise<geometry_msgs::Twist>("wheels_controller/cmd_vel", 10);

	// Initialization of ROS subscribers with corresponding callback functions
	ros::Subscriber harvestingSub = nh.subscribe("harvesting", 10, updateHarvesting);
	ros::Subscriber goalZSub = nh.subscribe("goal_z", 10, updateGoalZ);
	ros::Subscriber blueDetectedSub = nh.subscribe("blue_detected", 10, updateBlueDetected);
	ros::Subscriber harvestZoneDetectedSub = nh.subscribe("harvest_zone_detected", 10, updateHarvestZoneDetected);
	ros::Subscriber xOffsetSub = nh.subscribe("x_offset", 10, updateXOffset);
	ros::Subscriber liftingYSub = nh.subscribe("lifting_y", 10, updateLiftingY);
	ros::Subscriber columnDoneSub = nh.subscribe("column_done", 10, updateColumnDone);
	ros::Subscriber haltZServoingSub = nh.subscribe("halt_servoing", 10, updatehaltZServoing);
	ros::Subscriber allColumnsDoneSub = nh.subscribe("all_columns_done", 10, updateAllColumnsDone);
	ros::Subscriber cameraRunningSub = nh.subscribe("camera_running", 10, updateCameraRunning);

	// Initialization of advertised services
	ros::ServiceServer getPositionService = nh.advertiseService("get_position", getPosition);
	ros::ServiceServer goToPositionService = nh.advertiseService("go_to_position", goToPosition);
	ros::ServiceServer homeYService = nh.advertiseService("home_y", homeY);

	ros::Rate rate(10000); // Setting a high ROS rate due to high frequency motor commands

	int ySpeedCounter = 0;

	// Initialize y motor properties
	mechanism.yMotor.setSpeed(ySpeed);
	mechanism.yMotor.setAcceleration(100);


	while(testMotor) {
		std::cout << "moving at speed: " << ySpeedsMotorTesting[ySpeedCounter] << std::endl;
		for(int i = 0; i < 500000; i++) {
			std_msgs::Int8 lidarSafetyAreaMsg;
			geometry_msgs::Twist cmdVelMsg;
			
			mechanism.yMotor.motorDriveY();
			mechanism.updateCurrentPosition();
			mechanism.yMotor.controlLoopY();

			lidarSafetyAreaMsg.data = 1;

			cmdVelMsg.linear.x = float(ySpeed) * 0.001;
			cmdVelMsg.linear.y = 0.0;
			cmdVelMsg.linear.z = 0.0;

			cmdVelMsg.angular.x = 0.0;
			cmdVelMsg.angular.y = 0.0;
			cmdVelMsg.angular.z = 0.0;

			//tugbotLidarSafetyAreaPub.publish(lidarSafetyAreaMsg);
			//tugbotCmdVelPub.publish(cmdVelMsg);

			//ros::spinOnce();
			//rate.sleep();
		}

		ySpeedCounter++;
		if(ySpeedCounter == 4) {
			ySpeedCounter = 0;
		}

		mechanism.yMotor.setSpeed(ySpeedsMotorTesting[ySpeedCounter]);
	}

	// Blocking continuation of ndoe execution until image_processing_node publishes that camera is running
	while(!cameraRunning) {
		ros::spinOnce();
		rate.sleep();
	}

	// Calibration and moving to starting position at x=0, z=250
	mechanism.calibrateZero(100);
	mechanism.updateCurrentPosition();

	std::cout << "calibration done, time to harvest" << std::endl;

	mechanism.yMotor.setSpeed(ySpeed);
	mechanism.yMotor.setAcceleration(100);


	// main motor loop, execution stops when the entire wall has been completed signified by allColumnsDone
	while(ros::ok() && !allColumnsDone) {

		// Initialization of ROS messages
		eve_main::EndEffectorPosition endEffectorPositionMsg;
		std_msgs::Bool initialCenteringDoneMsg;
		std_msgs::Bool calibrationDoneMsg;

		// Check for if servoing should happen. Stops if the camera stream cannot find a target z-value to servo to or if the robot is in the middle of the harvesting operation
		if(mechanism.goalZ > 0 && !harvesting) {
			// Check for if x-axis servoing should happen.
			if(!findingZ && !harvesting) {
				// If x is centered sufficiently within the xDeadbandBuffer, stop servoing by setting x speed to 0
				if(abs(xOffset) < xDeadbandBuffer) {
					mechanism.xServoingSpeed = 0;
					findingZ = true;
				} else {
					// If x is not centered sufficiently within the xDeadbandBuffer, set servoing speed to be proportional to the offset, but cap values at value specified in mechanism.maxServoingSpeed so as to not fall outside of the positioning mechanism's capabilities
					mechanism.xServoingSpeed = std::min(mechanism.maxServoingSpeed, xOffset);
					
				}

				mechanism.moveInX(-mechanism.xServoingSpeed); // negative due to how the offset is calculated in image_processing_node
				mechanism.updateCurrentPosition();
			}

			// Check for if z-axis servoing should happen, does not execute if robot is x-servoing or if the robot sees a sufficient number of blue cup pixels, or if the robot is in the middle of the harvesting operation
			if(findingZ && !blueDetected && !haltZServoing) {
				zOffset = mechanism.goalZ - hardwareBuffer; // zOffset signifies the distance from the front of the vine rib to the end of the scissor sheath (flat portion that is farthest away from camera)
				// If z is centered sufficiently within the zDeadbandBuffer, stop servoing by setting z speed to 0
				if((abs(zOffset - desiredZDistance)) < zDeadbandBuffer) {
					mechanism.zServoingSpeed = 0;
					findingZ = false;
				} else {
					// If z is not centered sufficiently within the zDeadbandBuffer, set servoing speed to be proportional to the offset, but cap values at value specified in mechanism.maxServoingSpeed so as to not fall outside of the positioning mechanism's capabilities
					mechanism.zServoingSpeed = std::min(mechanism.maxServoingSpeed, zOffset - desiredZDistance);	
				}

				mechanism.moveInZ(mechanism.zServoingSpeed);
				mechanism.updateCurrentPosition();
			}

			// Since xServoingSpeed and zServoingSpeed are initialized to -1, this check is true once both x and z have been successfully centered at least once
			if(mechanism.xServoingSpeed == 0 && mechanism.zServoingSpeed == 0) {
				mechanism.centeredCounter++;
				// Prevention of integer overflow
				if(mechanism.centeredCounter >= std::numeric_limits<int>::max()) {
					mechanism.centeredCounter = 1;
				}
			}
		}

		// Checking for if y needs to move, and is blocked by the liftingY ROS message set to false, the column being finished (all plants in column have been harvested), and if the end effector has not hugged the vine yet
		if(liftingY && !columnDone && initialCenteringDone) {
			// std::cout << "liftingY" << std::endl;
			
			mechanism.yMotor.motorDriveY();
			mechanism.updateCurrentPosition();
			mechanism.yMotor.controlLoopY();
		}

		// If both x and z have servoed successfully at least once, this variable is here to signify lifting operations as well as to trigger looking for blue cup pixels/harvesting zone
		if(mechanism.centeredCounter > 0) {
			initialCenteringDone = true;
		}

		// Update ROS messages
		endEffectorPositionMsg.xPosition = mechanism.xPosition;
		endEffectorPositionMsg.yPosition = mechanism.yPosition;
		endEffectorPositionMsg.zPosition = mechanism.zPosition;

		initialCenteringDoneMsg.data = initialCenteringDone;

		// Publish ROS messages
		endEffectorPositionPub.publish(endEffectorPositionMsg);
		initialCenteringDonePub.publish(initialCenteringDoneMsg);		

		ros::spinOnce();
		rate.sleep();
	}

	return(0);
}
