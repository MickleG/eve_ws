#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int32.h"

#include <stdio.h>
#include <stdlib.h>
#include <wiringPi.h>
#include <iostream>
#include <vector>
#include <algorithm>
#include <unistd.h>
#include <opencv2/opencv.hpp>
#include <librealsense2/rs.hpp>


const int resolution[2] = {424, 240}; // resolution to capture original rgb and depth streams at
const int cropped_gripper_bounds[2] = {130, 400}; // resolution that crops out grippers and scissor sheath from field of view
const int num_min_vine_rib_points = 2000; // desired number of points to find the average minimum z value for along the vine's front rib
const int num_min_cup_points = 200; // desired number of cup pixels used to extract minimum depth values from to detect top of cup
const int blueThresholdPixels = 100; // desired number of pixels to be blue to signify the cup being detected


// local state variables for storing ROS message information
bool harvesting = false;
bool initialCenteringDone = false;
bool harvestZoneDetected = false;
bool blueDetected = false;
bool liftingY = false;
bool columnDone = false;
bool allColumnsDone = false;
bool cameraRunning = false;
bool leftBlueDetected = false;

int v_avg = 0;
int goalZ = -1;
int xOffset = 0;
int nextVineOffset = std::numeric_limits<int>::max();



using namespace std;
using namespace cv;

// Used to store depth measured from RealSense matched with its corresponding pixel location
struct PointValue {
	float value;
	int v;
	int u;
};


// ***ROS Subscriber Callbacks*** //
void updateHarvesting(const std_msgs::Bool::ConstPtr& msg) {
	harvesting = msg -> data;
}
void updateInitialCenteringDone(const std_msgs::Bool::ConstPtr& msg) {
	initialCenteringDone = msg -> data;
}

void updateColumnDone(const std_msgs::Bool::ConstPtr& msg) {
	columnDone = msg -> data;
}

void updateLiftingY(const std_msgs::Bool::ConstPtr& msg) {
	liftingY = msg -> data;
}
void updateAllColumnsDone(const std_msgs::Bool::ConstPtr& msg) {
	allColumnsDone = msg -> data;
}



int main(int argc, char **argv) {
	ros::init(argc, argv, "image_processing_node");
	ros::NodeHandle nh;

	// Initialization of ROS publishers
	ros::Publisher blueDetectedPub = nh.advertise<std_msgs::Bool>("blue_detected", 10);
	ros::Publisher harvestZoneDetectedPub = nh.advertise<std_msgs::Bool>("harvest_zone_detected", 10);
	ros::Publisher goalZPub = nh.advertise<std_msgs::Int32>("goal_z", 10);
	ros::Publisher xOffsetPub = nh.advertise<std_msgs::Int32>("x_offset", 10);
	ros::Publisher cameraRunningPub = nh.advertise<std_msgs::Bool>("camera_running", 10);
	ros::Publisher leftBlueDetectedPub = nh.advertise<std_msgs::Bool>("left_blue_detected", 10);

	// Initialization of ROS subscribers with corresponding callback functions
	ros::Subscriber harvestingSub = nh.subscribe("harvesting", 10, updateHarvesting);
	ros::Subscriber initialCenteringDoneSub = nh.subscribe("initial_centering_done", 10, updateInitialCenteringDone);
	ros::Subscriber liftingYSub = nh.subscribe("lifting_y", 10, updateLiftingY);
	ros::Subscriber allColumnsDoneSub = nh.subscribe("all_columns_done", 10, updateAllColumnsDone);
	ros::Subscriber columnDoneSub = nh.subscribe("column_done", 10, updateColumnDone);


	ros::Rate rate(15); // ROS rate set to 15 Hz in accordance with RealSense supported framerate for both rgb and depth streams

	// Initialize realsense stream and corresponding rgb and depth parameters
	rs2::pipeline pipe;
	rs2::config cfg;
	cfg.enable_stream(RS2_STREAM_COLOR, resolution[0], resolution[1], RS2_FORMAT_BGR8, 15);
	cfg.enable_stream(RS2_STREAM_DEPTH, resolution[0], resolution[1], RS2_FORMAT_Z16, 15);
	pipe.start(cfg);

	// Align depth image to color image through camera extrinsics
	rs2::align align_to_color(RS2_STREAM_COLOR);

	// Running code until all columns are harvested
	while(ros::ok() && !allColumnsDone) {
		// Initializing ROS messages for publishing
		std_msgs::Bool blueDetectedMsg;
		std_msgs::Bool harvestZoneDetectedMsg;
		std_msgs::Int32 goalZMsg;
		std_msgs::Int32 xOffsetMsg;
		std_msgs::Bool cameraRunningMsg;
		std_msgs::Bool leftBlueDetectedMsg;
		
		// Setting up realsense stream
		rs2::frameset frames = pipe.wait_for_frames();
		frames = align_to_color.process(frames);

		// If no frames collected, camera is not yet initialized, so send ROS message to indicate this to prevent breaking errors in other nodes that rely on the camera
		if(frames) {
			cameraRunning = true;
		} else {
			cameraRunning = false;
		}

		harvesting = false;
		harvestZoneDetected = false;

		// Publish cameraRunning message for other nodes instantly
		cameraRunningMsg.data = cameraRunning;
		cameraRunningPub.publish(cameraRunningMsg);
		ros::spinOnce();


		// collecting color and depth frames from RealSense camera
		rs2::frame color_frame = frames.get_color_frame();
		rs2::depth_frame depth_frame = frames.get_depth_frame().as<rs2::depth_frame>();

		// Convert RealSense frame to OpenCV matrix
		cv::Mat original_rgb_image(cv::Size(resolution[0], resolution[1]), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);
		cv::Mat hsv_image;
		cv::Mat original_depth_image(cv::Size(resolution[0], resolution[1]), CV_16UC1, (void*)depth_frame.get_data(), cv::Mat::AUTO_STEP);

		//cv::imshow("original_image", original_rgb_image);

		cv::Mat croppingMask = cv::Mat::zeros(cv::Size(resolution[0], resolution[1]), CV_8UC1);

		// Cropping image to remove gripper fingers
		cv::Rect roi(cropped_gripper_bounds[0], 0, cropped_gripper_bounds[1] - cropped_gripper_bounds[0], resolution[1]);
		croppingMask(roi) = 255;
		cv::Mat rgb_image(cv::Size(resolution[0], resolution[1]), CV_8UC1);
		cv::Mat depth_image(cv::Size(resolution[0], resolution[1]), CV_16UC1);
		cv::bitwise_and(original_rgb_image, original_rgb_image, rgb_image, croppingMask);
		cv::bitwise_and(original_depth_image, original_depth_image, depth_image, croppingMask);


		// Changing color space from BGR to HSV, allowing for more robust color segmentation
		cv::cvtColor(rgb_image, hsv_image, COLOR_BGR2HSV, 0);

		// cv::imshow("hsv", hsv_image);


		// Creating cup mask using HSV thresholding
		cv::Mat cupMaskBeforeLargest(cv::Size(resolution[0], resolution[1]), CV_8UC1);
		cv::Scalar lowerHSV(100, 200, 0); // lower bound for cup
		cv::Scalar upperHSV(150, 255, 255); // upper bound for cup
		cv::inRange(hsv_image, lowerHSV, upperHSV, cupMaskBeforeLargest);

		// cv::imshow("cupMaskBeforeLargest beforemorph", cupMaskBeforeLargest);

		// Morphology to clean image of noise
		cv::Mat kernel = cv::Mat::ones(5, 5, CV_8U);
		cv::morphologyEx(cupMaskBeforeLargest, cupMaskBeforeLargest, cv::MORPH_CLOSE, kernel);
		cv::morphologyEx(cupMaskBeforeLargest, cupMaskBeforeLargest, cv::MORPH_OPEN, kernel);

		// cv::imshow("cupMaskBeforeLargest", cupMaskBeforeLargest);


		// Cleaning up all noise by selecting the largest blob from the cup mask and discarding everything else
		std::vector<std::vector<cv::Point>> contours;
		std::vector<cv::Vec4i> hierarchy;
		cv::findContours(cupMaskBeforeLargest, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

		int largest_contour_index = -1;
		// double largest_area = 0.0;
		double largest_area = num_min_cup_points;
		for(size_t i = 0; i < contours.size(); i++) {
			double area = cv::contourArea(contours[i]);
			if(area > largest_area) {
				largest_area = area;
				largest_contour_index = i;
			}
		}

		cv::Mat cupMask = cv::Mat::zeros(cupMaskBeforeLargest.size(), CV_8UC1);

		// Adding only points part of largest blob to cupMask
		if(largest_contour_index != -1) {
			cv::drawContours(cupMask, contours, largest_contour_index, cv::Scalar(255), cv::FILLED);
		}

		// cv::imshow("cupMask", cupMask);

		// Finding average location of cup and counting pixels in order to see if cup is fully in image to stop
		int totalCupPixels = 0;
		float avgCupU = 0;
		float avgCupV = 0;

		vector<int> cupPoints;

		int topmostBlueBuffer = 5;

		for(int v = 0; v < cupMask.rows; v++) {
			for(int u = 0; u < cupMask.cols; u++) {
				if(cupMask.at<unsigned char>(v, u) == 255) {
					avgCupU += u;
					avgCupV += v;
					cupPoints.push_back(v);
					totalCupPixels++;
				}
			}
		}

		// Finding average x-coordinate of cupMask after filtration
		avgCupU = avgCupU / float(totalCupPixels);

		// cv::imshow("cup", cupMask);

		// Collecting smallest n cup points (currently n=100) to determine if cup is fully in frame. If these smallest v-values lie below a certain v-threshold, this means that the top of the cup is still coming into frame
		std::sort(cupPoints.begin(), cupPoints.end(), [](const int& a, const int& b) {
			return a > b;
		});

		std::vector<int> smallest_v_values(cupPoints.begin(), cupPoints.begin() + std::min(num_min_cup_points, static_cast<int>(cupPoints.size())));

		v_avg = 0;

		if(smallest_v_values.size() > 0) {

			if(smallest_v_values[0] > topmostBlueBuffer) { // Only find average v-location of the smallest n v-pixel locations, as we want to index the harvest location based off of the top of the cup (represented by the portion of the cup with the smallest v-values)

				float v_avg_storage = 0;
				int v_counter = 0;

				for(int i = 0; i < int(smallest_v_values.size()); i++) {
					v_avg_storage += smallest_v_values[i];
					v_counter++;
				}

				v_avg = int(v_avg_storage / float(v_counter));

				// 120 and 180 chosen as bounds for cutting zone from testing, feel free to change if hardware constraints change
				if(v_avg >= 120 && v_avg < 180 && !harvesting && initialCenteringDone) {
					harvestZoneDetected = true;
					// cv::circle(rgb_image, cv::Point(int(resolution[0] / 2), v_avg), 5, cv::Scalar(0, 255, 0), -1);

				} else {
					harvestZoneDetected = false;
					// cv::circle(rgb_image, cv::Point(int(resolution[0] / 2), v_avg), 5, cv::Scalar(255, 0, 255), -1);
				}

			} else {
				v_avg = 0;
			}
		}


		// Used to determine if the blue cup is exposed enough to trigger a message that blue has been detected - useful for other nodes. This simply counts the amount of pixels in the cup mask and checks against a pre-determined threshold
		if(!harvesting && liftingY && initialCenteringDone) {
			if(totalCupPixels > blueThresholdPixels) {
				blueDetected = true;
				// cout << "blue detected" << endl;
			} else {
				blueDetected = false;
				// cout << "blue NOT detected" << endl;
			}
		} else {
			blueDetected = false;
		}

		// Sometimes cups in the vines on either side can show up (specifically the leftmost vine), so indicate in message
		if(int(avgCupU) >= 220 && columnDone && int(avgCupU) <= 300) {
			leftBlueDetected = true;
		} else {
			leftBlueDetected = false;
		}


		// Creating vineMask and depthImage to use for visual servoing in x and z, respectively

		cv::Mat vineMask = cv::Mat::zeros(cv::Size(resolution[0], resolution[1]), CV_8UC1);
		cv::Mat depthImage = cv::Mat::zeros(cv::Size(resolution[0], resolution[1]), CV_64F);
            

		// Extracting vine using distance away from camera (cutting away far background points greater than 0.15m away) and HSV color thresholding (cutting away color mismatches from the vine)
		// Additional check val != 0 is added due to points lying outside of Realsense detection zone being labelled as distance = 0
		for (int v = 0; v < depth_image.rows; v++) {
			for (int u = 0; u < depth_image.cols; u++) {
				float val = depth_frame.get_distance(u, v);
				depthImage.at<double>(v, u) = val;
				cv::Vec3b hsv_pixel = hsv_image.at<cv::Vec3b>(v, u);

				//bool hue_condition = 0 <= hsv_pixel[0] && 180 >= hsv_pixel[0] && (30 > hsv_pixel[0] || 90 < hsv_pixel[0]);
				bool hue_condition = !(40 <= hsv_pixel[0] && 80 >= hsv_pixel[0]);
				bool sat_condition = 0 <= hsv_pixel[1] && 150 >= hsv_pixel[1];

				// change 0.15 if need be, this is the max distance in meters to consider for depth
				if (val < 0.15 && hue_condition && sat_condition && val != 0 && croppingMask.at<unsigned char>(v, u) == 255) { // hue_condition &&
					vineMask.at<unsigned char>(v, u) = 255;
				}
			}
		}

		// cv::imshow("vinemask premorph", vineMask);

		// // Removing cups from vineMask to improve vine masking (as the gray color on the vines is a blue-based hue)
		cv::subtract(vineMask, cupMask, vineMask);

		cv::Mat bigKernel = cv::Mat::ones(7, 7, CV_8U);

		// More morphology to clean vineMask of noise
		cv::morphologyEx(vineMask, vineMask, cv::MORPH_OPEN, bigKernel);

		// cv::imshow("vineMask prefilter", vineMask);


		// Collecting locations of n lowest depth points (currently n = 2000), adding to mask called smallest_values.
		// This mask will then average the distances of all n points in order to get a more robust estimation for
		// the minimum distance the vine is away from the end effector
		std::vector<PointValue> points;

		for(int v = 0; v < vineMask.rows; v++) {
			for(int u = 0; u < vineMask.cols; u++) {
				float val = depthImage.at<double>(v, u);

				if (vineMask.at<unsigned char>(v, u) != 0 && val != 0) {
					points.push_back({val, v, u});
				}        
			}
		}

		// Sorting points from lowest to highest value, then choosing the smallest n from that sorted list
		std::sort(points.begin(), points.end(), [](const PointValue& a, const PointValue& b) {
			return a.value < b.value;
		});

		std::vector<PointValue> smallest_z_values(points.begin(), points.begin() + std::min(num_min_vine_rib_points, static_cast<int>(points.size())));

		// Processing to find the vertical section of the smallest_values mask, corresponding to the front rib of the vine
		Mat smallest_values_raw = cv::Mat::zeros(cv::Size(resolution[0], resolution[1]), CV_8UC1);

		for(const auto& point : smallest_z_values) {
			smallest_values_raw.at<unsigned char>(point.v, point.u) = 255;
		}

		// Running morphology to clean up noise from smallest_values mask
		cv::morphologyEx(smallest_values_raw, smallest_values_raw, cv::MORPH_OPEN, kernel);

		// cv::imshow("smallest_values before thin convolution", smallest_values_raw);

		// Using a tall and thin kernel in order to extract similar tall and thin features from the image.
		// This extracts the front rib of the vine, which is what we want to servo off of
		cv::Mat verticalStructure = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(1, 20));
		cv::Mat filtered_smallest_values_raw;
		cv::erode(smallest_values_raw, smallest_values_raw, verticalStructure);
		cv::dilate(smallest_values_raw, smallest_values_raw, verticalStructure);

		std::vector<std::vector<cv::Point>> contoursRib;
		std::vector<cv::Vec4i> hierarchyRib;

		cv::findContours(smallest_values_raw, contoursRib, hierarchyRib, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

		// Find the largest contour which should be the front vine rib
		int largestContourIndex = -1;
		double largestContourArea = 0.0;
		for (size_t i = 0; i < contoursRib.size(); i++) {
			double area = cv::contourArea(contoursRib[i]);
			if (area > largestContourArea) {
				largestContourArea = area;
				largestContourIndex = static_cast<int>(i);
			}
		}

		cv::Mat smallest_values = cv::Mat::zeros(vineMask.size(), CV_8UC1);
		if (largestContourIndex != -1) {
			cv::drawContours(smallest_values, contoursRib, largestContourIndex, cv::Scalar(255), cv::FILLED);
		}

		cv::Mat smallest_values_filtered;

		// extracting the points from original smallest_values_raw mask that lie within the idealized vertical mask
		cv::bitwise_and(smallest_values, smallest_values_raw, smallest_values_filtered);


		// Finding the center of the vine by finding the max and min u and taking the midpoint between the two. Using midpoint of max and min rather than average u due to the leave coverage biasing the average towards one side of the vine a lot of the time (think about a leaf covering the vine diagonally, there will be higher bias towards the uncovered section but we want the true center of the vine)
		float min_z_u = 0;
		float min_z_v = 0;
		float minVal = 0;
		int average_counter = 0;

		int min_u = std::numeric_limits<int>::max();
		int max_u = 0;
            

		for (int v = 0; v < smallest_values_filtered.rows; v++) {
			for (int u = 0; u < smallest_values_filtered.cols; u++) {
				if(smallest_values_filtered.at<unsigned char>(v, u) == 255) {
					min_z_u += u;
					min_z_v += v;
					minVal += depthImage.at<double>(v, u);
					average_counter++;

					if(u > max_u) {
						max_u = u;
					}
					if(u < min_u) {
						min_u = u;
					}
				}
			}
		}

		min_z_u = int(min_z_u / average_counter);
		min_z_v = int(min_z_v / average_counter);
		minVal = minVal / average_counter;

		cv::Point minLoc(min_z_u, min_z_v);

		goalZ = minVal * 1000; // converting to mm to send over ROS message

		int avg_u = int((max_u + min_u) / 2);

		xOffset = int(resolution[0] / 2) + 50 - avg_u; // subtracting u_centerOfImage from u_detectedVineRib to get the vine's u offset (pixels in horizontal direction)


		cv::Point centerX(avg_u, int(resolution[1] / 2));
		cv::circle(rgb_image, centerX, 5, cv::Scalar(0, 0, 255), -1);

		//cv::imshow("image", rgb_image);

		// Updating messages and publishing
		blueDetectedMsg.data = blueDetected;
		harvestZoneDetectedMsg.data = harvestZoneDetected;
		goalZMsg.data = goalZ;
		xOffsetMsg.data = xOffset;
		leftBlueDetectedMsg.data = leftBlueDetected;

		blueDetectedPub.publish(blueDetectedMsg);
		harvestZoneDetectedPub.publish(harvestZoneDetectedMsg);
		goalZPub.publish(goalZMsg);
		xOffsetPub.publish(xOffsetMsg);
		leftBlueDetectedPub.publish(leftBlueDetectedMsg);

		cv::waitKey(1);

		ros::spinOnce();
		rate.sleep();
	}

	cv::destroyAllWindows();
	
	return(0);
}
