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

bool harvesting = false;
bool initialCenteringDone = false;
bool harvestZoneDetected = false;
bool blueDetected = false;
bool liftingY = false;
bool columnDone = false;

int v_avg = 0;
int goalZ = -1;
int xOffset = 0;

using namespace std;
using namespace cv;

struct PointValue {
	float value;
	int v;
	int u;
};


void updateHarvesting(const std_msgs::Bool::ConstPtr& msg) {
	harvesting = msg -> data;
}

void updateInitialCenteringDone(const std_msgs::Bool::ConstPtr& msg) {
	initialCenteringDone = msg -> data;
}

void updateLiftingY(const std_msgs::Bool::ConstPtr& msg) {
	liftingY = msg -> data;
}

void updateColumnDone(const std_msgs::Bool::ConstPtr& msg) {
	columnDone = msg -> data;
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "image_processing_node");
	ros::NodeHandle nh;

	ros::Publisher blueDetectedPub = nh.advertise<std_msgs::Bool>("blue_detected", 10);
	ros::Publisher harvestZoneDetectedPub = nh.advertise<std_msgs::Bool>("harvest_zone_detected", 10);
	ros::Publisher goalZPub = nh.advertise<std_msgs::Int32>("goal_z", 10);
	ros::Publisher xOffsetPub = nh.advertise<std_msgs::Int32>("x_offset", 10);



	ros::Subscriber harvestingSub = nh.subscribe("harvesting", 10, updateHarvesting);
	ros::Subscriber initialCenteringDoneSub = nh.subscribe("initial_centering_done", 10, updateInitialCenteringDone);
	ros::Subscriber liftingYSub = nh.subscribe("lifting_y", 10, updateLiftingY);
	ros::Subscriber columnDoneSub = nh.subscribe("column_done", 10, updateColumnDone);


	ros::Rate rate(15);

	// usleep(100000); // add delay to prevent motor jerk

	int resolution[2] = {424, 240};
	int cropped_gripper_bounds[2] = {130, 400};

	// printf("image processing running\n");

	rs2::pipeline pipe;
	rs2::config cfg;

	cfg.enable_stream(RS2_STREAM_COLOR, resolution[0], resolution[1], RS2_FORMAT_BGR8, 15);
	cfg.enable_stream(RS2_STREAM_DEPTH, resolution[0], resolution[1], RS2_FORMAT_Z16, 15);

	pipe.start(cfg);

	rs2::align align_to_color(RS2_STREAM_COLOR);
    
	int num_min_vine_rib_points = 2000;
	int num_min_cup_points = 200;

	int blueThresholdPixels = 100;
	int counter = 0;
	
	
	while(ros::ok() && !columnDone) {
		std_msgs::Bool blueDetectedMsg;
		std_msgs::Bool harvestZoneDetectedMsg;
		std_msgs::Int32 goalZMsg;
		std_msgs::Int32 xOffsetMsg;

		rs2::frameset frames = pipe.wait_for_frames();
		frames = align_to_color.process(frames);


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



		// Changing color space from BGR to HSV, allowing for more robust color detection
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

		cv::imshow("cup", cupMask);


		std::sort(cupPoints.begin(), cupPoints.end(), [](const int& a, const int& b) {
			return a > b;
		});

		std::vector<int> largest_v_values(cupPoints.begin(), cupPoints.begin() + std::min(num_min_cup_points, static_cast<int>(cupPoints.size())));

		v_avg = 0;
		float v_avg_storage = 0;
		int v_counter = 0;

		for(int i = 0; i < int(largest_v_values.size()); i++) {
			v_avg_storage += largest_v_values[i];
			v_counter++;
		}

		v_avg = int(v_avg_storage / float(v_counter));

		if(v_avg >= 200 && !harvesting) {
			harvestZoneDetected = true;
		} else {
			harvestZoneDetected = false;
		}

		cv::circle(rgb_image, cv::Point(int(resolution[0] / 2), v_avg), 5, cv::Scalar(0, 255, 0), -1);
            

		if(!harvesting && liftingY) {
			if(totalCupPixels > blueThresholdPixels) {
				blueDetected = true;
				// cout << "blue detected" << endl;
			} else {
				blueDetected = false;
				// cout << "blue NOT detected" << endl;
			}
		}

		// Creating vineMask and depthImage to use for visual servoing in x and z, respectively

		cv::Mat vineMask = cv::Mat::zeros(cv::Size(resolution[0], resolution[1]), CV_8UC1);
		cv::Mat depthImage = cv::Mat::zeros(cv::Size(resolution[0], resolution[1]), CV_64F);
            

		// Extracting vine using distance away from camera (cutting away far background points greater than 0.2 m away) and HSV color thresholding (cutting away color mismatches from the vine)
		// Additional check val != 0 is added due to points lying outside of Realsense detection zone being labelled as distance = 0
		for (int v = 0; v < depth_image.rows; v++) {
			for (int u = 0; u < depth_image.cols; u++) {
				float val = depth_frame.get_distance(u, v);
				depthImage.at<double>(v, u) = val;
				cv::Vec3b hsv_pixel = hsv_image.at<cv::Vec3b>(v, u);

				// bool hue_condition = 0 <= hsv_pixel[0] && 180 >= hsv_pixel[0] && (30 > hsv_pixel[0] || 90 < hsv_pixel[0]);
				bool hue_condition = !(40 <= hsv_pixel[0] && 80 >= hsv_pixel[0]);
				bool sat_condition = 0 <= hsv_pixel[1] && 150 >= hsv_pixel[1];

				if (val < 0.25 && hue_condition && sat_condition && val != 0 && croppingMask.at<unsigned char>(v, u) == 255) { // hue_condition &&
					vineMask.at<unsigned char>(v, u) = 255;
				}
			}
		}

		cv::imshow("vinemask premorph", vineMask);

		// // Removing cups from vineMask to improve vine masking (as the gray color on the vines is a blue-based hue)
		cv::subtract(vineMask, cupMask, vineMask);

		cv::Mat bigKernel = cv::Mat::ones(7, 7, CV_8U);

		// More morphology to clean vineMask of noise
		cv::morphologyEx(vineMask, vineMask, cv::MORPH_OPEN, bigKernel);

		// cv::imshow("vineMask prefilter", vineMask);


		// Collecting locations of n lowest depth points (n = 1000), adding to mask called smallest_values.
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

		// cv::imshow("vine_rib_filtered", smallest_values_filtered);

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

		xOffset = int(resolution[0] / 2) - avg_u; // subtracting u_centerOfImage from u_detectedVineRib to get the vine's u offset (pixels in horizontal direction)


		cv::Point centerX(avg_u, int(resolution[1] / 2));
		cv::circle(rgb_image, centerX, 5, cv::Scalar(0, 0, 255), -1);

		cv::imshow("image", rgb_image);


		counter++;

		blueDetectedMsg.data = blueDetected;
		harvestZoneDetectedMsg.data = harvestZoneDetected;
		goalZMsg.data = goalZ;
		xOffsetMsg.data = xOffset;
		
		blueDetectedPub.publish(blueDetectedMsg);
		harvestZoneDetectedPub.publish(harvestZoneDetectedMsg);
		goalZPub.publish(goalZMsg);
		xOffsetPub.publish(xOffsetMsg);

		cv::waitKey(1);

		ros::spinOnce();
		rate.sleep();
	}

	cv::destroyAllWindows();
	
	return(0);
}
