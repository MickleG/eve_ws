#include <ros/ros.h>
#include <librealsense2/rs.hpp>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_node");
    ros::NodeHandle nh;

    // Declare RealSense pipeline, colorizer, and config
    rs2::pipeline pipe;
    rs2::config cfg;

    // Add desired streams to configuration
    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);

    // Start the pipeline with the configuration
    pipe.start(cfg);

    // Main loop
    while (ros::ok())
    {
        // Wait for the next set of frames
        rs2::frameset frames = pipe.wait_for_frames();

        // Get the color frame
        rs2::video_frame color_frame = frames.get_color_frame();

        // Convert the frame to a cv::Mat
        cv::Mat color_image(cv::Size(color_frame.get_width(), color_frame.get_height()), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);

        // Convert the cv::Mat to a ROS message
        sensor_msgs::Image img_msg;
        img_msg.header.stamp = ros::Time::now();
        img_msg.header.frame_id = "realsense_camera";
        img_msg.height = color_frame.get_height();
        img_msg.width = color_frame.get_width();
        img_msg.encoding = sensor_msgs::image_encodings::BGR8;
        img_msg.step = color_image.cols * color_image.elemSize();
        size_t size = img_msg.step * color_image.rows;
        img_msg.data.resize(size);
        memcpy(img_msg.data.data(), color_image.data, size);

        // Publish the image
        ros::Publisher pub = nh.advertise<sensor_msgs::Image>("realsense/color/image_raw", 1);
        pub.publish(img_msg);

        ros::spinOnce();
    }

    // Stop the pipeline
    pipe.stop();

    return 0;
}

