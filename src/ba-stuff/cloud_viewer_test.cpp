/*
 * cloud_viewer_test.cpp
 *
 *  Created on: Sep 4, 2016
 *      Author: benny
 */

#include <ba-stuff/rosconnector.h>
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <ros/ros.h>

#include <opencv2/opencv.hpp>

const char* IR_TOPIC = "/kinect2/sd/image_ir";
const char* DEPTH_TOPIC = "/kinect2/sd/image_depth";

int main(int argc, char **argv) {
	std::cout << "hello world" << std::endl;
	ros::init(argc, argv, "cloudViewerTest");
	ros::NodeHandle nh;
	RosConnector con(nh, IR_TOPIC, DEPTH_TOPIC);
	cv::Mat ir, depth;
	con.getNewImage(ir, depth);

	const char* name = "image";
	cv::namedWindow(name, cv::WINDOW_AUTOSIZE);
	cv::imshow(name, ir);
	cv::waitKey();
}
