/*
 * Distance.cpp
 *
 *  Created on: Mar 2, 2016
 *      Author: benny
 */

#include "Distance.h"

#include <stdlib.h>
#include <stdio.h>
#include <iostream>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <ros/ros.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

Distance::Distance() {
	// TODO Auto-generated constructor stub

}

Distance::~Distance() {
	// TODO Auto-generated destructor stub
}

bool image_detected, found;

void readImage(const sensor_msgs::Image::ConstPtr msgImage, cv::Mat &image) {
	cv_bridge::CvImageConstPtr pCvImage;
	pCvImage = cv_bridge::toCvShare(msgImage, msgImage->encoding);
	pCvImage->image.copyTo(image);
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
	std::cout << "image callback" << std::endl;
	image_detected = true;
	cv::Mat mat, output;
	readImage(msg, mat);

//	std::cout << mat << std::endl;
	cv::imshow("Display window", mat);

	cv::Size boardDims(7, 5);
	found = cv::findChessboardCorners(mat, boardDims, output,
			cv::CALIB_CB_FAST_CHECK);
	std::cout << found << std::endl;
}

int main(int argc, char **argv) {
	std::cout << "hello world" << std::endl;
	ros::init(argc, argv, "distance");
	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);
	image_transport::Subscriber sub = it.subscribe("/kinect2/hd/image_mono", 1,
			imageCallback);
	cv::namedWindow("Display window", cv::WINDOW_AUTOSIZE); // Create a window for display.
	while (!found) {
		ros::spinOnce();
	}
//	ros::spin();
	cv::waitKey(0);
}

