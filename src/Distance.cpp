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
cv::Size boardDims(7, 5);
float boardSize = 0.03;

void readImage(const sensor_msgs::Image::ConstPtr msgImage, cv::Mat &image) {
	cv_bridge::CvImageConstPtr pCvImage;
	pCvImage = cv_bridge::toCvShare(msgImage, msgImage->encoding);
	pCvImage->image.copyTo(image);
}

void getDistance(cv::Mat points) {
	std::vector<cv::Point3f> board;
	board.resize(boardDims.width * boardDims.height);
	for (size_t r = 0, i = 0; r < (size_t) boardDims.height; ++r) {
		for (size_t c = 0; c < (size_t) boardDims.width; ++c, ++i) {
			board[i] = cv::Point3f(c * boardSize, r * boardSize, 0);
		}
	}

	cv::Mat cameraMatrix, distortion, rvec, rotation, translation;
	;
	cv::FileStorage fs;
	std::string path("/home/benny/kinect_cal_data/calib_color.yaml");
	if (fs.open(path, cv::FileStorage::READ)) {
		fs["cameraMatrix"] >> cameraMatrix;
		fs["distortionCoefficients"] >> distortion;
		fs.release();
	} else {
		std::cerr << "couldn't read calibration '" << path << "'!" << std::endl;
	}
	cv::solvePnPRansac(board, points, cameraMatrix, distortion, rvec,
			translation, false, 300, 0.05, board.size(), cv::noArray(),
			cv::ITERATIVE);

	std::cout << translation << std::endl;
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
	std::cout << "image callback" << std::endl;
	image_detected = true;
	cv::Mat mat, output;
	readImage(msg, mat);

//	std::cout << mat << std::endl;

	found = cv::findChessboardCorners(mat, boardDims, output,
			cv::CALIB_CB_FAST_CHECK);
	std::cout << found << std::endl;
	if (found) {
		getDistance(output);
		cv::drawChessboardCorners(mat, boardDims, output, found);
		cv::imshow("Display window", mat);
	}
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

