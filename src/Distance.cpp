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

bool chessBoardfound;
cv::Size boardDims(7, 5);
float boardSize = 0.03;
std::vector<cv::Point3f> board;
cv::Mat cameraMatrix, distortion, rvec, rotation, translation, normal;

void readImage(const sensor_msgs::Image::ConstPtr msgImage, cv::Mat &image) {
	cv_bridge::CvImageConstPtr pCvImage;
	pCvImage = cv_bridge::toCvShare(msgImage, msgImage->encoding);
	pCvImage->image.copyTo(image);
}

double getDistance(cv::Mat points) {
	cv::solvePnPRansac(board, points, cameraMatrix, distortion, rvec,
			translation, false, 300, 0.05, board.size(), cv::noArray(),
			cv::ITERATIVE);

	cv::Rodrigues(rvec, rotation);

	normal.at<double>(0) = 0;
	normal.at<double>(1) = 0;
	normal.at<double>(2) = 1;
	normal = rotation * normal;
	double distance = normal.dot(translation);

	return distance;
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
	std::cout << "callback" << std::endl;
	cv::Mat mat, output;
	readImage(msg, mat);
	chessBoardfound = false;
	chessBoardfound = cv::findChessboardCorners(mat, boardDims, output,
			cv::CALIB_CB_FAST_CHECK);
//	std::cout << chessBoardfound << std::endl;
//	if (chessBoardfound) {
//		double distance = getDistance(output);
//		cv::drawChessboardCorners(mat, boardDims, output, chessBoardfound);
//		std::string distanceString = patch::to_string(distance).append(" m");
//		cv::putText(mat, distanceString, cv::Point(50, 100), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 4);
//	}
	cv::imshow("Display window", mat);
	std::cout << "callback finished" << std::endl;
}

void readCalibrationData() {
	cv::FileStorage fs;
	std::string path("/home/benny/kinect_cal_data/calib_color.yaml");
	if (fs.open(path, cv::FileStorage::READ)) {
		fs["cameraMatrix"] >> cameraMatrix;
		fs["distortionCoefficients"] >> distortion;
		fs.release();
	} else {
		std::cerr << "couldn't read calibration '" << path << "'!" << std::endl;
	}
}

void createBoardPoints() {
	board.resize(boardDims.width * boardDims.height);
	for (size_t r = 0, i = 0; r < (size_t) (boardDims.height); ++r) {
		for (size_t c = 0; c < (size_t) (boardDims.width); ++c, ++i) {
			board[i] = cv::Point3f(c * boardSize, r * boardSize, 0);
		}
	}
}

int main(int argc, char **argv) {
	std::cout << "hello world" << std::endl;
	ros::init(argc, argv, "distance");
	ros::NodeHandle nh;
	ros::AsyncSpinner spinner(1);
	image_transport::ImageTransport it(nh);
	image_transport::Subscriber sub = it.subscribe("/kinect2/hd/image_mono", 1,
			imageCallback);
	cv::namedWindow("Display window", cv::WINDOW_AUTOSIZE); // Create a window for display.

	readCalibrationData();
	createBoardPoints();
	normal = cv::Mat(3, 1, CV_64F);

	spinner.start();
//	while (!chessBoardfound) {
//		ros::spinOnce();
//	}
	cv::waitKey(0);
}
