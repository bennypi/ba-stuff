/*
 * Distance.h
 *
 *  Created on: Mar 2, 2016
 *      Author: benny
 */

#ifndef BA_STUFF_SRC_DISTANCE_H_
#define BA_STUFF_SRC_DISTANCE_H_

#include <stdlib.h>
#include <stdio.h>
#include <iostream>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/time_synchronizer.h>

#include <ros/ros.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <pcl/point_cloud.h>

class Distance {
private:
	const float boardSize = 0.03;
	std::vector<cv::Point3f> board;
	cv::Mat cameraMatrix, distortion, rvec, rotation, translation;
	double fx, fy, cx, cy;
	const static char* KINECT_IMAGE;
	const static char* IR_IMAGE;
	const static char* COLOR_MAP;
	double distanceToNormal;

	void readCalibrationData();
	void createBoardPoints();

public:
	bool chessBoardFound, update, modeIr;
	cv::Mat color, ir, depth;
	cv::Mat normal, adjMap, colorMap, intersectionsInPicture, extrinsicsRotation,
			extrinsicsTranslation;
	const cv::Size boardDims;

	Distance(bool modeIr, const cv::Size &size, int argc, char **argv);
	virtual ~Distance();
	bool findChessboardColor();
	bool findChessboardIr();
	void createChessBoardPlane(cv::Mat &output);
	double getNormalWithDistance(cv::Mat points, cv::Mat &normal);
	double computeDistanceToPoint(const cv::Point &pointImage,
			const cv::Mat &normal, const double distance);
	void drawDetailsInImage(double normalDistance);
	void updateImages(cv::Mat &color, cv::Mat &ir, cv::Mat &depth);
	void createMatForNormal(cv::Mat &output);
};

#endif /* BA_STUFF_SRC_DISTANCE_H_ */

#include <string>
#include <sstream>

namespace patch {
template<typename T> std::string to_string(const T& n) {
	std::ostringstream stm;
	stm << n;
	return stm.str();
}
}
