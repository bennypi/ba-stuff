/*
 * Distance.cpp
 *
 *  Created on: Mar 2, 2016
 *      Author: benny
 */

#include "distance.h"

const char* Distance::KINECT_IMAGE = "KINECT_IMAGE";
const char* Distance::COLOR_MAP = "COLOR_MAP";

Distance::Distance(const cv::Size &size, std::string imageTopic, int argc, char **argv) :
		boardDims(size), chessBoardFound(false), update(false) {
	ros::init(argc, argv, "distance");
	ros::NodeHandle nh;
	Distance::connector(nh, "", "");
}

Distance::~Distance() {
	// TODO Auto-generated destructor stub
}

void readImage(const sensor_msgs::Image::ConstPtr msgImage, cv::Mat &image) {
	cv_bridge::CvImageConstPtr pCvImage;
	pCvImage = cv_bridge::toCvShare(msgImage, msgImage->encoding);
	pCvImage->image.copyTo(image);
}

double Distance::getNormalWithDistance(cv::Mat points, cv::Mat &normal) {
	cv::solvePnPRansac(Distance::board, points, Distance::cameraMatrix,
			Distance::distortion, Distance::rvec, Distance::translation, false,
			300, 0.05, Distance::board.size(), cv::noArray(), cv::ITERATIVE);

	cv::Rodrigues(Distance::rvec, Distance::rotation);

	normal.at<double>(0) = 0;
	normal.at<double>(1) = 0;
	normal.at<double>(2) = 1;
	normal = Distance::rotation * normal;
	double distance = normal.dot(Distance::translation);

	return distance;
}

double Distance::computeDistanceToPoint(const cv::Point &pointImage,
		const cv::Mat &normal, const double distance) {
	cv::Mat point = cv::Mat(3, 1, CV_64F);

	point.at<double>(0) = (pointImage.x - Distance::cx) / Distance::fx;
	point.at<double>(1) = (pointImage.y - Distance::cy) / Distance::fy;
	point.at<double>(2) = 1;

	double t = distance / normal.dot(point);
	point = point * t;

	return point.at<double>(2);
}

void Distance::readCalibrationData() {
	cv::FileStorage fs;
	std::string path("/home/benny/kinect_cal_data/calib_color.yaml");
	if (fs.open(path, cv::FileStorage::READ)) {
		fs["cameraMatrix"] >> Distance::cameraMatrix;
		fs["distortionCoefficients"] >> Distance::distortion;
		fs.release();
	} else {
		std::cerr << "couldn't read calibration '" << path << "'!" << std::endl;
	}
	path = ("/home/benny/kinect_cal_data/calib_pose.yaml");
	if (fs.open(path, cv::FileStorage::READ)) {
			fs["rotation"] >> Distance::extrinsicsRotation;
			fs["translation"] >> Distance::extrinsicsTranslation;
			fs.release();
		} else {
			std::cerr << "couldn't read calibration '" << path << "'!" << std::endl;
		}

//	do i need this?
//	cv::initUndistortRectifyMap(cameraMatrix, distortion, cv::Mat(), cameraMatrix, size, CV_32FC1, mapX, mapY);
	std::cout << Distance::cameraMatrix << std::endl;
	Distance::fx = Distance::cameraMatrix.at<double>(0, 0);
	Distance::fy = Distance::cameraMatrix.at<double>(1, 1);
	Distance::cx = Distance::cameraMatrix.at<double>(0, 2);
	Distance::cy = Distance::cameraMatrix.at<double>(1, 2);
}

void Distance::createBoardPoints() {
	Distance::board.resize(
			Distance::boardDims.width * Distance::boardDims.height);
	for (size_t r = 0, i = 0; r < (size_t) (Distance::boardDims.height); ++r) {
		for (size_t c = 0; c < (size_t) (Distance::boardDims.width); ++c, ++i) {
			Distance::board[i] = cv::Point3f(c * Distance::boardSize,
					r * Distance::boardSize, 0);
		}
	}
}

void Distance::drawDetailsInImage(double normalDistance) {
	cv::Mat coloredMat;
	cv::cvtColor(Distance::color, coloredMat, CV_GRAY2BGR);
	cv::drawChessboardCorners(coloredMat, Distance::boardDims, Distance::output,
			Distance::chessBoardFound);
	std::string distanceString = patch::to_string(normalDistance).append(" m");
	cv::putText(coloredMat, distanceString, cv::Point(50, 100),
			cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 4);
	cv::imshow(Distance::KINECT_IMAGE, coloredMat);
}

void Distance::imageCallback(const sensor_msgs::ImageConstPtr color) {
	readImage(color, Distance::color);
	Distance::chessBoardFound = false;
	Distance::chessBoardFound = cv::findChessboardCorners(Distance::color,
			Distance::boardDims, Distance::output, cv::CALIB_CB_FAST_CHECK);
	Distance::update = true;
}

void Distance::syncedImageCallback(const sensor_msgs::ImageConstPtr color, const sensor_msgs::ImageConstPtr depth) {
	readImage(color, Distance::color);
	readImage(depth, Distance::depth);
	Distance::chessBoardFound = false;
	Distance::chessBoardFound = cv::findChessboardCorners(Distance::color,
			Distance::boardDims, Distance::output, cv::CALIB_CB_FAST_CHECK);
	Distance::update = true;
}

int main(int argc, char **argv) {
	Distance d(cv::Size(7, 5), "/kinect2/hd/image_color", argc, argv);

	cv::namedWindow(Distance::KINECT_IMAGE, cv::WINDOW_AUTOSIZE);

	d.readCalibrationData();
	d.createBoardPoints();

	while (!d.chessBoardFound) {
		ros::spinOnce();
		if (d.update) {
			cv::imshow(Distance::KINECT_IMAGE, d.color);
			d.update = false;
		}
		cv::waitKey(100);
	}
	d.normal = cv::Mat(3, 1, CV_64F);
	double normalDistance = d.getNormalWithDistance(d.output, d.normal);
	d.drawDetailsInImage(normalDistance);
	cv::Mat distanceOfCorners(7, 5, CV_64F);
	for (int i = 0, idx = 0; i < distanceOfCorners.rows; i++) {
		for (int j = 0; j < distanceOfCorners.cols; j++, idx++) {
			double distanceOfPoint = d.computeDistanceToPoint(
					d.output.at<cv::Point2f>(idx), d.normal, normalDistance);
			distanceOfCorners.at<double>(i, j) = distanceOfPoint;
		}
	}

	// rotate and translate the normal to the ir frame
	cv::Mat normalInIR = d.extrinsicsRotation* d.normal;
	std::cout << d.normal.cols << " " << d.normal.rows << std::endl;
	std::cout << normalInIR.cols << " " << normalInIR.rows << std::endl;
//	cv::normalize(normalInIR);
	std::cout << normalInIR << std::endl;
	normalInIR = d.extrinsicsTranslation * normalInIR;



//	cv::namedWindow(COLOR_MAP, cv::WINDOW_AUTOSIZE);
	double min;
	double max;
	cv::minMaxIdx(d.depth, &min, &max);
	double scale = 255 / (max - min);
	std::cout << min << " " << max << std::endl;
	// expand your range to 0..255. Similar to histEq();
	d.depth.convertTo(d.adjMap, CV_8UC1, scale, -min * scale);
//	std::cout << d.adjMap << std::endl;
	cv::applyColorMap(d.adjMap, d.colorMap, cv::COLORMAP_JET);
//	cv::Mat largeColorMap;
//	cv::resize(d.colorMap, largeColorMap,
//			cv::Size(d.colorMap.cols * 100, d.colorMap.rows * 100));
//	cv::imshow(Distance::COLOR_MAP, d.colorMap);
	cv::waitKey();
}
