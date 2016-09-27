/*
 * cloud_viewer_test.cpp
 *
 *  Created on: Sep 4, 2016
 *      Author: benny
 */

#include <ba-stuff/rosconnector.h>
#include <ba-stuff/distance.h>

#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <ros/ros.h>

#include <opencv2/opencv.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>

const char* COLOR_TOPIC = "/kinect2/sd/image_color_rect";
const char* IR_TOPIC = "/kinect2/sd/image_ir_rect";
const char* DEPTH_TOPIC = "/kinect2/sd/image_depth_rect";
float fx, fy, cx, cy;
cv::Mat cameraMatrix, distortion, rvec, rotation, translation,
		extrinsicsRotation, extrinsicsTranslation, lookupYDepth, lookupXDepth, lookupYCalculated, lookupXCalculated;
bool running;

void readCalibrationData() {
	cv::FileStorage fs;
	std::string path;
	path = ("/home/benny/kinect_cal_data/calib_ir.yaml");
	if (fs.open(path, cv::FileStorage::READ)) {
		fs["cameraMatrix"] >> cameraMatrix;
		fs["distortionCoefficients"] >> distortion;
		fs.release();
	} else {
		std::cerr << "couldn't read calibration '" << path << "'!" << std::endl;
	}
	path = ("/home/benny/kinect_cal_data/calib_pose.yaml");
	if (fs.open(path, cv::FileStorage::READ)) {
		fs["rotation"] >> extrinsicsRotation;
		fs["translation"] >> extrinsicsTranslation;
		fs.release();
	} else {
		std::cerr << "couldn't read calibration '" << path << "'!" << std::endl;
	}

	fx = 1.0f / cameraMatrix.at<double>(0, 0);
	fy = 1.0f / cameraMatrix.at<double>(1, 1);
	cx = cameraMatrix.at<double>(0, 2);
	cy = cameraMatrix.at<double>(1, 2);
}

void createCloudDepth(const cv::Mat &depth, int b, int g, int r,
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud) {
	const float badPoint = std::numeric_limits<float>::quiet_NaN();

#pragma omp parallel for
	std::cout << "number of points: " << depth.rows * depth.cols << std::endl;
	int count = 0;
	for (int r = 0; r < depth.rows; ++r) {
		pcl::PointXYZRGBA *itP = &cloud->points[r * depth.cols];
		const uint16_t *itD = depth.ptr<uint16_t>(r);
		const float y = lookupYDepth.at<float>(0, r);
		const float *itX = lookupXDepth.ptr<float>();

		for (size_t c = 0; c < (size_t) depth.cols; ++c, ++itP, ++itD, ++itX) {
			register const float depthValue = *itD / 1000.0f;
			// Check for invalid measurements
			if (*itD == 0) {
				// not valid
				itP->x = itP->y = itP->z = badPoint;
				itP->rgba = 0;
				count++;
				continue;
			}
			itP->z = depthValue;
			itP->x = *itX * depthValue;
			itP->y = y * depthValue;
			itP->b = b;
			itP->g = g;
			itP->r = r;
			itP->a = 255;
		}
	}
	std::cout << count << std::endl;
}

void createCloudCalculated(const cv::Mat &depth, int b, int g, int r,
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud) {
	const float badPoint = std::numeric_limits<float>::quiet_NaN();

#pragma omp parallel for
	std::cout << "number of points: " << depth.rows * depth.cols << std::endl;
	int count = 0;
	for (int r = 0; r < depth.rows; ++r) {
		pcl::PointXYZRGBA *itP = &cloud->points[r * depth.cols];
		const uint16_t *itD = depth.ptr<uint16_t>(r);
		const float y = lookupYCalculated.at<float>(0, r);
		const float *itX = lookupXCalculated.ptr<float>();

		for (size_t c = 0; c < (size_t) depth.cols; ++c, ++itP, ++itD, ++itX) {
			register const float depthValue = *itD / 1000.0f;
			// Check for invalid measurements
			if (*itD == 0) {
				// not valid
				itP->x = itP->y = itP->z = badPoint;
				itP->rgba = 0;
				count++;
				continue;
			}
			itP->z = depthValue;
			itP->x = *itX * depthValue;
			itP->y = y * depthValue;
			itP->b = b;
			itP->g = g;
			itP->r = r;
			itP->a = 255;
		}
	}
	std::cout << count << std::endl;
}

void createLookupDepth(size_t width, size_t height) {
	float *it;

	lookupYDepth = cv::Mat(1, height, CV_32F);
	it = lookupYDepth.ptr<float>();
	for (size_t r = 0; r < height; ++r, ++it) {
		*it = (r - cy) * fy;
	}

	lookupXDepth = cv::Mat(1, width, CV_32F);
	it = lookupXDepth.ptr<float>();
	for (size_t c = 0; c < width; ++c, ++it) {
		*it = (c - cx) * fx;
	}
}

void createLookupCalculated(size_t width, size_t height) {
	float *it;

	lookupYCalculated = cv::Mat(1, height, CV_32F);
	it = lookupYCalculated.ptr<float>();
	for (size_t r = 0; r < height; ++r, ++it) {
		*it = (r - cy) * fy;
	}

	lookupXCalculated = cv::Mat(1, width, CV_32F);
	it = lookupXCalculated.ptr<float>();
	for (size_t c = 0; c < width; ++c, ++it) {
		*it = (c - cx) * fx;
	}
}
void keyboardEvent(const pcl::visualization::KeyboardEvent &event, void *) {
	if (event.keyUp()) {
		switch (event.getKeyCode()) {
		case 27:
		case 'q':
			running = false;
			break;
		}
	}
}

void initializeCloud(const cv::Mat& ir,
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& cloud) {
	cloud = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(
			new pcl::PointCloud<pcl::PointXYZRGBA>());
	cloud->height = ir.rows;
	cloud->width = ir.cols;
	cloud->is_dense = false;
	cloud->points.resize(cloud->height * cloud->width);
}

void initializeVisualizer(
		const pcl::visualization::PCLVisualizer::Ptr& visualizer,
		const std::string& cloudName,
		const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& cloud) {
	visualizer->setPointCloudRenderingProperties(
			pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, cloudName);
	visualizer->initCameraParameters();
	visualizer->setBackgroundColor(0, 0, 0);
	visualizer->setPosition(cloud->width, 0);
	visualizer->setSize(cloud->width, cloud->height);
	visualizer->setShowFPS(true);
	visualizer->setCameraPosition(0, 0, 0, 0, -1, 0);
	visualizer->registerKeyboardCallback(keyboardEvent);
}

void showStatistics(const cv::Mat& mat, double& low, double& high) {
	cv::minMaxLoc(mat, &low, &high);
	std::cout << "min value: " << low << std::endl;
	std::cout << "max value: " << high << std::endl;
}

int main(int argc, char **argv) {
	std::cout << "hello world" << std::endl;
	ros::init(argc, argv, "cloudViewerTest");
	ros::NodeHandle nh;
	RosConnector con(nh, COLOR_TOPIC, IR_TOPIC, DEPTH_TOPIC);
	cv::Mat color, ir, depth;
	Distance d(true, cv::Size(7, 5), argc, argv);

	readCalibrationData();

	con.getColorIrDepth(color, ir, depth);
	cv::Mat output;
	d.updateImages(color, ir, depth);
	bool found = d.findChessboardIr();
	std::cout << found << std::endl;

	cv::Mat normal;
	d.createChessBoardPlane(normal);
	std::cout << normal << std::endl;

	cv::Mat calculatedMat(ir.rows, ir.cols, CV_64F);
	d.createMatForNormal(calculatedMat);

	double low, high;
	showStatistics(calculatedMat, low, high);

	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr depthCloud, computedCloud;
	initializeCloud(ir, depthCloud);
	initializeCloud(calculatedMat, computedCloud);
	createLookupDepth(depth.cols, depth.rows);
	createLookupCalculated(calculatedMat.cols, calculatedMat.rows);

	pcl::visualization::PCLVisualizer::Ptr visualizer(
			new pcl::visualization::PCLVisualizer("Cloud Viewer"));
	const std::string depthCloudName = "depth";
	const std::string calculatedCloudName = "calculated";

	createCloudDepth(depth, 255, 0, 0, depthCloud);
	createCloudCalculated(calculatedMat, 0, 255, 0, computedCloud);
	visualizer->addPointCloud(depthCloud, depthCloudName);
	visualizer->addPointCloud(computedCloud, calculatedCloudName);

	initializeVisualizer(visualizer, depthCloudName, depthCloud);

	running = true;
	while (running) {
		visualizer->spinOnce(10);
		visualizer->updatePointCloud(depthCloud, depthCloudName);
		visualizer->updatePointCloud(computedCloud, calculatedCloudName);
	}

}
