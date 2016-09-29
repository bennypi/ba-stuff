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
		extrinsicsRotation, extrinsicsTranslation, lookupYDepth, lookupXDepth,
		lookupYCalculated, lookupXCalculated;
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

void createCloud(const cv::Mat &mat, int blue, int green, int red,
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud, cv::Mat &lookupX,
		cv::Mat &lookupY) {
	const float badPoint = std::numeric_limits<float>::quiet_NaN();

#pragma omp parallel for
	for (int r = 0; r < mat.rows; ++r) {
		pcl::PointXYZRGBA *itP = &cloud->points[r * mat.cols];
		const uint16_t *itD = mat.ptr<uint16_t>(r);
		const float y = lookupY.at<float>(0, r);
		const float *itX = lookupX.ptr<float>();

		for (size_t c = 0; c < (size_t) mat.cols; ++c, ++itP, ++itD, ++itX) {
			register const float depthValue = *itD / 1000.0f;
			// Check for invalid measurements
			if (*itD == 0) {
				// not valid
				itP->x = itP->y = itP->z = badPoint;
				itP->rgba = 0;
				continue;
			}
			itP->z = depthValue;
			itP->x = *itX * depthValue;
			itP->y = y * depthValue;
			itP->b = blue;
			itP->g = green;
			itP->r = red;
			itP->a = 255;
		}
	}
}

void createLookup(size_t width, size_t height, cv::Mat &lookupX,
		cv::Mat &lookupY) {
	float *it;
	lookupY = cv::Mat(1, height, CV_32F);
	it = lookupY.ptr<float>();
	for (size_t r = 0; r < height; ++r, ++it) {
		*it = (r - cy) * fy;
	}
	lookupX = cv::Mat(1, width, CV_32F);
	it = lookupX.ptr<float>();
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

void createCloudFromHessian(cv::Mat &normal, double distance,
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& hessianCloud,
		const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& originCloud) {
	double norm[3];
	norm[0] = normal.at<double>(0, 0);
	norm[1] = normal.at<double>(0, 1);
	norm[2] = normal.at<double>(0, 2);

	for (int r = 0; r < hessianCloud->height; r++) {
		pcl::PointXYZRGBA *itHessian = &hessianCloud->points[r
				* hessianCloud->width];
		pcl::PointXYZRGBA *itOrigin = &originCloud->points[r
				* originCloud->width];
		for (int c = 0; c < hessianCloud->width; c++, ++itHessian, ++itOrigin) {
			double z = (-norm[0] * itOrigin->x - norm[1] * itOrigin->y
					+ distance) / norm[2];
			itHessian->x = itOrigin->x;
			itHessian->y = itOrigin->y;
			itHessian->z = z;
			itHessian->b = 0;
			itHessian->g = 0;
			itHessian->r = 255;
			itHessian->a = 255;
//			std::cout << itOrigin->x << " " << itOrigin->y << " " << itOrigin->z << std::endl;
		}
	}
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
	double distance;
	d.createChessBoardPlane(normal, distance);


	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr depthCloud, calculatedCloud,
			hessianCloud;
	initializeCloud(ir, depthCloud);
	createLookup(depth.cols, depth.rows, lookupXDepth, lookupYDepth);

	// create cloud from hessian
	hessianCloud = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(
			new pcl::PointCloud<pcl::PointXYZRGBA>());
	hessianCloud->height = ir.rows;
	hessianCloud->width = ir.cols;
	hessianCloud->is_dense = false;
	hessianCloud->points.resize(hessianCloud->height * hessianCloud->width);

	pcl::visualization::PCLVisualizer::Ptr visualizer(
			new pcl::visualization::PCLVisualizer("Cloud Viewer"));
	const std::string depthCloudName = "depth";
	const std::string hessianCloudName = "hessian";

	createCloud(depth, 255, 0, 0, depthCloud, lookupXDepth, lookupYDepth);
	createCloudFromHessian(normal, distance, hessianCloud, depthCloud);

	visualizer->addPointCloud(depthCloud, depthCloudName);
	visualizer->addPointCloud(hessianCloud, hessianCloudName);

	initializeVisualizer(visualizer, depthCloudName, depthCloud);

	running = true;
	while (running) {
		visualizer->spinOnce(10);
		visualizer->updatePointCloud(depthCloud, depthCloudName);
		visualizer->updatePointCloud(hessianCloud, hessianCloudName);
	}

}
