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

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>

const char* IR_TOPIC = "/kinect2/sd/image_ir";
const char* DEPTH_TOPIC = "/kinect2/sd/image_depth";
double fx, fy, cx, cy;
cv::Mat cameraMatrix, distortion, rvec, rotation, translation,
		extrinsicsRotation, extrinsicsTranslation, lookupY, lookupX;

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

	fx = cameraMatrix.at<double>(0, 0);
	fy = cameraMatrix.at<double>(1, 1);
	cx = cameraMatrix.at<double>(0, 2);
	cy = cameraMatrix.at<double>(1, 2);
}

void createCloud(const cv::Mat &depth, const cv::Mat &color,
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud) {
	const float badPoint = std::numeric_limits<float>::quiet_NaN();

#pragma omp parallel for
	std::cout << "number of points: " << depth.rows *depth.cols << std::endl;
	int count = 0;
	for (int r = 0; r < depth.rows; ++r) {
		pcl::PointXYZRGBA *itP = &cloud->points[r * depth.cols];
		const uint16_t *itD = depth.ptr<uint16_t>(r);
		const cv::Vec3b *itC = color.ptr<cv::Vec3b>(r);
		const float y = lookupY.at<float>(0, r);
		const float *itX = lookupX.ptr<float>();

		for (size_t c = 0; c < (size_t) depth.cols;
				++c, ++itP, ++itD, ++itC, ++itX) {
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
			itP->b = 100;
			itP->g = 0;
			itP->r = 0;
			itP->a = 255;
		}
	}
	std::cout << count << std::endl;
}

void createLookup(size_t width, size_t height) {
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

int main(int argc, char **argv) {
	std::cout << "hello world" << std::endl;
	ros::init(argc, argv, "cloudViewerTest");
	ros::NodeHandle nh;
	RosConnector con(nh, IR_TOPIC, DEPTH_TOPIC);
	cv::Mat ir, depth;

	readCalibrationData();

	con.getNewImage(ir, depth);

	const char* name = "image";
	cv::namedWindow(name, cv::WINDOW_AUTOSIZE);
	cv::imshow(name, ir);
//	cv::waitKey();

	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud = pcl::PointCloud<
			pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>());
	cloud->height = ir.rows;
	cloud->width = ir.cols;
	cloud->is_dense = false;
	cloud->points.resize(cloud->height * cloud->width);
	createLookup(ir.cols, ir.rows);

	pcl::visualization::PCLVisualizer::Ptr visualizer(
			new pcl::visualization::PCLVisualizer("Cloud Viewer"));
	const std::string cloudName = "rendered";

	createCloud(depth, ir, cloud);

	visualizer->addPointCloud(cloud, cloudName);
	visualizer->setPointCloudRenderingProperties(
			pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, cloudName);
	visualizer->initCameraParameters();
	visualizer->setBackgroundColor(100, 0, 0);
	visualizer->setPosition(ir.cols, 0);
	visualizer->setSize(ir.cols, ir.rows);
	visualizer->setShowFPS(true);
	visualizer->setCameraPosition(0, 0, 0, 0, -1, 0);

	cv::waitKey();
}