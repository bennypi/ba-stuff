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
#include <thread>
#include <fstream>

#include <ros/ros.h>

#include <opencv2/opencv.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>

const char* COLOR_TOPIC = "/kinect2/hd/image_mono";
const char* IR_TOPIC = "/kinect2/sd/image_ir";
const char* DEPTH_TOPIC = "/kinect2/sd/image_depth";
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

void printMinAndMax(const cv::Mat& mat) {
	double low = 0;
	double high = 0;
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
		}
	}
}

void createCloud(const cv::Mat &mat, int blue, int green, int red,
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud, const cv::Mat &lookupX,
		const cv::Mat &lookupY, cv::Mat &mask) {
	const float badPoint = std::numeric_limits<float>::quiet_NaN();
#pragma omp parallel for
	for (int r = 0; r < mat.rows; ++r) {
		pcl::PointXYZRGBA *itP = &cloud->points[r * mat.cols];
		const uint16_t *itD = mat.ptr<uint16_t>(r);
		uchar *itMask = mask.ptr<uchar>(r);
		const float y = lookupY.at<float>(0, r);
		const float *itX = lookupX.ptr<float>();

		for (size_t c = 0; c < (size_t) mat.cols;
				++c, ++itP, ++itD, ++itX, ++itMask) {
			register const float depthValue = *itD / 1000.0f;
			// Check for invalid measurements
			if (*itD == 0) {
				// not valid
				itP->x = itP->y = itP->z = badPoint;
				itP->rgba = 0;
				*itMask = 0;
				continue;
			}
			itP->z = depthValue;
			itP->x = *itX * depthValue;
			itP->y = y * depthValue;
			itP->b = blue;
			itP->g = green;
			itP->r = red;
			itP->a = 255;
			*itMask = 1;
		}
	}
}

void getDistancePlaneDepth(const cv::Mat &normal, const double distance,
		const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& depth, cv::Mat &result,
		cv::Mat &mask) {
	cv::Mat point(3, 1, CV_64F);
	double* p;
	for (int r = 0; r < depth->height; r++) {
		pcl::PointXYZRGBA *itDepth = &depth->points[r * depth->width];
		p = result.ptr<double>(r);
		uchar *itMask = mask.ptr<uchar>(r);
		for (int j = 0, c = 0; c < depth->width;
				c++, ++itDepth, ++j, ++itMask) {
			if (*itMask) {
				point.at<double>(0, 0) = itDepth->x;
				point.at<double>(0, 1) = itDepth->y;
				point.at<double>(0, 2) = itDepth->z;
				double d = point.dot(normal) - distance;
				// if value is too far outside
				if (d < -0.2 || d > 0.2) {
					p[j] = 0;
					*itMask = 0;
				} else {
					p[j] = d;
				}
			} else {
				p[j] = 0;
			}
		}
	}
}

void prepareMatForColorMap(const cv::Mat &input, cv::Mat &output) {
	CV_Assert(output.depth() == CV_8U);
	CV_Assert(input.size == output.size);
	double outMin = 0;
	double outMax = 255;
	double inMin, inMax;
	cv::minMaxLoc(input, &inMin, &inMax);
	std::cout << inMin << " " << inMax << std::endl;
	uchar* pOutput;
	for (int i = 0; i < input.rows; ++i) {
		const int* pInput = input.ptr<int>(i);
		pOutput = output.ptr<uchar>(i);
		for (int j = 0; j < input.cols; ++j, ++pInput, ++pOutput) {
			double tmpValue = *pInput;
			tmpValue = (tmpValue - inMin) *((outMax-outMin)/(inMax - inMin)) + outMin;
			char value = tmpValue;
			*pOutput = value;
		}
	}
}

void showColorMap(const cv::Mat &input) {
	cv::Mat colorMap;
	cv::namedWindow("Color map", cv::WINDOW_AUTOSIZE);
	cv::applyColorMap(input, colorMap, cv::COLORMAP_RAINBOW);
	cv::imshow("Color Map", colorMap);
	cv::waitKey();
}

void writeDataFile(const cv::Mat &input, const cv::Mat &mask) {
	std::ofstream myfile;
	myfile.open("data.txt");
	myfile
			<< "# distance between calculated plane from chessboard and measured depth values\n";
	myfile << "# values are in meters\n";
	myfile << "# x - y - distance\n";
	for (int row = 0; row < input.rows; ++row) {
		const double* p = input.ptr<double>(row);
		const uchar* pMask = mask.ptr<uchar>(row);
		for (int col = 0; col < input.cols; ++col, ++p, ++pMask) {
			if (*pMask) {
				myfile << row << ' ' << col << ' ' << *p << "\n";
			}
		}
	}
	myfile.close();
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "cloudViewerTest");
	ros::NodeHandle nh;
	RosConnector con(nh, COLOR_TOPIC, IR_TOPIC, DEPTH_TOPIC);
	cv::Mat color, ir, depth;
	Distance distanceInstance(cv::Size(7, 5), argc, argv);

	readCalibrationData();

	std::cout << "searching for chessboards" << std::endl;
	bool found = false;
	while (!found) {
		con.getColorIrDepth(color, ir, depth);
		distanceInstance.updateImages(color, ir, depth);
		bool foundIr = distanceInstance.findChessboardIr();
		bool foundColor = distanceInstance.findChessboardColor();
		std::cout << foundIr << " - " << foundColor << std::endl;
		found = foundIr && foundColor;
	}

	distanceInstance.createColorPlane();
	distanceInstance.createIrPlane();

	cv::Mat normalIr, normalColor;
	double dIr, dColor;
	normalIr = distanceInstance.irCam.normal;
	dIr = distanceInstance.irCam.d;
	normalColor = distanceInstance.colorCam.normal;
	dColor = distanceInstance.colorCam.d;

	std::cout << "distance to chessboardplane in ir: " << dIr << std::endl;
	std::cout << "distance to chessboardplane in color: " << dColor << std::endl;
/*
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr depthCloud, hessianCloud;
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

	cv::Mat mask(depth.rows, depth.cols, CV_8U);
	createCloud(depth, 255, 0, 0, depthCloud, lookupXDepth, lookupYDepth, mask);
	createCloudFromHessian(normal, distance, hessianCloud, depthCloud);

	cv::Mat distanceMat(depth.rows, depth.cols, CV_64F);
	getDistancePlaneDepth(normal, distance, depthCloud, distanceMat, mask);

	double low, high;
	printMinAndMax(distanceMat);
	cv::Scalar meanScalar = cv::mean(distanceMat, mask);
	std::cout << meanScalar << std::endl;

	cv::Mat colorMap(depth.rows, depth.cols, CV_8U);
	prepareMatForColorMap(distanceMat, colorMap);
	printMinAndMax(colorMap);
	meanScalar = cv::mean(colorMap, mask);
	std::cout << meanScalar << std::endl;
	std::thread t(showColorMap, colorMap);

	visualizer->addPointCloud(depthCloud, depthCloudName);
	visualizer->addPointCloud(hessianCloud, hessianCloudName);

	initializeVisualizer(visualizer, depthCloudName, depthCloud);

	running = true;
	while (running) {
		visualizer->spinOnce(10);
		visualizer->updatePointCloud(depthCloud, depthCloudName);
		visualizer->updatePointCloud(hessianCloud, hessianCloudName);
	}
	t.join();
	writeDataFile(distanceMat, mask);
	*/
}
