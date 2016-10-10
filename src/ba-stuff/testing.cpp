/*
 * testing.cpp
 *
 *  Created on: Oct 10, 2016
 *      Author: benny
 */

#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <ros/ros.h>

#include <opencv2/opencv.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>

int main(int argc, char **argv) {
	std::cout << "huhu" << std::endl;
	cv::Mat ref(5, 3, CV_8U);
	cv::Mat dest(5, 3, CV_8U);
	int counter = 0;

	for (int r = 0; r < ref.rows; ++r) {
		int *itD = ref.ptr<int>(r);
		int *itMask = dest.ptr<int>(r);

		for (size_t c = 0; c < (size_t) ref.cols; ++c, ++itD, ++itMask) {
			itMask[c] = counter;
			*itD = counter;
			std::cout << counter << std::endl;
			std::cout << c << std::endl;
			counter++;
		}
	}
	std::cout << ref << std::endl;
	std::cout << dest << std::endl;

	counter = 0;

	cv::Mat I(5, 3, CV_8U);
	int nRows = I.rows;
	int nCols = I.cols;

	int i, j;
	uchar* p;
	for (i = 0; i < I.rows; ++i) {
		p = I.ptr<uchar>(i);
		for (j = 0; j < I.cols; ++j) {
			p[j] = counter;
			counter++;
		}
	}
	std::cout << I << std::endl;
	std::cout << sizeof(uchar) << std::endl;
}

