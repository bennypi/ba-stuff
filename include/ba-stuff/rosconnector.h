/*
 * rosconnector.h
 *
 *  Created on: Jun 13, 2016
 *      Author: benny
 */

#ifndef BA_STUFF_SRC_ROSCONNECTOR_H_
#define BA_STUFF_SRC_ROSCONNECTOR_H_

#include <stdlib.h>
#include <stdio.h>
#include <mutex>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/time_synchronizer.h>

#include <ros/ros.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class RosConnector {
public:
	RosConnector(ros::NodeHandle nh, std::string colorTopic, std::string depthTopic);
	virtual ~RosConnector();
	void getNewImage(cv::Mat &image);
	void getNewImage(cv::Mat &color, cv::Mat &depth);

private:
	void syncedImageCallback(const sensor_msgs::ImageConstPtr color, const sensor_msgs::ImageConstPtr depth);
	void readImage(const sensor_msgs::Image::ConstPtr msgImage, cv::Mat &image);
	cv::Mat colorMat, depthMat, imageMat;
	ros::NodeHandle nh;
	std::mutex lock;
	bool update;
};

#endif /* BA_STUFF_SRC_ROSCONNECTOR_H_ */
