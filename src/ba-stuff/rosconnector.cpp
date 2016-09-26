/*
 * rosconnector.cpp
 *
 *  Created on: Jun 13, 2016
 *      Author: benny
 */

#include <ba-stuff/rosconnector.h>

RosConnector::RosConnector(ros::NodeHandle nh, std::string colorTopic,
		std::string irTopic, std::string depthTopic) {
	update = false;
	this->nh = nh;
	image_transport::ImageTransport it(this->nh);

	image_transport::TransportHints hints("compressed");
	typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image,
			sensor_msgs::Image, sensor_msgs::Image> ColorIrDepthSyncPolicy;

	image_transport::SubscriberFilter *subImageColor =
			new image_transport::SubscriberFilter(it, colorTopic, 4, hints);
	image_transport::SubscriberFilter *subImageIr =
			new image_transport::SubscriberFilter(it, irTopic, 4, hints);
	image_transport::SubscriberFilter *subImageDepth =
			new image_transport::SubscriberFilter(it, depthTopic, 4, hints);

	message_filters::Synchronizer<ColorIrDepthSyncPolicy> *sync =
			new message_filters::Synchronizer<ColorIrDepthSyncPolicy>(
					ColorIrDepthSyncPolicy(4), *subImageColor, *subImageIr,
					*subImageDepth);
	sync->registerCallback(
			boost::bind(&RosConnector::syncedImageCallback, this, _1, _2, _3));
	while (!update) {
		ros::spinOnce();
	}
}

RosConnector::~RosConnector() {
	// TODO Auto-generated destructor stub
}

void RosConnector::syncedImageCallback(const sensor_msgs::ImageConstPtr color,
		const sensor_msgs::ImageConstPtr ir,
		const sensor_msgs::ImageConstPtr depth) {
	update = true;
	cv::Mat colorMat, irMat, depthMat;
	readImage(color, colorMat);
	readImage(ir, irMat);
	readImage(depth, depthMat);

	// IR image input
	if (irMat.type() == CV_16U) {
		cv::Mat tmp;
		irMat.convertTo(tmp, CV_8U, 0.02);
		cv::cvtColor(tmp, irMat, CV_GRAY2BGR);
	}
	lock.lock();
	this->colorMat = colorMat;
	this->irMat = irMat;
	this->depthMat = depthMat;
	lock.unlock();
}

void RosConnector::readImage(const sensor_msgs::Image::ConstPtr msgImage,
		cv::Mat &image) {
	cv_bridge::CvImageConstPtr pCvImage;
	pCvImage = cv_bridge::toCvShare(msgImage, msgImage->encoding);
	pCvImage->image.copyTo(image);
}

void RosConnector::getColor(cv::Mat &color) {
	ros::spinOnce();
	lock.lock();
	color = this->colorMat;
	lock.unlock();
}

void RosConnector::getIr(cv::Mat &ir) {
	ros::spinOnce();
	lock.lock();
	ir = this->irMat;
	lock.unlock();
}

void RosConnector::getDepth(cv::Mat &depth) {
	ros::spinOnce();
	lock.lock();
	depth = this->depthMat;
	lock.unlock();
}

void RosConnector::getColorDepth(cv::Mat &color, cv::Mat &depth) {
	ros::spinOnce();
	lock.lock();
	color = this->colorMat;
	depth = this->depthMat;
	lock.unlock();
}

void RosConnector::getIrDepth(cv::Mat &ir, cv::Mat &depth) {
	ros::spinOnce();
	lock.lock();
	ir = this->irMat;
	depth = this->depthMat;
	lock.unlock();
}

void RosConnector::getColorIrDepth(cv::Mat &color, cv::Mat &ir,
		cv::Mat &depth) {
	ros::spinOnce();
	lock.lock();
	color = this->colorMat;
	ir = this->irMat;
	depth = this->depthMat;
	lock.unlock();
}
