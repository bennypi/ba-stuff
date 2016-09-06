/*
 * rosconnector.cpp
 *
 *  Created on: Jun 13, 2016
 *      Author: benny
 */

#include <ba-stuff/rosconnector.h>

RosConnector::RosConnector(ros::NodeHandle nh, std::string colorTopic,
		std::string depthTopic) {
	update = false;
	this->nh = nh;
	image_transport::ImageTransport it(this->nh);

	image_transport::TransportHints hints("compressed");
	typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image,
			sensor_msgs::Image> ColorIrDepthSyncPolicy;

	image_transport::SubscriberFilter *subImageColor =
			new image_transport::SubscriberFilter(it, colorTopic, 4, hints);
	image_transport::SubscriberFilter *subImageDepth =
			new image_transport::SubscriberFilter(it, depthTopic, 4, hints);

	message_filters::Synchronizer<ColorIrDepthSyncPolicy> *sync =
			new message_filters::Synchronizer<ColorIrDepthSyncPolicy>(
					ColorIrDepthSyncPolicy(4), *subImageColor, *subImageDepth);
	sync->registerCallback(
			boost::bind(&RosConnector::syncedImageCallback, this, _1, _2));
	while (!update)
	{
		ros::spinOnce();
	}
}

RosConnector::~RosConnector() {
	// TODO Auto-generated destructor stub
}

void RosConnector::syncedImageCallback(const sensor_msgs::ImageConstPtr color,
		const sensor_msgs::ImageConstPtr depth) {
	update = true;
	lock.lock();
	readImage(color, colorMat);
	readImage(depth, depthMat);
	lock.unlock();
}

void RosConnector::readImage(const sensor_msgs::Image::ConstPtr msgImage,
		cv::Mat &image) {
	cv_bridge::CvImageConstPtr pCvImage;
	pCvImage = cv_bridge::toCvShare(msgImage, msgImage->encoding);
	pCvImage->image.copyTo(image);
}

void RosConnector::getNewImage(cv::Mat &image) {
	ros::spinOnce();
	lock.lock();
	image = this->imageMat;
	lock.unlock();
}

void RosConnector::getNewImage(cv::Mat &color, cv::Mat &depth) {
	ros::spinOnce();
	lock.lock();
	color = this->colorMat;
	depth = this->depthMat;
	lock.unlock();
}
