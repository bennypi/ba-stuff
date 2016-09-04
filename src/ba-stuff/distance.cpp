/*
 * Distance.cpp
 *
 *  Created on: Mar 2, 2016
 *      Author: benny
 */

#include <ba-stuff/distance.h>

const char* Distance::KINECT_IMAGE = "COLOR_IMAGE";
const char* Distance::COLOR_MAP = "COLOR_MAP";
const char* Distance::IR_IMAGE = "IR_IMAGE";
const char* IR_TOPIC = "/kinect2/sd/image_ir";
const char* DEPTH_TOPIC = "/kinect2/sd/image_depth";
const char* COLOR_TOPIC = "/kinect2/hd/image_color";

void Distance::createSimpleSubscriber(ros::NodeHandle nh, char const *topic) {
	Distance::sub = nh.subscribe(topic, 1000, &Distance::imageCallback, this);
}

void Distance::createSyncedSubscriber(ros::NodeHandle& nh, char const *topic) {
	image_transport::ImageTransport it(nh);
	image_transport::TransportHints hints("compressed");
	typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image,
			sensor_msgs::Image> ColorIrDepthSyncPolicy;
	image_transport::SubscriberFilter* subImageColor =
			new image_transport::SubscriberFilter(it, IR_TOPIC, 4, hints);
	image_transport::SubscriberFilter* subImageDepth =
			new image_transport::SubscriberFilter(it, DEPTH_TOPIC, 4, hints);
	message_filters::Synchronizer<ColorIrDepthSyncPolicy>* sync =
			new message_filters::Synchronizer<ColorIrDepthSyncPolicy>(
					ColorIrDepthSyncPolicy(4), *subImageColor, *subImageDepth);
	sync->registerCallback(
			boost::bind(&Distance::syncedImageCallback, this, _1, _2));
}

Distance::Distance(bool mode_ir, bool mode_synced, const cv::Size &size,
		int argc, char **argv) :
		mode_ir(mode_ir), mode_synced(mode_synced), boardDims(size), chessBoardFound(
				false), update(false) {
	ros::init(argc, argv, "distance");
	ros::NodeHandle nh;
//	ros::AsyncSpinner spinner(1);

	if (mode_ir && !mode_synced) {
		createSimpleSubscriber(nh, IR_TOPIC);
	} else if (!mode_ir && !mode_synced) {
		createSimpleSubscriber(nh, COLOR_TOPIC);
	} else if (mode_ir && mode_synced) {
		createSyncedSubscriber(nh, IR_TOPIC);
	} else {
		createSyncedSubscriber(nh, COLOR_TOPIC);
	}
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
	std::string path;
	if (mode_ir) {
		path = ("/home/benny/kinect_cal_data/calib_ir.yaml");
	} else {
		path = ("/home/benny/kinect_cal_data/calib_color.yaml");
	}
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
//	std::cout << Distance::cameraMatrix << std::endl;
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

void Distance::findChessboardCorners() {
	if (Distance::color.type() == 2) {
		std::cout << "converted from 16bit to 8 bit" << std::endl;
		Distance::color.convertTo(Distance::color, CV_8U, 0.00390625);
	}
	if (mode_ir) {
		Distance::chessBoardFound = cv::findChessboardCorners(Distance::color,
				Distance::boardDims, Distance::output,
				cv::CALIB_CB_ADAPTIVE_THRESH);
	}
	Distance::update = true;
}

void Distance::imageCallback(const sensor_msgs::ImageConstPtr color) {
	readImage(color, Distance::color);
	findChessboardCorners();
}

void Distance::syncedImageCallback(const sensor_msgs::ImageConstPtr color,
		const sensor_msgs::ImageConstPtr depth) {
	readImage(color, Distance::color);
	readImage(depth, Distance::depth);
	findChessboardCorners();
}

void calculateDistanceToChessboardCorners(double normalDistance, Distance& d) {
	cv::Mat distanceOfCorners(7, 5, CV_64F);
	for (int i = 0, idx = 0; i < distanceOfCorners.rows; i++) {
		for (int j = 0; j < distanceOfCorners.cols; j++, idx++) {
			double distanceOfPoint = d.computeDistanceToPoint(
					d.output.at<cv::Point2f>(idx), d.normal, normalDistance);
			distanceOfCorners.at<double>(i, j) = distanceOfPoint;
		}
	}
}

void calculateDistanceToPlane(cv::Mat mat, double normalDistance, Distance& d) {
	for (int i = 0, idx = 0; i < mat.rows; i++) {
		for (int j = 0; j < mat.cols; j++, idx++) {
			double distanceOfPoint = d.computeDistanceToPoint(cv::Point2i(i, j),
					d.normal, normalDistance);
			mat.at<double>(i, j) = distanceOfPoint;
		}
	}
}

void stupidColorMap(cv::Mat mat) {
	if (mat.type() == 3) {
		mat.convertTo(mat, CV_8U);
	}
	cv::namedWindow(Distance::COLOR_MAP, cv::WINDOW_AUTOSIZE);
	cv::applyColorMap(mat, mat, cv::COLORMAP_RAINBOW);
	cv::imshow(Distance::COLOR_MAP, mat);
}

void shrinkMatrix(cv::Mat &mat, int min, int max) {
	int nRows = mat.rows;
	int nCols = mat.cols;

	if (mat.isContinuous()) {
		nCols *= nRows;
		nRows = 1;
	}

	int i, j;
	short* p;
	for (i = 0; i < nRows; ++i) {
		p = mat.ptr<short>(i);
		for (j = 0; j < nCols; ++j) {
			p[j] = ((int16_t) p[j] < min) ? min : p[j];
			p[j] = ((int16_t) p[j] > max) ? max : p[j];
		}
	}
}

void showStatistics(const cv::Mat& mat, double& low, double& high) {
	cv::minMaxLoc(mat, &low, &high);
	std::cout << "min value: " << low << std::endl;
	std::cout << "max value: " << high << std::endl;
}

int main(int argc, char **argv) {
	Distance d(true, true, cv::Size(7, 5), argc, argv);
	d.cloudEnabled = false;
	if (argc == 2) {
		std::string arg = argv[1];
		if (arg == "cloud") {
			std::cout << "displaying cloud" << std::endl;
			d.cloudEnabled = true;
		}
	}

	cv::namedWindow(Distance::KINECT_IMAGE, cv::WINDOW_AUTOSIZE);

	d.readCalibrationData();
	d.createBoardPoints();

	ros::Rate r(10);

	ros::spinOnce();
	while (!d.chessBoardFound) {
		if (d.update) {
			cv::imshow(Distance::KINECT_IMAGE, d.color);
			d.update = false;
		}
		cv::waitKey(100);
		ros::spinOnce();
		r.sleep();
	}
	std::cout << "chessboard found" << std::endl;

	// trying to show IR image
	cv::Mat depth;
	d.depth.copyTo(depth);
	double low, high;
	std::cout << "values for the depth:" << std::endl;
	showStatistics(depth, low, high);
	shrinkMatrix(depth, 0, 1500);
	showStatistics(depth, low, high);
	double alpha = 255 / high;
	depth.convertTo(depth, 0, alpha);
	std::cout << "values for the depth:" << std::endl;
	showStatistics(depth, low, high);
	cv::Mat coloredDepth;
	cv::applyColorMap(depth, coloredDepth, cv::COLORMAP_RAINBOW);
	cv::namedWindow(Distance::IR_IMAGE, cv::WINDOW_AUTOSIZE);
	cv::imshow(Distance::IR_IMAGE, coloredDepth);

	// get the normal and distance
	d.normal = cv::Mat(3, 1, CV_64F);
	double normalDistance = d.getNormalWithDistance(d.output, d.normal);

	// draw the beautiful cv image
	d.drawDetailsInImage(normalDistance);

	// calculate the distance to every single pixel
	cv::Mat calculatedPlane(d.color.rows, d.color.cols, CV_64F);
	calculateDistanceToPlane(calculatedPlane, normalDistance, d);
	double min, max;

	// make the values millimeters and transform them to CV_16S
	calculatedPlane.convertTo(calculatedPlane, CV_16S, 1000);
	std::cout << "values for the calculated plane:" << std::endl;
	showStatistics(calculatedPlane, min, max);
	// create a new mat with signed ints
	cv::Mat signedDepth;
	d.depth.convertTo(signedDepth, CV_16S);

	cv::Mat mask = cv::Mat::zeros(signedDepth.size(), CV_8UC1);
	mask.setTo(255, signedDepth > 0);

	std::cout << "values for the signed depth image:" << std::endl;
	cv::minMaxLoc(signedDepth, &min, &max, 0, 0, mask);
	std::cout << "min value: " << min << std::endl;
	std::cout << "max value: " << max << std::endl;

	// subtract the calculated and measured distances
	cv::Mat difference(d.color.rows, d.color.cols, CV_16S);
	difference = signedDepth - calculatedPlane;

	// find minmax
	std::cout << "values for the intial difference:" << std::endl;
	showStatistics(difference, min, max);
	std::cout << "mean: " << cv::mean(difference) << std::endl;

	// cut off every value smaller and bigger than -128 and 127
	shrinkMatrix(difference, -128, 127);
	// shift values to 0...255
	cv::convertScaleAbs(difference, difference, 1, 128);
	std::cout << "values after shrinking:" << std::endl;
	showStatistics(difference, min, max);
	std::cout << "mean: " << cv::mean(difference) << std::endl;

	// show the colormap
	stupidColorMap(difference);

	cv::waitKey();
}
