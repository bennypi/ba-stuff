/*
 * Distance.cpp
 *
 *  Created on: Mar 2, 2016
 *      Author: benny
 */

#include <ba-stuff/distance.h>

const char* KINECT_IMAGE = "COLOR_IMAGE";
const char* COLOR_MAP = "COLOR_MAP";
const char* IR_IMAGE = "IR_IMAGE";

void Distance::readCalibrationData() {
	cv::FileStorage fs;
	std::string path;

	path = ("/home/benny/kinect_cal_data/calib_ir.yaml");
	if (fs.open(path, cv::FileStorage::READ)) {
		fs["cameraMatrix"] >> irCam.cameraMatrix;
		fs["distortionCoefficients"] >> irCam.distortion;
		fs.release();
		irCam.fx = irCam.cameraMatrix.at<double>(0, 0);
		irCam.fy = irCam.cameraMatrix.at<double>(1, 1);
		irCam.cx = irCam.cameraMatrix.at<double>(0, 2);
		irCam.cy = irCam.cameraMatrix.at<double>(1, 2);
	} else {
		std::cerr << "couldn't read calibration '" << path << "'!" << std::endl;
	}

	path = ("/home/benny/kinect_cal_data/calib_color.yaml");
	if (fs.open(path, cv::FileStorage::READ)) {
		fs["cameraMatrix"] >> colorCam.cameraMatrix;
		fs["distortionCoefficients"] >> colorCam.distortion;
		fs.release();
		colorCam.fx = colorCam.cameraMatrix.at<double>(0, 0);
		colorCam.fy = colorCam.cameraMatrix.at<double>(1, 1);
		colorCam.cx = colorCam.cameraMatrix.at<double>(0, 2);
		colorCam.cy = colorCam.cameraMatrix.at<double>(1, 2);
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
//	do i need this?
//	cv::initUndistortRectifyMap(cameraMatrix, distortion, cv::Mat(), cameraMatrix, size, CV_32FC1, mapX, mapY);
}

Distance::Distance(const cv::Size &size, int argc, char **argv) :
		boardDims(size) {
	ros::init(argc, argv, "distance");
	ros::NodeHandle nh;
	chessBoardFound = false;
	update = false;
	readCalibrationData();
	createBoardPoints();
}

Distance::~Distance() {
	// TODO Auto-generated destructor stub
}

void Distance::createBoardPoints() {
	board.resize(boardDims.width * boardDims.height);
	for (size_t r = 0, i = 0; r < (size_t) (boardDims.height); ++r) {
		for (size_t c = 0; c < (size_t) (boardDims.width); ++c, ++i) {
			board[i] = cv::Point3f(c * boardSize, r * boardSize, 0);
		}
	}
}

bool Distance::findChessboardColor() {
	bool found = cv::findChessboardCorners(color, boardDims, pointsColor,
			cv::CALIB_CB_FAST_CHECK);
	if (found) {
		const cv::TermCriteria termCriteria(
				cv::TermCriteria::COUNT + cv::TermCriteria::COUNT, 100,
				DBL_EPSILON);
		cv::cornerSubPix(color, pointsColor, cv::Size(11, 11), cv::Size(-1, -1),
				termCriteria);
	}
	return found;
}

bool Distance::findChessboardIr() {
	if (ir.type() == 2) {
		std::cout << "converted ir mat from 16bit to 8 bit" << std::endl;
		ir.convertTo(ir, CV_8U, 0.00390625);
	}
	bool found = cv::findChessboardCorners(ir, boardDims, pointsIr,
			cv::CALIB_CB_ADAPTIVE_THRESH);
	if (found) {
		const cv::TermCriteria termCriteria(
				cv::TermCriteria::COUNT + cv::TermCriteria::COUNT, 100,
				DBL_EPSILON);
		cv::cornerSubPix(ir, pointsIr, cv::Size(11, 11), cv::Size(-1, -1),
				termCriteria);
	}
	return found;
}

void Distance::createIrPlane() {
	normal = cv::Mat(3, 1, CV_64F);
	cv::Mat rvec, translation, rotation;
	cv::solvePnPRansac(board, pointsIr, irCam.cameraMatrix, irCam.distortion,
			rvec, translation, false, 300, 0.05, board.size(), cv::noArray(),
			cv::ITERATIVE);

	cv::Rodrigues(rvec, rotation);

	normal.at<double>(0) = 0;
	normal.at<double>(1) = 0;
	normal.at<double>(2) = 1;
	normal = rotation * normal;
	irCam.normal = normal;
	irCam.d = normal.dot(translation);
}

void Distance::createColorPlane() {
	normal = cv::Mat(3, 1, CV_64F);
	cv::Mat rvec, translation, rotation;
	cv::solvePnPRansac(board, pointsColor, colorCam.cameraMatrix,
			colorCam.distortion, rvec, translation, false, 300, 0.05,
			board.size(), cv::noArray(), cv::ITERATIVE);

	cv::Rodrigues(rvec, rotation);

	normal.at<double>(0) = 0;
	normal.at<double>(1) = 0;
	normal.at<double>(2) = 1;
	normal = rotation * normal;
	colorCam.normal = normal;
	colorCam.d = normal.dot(translation);
}

void Distance::updateImages(cv::Mat &color, cv::Mat &ir, cv::Mat &depth) {
	this->color = color;
	this->ir = ir;
	this->depth = depth;
}

/*
 void stupidColorMap(cv::Mat mat) {
 if (mat.type() == 3) {
 mat.convertTo(mat, CV_8U);
 }
 cv::namedWindow(COLOR_MAP, cv::WINDOW_AUTOSIZE);
 cv::applyColorMap(mat, mat, cv::COLORMAP_RAINBOW);
 cv::imshow(COLOR_MAP, mat);
 }
 */

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

/*
 void showStatistics(const cv::Mat& mat, double& low, double& high) {
 cv::minMaxLoc(mat, &low, &high);
 std::cout << "min value: " << low << std::endl;
 std::cout << "max value: " << high << std::endl;
 }
 */

/*
 int main(int argc, char **argv) {
 Distance d(true, cv::Size(7, 5), "/kinect2/sd/image_ir",
 "/kinect2/sd/image_dept", "/kinect2/hd/image_color", argc, argv);
 d.cloudEnabled = false;
 if (argc == 2) {
 std::string arg = argv[1];
 if (arg == "cloud") {
 std::cout << "displaying cloud" << std::endl;
 d.cloudEnabled = true;
 }
 }

 cv::namedWindow(KINECT_IMAGE, cv::WINDOW_AUTOSIZE);

 d.readCalibrationData();
 d.createBoardPoints();

 ros::Rate r(10);

 ros::spinOnce();
 while (!d.chessBoardFound) {
 if (d.update) {
 cv::imshow(KINECT_IMAGE, d.color);
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
 cv::namedWindow(IR_IMAGE, cv::WINDOW_AUTOSIZE);
 cv::imshow(IR_IMAGE, coloredDepth);

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
 */
