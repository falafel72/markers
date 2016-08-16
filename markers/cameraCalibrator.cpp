#include "cameraCalibrator.h"

//using namespace CameraCalibrator;

CameraCalibrator::CameraCalibrator(): flag(0) , mustInitUndistort(true) {};

int CameraCalibrator::addChessboardPoints(const vector<string> &filelist,
			Size &boardSize) {
    //chessboard points
    vector<Point2f> imageCorners;
    vector<Point3f> objectCorners;

    //init corners
    for(int i = 0; i < boardSize.height; i++) {
	for(int j = 0; j < boardSize.width; j++) {
	    objectCorners.push_back(Point3f(i,j,0.0f));
	}
    }

    Mat image; 
    int successes = 0;
    //go through each chessboard image
    for(int i = 0; i < filelist.size(); i++) {
	image = imread("calibrate/" + filelist[i],0);
	bool found = findChessboardCorners(image,boardSize,imageCorners);
	//get corners w/ subpixel accuracy
	cornerSubPix(image, imageCorners, Size(5,5), Size(-1,-1), 
		    TermCriteria(TermCriteria::MAX_ITER + TermCriteria::EPS,
				30, //max iterations
				0.1)); //accuracy
	if(imageCorners.size() == boardSize.area()) {
	    addPoints(imageCorners,objectCorners);
	    successes++;
	}
    }
    return successes;
}

void CameraCalibrator::addPoints(const vector<Point2f> &imageCorners, 
		const vector<Point3f> &objectCorners) {
    imagePoints.push_back(imageCorners);
    objectPoints.push_back(objectCorners);
}

double CameraCalibrator::calibrate(Size &imageSize) {
    mustInitUndistort = true; 
    vector<Mat> rvecs, tvecs;
    
    return calibrateCamera(objectPoints,
			    imagePoints,
			    imageSize, 
			    cameraMatrix, 
			    distCoeffs,
			    rvecs, tvecs, 
			    flag);
}

Mat CameraCalibrator::remap(const Mat &image) {
    Mat undistorted;
    if(mustInitUndistort) {
	initUndistortRectifyMap(
		cameraMatrix,
		distCoeffs,
		Mat(),
		Mat(),
		image.size(), 
		CV_32FC1,
		map1,map2);
	mustInitUndistort = false;
    }
    cv::remap(image,undistorted,map1,map2,INTER_LINEAR);
    return undistorted; 
}

Mat CameraCalibrator::getCameraMatrix() {
    return cameraMatrix;
}

void CameraCalibrator::setCameraMatrix(Mat &in) {
    cameraMatrix = in;
}

Mat CameraCalibrator::getDistCoeffs() {
    return distCoeffs;
}

void CameraCalibrator::setDistCoeffs(Mat &in) {
    distCoeffs = in;
}
