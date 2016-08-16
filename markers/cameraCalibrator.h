#include <opencv2/core/core.hpp> 
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

using namespace cv;
using namespace std; 

class CameraCalibrator{ 
    vector<vector<Point3f> > objectPoints; //points in generated 3d environment
    vector<vector<Point2f> > imagePoints; //points in pixels
    Mat cameraMatrix; //matrix with camera values
    Mat distCoeffs; //self explanatory
    int flag; 
    Mat map1,map2; //used in undistortion
    bool mustInitUndistort;
    public: 
	CameraCalibrator(); //constructor

	int addChessboardPoints(const vector<string> &filelist,  //extracts corner points from chessboard images
			    Size &boardSize);
	void addPoints(const vector<Point2f> &imageCorners, //adds scene points and corresponding image points
			const vector<Point3f> &objectCorners);

	double calibrate(Size &imageSize); //self explanatory

	Mat remap(const Mat &image); //removes distortion after calibration

	Mat getCameraMatrix();

	void setCameraMatrix(Mat &in);

	Mat getDistCoeffs();

	void setDistCoeffs(Mat &in);
};
