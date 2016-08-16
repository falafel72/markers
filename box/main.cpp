#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp> 
#include <opencv2/imgproc/imgproc.hpp> 
#include <iostream>
#include <string.h> 
#include <math.h>
#include <dirent.h>

#include "cameraCalibrator.cpp" 

using namespace cv; 
using namespace std; 

Point2f center;

vector<string> getCalibFiles() {
	vector<string> filenames;
	DIR *dir;
	struct dirent *ent; 
	if((dir = opendir("./calibrate")) != NULL) {
		while((ent = readdir(dir)) != NULL) {
			if(strncmp(ent->d_name,".",2) != 0 && strncmp(ent->d_name,"..",2)) {
				filenames.push_back(ent->d_name);
			}
		}
		closedir(dir);
	}
	return filenames;
}

void calibrateCamera(CameraCalibrator &c, Size &imageSize) {
	vector<string> filenames = getCalibFiles();		
	Size boardSize = Size(7,5);
	c.addChessboardPoints(filenames,boardSize); 
	double reprojerr = c.calibrate(imageSize);
}

float getAngle(Point2f &a, Point2f &b) {
	//float gradient = (b.y - a.y)/(b.x-a.x);
	float angleRad = atan2(a.y - b.y, a.x - b.x);
	float angleDeg = angleRad * 180/M_PI;
	return angleDeg;
}

bool clockwiseComp(Point2f& p1, Point2f& p2) { //starting from bottom right
	if(p1.x - center.x >= 0 && p2.x - center.x < 0) return true;
	if(p1.x - center.x < 0 && p2.x - center.x >= 0) return false;

	if(p1.x - center.x == 0 && p2.x - center.x == 0) {
	if(p1.y - center.y >= 0 || p2.y - center.y >= 0) return p1.y > p2.y;
		return p2.y > p1.y;
	}	

	//calculate cross product, should be negative if p2 is clockwise from p1
	float det = (p1.x - center.x) * (p2.y - center.y) - (p2.x - center.x) * (p1.y - center.y);
	return (det < 0) ? true : false;

	//if p1 and p2 are on same radial line, pick the one further out
	float d1 = (p1.x - center.x) * (p1.x - center.x) + (p1.y - center.y) * (p1.y - center.y);
	float d2 = (p2.x - center.x) * (p2.x - center.x) + (p2.y - center.y) * (p2.y - center.y);
	return d1 > d2;
}

void getCenter(vector<Point2f> poly) {
	float centerX = 0;
	float centerY = 0;
	for(int i = 0; i < poly.size(); i++) {
		centerX += poly[i].x;
		centerY += poly[i].y;	
	}
	center = Point2f(centerX/poly.size(), centerY/poly.size());
};


int main(int argc, char **argv) {
	string filename(argv[1]);
	Mat i = imread("images/"+filename,0);

	CameraCalibrator c; 
	Mat cameraMat = imread("./params/mat1.bmp",0);
	Mat distCoeffs = imread("./params/dist1.bmp",0);
	Size imageSize = i.size();
	if(!cameraMat.data) {
		calibrateCamera(c, imageSize);
	}
	else {
		c.setCameraMatrix(cameraMat);
		c.setDistCoeffs(distCoeffs);
	}

	//undistort image	
	Mat image = c.remap(i); 


	Mat thresholded;
	//adaptiveThreshold(image,thresholded,255,ADAPTIVE_THRESH_MEAN_C,THRESH_BINARY_INV,7,0);	
	threshold(image,thresholded, 60, 255, THRESH_BINARY_INV);

	imshow("thresholded", thresholded);

	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	findContours(thresholded, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE, Point(0,0));

	Mat drawing = Mat::zeros(thresholded.size(), CV_8UC3);
	drawContours(drawing,contours,-1,Scalar(255,255,255));
	imshow("contours", drawing);

	vector<vector<Point> > polygons;
	for(int i = 0; i < contours.size(); i++) {
		vector<Point> poly;
		approxPolyDP(Mat(contours[i]), poly, 5, true);
		if(poly.size() == 4 && contourArea(contours[i]) > 2000) {
			polygons.push_back(poly);
		}
	}
	
	if(polygons.size() == 0) {
		cerr << "No box detected!" << endl;
		return -1;
	}
	
	for(int i = 0; i < polygons.size(); i++) {
		polylines(image, polygons[i], true, Scalar(255,255,255), 1, 8);
	}

	imshow("polygons", image);

	vector<Point2f> markPolyf;
	Mat(polygons[0]).copyTo(markPolyf);
	
	getCenter(markPolyf);	
	sort(markPolyf.begin(), markPolyf.end(), clockwiseComp);

	float angle = getAngle(markPolyf[1], markPolyf[2]);
	cout << "rotation: " << angle << endl;
	
	waitKey();
	return 0;
}
