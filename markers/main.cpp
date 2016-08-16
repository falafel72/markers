#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <string>
#include <iostream>
#include <math.h>
#include <dirent.h>
#include "cameraCalibrator.cpp" 

using namespace cv;
using namespace std;

Point center;

bool clockwiseComp(Point2f p1, Point2f p2) { //starting from bottom right
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

bool isMarker(Mat &markerCode) {
	Mat markBin;
	markerCode.convertTo(markBin,CV_8UC1);
	Mat mask(markBin.rows, markBin.cols, CV_8UC1, Scalar(255));
	Mat_<unsigned char>::iterator it = mask.begin<unsigned char>();
	Mat_<unsigned char>::iterator itend = mask.end<unsigned char>();
	int counter = 0;

	for (int counter = 0; it != itend; ++counter, ++it) {
		int row = counter/mask.rows;
		int col = counter % mask.cols;
		if(row == 0 || row == mask.rows-1 || row == mask.rows/2 - 1 || (mask.rows % 2 == 0 && row == mask.rows/2) ) {
			(*it) = 0;
		}
		if(col == 0 || col == mask.cols-1 || col == mask.cols/2 - 1 || (mask.cols % 2 == 0 && col == mask.cols/2)) {
			(*it) = 0;
		}
		//cout << counter << " " << *it << endl;	
	}
	Mat bitOr = Mat(markerCode.size(), CV_8UC1);
	//cout << mask << endl;
	bitwise_or(markBin,mask,bitOr);
	//cout << markBin << endl;
	//cout << mask << endl;
	//cout << bitOr << endl;
	//cout << (mask!=bitOr) << endl;
	
	return countNonZero(mask!=bitOr) == 0;
}

Mat getCode(Mat &marker, int markerSize=14) {
	Mat code(Size(markerSize,markerSize), CV_8UC1);
	Mat_<unsigned char>::iterator it = code.begin<unsigned char>(); 
	for(int row = 0; row < markerSize; row++) {
		for(int col = 0; col < markerSize; col++) {
			Mat region = marker(Rect((col * marker.cols/markerSize), (row * marker.rows/markerSize), marker.cols/markerSize,marker.rows/markerSize));
			Scalar output; 
			(*it) = (mean(region)[0] < 90) ? 255 : 0;
			it++;
		}
	}
	//code.convertTo(code,);
	return code;
}

int getQuadrant(Mat &marker) {
	Mat temp;
	for(int i = 0; i < 4; i++) {
		temp = imread("templates/" + to_string(i) + ".bmp", CV_8UC1);
		if(countNonZero(temp!=marker) == 0) {
			return (i >= 90) ? 180-i : i ;
		}
	}

	return -1;
}

void visSections(Mat &marker, int markerSize = 14) {
	for(int row = 0; row < markerSize; row++) {
		line(marker,Point(0,(row * marker.rows/markerSize)),Point(marker.rows,(row * marker.rows/markerSize)), Scalar(0,0,0));
	}
	for(int col = 0; col < markerSize; col++) {
		line(marker,Point((col * marker.cols/markerSize),0),Point((col * marker.cols/markerSize),marker.cols), Scalar(0,0,0));
	}
}

vector<int> getDimensions(vector<Point2f> &Marker) {
	vector<int> dimensions;
	int width1 = round(sqrt(pow(Marker[0].x - Marker[1].x,2) + pow(Marker[0].y - Marker[1].y,2)));
	int width2 = round(sqrt(pow(Marker[2].x - Marker[3].x,2) + pow(Marker[2].y - Marker[3].y,2)));
	dimensions.push_back((width1 >= width2) ? width1 : width2);

	int height1 = round(sqrt(pow(Marker[0].x - Marker[3].x,2) + pow(Marker[0].y - Marker[3].y,2)));
	int height2 = round(sqrt(pow(Marker[1].x - Marker[2].x,2) + pow(Marker[1].y - Marker[2].y,2)));
	dimensions.push_back((height1 >= height2) ? height1 : height2);

	return dimensions;
}

void denoisify(Mat &in, Mat &out) {
	//blur(in, out, Size(3,3));
	erode (in,out,Mat(Size(3,3), CV_8U));
}

float getAngle(Point2f &a, Point2f &b) {
	//float gradient = (b.y - a.y)/(b.x-a.x);
	float angleRad = atan2(a.y - b.y, a.x - b.x);
	float angleDeg = angleRad * 180/M_PI;
	return angleDeg;
}

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
	//CameraCalibrator c;
	vector<string> filenames = getCalibFiles();
	
	Size boardSize = Size(7,5);
	c.addChessboardPoints(filenames,boardSize); 
	double reprojerr = c.calibrate(imageSize);
	//cout << reprojerr << endl;	
	//return c.getCameraMatrix();
}

int main(int argc, char **argv) {
	string str(argv[1]);
	Mat i = imread("./images/marker"+str+".jpg",0);
	//resize(i,i,Size(0,0),0.75,0.75);
	
	//get camera parameters
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

	//minimise noise with erosion
	Mat denoised;
	denoisify(image,denoised);


	//threshold image
	int thresh = 40;

	Mat thresholded;
	threshold(image,thresholded,90,255,THRESH_BINARY_INV);
	imshow("thresholded",thresholded);
	
	//perform opening of image to allow for better edge detection
	Mat element = Mat(Size(3,3),CV_8U);
	Mat element2 = Mat(Size(5,5),CV_8U);
	erode(thresholded,thresholded,element);
	dilate(thresholded,thresholded,element2);
	
	//get contours
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy; 
	findContours(thresholded, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE, Point(0,0));
	
	Mat drawing = Mat::zeros(thresholded.size(), CV_8UC3);
	Mat drawing2 = image.clone();

	drawContours(drawing,contours,-1,Scalar(255,255,255));
	
	//find suitable polygons that could be markers
	vector<vector<Point> > polygons; 
	vector<Rect> boundingRects;
	for(int i = 0; i < contours.size() ; i++) {
	   vector<Point> poly;
	   approxPolyDP(Mat(contours[i]), poly, 5, true);
	   Scalar color(255,255,255);
	   if(poly.size() == 4) {
		   Rect bound = boundingRect(Mat(contours[i]));
		   if(contourArea(contours[i]) > 2000) {
			   polylines(drawing2, poly, true, color, 1, 8);
			   boundingRects.push_back(bound);
			   polygons.push_back(poly);
			   rectangle(drawing2,bound.tl(),bound.br(),Scalar(0,255,0));
		   }
		}
	}

	//imshow("contours",drawing);
	imshow("polygons", drawing2);
	

	//copies all potential markers to their own images
	vector<Mat> marks; 
	for(int i = 0; i < boundingRects.size(); i++) {
		Mat imageROI = image(Rect(boundingRects[i].x,boundingRects[i].y,boundingRects[i].width,boundingRects[i].height));
		Mat marker;
		imageROI.copyTo(marker);	
		marks.push_back(marker);
	}

	for(int i = 0; i < marks.size(); i++) {
		//threshold to get contours
		
		Mat markThresh;
		threshold(marks[i],markThresh,90,255,THRESH_BINARY_INV);
		Mat markThresh2 = markThresh.clone();


		vector<vector<Point> > markContours;
		vector<Vec4i> markHierarchy; 
		findContours(markThresh, markContours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE, Point(0,0));


		Mat markdraw = Mat::zeros(marks[i].size(), CV_8UC3);

		drawContours(markdraw,markContours,-1,Scalar(255,255,255));


		vector<Point> markPoly; 
		vector<Point> markContour;
		//vector<Rect> markRect;
		for(int i = 0; i < markContours.size() ; i++) {
				//estimate with polygon to get new coordinates
			vector<Point> poly;
			approxPolyDP(Mat(markContours[i]), poly, 5, true);
			Scalar color(255,255,255);
			if(poly.size() == 4) {
				if(contourArea(markContours[i]) > 2000) {
					markPoly = poly;
					markContour = markContours[i];
					//drawContours(markdraw,markContours,i,Scalar(255,255,255));
				}
			}
		}
		
		imshow("poly"+to_string(i),markdraw);

		//if no polygons detected, move to the next "marker"
		if(markContour.size() == 0) {
			cerr << "Region invalid" << endl;
			continue;
		}

		//get center of polygon
	  	Moments m = moments(markContour);
		center = Point(m.m10/m.m00, m.m01/m.m00);
		vector<Point2f> markPolyf;
	 	Mat(markPoly).copyTo(markPolyf);
		//sort corners clockwise	  
		sort(markPolyf.begin(), markPolyf.end(), clockwiseComp);
		
		//get width and height of marker image
		vector<int> dimensions = getDimensions(markPolyf);	
		
		//order rectangle corners clockwise (bottom right first)
		vector<Point2f> markRect;
		markRect.push_back(Point2f(dimensions[0],dimensions[1]));
		markRect.push_back(Point2f(dimensions[0],0));
		markRect.push_back(Point2f(0,0));
		markRect.push_back(Point2f(0, dimensions[1]));


		//get transformation matrix and wrap perspective
		Mat transform = getPerspectiveTransform(markPolyf,markRect);
		Mat warped;
		warpPerspective(markThresh2,warped,transform,Size(dimensions[0], dimensions[1]));

		//draw lines
		visSections(warped);

		//only display if it is valid
		Mat code = getCode(warped);
		if(isMarker(code)) 
			imshow(to_string(i),warped);	
		
		int quad = getQuadrant(code);
		float offsetAngle = getAngle(markPolyf[1], markPolyf[2]);
		cout << offsetAngle << endl;
		float angle = 90*quad - offsetAngle;
		if(angle >= 360) angle -= 360;
		if(angle < 0) angle += 360;
		cout << "rotation: " << angle << endl;
	}
	waitKey();
	return 0;
}
