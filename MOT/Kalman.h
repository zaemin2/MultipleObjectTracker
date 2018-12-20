#pragma once
#include "opencv2/opencv.hpp"
#include <opencv/cv.h>

using namespace std;

// http://www.morethantechnical.com/2011/06/17/simple-kalman-filter-for-tracking-using-opencv-2-2-w-code/
class TKalmanFilter
{
public:
	cv::KalmanFilter* kalman;
	double deltatime; //Time Increment
	cv::Point2f LastResult;

	TKalmanFilter(cv::Point2f p,float dt=0.2,float Accel_noise_mag=0.5);
	~TKalmanFilter();
	cv::Point2f GetPrediction();
	cv::Point2f Update(cv::Point2f p, bool DataCorrect);
};

