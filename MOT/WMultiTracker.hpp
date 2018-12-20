
//
//  WMultiTracker.hpp
//
//  Created by Jaemin LEE on 2018. 12. 4.
//  Copyright © 2018년 Jaemin LEE. All rights reserved.
//

#ifndef WMultiTracker_hpp
#define WMultiTracker_hpp

#include "Kalman.h"
#include "HungarianAlg.h"
#include "../config.h"

#include <fstream>
#include <stack>

#define COLOR(ID) cv::Scalar(255-(5*ID)%256,255-(57*ID)%256,255-(1035*ID)%256)

// state of tracking 
enum STATE { INIT = 0, RUNNING = 1, OCCLUD = 2, STOP = 3 };

class t_object
{
public:
	t_object(int id, cv::Rect2d r, float dt, float acceleration)
	{
		cv::Point2f p(r.x + r.width / 2, r.y + r.height / 2);
		this->id = id;
		this->n_miss_frame = 0;
		this->rect = r;
		this->trajectory.push_back(p);
		this->state = RUNNING;
		this->begin_time = clock();
		this->KF = new TKalmanFilter(p, dt, acceleration);
		this->prediction = p;
	};
	~t_object()
	{
		// Free resources.
		delete this->KF;
	};

	int id;									// ID 
	int n_miss_frame = 0;					// number of missing frame
	int state = INIT;						// iniation state
	cv::Rect2d rect;						// current rect;
	std::vector<cv::Point2d> trajectory;	// trace of tracking object
	time_t begin_time;						// time of start point
	int suspicious;							// 
	TKalmanFilter* KF;
	cv::Point2f prediction;
};


class WMultiTracker {
    
public:
    
	// initialize by using frame and object image
	WMultiTracker() {};
	WMultiTracker(float acceleration, float dt)
	{
		this->acceleration = acceleration;
		this->dt = dt;
	};
	~WMultiTracker() {};

	int next_id = 0;
	int frameNumber = 1;
	std::vector<t_object*> trackers;

	void run(const cv::Mat& _frame, const std::vector<cv::Rect2d>& detections);
	void draw(cv::Mat& _src);

    
private:
	float acceleration = 0.5;
	float dt = 0.2;

	void add(const cv::Mat& _frame, const cv::Rect2d& _rect);
	bool isEmpty();
	void solveAssignment(const cv::Mat& _frame, std::vector<cv::Rect2d> detections);
};

#endif /* WMultiTracker_hpp */
