//
//  WMultiTracker.cpp
//
//  Created by Jaemin LEE on 2018. 12. 4.
//  Copyright © 2018년 Jaemin LEE. All rights reserved.
//

#include "WMultiTracker.hpp"

// -----------------------------------
// Add new object to trackers
// -----------------------------------
void WMultiTracker::add(const cv::Mat& _frame, const cv::Rect2d& _rect)
{
	t_object *t = new t_object(this->next_id++, _rect, this->dt, this->acceleration);
	this->trackers.push_back(t);
}

bool WMultiTracker::isEmpty()
{
	return this->trackers.empty();
}

void WMultiTracker::run(const cv::Mat& _frame, const std::vector<cv::Rect2d>& detections)
{
	if (!this->isEmpty()) {
		// update tracker
		this->solveAssignment(_frame, detections);
	}
	else {
		for (auto& d_obj : detections) {
			this->add(_frame, d_obj);
		}
	}
}

void WMultiTracker::solveAssignment(const cv::Mat& _frame, std::vector<cv::Rect2d> detections)
{
	int N = this->trackers.size();
	int M = detections.size();

	vector< vector<double> > Cost(N, vector<double>(M));
	vector<int> assignment;

	// -----------------------------------
	// Calculate cost between tracked and detected obejct
	// -----------------------------------
	double dist;
	for (int i = 0; i<this->trackers.size(); i++)
	{
		for (int j = 0; j<detections.size(); j++)
		{

			cv::Rect2d d = detections[j];
			cv::Point2d diff = (this->trackers[i]->prediction - cv::Point2f(d.x+d.width/2, d.y+d.height/2));
			dist = sqrtf(diff.x*diff.x + diff.y*diff.y);    // can be changed as the way you define.
			Cost[i][j] = dist;			// can change
		}
	}

	// -----------------------------------
	// Solving assignment problem (tracks and predictions of Kalman filter)
	// -----------------------------------
	AssignmentProblemSolver APS;
	APS.Solve(Cost, assignment, AssignmentProblemSolver::optimal);

	// -----------------------------------
	// clean assignment from pairs with large distance
	// -----------------------------------
	// Not assigned tracks
	vector<int> not_assigned_tracks;

	for (int i = 0; i<assignment.size(); i++)
	{
		if (assignment[i] != -1)
		{
			if (Cost[i][assignment[i]]> MAX_DISTANCE)
			{
				assignment[i] = -1;
				// Mark unassigned tracks, and increment skipped frames counter,
				// when skipped frames counter will be larger than threshold, track will be deleted.
				not_assigned_tracks.push_back(i);

			}
		}
		else
		{
			// If track have no assigned detect, then increment skipped frames counter.
			this->trackers[i]->n_miss_frame++;
		}
	}

	// -----------------------------------
	// If track didn't get detects long time, remove it.
	// -----------------------------------
	for (int i = 0; i<this->trackers.size(); i++)
	{
		if (this->trackers[i]->n_miss_frame > MAX_MISS_FRAME)
		{
			delete this->trackers[i];
			this->trackers.erase(this->trackers.begin() + i);
			assignment.erase(assignment.begin() + i);
			i--;
		}
	}


	// -----------------------------------
	// Search for unassigned detects
	// -----------------------------------
	vector<int> not_assigned_detections;
	vector<int>::iterator it;
	for (int i = 0; i<detections.size(); i++)
	{
		it = find(assignment.begin(), assignment.end(), i);
		if (it == assignment.end())
		{
			not_assigned_detections.push_back(i);
		}
	}
	
	// -----------------------------------
	// and start new tracks for them.
	// -----------------------------------
	if (not_assigned_detections.size() != 0)
	{
		for (int i = 0; i<not_assigned_detections.size(); i++)
		{
			this->add(_frame, detections[not_assigned_detections[i]]);
		}
	}

	// -----------------------------------
	// Update Kalman Filters state
	// -----------------------------------
	for (int i = 0; i < assignment.size(); i++)
	{
		// If track updated less than one time, than filter state is not correct.

		this->trackers[i]->KF->GetPrediction();

		if (assignment[i] != -1) // If we have assigned detect, then update using its coordinates,
		{
			this->trackers[i]->n_miss_frame = 0;
			cv::Rect2d d = detections[assignment[i]];
			cv::Point2f p(d.x + d.width / 2, d.y + d.height / 2);
			this->trackers[i]->rect = d;
			this->trackers[i]->prediction = this->trackers[i]->KF->Update(p, 1);
			this->trackers[i]->trajectory.push_back(p);
		}
		else			  // if not continue using predictions
		{
			this->trackers[i]->prediction = this->trackers[i]->KF->Update(cv::Point2f(0, 0), 0);
			this->trackers[i]->rect.x = this->trackers[i]->prediction.x - this->trackers[i]->rect.width/2;
			this->trackers[i]->rect.y = this->trackers[i]->prediction.y - this->trackers[i]->rect.height/2;
			this->trackers[i]->trajectory.push_back(this->trackers[i]->prediction);
		}

		this->trackers[i]->KF->LastResult = this->trackers[i]->prediction;
	}
}

void WMultiTracker::draw(cv::Mat& _src) {
	for (int i = 0; i<this->trackers.size(); i++)
	{
		cv::rectangle(_src, this->trackers[i]->rect, COLOR(this->trackers[i]->id), 2, 1);
		putText(_src, cv::format("%03d", this->trackers[i]->id), cv::Point(this->trackers[i]->prediction.x, 
			this->trackers[i]->prediction.y), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 1, CV_AA);
	}
}




