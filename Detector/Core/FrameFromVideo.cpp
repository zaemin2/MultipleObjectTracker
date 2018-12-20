//============================================================================
// Author      : F. De Smedt @ EAVISE
// Copyright   : This code is written for the publication of  "Open Framework for Combined Pedestrian detection". This code is for research and educational purposes only. For a different license, please contact the contributors directly. More information about the license can be fount in license.txt
//============================================================================


#include "FrameFromVideo.h"

FrameFromVideo::FrameFromVideo(std::string video_name) :cap(cv::VideoCapture(video_name)) {

}

FrameFromVideo::FrameFromVideo(std::string video_name, float angle):cap(cv::VideoCapture(video_name)) {
	this->angle = angle;
}

FrameFromVideo::~FrameFromVideo() {
}

void rotate_image(cv::Mat& _frame, float angle)
{
	// if need to rotate 
	float scale = 1.0;
	cv::Point2f center(_frame.cols*0.5, _frame.rows*0.5);
	const cv::Mat affine_matrix = cv::getRotationMatrix2D(center, angle, scale);
	cv::warpAffine(_frame, _frame, affine_matrix, _frame.size());
}

cv::Mat FrameFromVideo::giveFrame() {
    cv::Mat frame;

    if(cap.get(CV_CAP_PROP_POS_FRAMES)  < cap.get(CV_CAP_PROP_FRAME_COUNT)) {
        cap >>frame;
    }

	if (angle != 0.0) {
		rotate_image(frame, angle);
	}

    return frame;
}

bool FrameFromVideo::isend() {
    return cap.get(CV_CAP_PROP_POS_FRAMES)  >= cap.get(CV_CAP_PROP_FRAME_COUNT);
}
