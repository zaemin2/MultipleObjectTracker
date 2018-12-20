#include "HAARDetector.h"

using namespace cv;

HAARDetector::HAARDetector() {
	this->haar = new CascadeClassifier();
	if (!this->haar->load("./Models/HAAR/cascades.xml")) { printf("--(!)Error loading\n"); };
}


DetectionList HAARDetector::applyDetector(const cv::Mat &Frame) const{
DetectionList DL;

    float Upscale = 1.0;	// modify to up scale if target is small 
    cv::Mat U;
    cv::resize(Frame,U,cv::Size(),Upscale,Upscale);

// HAAR
    std::vector<cv::Rect> found, found_filtered;
    std::vector<double> levelweights;
	std::vector<int> Reject_level;

	cv::cvtColor(U, U, CV_BGR2GRAY);
	this->haar->detectMultiScale(U, found, Reject_level, levelweights, 1.1, 4, 0 | CV_HAAR_SCALE_IMAGE, Size(22, 20), cv::Size(85, 80), true);

    for(int i = 0; i < found.size(); i++ )
    {
        Rect r = found[i];
        DL.addDetection(r.x,r.y,r.width,r.height, (double)levelweights[i]);
    }

    DL.resizeDetections(Upscale);
    DL.setDetectorNames(this->getName());

    return DL;
}
