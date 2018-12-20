#include "HOGDetector.h"

using namespace cv;

HOGDetector::HOGDetector() {
    hog.setSVMDetector(HOGDescriptor::getDefaultPeopleDetector());
    cv::setNumThreads(1);
}


DetectionList HOGDetector::applyDetector(const cv::Mat &Frame) const{
DetectionList DL;

    float Upscale = 3.0;

    cv::Mat U;
    cv::resize(Frame,U,cv::Size(),Upscale,Upscale);

// HOG
    std::vector<cv::Rect> found, found_filtered;
    std::vector<double> Scores, Scores_filtered;

    hog.detectMultiScale(U, found, Scores, 0, Size(8,8), Size(32,32), 1.05, 0);

    for(int i = 0; i < found.size(); i++ )
    {
        Rect r = found[i];
        // the HOG detector returns slightly larger rectangles than the real objects.
        // so we slightly shrink the rectangles to get a nicer output.
        r.x += cvRound(r.width*0.1);
        r.width = cvRound(r.width*0.8);
        r.y += cvRound(r.height*0.07);
        r.height = cvRound(r.height*0.8);
        DL.addDetection(r.x,r.y,r.width,r.height,Scores[i]);
    }

    DL.resizeDetections(Upscale);
    DL.setDetectorNames(this->getName());

    return DL;
}
