#ifndef _H_HAARDETECTOR
#define _H_HAARDETECTOR

#include <vector>
#include <string>
#include <iostream>

#include "../Core/detection.h"
#include "../Core/DetectionList.h"
#include "../Core/detector.h"

class HAARDetector: public Detector {
public:
    HAARDetector();
    std::string getName() const {
        return "HAAR";
    }

    DetectionList applyDetector(const cv::Mat &Frame) const;
private:
	cv::CascadeClassifier *haar;


};


#endif
