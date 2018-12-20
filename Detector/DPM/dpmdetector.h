//============================================================================
// Author      : F. De Smedt @ EAVISE
// Copyright   : This code is written for the publication of  "Open Framework for Combined Pedestrian detection". This code is for research and educational purposes only. For a different license, please contact the contributors directly. More information about the license can be fount in license.txt
//============================================================================



#ifndef DPMDETECTOR_H
#define DPMDETECTOR_H

#include "../Core/detector.h"
#include "../Core/DetectionList.h"

#include "CModel.h"

class DPMDetector : public Detector
{
public:
    std::string getName() const {
        return "DPM";
    }
    DPMDetector();
    ~DPMDetector();

    DetectionList applyDetector(const cv::Mat &Frame) const;
private:
    CModel M;
    float thresh;
};

#endif // DPMDETECTOR_H
