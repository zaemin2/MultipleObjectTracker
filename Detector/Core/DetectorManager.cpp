//============================================================================
// Author      : F. De Smedt @ EAVISE
// Copyright   : This code is written for the publication of  "Open Framework for Combined Pedestrian detection". This code is for research and educational purposes only. For a different license, please contact the contributors directly. More information about the license can be fount in license.txt
//============================================================================




#include "DetectorManager.h"

DetectorManager::DetectorManager(int algorithm) {

	switch (algorithm)
	{
	case ACF:
		Det = new ACFDetector();
		break;
	case DPM:
		Det = new DPMDetector();
		break;
	case ICF:
		Det = new ChnFtrsDetector();
		break;
	case HOG:
		Det = new HOGDetector();
		break;
	default:
		Det = new ACFDetector();
		break;
	};
}

DetectorManager::~DetectorManager() {
    // cleanly delete the created detector
    delete Det;
}

DetectionList DetectorManager::applyDetector(cv::Mat &Frame) {
    // forward the detection request to the detector
    return Det->applyDetector(Frame);
}

