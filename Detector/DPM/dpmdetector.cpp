//============================================================================
// Author      : F. De Smedt @ EAVISE
// Copyright   : This code is written for the publication of  "Open Framework for Combined Pedestrian detection". This code is for research and educational purposes only. For a different license, please contact the contributors directly. More information about the license can be fount in license.txt
//============================================================================



#include "dpmdetector.h"
#include "DPMFunctions.h"


DPMDetector::DPMDetector() {

    M = LoadCascadeModel(thresh, "Models/DPM/personmodel.bin");

    thresh = -5.0;

    M.thresh = thresh;
}

DetectionList DPMDetector::applyDetector(const cv::Mat &Frame) const {
    DetectionList DL;

    float Upscale = 1.0;
    cv::Mat U;
    cv::resize(Frame,U,cv::Size(),Upscale,Upscale);

    DPMPyramid DPyr(U, M.getPadx(), M.getPady(), M.getInterval());

    vector<double> dets,detsT;
    vector<double> boxes,boxesT;

    int Height = U.rows;
    int Width = U.cols;

    int W = cascade_detect(&M,thresh,dets,boxes,DPyr);


    if (dets.size() > 0) {
        clipboxes(Width,Height,dets,boxes,W);

        for (int Q=0; Q<dets.size()/6; Q++) {
            DL.addDetection(dets[Q*6+0], dets[Q*6+1], dets[Q*6+2] - dets[Q*6+0], dets[Q*6+3] - dets[Q*6+1], dets[Q*6+5]);
        }
    }
    DL.resizeDetections(Upscale);
    DL.setDetectorNames(this->getName());

    return DL;
}


DPMDetector::~DPMDetector() {

}
