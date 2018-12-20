//============================================================================
// Author      : F. De Smedt @ EAVISE
// Copyright   : This code is written for the publication of  "Open Framework for Combined Pedestrian detection". This code is for research and educational purposes only. For a different license, please contact the contributors directly. More information about the license can be fount in license.txt
//============================================================================



#ifndef DPMDETECTOROLD_H
#define DPMDETECTOROLD_H

#include "../Core/detector.h"

#include "NewCascadeModelOld.h"
#include "CascadeModelOld.h"
#include "cascadeOld.h"

#include "../Core/DetectionList.h"

class DPMDetectorOld : public Detector
{
public:

    DPMDetectorOld();
    ~DPMDetectorOld();

    DetectionList applyDetector(const cv::Mat &Frame) const;

private:

    CascadeModelOld *Model;
    NewCascadeModelOld CModel;
    CascadeArrays_NonVolatileOld NonVolatile;

    std::vector<DetectorModel*> m_models;
    int m_density;
    int m_shrinking;
};

#endif // CHNFTRSDETECTOR_H
