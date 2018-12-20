//============================================================================
// Author      : F. De Smedt @ EAVISE
// Copyright   : This code is written for the publication of  "Open Framework for Combined Pedestrian detection". This code is for research and educational purposes only. For a different license, please contact the contributors directly. More information about the license can be fount in license.txt
//============================================================================

#ifndef CHNFTRSDETECTOR_H
#define CHNFTRSDETECTOR_H


#include "../Core/detector.h"
#include "../ICF/chnftrsmodel.h"
#include "../Channel/channelold.h"
#include "../ICF/chnftrsfeaturelayer.h"
#include "../Core/Image.hpp"

/*! This class, based on the Detector class, represents a ChnFtrs-detector. It is created by giving a vector of models, which should have the same features (same channels, ...) but can have different sizes. By using the Apply-function, each model of this detector will be evaluated on the same feature-pyramid (as been done by VeryFast-detector).

*/
class ChnFtrsDetector :public Detector
{
public:
	std::string getName() const {
		return "ICF";
	}
//
    ChnFtrsDetector();
    ~ChnFtrsDetector();

    // interface function to run the detector on the frame
    DetectionList applyDetector(const cv::Mat &Frame) const;

private:
    // private constructor that actually initialises the detector
    ChnFtrsDetector(std::vector<std::string> ModelFiles);

    // private function that performs the actual detections
    std::vector<Detection*> applyDetector(const cv::Mat &image, float threshold, float ScaleFactorX, float ScalefactorY, bool pruning) const;


    ChnFtrsFeatureLayer* CalculateFeatureImages(const cv::Mat& image) const;

    void findDetection(ChnFtrsFeatureLayer* FeatLay ,std::vector<Detection*> &Detections, float ScaleFactorX, float ScaleFactorY, bool pruning, float threshold, int padding) const;

    // to store the models
    std::vector<ChnFtrsModel*> m_models;
    int m_density;
    int m_shrinking;
};

#endif // CHNFTRSDETECTOR_H
