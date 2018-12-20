//============================================================================
// Author      : F. De Smedt @ EAVISE
// Copyright   : This code is written for the publication of  "Open Framework for Combined Pedestrian detection". This code is for research and educational purposes only. For a different license, please contact the contributors directly. More information about the license can be fount in license.txt
//============================================================================



#ifndef FEATPYRAMID_H
#define FEATPYRAMID_H
#include "PyramidOld.h"
#include "NewCascadeModelOld.h"
// calculate the feature pyramid
PyramidOld featpyramidcpu(cv::Mat &im, NewCascadeModelOld *Model, int level, int method) ;

#endif // FEATPYRAMID_H
