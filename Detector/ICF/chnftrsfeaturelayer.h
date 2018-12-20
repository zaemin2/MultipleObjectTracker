//============================================================================
// Author      : F. De Smedt @ EAVISE
// Copyright   : This code is written for the publication of  "Open Framework for Combined Pedestrian detection". This code is for research and educational purposes only. For a different license, please contact the contributors directly. More information about the license can be fount in license.txt
//============================================================================

#ifndef CHNFTRSFEATURELAYER_H
#define CHNFTRSFEATURELAYER_H


#include <iostream>
#include <vector>

#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"

#include "../Core/featurelayer.h"

/*!
	The respresentation of the features of a ChnFtrs-based detector. In this case these will be Integral-images (ICF). The number of feature-layers is dependend on the number of channels used, which is model-dependend
*/

class ChnFtrsFeatureLayer : public FeatureLayer
{
public:
    ChnFtrsFeatureLayer();
    ~ChnFtrsFeatureLayer();
private:

};

#endif // CHNFTRSFEATURELAYER_H
