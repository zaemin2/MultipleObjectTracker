//============================================================================
// Author      : F. De Smedt @ EAVISE
// Copyright   : This code is written for the publication of  "Open Framework for Combined Pedestrian detection". This code is for research and educational purposes only. For a different license, please contact the contributors directly. More information about the license can be fount in license.txt
//============================================================================


#ifndef FCONV_H
#define FCONV_H
#include "NewCascadeModelOld.h"

float **fconvOld(float* A, float** B, int start, int end, int WA,int HA, int DA, RootfilterOld* rootfilters, int** sizeC);

#endif // FCONV_H
