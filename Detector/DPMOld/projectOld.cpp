//============================================================================
// Author      : F. De Smedt @ EAVISE
// Copyright   : This code is written for the publication of  "Open Framework for Combined Pedestrian detection". This code is for research and educational purposes only. For a different license, please contact the contributors directly. More information about the license can be fount in license.txt
//============================================================================



/*Project*/
#include <iostream>
#include "matmultOld.h"
using namespace std;


float *projectOld(float *features, float* coeff, int *featsizes) {
    float *p ;

    int nsize[2] = {featsizes[0]*featsizes[1], featsizes[2]};

    p = matmultOld(features,coeff,nsize[1],6,nsize[0],32);

    featsizes[2] = 6;

    return p;
}
