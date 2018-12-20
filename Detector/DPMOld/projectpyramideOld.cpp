//============================================================================
// Author      : F. De Smedt @ EAVISE
// Copyright   : This code is written for the publication of  "Open Framework for Combined Pedestrian detection". This code is for research and educational purposes only. For a different license, please contact the contributors directly. More information about the license can be fount in license.txt
//============================================================================



/*ProjectPyramide*/
#include <iostream>
#include "projectOld.h"
#include "PyramidOld.h"
#include "CascadeModelOld.h"
#include "NewCascadeModelOld.h"
using namespace std;

void projectpyramideOld(PyramidOld *pyra,NewCascadeModelOld *NModel) {
    for(int i=0; i<pyra->lenghtFeatures; i++) {
        pyra->featold[i] = pyra->feat[i];
        pyra->featsizesold[i][0] = pyra->featsizes[i][0];
        pyra->featsizesold[i][1] = pyra->featsizes[i][1];
        pyra->featsizesold[i][2] = pyra->featsizes[i][2];
        pyra->feat[i] = projectOld(pyra->feat[i],NModel->coeff,pyra->featsizes[i]);
    }
}

