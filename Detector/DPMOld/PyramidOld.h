//============================================================================
// Author      : F. De Smedt @ EAVISE
// Copyright   : This code is written for the publication of  "Open Framework for Combined Pedestrian detection". This code is for research and educational purposes only. For a different license, please contact the contributors directly. More information about the license can be fount in license.txt
//============================================================================



#ifndef __PYRAMID
#define __PYRAMID
#include <vector>
#include <cmath>



struct PyramidOld {
    int padx;
    int pady;
    int imsize[2];
    int lenghtFeatures;
    float **feat; //
    float **featold; //
    int (**featsizes);//[height width offset]
    int (**featsizesold);//[height width offset]
    float *scales; //
/*
    float GetElement(int index,int x,int y,int z) {
        return 3;// feat[index][y*featsizes[index][1]*32 + z * featsizes[index][1] + x ];
    }
*/
    void SetElement(int index,int x,int y,int z,float W) {
        feat[index][z*featsizes[index][0]*featsizes[index][1]+y*featsizes[index][1]+x  ] = W;
    }

};


#endif
