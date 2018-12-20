//============================================================================
// Author      : F. De Smedt @ EAVISE
// Copyright   : This code is written for the publication of  "Open Framework for Combined Pedestrian detection". This code is for research and educational purposes only. For a different license, please contact the contributors directly. More information about the license can be fount in license.txt
//============================================================================



/*padarray.cpp*/
#include <stdlib.h>
#include <iostream>
#include <cstring>
using namespace std;

float *padarray(float *A, int padding[2], int sizes[3]) {

    int Wpadding = padding[1];
    int Hpadding = padding[0];
    int width = sizes[1];
    int height = sizes[0];
    int planes = sizes[2];

    int planesize = width*height;
    int NWidth = width+2*Wpadding;
    int NHeight = height+2*Hpadding;
    int Nplanesize = NWidth*NHeight;

    float *newA = (float*)malloc(NWidth*NHeight*planes*sizeof(float));
    memset(newA,0,NWidth*NHeight*planes*sizeof(float));

    float *t1;
    float *t2;

    for(int Q=0; Q<planes; Q++) {
        for(int i=0; i<width; i++) {
            t2 = newA + Q*Nplanesize + (Wpadding+i)*NHeight + Hpadding;
            t1 = A + Q*planesize + i*height;
            memcpy(t2,t1,height*sizeof(float));
        }
    }

    sizes[1] = NWidth;
    sizes[0] = NHeight;

    free(A);
    return newA;
}
