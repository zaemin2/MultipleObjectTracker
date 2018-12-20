//============================================================================
// Author      : F. De Smedt @ EAVISE
// Copyright   : This code is written for the publication of  "Open Framework for Combined Pedestrian detection". This code is for research and educational purposes only. For a different license, please contact the contributors directly. More information about the license can be fount in license.txt
//============================================================================



/*matrix multiplication*/
#include <stdlib.h>
#include <iostream>
#include <cstring>

using namespace std;

float *matmultOld(float *A,float *B,int colsA,int colsB, int rowsA,int rowsB) {
    float *res = (float*)malloc(sizeof(float)*colsB*rowsA);
    memset(res,0,sizeof(float)*colsB*rowsA);
    for(int i=0; i<rowsA; i++) { //vert
        for(int k=0; k<colsB; k++) { //hor
            for(int j=0; j<colsA; j++) { //hor  = vert van B
                res[k*rowsA+i] += (float)A[j*rowsA+i]*B[k*rowsB+j];
            }
        }
    }


    return res;
}
