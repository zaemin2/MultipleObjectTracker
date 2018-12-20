/*Project*/
#include "DPMFunctions.h"
#include <iostream>
using namespace std;


double *project(double *features, double* coeff, int *featsizes) {
    double *p ;//= (double*)malloc(sizeof(double)*featsizes[0]*featsizes[1]*featsizes[2]);

    int nsize[2] = {featsizes[0]*featsizes[1], featsizes[2]};

    p = matmult(features,coeff,nsize[1],6,nsize[0],32);

    featsizes[2] = 6;

    return p;
}
