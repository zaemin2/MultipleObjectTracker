/*matrix multiplication*/
//#include "Functions.h"
#include <stdlib.h>
#include <iostream>
#include <cstring>

using namespace std;

double *matmult(double *A,double *B,int colsA,int colsB, int rowsA,int rowsB) {
    double *res = (double*)malloc(sizeof(double)*colsB*rowsA);
    memset(res,0,sizeof(double)*colsB*rowsA);
    for(int i=0; i<rowsA; i++) { //vert
        for(int k=0; k<colsB; k++) { //hor
            for(int j=0; j<colsA; j++) { //hor  = vert van B
                res[k*rowsA+i] += (double)A[j*rowsA+i]*B[k*rowsB+j];
            }
        }
    }

    return res;
}
