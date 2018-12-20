/*padarray.cpp*/
#include <stdlib.h>
#include <iostream>
#include <cstring>
using namespace std;

double *padarray(double *A, int padding[2], int sizes[3]) {

    int Wpadding = padding[1];
    int Hpadding = padding[0];
    int width = sizes[1];
    int height = sizes[0];
    int planes = sizes[2];

    int planesize = width*height;
    int NWidth = sizes[1]+2*Wpadding;
    int NHeight = sizes[0]+2*Hpadding;
    int Nplanesize = NWidth*NHeight;

    double *newA = (double*)malloc(NWidth*NHeight*planes*sizeof(double));
    memset(newA,0,NWidth*NHeight*planes*sizeof(double));

    double *t1;
    double *t2;

    for(int Q=0; Q<planes; Q++) {

        for(int i=0; i<width; i++) { //height
            t2 = newA + Q*Nplanesize + (Wpadding+i)*NHeight + Hpadding;
            t1 = A + Q*planesize + i*height;
            memcpy(t2,t1,height*sizeof(double));
        }
    }

    sizes[1] = NWidth;
    sizes[0] = NHeight;



    free(A);
    return newA;

}
