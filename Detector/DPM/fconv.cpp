#include <math.h>
#include <iostream>
#include <string.h>
#include <stdlib.h>

#include "CModel.h"


/*
 * This code is used for computing filter responses.  It computes the
 * response of a set of filters with a feature map.
 *
 * Basic version, relatively slow but very compatible.
 */
using namespace std;

struct thread_data {
    double *A;
    double *B;
    double *C;
    double *mxC;
    int A_dims[3];
    int B_dims[3];
    int C_dims[2];
};

// convolve A and B
//void *process(void *thread_arg) {
void process(void *thread_arg) {
    thread_data *args = (thread_data *)thread_arg;
    double *A = args->A; //get A
    double *B = args->B; //get B
    double *C = args->C; //get result array
    int A_dims[3] = {args->A_dims[0], args->A_dims[1],args->A_dims[2]}; //get size A
    int B_dims[3] = {args->B_dims[0], args->B_dims[1],args->B_dims[2]}; //get size A
    int C_dims[2] = {args->C_dims[0], args->C_dims[1]}; //get size C
    int num_features = args->A_dims[2];

    for (int f = 0; f < num_features; f++) {
        double *dst = C; //dst points to result array
        double *A_src = A + f*A_dims[0]*A_dims[1]; //always size of A further
        double *B_src = B + f*B_dims[0]*B_dims[1]; //always size of B further
        for (int x = 0; x < C_dims[1]; x++) {   //run over C x-direction
            for (int y = 0; y < C_dims[0]; y++) {  //run over C y-direction
                double val = 0;
                for (int xp = 0; xp < B_dims[1]; xp++) {
                    double *A_off = A_src + (x+xp)*A_dims[0] + y;
                    double *B_off = B_src + xp*B_dims[0];
                    switch(B_dims[0]) {
                    case 20:
                        val += A_off[19] * B_off[19];
                    case 19:
                        val += A_off[18] * B_off[18];
                    case 18:
                        val += A_off[17] * B_off[17];
                    case 17:
                        val += A_off[16] * B_off[16];
                    case 16:
                        val += A_off[15] * B_off[15];
                    case 15:
                        val += A_off[14] * B_off[14];
                    case 14:
                        val += A_off[13] * B_off[13];
                    case 13:
                        val += A_off[12] * B_off[12];
                    case 12:
                        val += A_off[11] * B_off[11];
                    case 11:
                        val += A_off[10] * B_off[10];
                    case 10:
                        val += A_off[9] * B_off[9];
                    case 9:
                        val += A_off[8] * B_off[8];
                    case 8:
                        val += A_off[7] * B_off[7];
                    case 7:
                        val += A_off[6] * B_off[6];
                    case 6:
                        val += A_off[5] * B_off[5];
                    case 5:
                        val += A_off[4] * B_off[4];
                    case 4:
                        val += A_off[3] * B_off[3];
                    case 3:
                        val += A_off[2] * B_off[2];
                    case 2:
                        val += A_off[1] * B_off[1];
                    case 1:
                        val += A_off[0] * B_off[0];
                        break;
                    default:
                        for (int yp = 0; yp < B_dims[0]; yp++) {
                            val += *(A_off++) **(B_off++);
                        }
                    }
                }
                *(dst++) += val;
            }
        }
    }
}

// matlab entry point
// C = fconv(A, cell of B, start, end);
double **fconv(double* A, double** B, int start, int end, int WA,int HA, int DA,  Rootfilter* rootfilters, int **sizeC) {

    int len = end-start+1;

    // output cell
//  plhs[0] = mxCreateCellMatrix(1, len); //uitvoer
    double** Back = (double**)malloc(sizeof(double*)*len);
    // do convolutions
    thread_data td; //create td
    for (int i = 0; i < len; i++) { //loop over lenght
        double *mxB = B[i/*+start*/];
        td.A_dims[0] = HA;
        td.A_dims[1] = WA;
        td.A_dims[2] = DA;
        td.A = A;
        td.B_dims[0] = rootfilters[i].size[0];
        td.B_dims[1] = rootfilters[i].size[1];
        td.B_dims[2] = 6; //PCA

        td.B = mxB;

        // compute size of output
        int height = td.A_dims[0] - td.B_dims[0] + 1; //hoogte
        int width = td.A_dims[1] - td.B_dims[1] + 1; //breedte

        td.C_dims[0] = height; //hoogte
        td.C_dims[1] = width; //breedte
        td.mxC = (double *)malloc(td.C_dims[0]*td.C_dims[1]*sizeof(double));//   td.C_dims, mxDOUBLE_CLASS, mxREAL); //create result array store
        memset(td.mxC,'\0',td.C_dims[0]*td.C_dims[1]*sizeof(double));/*to be sure ...*/
        td.C = td.mxC;
        process((void *)&td); //do actual convolution
//   mxSetCell(plhs[0], i, td.mxC); //put result on output
        Back[i] = td.mxC;
        sizeC[i][0] = height;//td.C_dims[0];
        sizeC[i][1] = width;//td.C_dims[1];

    }
    return Back;
}


