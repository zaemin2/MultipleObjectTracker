//============================================================================
// Author      : F. De Smedt @ EAVISE
// Copyright   : This code is written for the publication of  "Open Framework for Combined Pedestrian detection". This code is for research and educational purposes only. For a different license, please contact the contributors directly. More information about the license can be fount in license.txt
//============================================================================



/*features.cpp*/

//#include <cmath>
#include <math.h>
#include <iostream>
#include <cstring>
#include <vector>
#include <stdlib.h>

#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"


using namespace std;
using namespace cv;

#define eps 0.0001

// unit vectors used to compute gradient orientation
float uu_[9] = {1.0000,
               0.9397,
               0.7660,
               0.500,
               0.1736,
               -0.1736,
               -0.5000,
               -0.7660,
               -0.9397
              };
float vv_[9] = {0.0000,
               0.3420,
               0.6428,
               0.8660,
               0.9848,
               0.9848,
               0.8660,
               0.6428,
               0.3420
              };


float* featurescpuOld(cv::Mat &im, int sbin, int width, int height, int *sizesfeat) {

    int dims[3] = {im.rows, im.cols, 3};

    int blocks[2];
    blocks[0] = (int)floor((float)dims[0]/(float)sbin);
    blocks[1] = (int)floor((float)dims[1]/(float)sbin);


    float *hist = (float *)malloc(blocks[0]*blocks[1]*18*sizeof(float));

    float *hist1 = (float *)malloc(blocks[0]*blocks[1]*18*sizeof(float));
    float *hist2 = (float *)malloc(blocks[0]*blocks[1]*18*sizeof(float));
    float *hist3 = (float *)malloc(blocks[0]*blocks[1]*18*sizeof(float));
    float *hist4 = (float *)malloc(blocks[0]*blocks[1]*18*sizeof(float));

    memset(hist1,0,blocks[0]*blocks[1]*18*sizeof(float));
    memset(hist2,0,blocks[0]*blocks[1]*18*sizeof(float));
    memset(hist3,0,blocks[0]*blocks[1]*18*sizeof(float));
    memset(hist4,0,blocks[0]*blocks[1]*18*sizeof(float));

    float *norm = (float *)malloc(blocks[0]*blocks[1] * sizeof(float));


    float *T1 = (float*)malloc(blocks[0]*blocks[1] * sizeof(float));
    float *T2 = (float*)malloc(blocks[0]*blocks[1] * sizeof(float));
    float *T3 = (float*)malloc(blocks[0]*blocks[1] * sizeof(float));
    float *T4 = (float*)malloc(blocks[0]*blocks[1] * sizeof(float));

    // memory for HOG features
    int out[3];
    out[0] = max(blocks[0]-2, 0);

    out[1] = max(blocks[1]-2, 0);
    out[2] = 27+4+1;
    float *feat = (float*)malloc(out[0]*out[1]*out[2]*sizeof(float));
    memset(feat,0,out[0]*out[1]*out[2]*sizeof(float));

    sizesfeat[0] = out[0];
    sizesfeat[1] = out[1];
    sizesfeat[2] = out[2];

    int visible[2];
    visible[0] = blocks[0]*sbin;
    visible[1] = blocks[1]*sbin;


    for(int b0=0; b0<blocks[0]; b0++) {
        for(int b1=0; b1<blocks[1]; b1++) {
            float H1[18]= {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
            float H2[18]= {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
            float H3[18]= {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
            float H4[18]= {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

            int ixp,iyp;
            bool change;

            for (int x=b1*sbin-sbin/2; x<b1*sbin+sbin/2; x++) {
                for (int y=b0*sbin-sbin/2; y<b0*sbin+sbin/2; y++) {
                    change=false;

                    if (x < 1 || y < 1)
                        continue;
                    if (x >= visible[1]-2 || y >= visible[0]-2)
                        continue;
                    change=true;

                    unsigned char *sm1 = im.ptr(y-1) + x*3;
                    unsigned char *s = im.ptr(y) + x*3;
                    unsigned char *sp1 = im.ptr(y+1) + x*3;

                    float dy = *(sp1) - *(sm1);
                    float dx = *(s+3) - *(s-3);
                    float v = dx*dx + dy*dy;

                    s++;
                    sm1++;
                    sp1++;

                    float dy2 = *(sp1) - *(sm1);
                    float dx2 = *(s+3) - *(s-3);
                    float v2 = dx2*dx2 + dy2*dy2;

                    s++;
                    sm1++;
                    sp1++;

                    float dy3 = *(sp1) - *(sm1);
                    float dx3 = *(s+3) - *(s-3);
                    float v3 = dx3*dx3+ dy3*dy3;

                    // pick channel with strongest gradient
                    if (v2 > v) {
                        v = v2;
                        dx = dx2;
                        dy = dy2;
                    }
                    if (v3 > v) {
                        v = v3;
                        dx = dx3;
                        dy = dy3;
                    }

                    /*Snap to strongest gradient*/
                    int best_o = 0;
                    float BD = 0;
                    float B = 0;
                    for(int Q=0; Q<9; Q++) {
                        float D = uu_[Q]*dx + vv_[Q]*dy;
                        if(D > BD) {
                            BD = D;
                            B = Q;
                        }
                        else if(-D > BD) {
                            BD = -D;
                            B = Q+9;
                        }
                    }
                    best_o = B;
                    float xp = ((float)x+0.5)/(float)sbin - 0.5;
                    float yp = ((float)y+0.5)/(float)sbin - 0.5;
                    ixp = (int)floor(xp);
                    iyp = (int)floor(yp);
                    float vx0 = xp-ixp;
                    float vy0 = yp-iyp;
                    float vx1 = 1.0-vx0;
                    float vy1 = 1.0-vy0;
                    v = sqrt(v);


                    /*write to histogram*/
                    if (ixp >= 0 && iyp >= 0) {
                        H1[best_o] += vx1*vy1*v;
                    }

                    if (ixp+1 < blocks[1] && iyp >= 0) {
                        H2[best_o] += vx0*vy1*v;
                    }

                    if (ixp >= 0 && iyp+1 < blocks[0]) {
                        H3[best_o] +=  vx1*vy0*v;
                    }

                    if (ixp+1 < blocks[1] && iyp+1 < blocks[0]) {
                        H4[best_o] += vx0*vy0*v;
                    }
                }
            }

            if(change && ixp >=0 && iyp >=0 && ixp < blocks[1]-1 && iyp < blocks[0]-1 ) {
                for(int Q=0; Q<18; Q++) {
                    *(hist1 + ixp*18*blocks[0] + iyp*18 + Q) = H1[Q];
                    *(hist2 + (ixp+1)*18*blocks[0] + iyp*18 + Q) = H2[Q];
                    *(hist3 + ixp*18*blocks[0]+ (iyp+1)*18 + Q) = H3[Q];
                    *(hist4 + (ixp+1)*18*blocks[0] + (iyp+1)*18 + Q) = H4[Q];
                }
            }
        }
    }


    for(int i=0; i<blocks[0]*blocks[1]*18; i++) {
        hist[i] = hist1[i]+hist2[i]+hist3[i]+hist4[i];
    }


    for(int bxt=0; bxt<blocks[1]; bxt++) {
        for(int byt=0; byt<blocks[0]; byt++) {
            int Pos = bxt*blocks[0]+byt;
            float *Hist = hist+bxt*blocks[0]*18+byt*18;
            float Som=0;
            for(int i=0; i<9; i++) {
                float s1 = Hist[i];

                float s2 = Hist[i+9];
                Som += (s1+s2)*(s1+s2);
            }
            norm[Pos] = Som;
        }
    }

    /*Summation*/
    float R =0;
    float *Sum = (float*)malloc(blocks[0]*blocks[1] * sizeof(float));
    memset(Sum,'\0',blocks[0]*blocks[1] * sizeof(float));


    for(int x=0; x<blocks[1]-1; x++) {
        for(int y=0; y<blocks[0]-1; y++) {

            R = norm[x*blocks[0]+y] + norm[x*blocks[0]+y+1] + norm[(x+1)*blocks[0]+y] +  norm[(x+1)*blocks[0]+y+1];

            Sum[x*blocks[0]+y] = 1.0/sqrt(R);
        }
    }

    for(int x=0; x<out[1]; x++) {
        for(int y=0; y<out[0]; y++) {
            float* src = hist + (x+1)*18*blocks[0] + (y+1)*18;
            float* dst = feat + x*out[0] + y;

            float n1 = Sum[(x+1)*blocks[0]+y+1];
            float n2 = Sum[(x+1)*blocks[0]+y];
            float n3 = Sum[x*blocks[0]+y+1];
            float n4 = Sum[x*blocks[0]+y];

            float t1=0;
            float t2=0;
            float t3=0;
            float t4=0;
            for(int o=0; o < 18; o++) {
                float h1 = std::min(*src * n1, (float)0.2);
                float h2 = std::min(*src * n2, (float)0.2);
                float h3 = std::min(*src * n3, (float)0.2);
                float h4 = std::min(*src * n4, (float)0.2);
                *dst = 0.5 * (h1+h2+h3+h4);
                t1 += h1;
                t2 += h2;
                t3 += h3;
                t4 += h4;
                dst += out[0]*out[1];
                src++;
            }
            T1[x*blocks[0]+y] = t1;
            T2[x*blocks[0]+y] = t2;
            T3[x*blocks[0]+y] = t3;
            T4[x*blocks[0]+y] = t4;

            src = hist + (x+1)*18*blocks[0] + (y+1)*18;

            float sum,h1,h2,h3,h4;

            for(int o=0; o<9; o++) {
                sum = *src + *(src + 9);
                h1 = std::min(sum * n1, (float)0.2);
                h2 = std::min(sum * n2, (float)0.2);
                h3 = std::min(sum * n3, (float)0.2);
                h4 = std::min(sum * n4, (float)0.2);

                *dst = 0.5*(h1+h2+h3+h4);
                dst += out[0]*out[1];
                src++;
            }
        }
    }



    float* dst = feat+27*out[0]*out[1];

    for(int x=0; x<out[1]; x++) {
        for(int y=0; y<out[0]; y++) {
            dst[x*out[0]+y] = 0.2357 * T1[x*blocks[0]+y];
            dst[out[0]*out[1]+x*out[0]+y] = 0.2357 * T2[x*blocks[0]+y];
            dst[out[0]*out[1]*2+x*out[0]+y] = 0.2357 * T3[x*blocks[0]+y];
            dst[out[0]*out[1]*3+x*out[0]+y] = 0.2357 * T4[x*blocks[0]+y];
        }
    }

    free(hist);
    free(hist1);
    free(hist2);
    free(hist3);
    free(hist4);
    free(Sum);

    free(T1);
    free(T2);
    free(T3);
    free(T4);

    free(norm);

    return feat;
}

