/*features.cpp*/

//#include <cmath>
#include <math.h>
#include <iostream>
#include <cstring>
#include <vector>
#include <stdlib.h>
#include "../Channel/Functions.h"

using namespace std;
//#include "mex.h"

// small value, used to avoid division by zero
#define eps 0.0001

// unit vectors used to compute gradient orientation
extern double uu[9];/* = {1.0000,
		0.9397,
		0.7660,
		0.500,
		0.1736,
		-0.1736,
		-0.5000,
		-0.7660,
		-0.9397};*/

extern double vv[9];/* = {0.0000,
		0.3420,
		0.6428,
		0.8660,
		0.9848,
		0.9848,
		0.8660,
		0.6428,
		0.3420};
*/
static inline double min(double x, double y) {
    return (x <= y ? x : y);
}
static inline double max(double x, double y) {
    return (x <= y ? y : x);
}

static inline int min(int x, int y) {
    return (x <= y ? x : y);
}
static inline int max(int x, int y) {
    return (x <= y ? y : x);
}


// main function:
// takes a double color image and a bin size
// returns HOG features
double* features(int sbin/*mxsbin*/, int width, int height, int *sizesfeat, const cv::Mat &Scaled) {

    int dims[3] = {height, width, 3};


    cv::Mat im_float;
    Scaled.convertTo(im_float, CV_32FC3);

    // memory for caching orientation histograms & their norms
    int blocks[2];
    blocks[0] = (int)round((double)dims[0]/(double)sbin);
    blocks[1] = (int)round((double)dims[1]/(double)sbin);


    double *hist = (double *)malloc(blocks[0]*blocks[1]*18*sizeof(double));
    memset(hist,0,blocks[0]*blocks[1]*18*sizeof(double));
    double *norm = (double *)malloc(blocks[0]*blocks[1] * sizeof(double));
    memset(norm,0,blocks[0]*blocks[1]*sizeof(double));
    // memory for HOG features
    int out[3];
    out[0] = max(blocks[0]-2, 0);

    out[1] = max(blocks[1]-2, 0);
    out[2] = 27+4+1;

    double *feat = (double*)malloc(out[0]*out[1]*out[2]*sizeof(double));
    sizesfeat[0] = out[0];
    sizesfeat[1] = out[1];
    sizesfeat[2] = out[2];

    int visible[2];
    visible[0] = blocks[0]*sbin;
    visible[1] = blocks[1]*sbin;
    for (int x = 1; x < visible[1]-1; x++) {
        for (int y = 1; y < visible[0]-1; y++) {


            if(x >= Scaled.cols-1 || y >= Scaled.rows-1 || x < 1 || y < 1 ) {
                continue;
            }

            cv::Vec3f Yp1 = im_float.at<cv::Vec3f>(y+1, x);
            cv::Vec3f Ym1 = im_float.at<cv::Vec3f>(y-1, x);

            cv::Vec3f Xp1 = im_float.at<cv::Vec3f>(y, x+1);
            cv::Vec3f Xm1 = im_float.at<cv::Vec3f>(y, x-1);

            // Blue
            double dy = Yp1.val[0] - Ym1.val[0];
            double dx = Xp1.val[0] - Xm1.val[0];
            double v = dx*dx + dy*dy;

            // Green
            double dy2 = Yp1.val[1] - Ym1.val[1];
            double dx2 = Xp1.val[1] - Xm1.val[1];
            double v2 = dx2*dx2 + dy2*dy2;

            // Red
            double dy3 = Yp1.val[2] - Ym1.val[2];
            double dx3 = Xp1.val[2] - Xm1.val[2];
            double v3 = dx3*dx3 + dy3*dy3;



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


            // snap to one of 18 orientations
            double best_dot = 0;
            int best_o = 0;
            for (int o = 0; o < 9; o++) {
                double dot = uu[o]*dx + vv[o]*dy;
                if (dot > best_dot) {
                    best_dot = dot;
                    best_o = o;
                }
                else if (-dot > best_dot) {
                    best_dot = -dot;
                    best_o = o+9;
                }
            }

            // add to 4 histograms around pixel using linear interpolation
            double xp = ((double)x+0.5)/(double)sbin - 0.5;
            double yp = ((double)y+0.5)/(double)sbin - 0.5;
            int ixp = (int)floor(xp);
            int iyp = (int)floor(yp);
            double vx0 = xp-ixp;
            double vy0 = yp-iyp;
            double vx1 = 1.0-vx0;
            double vy1 = 1.0-vy0;
            v = sqrt(v);

            if (ixp >= 0 && iyp >= 0) {
                *(hist + ixp*blocks[0] + iyp + best_o*blocks[0]*blocks[1]) += vx1*vy1*v;
            }

            if (ixp+1 < blocks[1] && iyp >= 0) {
                *(hist + (ixp+1)*blocks[0] + iyp + best_o*blocks[0]*blocks[1]) += vx0*vy1*v;
            }

            if (ixp >= 0 && iyp+1 < blocks[0]) {
                *(hist + ixp*blocks[0] + (iyp+1) + best_o*blocks[0]*blocks[1]) +=  vx1*vy0*v;
            }

            if (ixp+1 < blocks[1] && iyp+1 < blocks[0]) {
                *(hist + (ixp+1)*blocks[0] + (iyp+1) + best_o*blocks[0]*blocks[1]) += vx0*vy0*v;
            }
        }
    }

    // compute energy in each block by summing over orientations
    for (int o = 0; o < 9; o++) {
        double *src1 = hist + o*blocks[0]*blocks[1];
        double *src2 = hist + (o+9)*blocks[0]*blocks[1];
        double *dst = norm;
        double *end = norm + blocks[1]*blocks[0];
        while (dst < end) {
            *(dst++) += (*src1 + *src2) * (*src1 + *src2);
            src1++;
            src2++;
        }
    }

    // compute features
    for (int x = 0; x < out[1]; x++) {
        for (int y = 0; y < out[0]; y++) {
            double *dst = feat + x*out[0] + y;
            double *src, *p, n1, n2, n3, n4;

            p = norm + (x+1)*blocks[0] + y+1;
            n1 = 1.0 / sqrt(*p + *(p+1) + *(p+blocks[0]) + *(p+blocks[0]+1) + eps);
            p = norm + (x+1)*blocks[0] + y;
            n2 = 1.0 / sqrt(*p + *(p+1) + *(p+blocks[0]) + *(p+blocks[0]+1) + eps);
            p = norm + x*blocks[0] + y+1;
            n3 = 1.0 / sqrt(*p + *(p+1) + *(p+blocks[0]) + *(p+blocks[0]+1) + eps);
            p = norm + x*blocks[0] + y;
            n4 = 1.0 / sqrt(*p + *(p+1) + *(p+blocks[0]) + *(p+blocks[0]+1) + eps);

            double t1 = 0;
            double t2 = 0;
            double t3 = 0;
            double t4 = 0;

            // contrast-sensitive features
            src = hist + (x+1)*blocks[0] + (y+1);
            for (int o = 0; o < 18; o++) {
                double h1 = min(*src * n1, 0.2);
                double h2 = min(*src * n2, 0.2);
                double h3 = min(*src * n3, 0.2);
                double h4 = min(*src * n4, 0.2);
                *dst = 0.5 * (h1 + h2 + h3 + h4);
                t1 += h1;
                t2 += h2;
                t3 += h3;
                t4 += h4;
                dst += out[0]*out[1];
                src += blocks[0]*blocks[1];
            }

            // contrast-insensitive features
            src = hist + (x+1)*blocks[0] + (y+1);
            for (int o = 0; o < 9; o++) {
                double sum = *src + *(src + 9*blocks[0]*blocks[1]);
                double h1 = min(sum * n1, 0.2);
                double h2 = min(sum * n2, 0.2);
                double h3 = min(sum * n3, 0.2);
                double h4 = min(sum * n4, 0.2);


                *dst = 0.5 * (h1 + h2 + h3 + h4);
                dst += out[0]*out[1];
                src += blocks[0]*blocks[1];
            }

            // texture features
            *dst = 0.2357 * t1;
            dst += out[0]*out[1];
            *dst = 0.2357 * t2;
            dst += out[0]*out[1];
            *dst = 0.2357 * t3;
            dst += out[0]*out[1];
            *dst = 0.2357 * t4;

            // truncation feature
            dst += out[0]*out[1];
            *dst = 0;
        }

    }

    free(hist);
    free(norm);

    return feat;
}


