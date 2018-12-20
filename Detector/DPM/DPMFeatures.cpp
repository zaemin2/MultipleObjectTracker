/*
 * DPMFeatures.cpp
 *
 *  Created on: Mar 20, 2015
 *      Author: fds
 */

#include "DPMFeatures.h"
#include <cmath>

#include "DPMFunctions.h"


#define eps 0.0001

/*To determine the orientation*/

double uu[9] = {1.0000,
                0.9397,
                0.7660,
                0.500,
                0.1736,
                -0.1736,
                -0.5000,
                -0.7660,
                -0.9397
               };
double vv[9] = {0.0000,
                0.3420,
                0.6428,
                0.8660,
                0.9848,
                0.9848,
                0.8660,
                0.6428,
                0.3420
               };


static inline PRECISIONTYPE min(PRECISIONTYPE x, PRECISIONTYPE y) {
    return (x <= y ? x : y);
}
static inline PRECISIONTYPE max(PRECISIONTYPE x, PRECISIONTYPE y) {
    return (x <= y ? y : x);
}

static inline int min(int x, int y) {
    return (x <= y ? x : y);
}
static inline int max(int x, int y) {
    return (x <= y ? y : x);
}



DPMFeatures::DPMFeatures(const cv::Mat &Frame, int sbin, int padx, int pady): padx(padx), pady(pady) {
    /*Convert image to float?*/
    cv::Mat im_float;
    //Frame.convertTo(im_float, CV_32FC3);
    Frame.convertTo(im_float, CV_32FC3);

    if(Frame.channels() != 3) {
        std::cerr << "We expect a 3-channel image for the pyramid!" << std::endl;
        exit(1);
    }

    int dims[3] = {Frame.rows, Frame.cols, Frame.channels()};

    int S[3];
    double *feat = features(sbin, Frame.cols, Frame.rows, S, Frame);

    width = S[1];
    height = S[0];
    depth = S[2];

// Pad the features
    int pad[2] = {pady+1, padx+1};
    int size[] = {height, width, depth};
    feat = padarray(feat, pad, size);

    width = size[1];
    height = size[0];
    depth = size[2];

    int place=31;

    for(int W=0; W<=pady; W++) {
        for(int Q=0; Q<width; Q++) {
            feat[place*width*height+ Q * height + W ] = 1;
        }
    }
    for(int W=height-pady-1; W<height; W++) {
        for(int Q=0; Q<width; Q++) {
            feat[place*height*width+ Q * height + W ] = 1;
        }
    }

    for(int W=0; W<height; W++) {
        for(int Q=0; Q<=padx; Q++) {
            feat[place*height*width+ Q * height + W ] = 1;
        }
    }

    for(int W=0; W<height; W++) {
        for(int Q=width-padx-1; Q<width; Q++) {
            feat[place*width*height+ Q * height + W ] = 1;
        }
    }

    this->FullFeatures = feat;
}

DPMFeatures::~DPMFeatures() {
    free(FullFeatures);
}

