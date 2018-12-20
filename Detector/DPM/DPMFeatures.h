/*
 * DPMFeatures.h
 *
 *  Created on: Mar 20, 2015
 *      Author: fds
 */

#ifndef DPMFEATURES_H_
#define DPMFEATURES_H_

#define PRECISIONTYPE double

#include <vector>
#include <iostream>

//OpenCV headers
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"

class DPMFeatures {
    friend class DPMPcaFeatures;

public:
    double *getFeatures() const {
        return this->FullFeatures;
    }

    DPMFeatures(const cv::Mat &Frame, int sbin, int padx, int pady);
    virtual ~DPMFeatures();

    int getWidth() {
        return this->width;
    }
    int getHeight() {
        return this->height;
    }

    int getDepth() {
        return this->depth;
    }


    void PrintLayer() {
        for(int c=0; c<1; c++) {
            for(int y=0; y<this->height; y++) {
                for(int x=0; x<width; x++) {
                    std::cout << FullFeatures[c*width*height+x*height+y] << " ";
                }
                std::cout << std::endl;
            }
            std::cout << std::endl << std::endl;
        }
    }

private:
    PRECISIONTYPE *FullFeatures; //becomes a H x W x 32 array
    int width;
    int height;
    int depth; //normally 32

    int padx;
    int pady;

};

#endif /* DPMFEATURES_H_ */
