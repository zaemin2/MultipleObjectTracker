/*
 * DPMPyramid.h
 *
 *  Created on: Mar 20, 2015
 *      Author: fds
 */

#ifndef DPMPYRAMID_H_
#define DPMPYRAMID_H_

#include "DPMFeatures.h"
#include <iostream>

//OpenCV headers
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"

class DPMPyramid {
public:
    DPMPyramid(const cv::Mat &Frame,int padx, int pady, int interval);

	DPMPyramid(const cv::Mat &Frame, int padx, int pady, int interval, int minheight, int maxheight, int modelheight);


    virtual ~DPMPyramid();
    int getNumLayers() const {
        return scales.size();
    }

    double getScale(int l) const {
        if(l >= 0 && l < this->getNumLayers()) {
            return scales[l];
        }
        else {
            std::cerr << "Requesting scale out of range" << std::endl;
            exit(6);
        }
    }


   int getInterval() const{
	return this->interval;
	}

    DPMFeatures* getLayer(int l) const {
        if(l >=0 && l < this->getNumLayers()) {
            return Features[l];
        }
        else {

            std::cout << "Requested " << l <<  " while size is " << this->getNumLayers() << std::endl;
            std::cerr << "Requesting pyramid layer outside of the scope" << std::endl;
            exit(2);
        }
    }

    int getPadx() const {
        return this->padx;
    }
    int getPady() const {
        return this->pady;
    }

private:
    std::vector<double> scales;
    std::vector<double> imscale;
    std::vector<DPMFeatures*> Features;
    std::vector<int> sbins;

    int padx;
    int pady;
    int interval;
};

#endif /* DPMPYRAMID_H_ */
