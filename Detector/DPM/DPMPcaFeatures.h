/*
 * DPMPcaFeatures.h
 *
 *  Created on: Mar 20, 2015
 *      Author: fds
 */

#ifndef DPMPCAFEATURES_H_
#define DPMPCAFEATURES_H_

#include "DPMFeatures.h"
#include "DPMFunctions.h"

class DPMPcaFeatures {
public:
    DPMPcaFeatures(DPMFeatures *Feat,const CModel *model);
    virtual ~DPMPcaFeatures();

    int getWidth() {
        return width;
    }
    int getHeight() {
        return height;
    }
    int getDepth() {
        return this->depth;
    }

    double *getFeatures() {
        return PcaFeatures;
    }


private:
    double *PcaFeatures;
    int width;
    int height;
    int depth;

};

#endif /* DPMPCAFEATURES_H_ */
