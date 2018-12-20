/*
 * DPMPcaFeatures.cpp
 *
 *  Created on: Mar 20, 2015
 *      Author: fds
 */

#include "DPMPcaFeatures.h"

DPMPcaFeatures::DPMPcaFeatures(DPMFeatures *Feat,const CModel *model) {
    // TODO Auto-generated constructor stub
    int S[] = {Feat->getHeight(),Feat->getWidth(),Feat->getDepth()};
    this->PcaFeatures = project(Feat->FullFeatures, model->coeff,S);
    /*Set sizes*/

    height = S[0];
    width = S[1];
    depth = S[2];

}

DPMPcaFeatures::~DPMPcaFeatures() {
    free(PcaFeatures);
}

