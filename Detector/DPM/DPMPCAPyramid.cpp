#include "DPMPCAPyramid.h"
#include "DPMPyramid.h"
#include "DPMPcaFeatures.h"

DPMPcaPyramid::DPMPcaPyramid(const DPMPyramid &DPyr,const CModel *m) {
    this->Features.resize(DPyr.getNumLayers());
    /*Project each layer of the pyramid*/

    for(int l=0; l<DPyr.getNumLayers(); l++) {
        Features[l] = new DPMPcaFeatures(DPyr.getLayer(l), m);
    }

}
