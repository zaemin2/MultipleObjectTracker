#ifndef _H_DPMPCAPyramid
#define _H_DPMPCAPyramid
#include "DPMFunctions.h"
#include "DPMPyramid.h"
#include "DPMPcaFeatures.h"

class DPMPcaPyramid {
public:
    DPMPcaPyramid(const DPMPyramid &DPyr,const CModel *m);
    DPMPcaFeatures* getLayer(int l) const {
        if(l >= 0 && l < this->getNumLayers()) {
            return this->Features[l];
        }
        else {
            std::cerr << "Request PCA-layer out of range" << std::endl;
            exit(3);
        }
    }
    int getNumLayers() const {
        return this->Features.size();
    }

    ~DPMPcaPyramid() {
        for(int l=0; l<this->getNumLayers(); l++) {
            delete Features[l];
        }
    }
private:
    std::vector<DPMPcaFeatures*> Features;
};

#endif
