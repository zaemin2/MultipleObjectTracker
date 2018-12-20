#ifndef CASCADE_H
#define CASCADE_H
#include "CascadeModelOld.h"
#include "NewCascadeModelOld.h"




struct CascadeArrays_VolatileOld {
    // convolution values
    float *PCONV[2];
    // distance transform values
    float *DT[2];
    // distance transform argmaxes, x dimension
    int *DXAM[2];
    // distance transform argmaxes, y dimension
    int *DYAM[2];

	int *LOFFCONV;
    // pyramid level offsets for distance transform
    int *LOFFDT;
};

struct CascadeArrays_NonVolatileOld {

	ModelOld *MODEL;

    // half-width of distance transform window
    int S;
    // padding used in the feature pyramid
    int padx, pady;
    // precomputed deformation costs
    float **DXDEFCACHE;
    float **DYDEFCACHE;

};

CascadeArrays_NonVolatileOld InitNonVolatileDataOld(NewCascadeModelOld *CModel);
void CleanupNonVolatileDataOld(CascadeArrays_NonVolatileOld &Mod);

int cascadeOld(NewCascadeModelOld *model, PyramidOld *pyra,float*** rootscores,/* int numrootlocs,*/ int padxN, int padyN, int s, int ***SizesC/*rootscoredims*/,std::vector<float> &coords,const CascadeArrays_NonVolatileOld &NonVolatile);
void initNonVolatileOld(PyramidOld *pyra, CascadeArrays_NonVolatileOld &NonVolStruct,NewCascadeModelOld &NModel);
void cleanupNonVolatileOld(CascadeArrays_NonVolatileOld &NonVolStruct, NewCascadeModelOld &NModel);
#endif // CASCADE_H
