#ifndef _H_CASCADE
#define _H_CASCADE

struct Cascade_Volatile {
    // convolution values
    double *PCONV[2];
    // distance transform values
    double *DT[2];
    // distance transform argmaxes, x dimension
    int *DXAM[2];
    // distance transform argmaxes, y dimension
    int *DYAM[2];

    int *LOFFCONV;
    // pyramid level offsets for distance transform
    int *LOFFDT;
};

struct Cascade_NonVolatile {
    // precomputed deformation costs
    double **DXDEFCACHE;
    double **DYDEFCACHE;
};

#endif
