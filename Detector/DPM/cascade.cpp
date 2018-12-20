#include "CModel.h"
#include <vector>
#include <cmath>
#include "model.h"
#include <iostream>
#include <stdio.h>

#include "DPMPCAPyramid.h"
#include "DPMPyramid.h"

#include "cascade.h"

using namespace std;

// half-width of distance transform window
static const int S = 5;

// square an int
static inline int square(int x) {
    return x*x;
}

// compute convolution value of filter B on data A at a single
// location (x, y)
static inline double conv(int x, int y,
                          const double *A, const int *A_dims,
                          const double *B, const int *B_dims,
                          int num_features) {
    double val = 0;
    const double *A_src = A + x*A_dims[0] + y;
    const double *B_off = B;
    int A_inc = A_dims[0]*A_dims[1];

    for (int f = 0; f < num_features; f++) {
        const double *A_off = A_src;
        for (int xp = 0; xp < B_dims[1]; xp++) {
            // valid only for filters with <= 20 rows
            switch(B_dims[0]) {
            case 20:
                val += A_off[19] * B_off[19];
            case 19:
                val += A_off[18] * B_off[18];
            case 18:
                val += A_off[17] * B_off[17];
            case 17:
                val += A_off[16] * B_off[16];
            case 16:
                val += A_off[15] * B_off[15];
            case 15:
                val += A_off[14] * B_off[14];
            case 14:
                val += A_off[13] * B_off[13];
            case 13:
                val += A_off[12] * B_off[12];
            case 12:
                val += A_off[11] * B_off[11];
            case 11:
                val += A_off[10] * B_off[10];
            case 10:
                val += A_off[9] * B_off[9];
            case 9:
                val += A_off[8] * B_off[8];
            case 8:
                val += A_off[7] * B_off[7];
            case 7:
                val += A_off[6] * B_off[6];
            case 6:
                val += A_off[5] * B_off[5];
            case 5:
                val += A_off[4] * B_off[4];
            case 4:
                val += A_off[3] * B_off[3];
            case 3:
                val += A_off[2] * B_off[2];
            case 2:
                val += A_off[1] * B_off[1];
            case 1:
                val += A_off[0] * B_off[0];

            }
            A_off += A_dims[0];
            B_off += B_dims[0];
        }
        A_src += A_inc;
    }
    return val;
}


// compute convolution value for a root filter at a fixed location
static inline double rconv(int L, int filterind, int x, int y, int pca,Model *MODEL) {
    const int *A_dims = MODEL->featdims[L];

    const double *A = MODEL->feat[pca][L];
    const int *B_dims = MODEL->rootfilterdims[filterind];

    const double *B = MODEL->rootfilters[filterind];
    int num_features = MODEL->numfeatures;
    // compute convolution
    return conv(x, y, A, A_dims, B, B_dims, num_features);
}


// compute convolution of a filter and distance transform of over the resulting
// values using memoized convolutions and deformation pruning
static inline double pconvdt(int L, int probex, int probey, int filterind, int defindex, int xstart, int xend, int ystart, int yend, int pca, double defthresh, Model *MODEL, Cascade_Volatile &CascVol, Cascade_NonVolatile &CascNVol)
{
    const int *A_dims = MODEL->featdims[L];
    const double *A = MODEL->feat[pca][L];
    const int *B_dims = MODEL->partfilterdims[filterind];
    const double *B = MODEL->partfilters[pca][filterind];
    int num_features = (pca == 1 ? MODEL->pcadim : MODEL->numfeatures);

    double *ptrbase = CascVol.PCONV[pca] + CascVol.LOFFCONV[L] + filterind*MODEL->featdimsprod[L];


    for (int x = xstart; x <= xend; x++) {
        double *ptr = ptrbase + x*MODEL->featdims[L][0] + ystart-1;
        for (int y = ystart; y <= yend; y++) {
            ptr++;
            // skip if already computed
            if (*ptr > -INFINITY)
                continue;

            // check for deformation pruning
            double defcost = CascNVol.DXDEFCACHE[defindex][probex-x+S] + CascNVol.DYDEFCACHE[defindex][probey-y+S];
            if (defcost < defthresh)
                continue;
            // compute convolution
            *ptr = conv(x, y, A, A_dims, B, B_dims, num_features);
        }
    }

    // do distance transform over the region.
    // the region is small enough that brute force DT
    // is the fastest method.
    double max = -INFINITY;
    int xargmax = 0;
    int yargmax = 0;

    for (int x = xstart; x <= xend; x++) {
        double *ptr = ptrbase + x*MODEL->featdims[L][0] + ystart-1;
        for (int y = ystart; y <= yend; y++) {
            ptr++;
            double val = *ptr + CascNVol.DXDEFCACHE[defindex][probex-x+S]
                         + CascNVol.DYDEFCACHE[defindex][probey-y+S];
            if (val > max) {
                max = val;
                xargmax = x;
                yargmax = y;
            }
        }
    }
    int offset = defindex*MODEL->featdimsprod[L]
                 + probex*MODEL->featdims[L][0]
                 + probey;

    // record max and argmax for DT
    *(CascVol.DXAM[pca] + CascVol.LOFFDT[L] + offset) = xargmax;
    *(CascVol.DYAM[pca] + CascVol.LOFFDT[L] + offset) = yargmax;
    *(CascVol.DT[pca] + CascVol.LOFFDT[L] + offset) = max;
    return max;
}

// lookup or compute the score of a part at a location
static inline double partscore(int L, int defindex, int pfind,
                               int x, int y, int pca, double defthresh, int padx, int pady, Model *MODEL,Cascade_Volatile &CascVol, Cascade_NonVolatile &CascNVol)
{
    // remove virtual padding
    x -= padx;
    y -= pady;

    // check if already computed...
    int offset = defindex*MODEL->featdimsprod[L]
                 + x*MODEL->featdims[L][0]
                 + y;

    double *ptr = CascVol.DT[pca] + CascVol.LOFFDT[L] + offset;

    if (*ptr > -INFINITY)
        return *ptr;

    // ...nope, define the bounds of the convolution and
    // distance transform region
    int xstart = x-S;
    xstart = (xstart < 0 ? 0 : xstart);
    int xend = x+S;

    int ystart = y-S;
    ystart = (ystart < 0 ? 0 : ystart);
    int yend = y+S;

    const int *A_dims = MODEL->featdims[L];
    const int *B_dims = MODEL->partfilterdims[pfind];
    yend = (B_dims[0] + yend > A_dims[0])
           ? yend = A_dims[0] - B_dims[0]
                    : yend;
    xend = (B_dims[1] + xend > A_dims[1])
           ? xend = A_dims[1] - B_dims[1]
                    : xend;

    // do convolution and distance transform in region
    // [xstart, xend] x [ystart, yend]
    return pconvdt(L, x, y,
                   pfind, defindex,
                   xstart, xend,
                   ystart, yend,
                   pca, defthresh,MODEL,CascVol, CascNVol);
}

// initialize global data
static void init(const CModel *model, int s,const DPMPyramid &DPyr,const DPMPcaPyramid &PCAPyr, Model*& MODEL, Cascade_Volatile &CascVol, Cascade_NonVolatile &CascNVol ) {
    // inittimer->tic();

    // init model and feature pyramid
    MODEL = new Model(model); //creating a NEW model
    MODEL->interval = DPyr.getInterval();

    //MODEL->initpyramid(pyra,DPyr, PCAPyr); //2 pyramids, old new
    MODEL->initpyramid(DPyr, PCAPyr); //2 pyramids, old new

    // allocate memory for storing convolution and
    // distance transform data pyramids
    int N    = s;//(int)mxGetScalar(prhs[8]); //s argment (last)
    CascVol.PCONV[0] = new double[N];
    CascVol.PCONV[1] = new double[N];
    CascVol.DT[0]    = new double[N];
    CascVol.DT[1]    = new double[N];
    for (int i = 0; i < N; i++) {
        CascVol.PCONV[0][i] = -INFINITY;
        CascVol.PCONV[1][i] = -INFINITY;
        CascVol.DT[0][i]    = -INFINITY;
        CascVol.DT[1][i]    = -INFINITY;
    }

    // each data pyramid (convolution and distance transform)
    // is stored in a 1D array.  since pyramid levels have
    // different sizes, we build an array of offset values
    // in order to index by level.  the last offset is the
    // total length of the pyramid storage array.
    CascVol.LOFFCONV = new int[MODEL->numlevels+1];
    CascVol.LOFFDT = new int[MODEL->numlevels+1];
    CascVol.LOFFCONV[0] = 0;
    CascVol.LOFFDT[0] = 0;
    for (int i = 1; i < MODEL->numlevels+1; i++) {
        CascVol.LOFFCONV[i] = CascVol.LOFFCONV[i-1] + MODEL->numpartfilters*MODEL->featdimsprod[i-1];
        CascVol.LOFFDT[i]   = CascVol.LOFFDT[i-1]   + MODEL->numdefparams*MODEL->featdimsprod[i-1];
    }

    // cache of precomputed deformation costs
    CascNVol.DXDEFCACHE = new double*[MODEL->numdefparams];
    CascNVol.DYDEFCACHE = new double*[MODEL->numdefparams];
    for (int i = 0; i < MODEL->numdefparams; i++) {
        const double *def = MODEL->defs[i];
        CascNVol.DXDEFCACHE[i] = new double[2*S+1];
        CascNVol.DYDEFCACHE[i] = new double[2*S+1];
        for (int j = 0; j < 2*S+1; j++) {
            CascNVol.DXDEFCACHE[i][j] = -def[0]*square(j-S) - def[1]*(j-S);
            CascNVol.DYDEFCACHE[i][j] = -def[2]*square(j-S) - def[3]*(j-S);
        }
    }

    for (int p = 0; p < 2; p++) {
        // allocate memory (left uninitialized intentionally)
        CascVol.DXAM[p] = new int[CascVol.LOFFDT[MODEL->numlevels]];
        CascVol.DYAM[p] = new int[CascVol.LOFFDT[MODEL->numlevels]];
    }
}

// free global data
static void cleanup(Model *MODEL, Cascade_Volatile &CascVol, Cascade_NonVolatile &CascNVol) {
    delete [] CascVol.LOFFCONV;
    delete [] CascVol.LOFFDT;
    for (int i = 0; i < MODEL->numdefparams; i++) {
        delete [] CascNVol.DXDEFCACHE[i];
        delete [] CascNVol.DYDEFCACHE[i];
    }
    for (int i = 0; i < 2; i++) {
        delete [] CascVol.DXAM[i];
        delete [] CascVol.DYAM[i];
    }
    delete [] CascNVol.DXDEFCACHE;
    delete [] CascNVol.DYDEFCACHE;
    delete MODEL;
    delete [] CascVol.PCONV[0];
    delete [] CascVol.PCONV[1];
    delete [] CascVol.DT[0];
    delete [] CascVol.DT[1];
}



//void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
int cascade(const CModel *model ,double*** rootscores, int numrootlocs, int padxN, int padyN, int s, int ***SizesC/*rootscoredims*/,vector<double> &coords,const DPMPyramid &DPyr,const DPMPcaPyramid &PCAPyr) {

    // Create instance for model
    Model *MODEL;

    Cascade_Volatile CascVol;
    Cascade_NonVolatile CascNVol;

    //initialise the MODEL variable
    init(model, s, DPyr, PCAPyr,MODEL,CascVol, CascNVol);

    int padx = padxN;
    int pady = padyN;
    // we need to keep track of the PCA scores for each PCA filter.
    // allocate some memory for storing these values.
    double **pcascore = new double*[MODEL->numcomponents];
    for (int c = 0; c < MODEL->numcomponents; c++)
        pcascore[c] = new double[MODEL->numparts[c]+1];

    int nlevels = MODEL->numlevels-MODEL->interval;

    // process each model component and pyramid level (note: parallelize here)
    for (int comp = 0; comp < MODEL->numcomponents; comp++) {
        for (int plevel = 0; plevel < nlevels; plevel++) {
            // root filter pyramid level
            int rlevel = plevel+MODEL->interval;
            double *mxA = rootscores[comp][rlevel];

            int dim[2] = {SizesC[comp][rlevel][0],SizesC[comp][rlevel][1]};//mxGetDimensions(mxA);
            const double *rtscore = mxA;

            for (int rx = ceil(padx/2.0); rx < dim[1] - ceil(padx/2.0); rx++) {
                for (int ry = ceil(pady/2.0); ry < dim[0] - ceil(pady/2.0); ry++) {
                    // get stage 0 score (PCA root + component offset)
                    double score = *(rtscore + rx*dim[0] + ry);

                    // record score of PCA filter ('score' has the component offset added
                    // to it, so we subtract it here to get just the PCA filter score)
                    pcascore[comp][0] = score - MODEL->offsets[comp];

                    // cascade stages 1 through 2*numparts+2
                    int stage = 1;
                    int numstages = 2*MODEL->numparts[comp]+2;
                    for (; stage < numstages; stage++) {


                        // check for hypothesis pruning
                        if (score < MODEL->t[comp][2*stage-1]) {
                            break;
                        }

                        // pca == 1 if we're placing pca filters
                        // pca == 0 if we're placing "full"/non-pca filters
                        int pca = (stage < MODEL->numparts[comp]+1 ? 1 : 0);
                        // get the part# used in this stage
                        // root parts have index -1, non-root parts are indexed 0:numparts
                        int part = MODEL->partorder[comp][stage];

                        if (part == -1) {
                            // we just finished placing all PCA filters, now replace the PCA root
                            // filter with the non-PCA root filter
                            double rscore = rconv(rlevel, comp, rx, ry, pca,MODEL);
                            score += rscore - pcascore[comp][0];
                        } else {

                            // place a non-root filter (either PCA or non-PCA)

                            int px = 2*rx + (int)MODEL->anchors[comp][part][0];
                            int py = 2*ry + (int)MODEL->anchors[comp][part][1];
                            // lookup the filter and deformation model used by this part
                            int filterind = MODEL->pfind[comp][part];
                            int defind = MODEL->defind[comp][part];
                            double defthresh = MODEL->t[comp][2*stage] - score;
                            double ps = partscore(plevel, defind, filterind,
                                                  px, py, pca, defthresh, padx, pady,MODEL,CascVol, CascNVol);

                            if (pca == 1) {
                                // record PCA filter score and update hypothesis score with ps
                                pcascore[comp][part+1] = ps;
                                score += ps;
                            } else {
                                // update hypothesis score by replacing the PCA filter score with ps
                                score += ps - pcascore[comp][part+1];
                            }
                        }
                    }
                    // check if the hypothesis passed all stages with a final score over
                    // the global threshold
                    if (stage == numstages && score >= MODEL->thresh) {
                        // compute and record image coordinates of the detection window
                        double scale = MODEL->sbin/DPyr.getScale(rlevel);
                        //double scale = MODEL->sbin/scales[rlevel];
                        double x1 = (rx-padx)*scale;
                        double y1 = (ry-pady)*scale;
                        double x2 = x1 + MODEL->rootfilterdims[comp][1]*scale - 1;
                        double y2 = y1 + MODEL->rootfilterdims[comp][0]*scale - 1;
                        // add 1 for matlab 1-based indexes
                        coords.push_back(x1+1);
                        coords.push_back(y1+1);
                        coords.push_back(x2+1);
                        coords.push_back(y2+1);
                        // compute and record image coordinates of the part filters
                        scale = MODEL->sbin/DPyr.getScale(plevel);
                        for (int P = 0; P < MODEL->numparts[comp]; P++) {
                            int probex = 2*rx + (int)MODEL->anchors[comp][P][0];
                            int probey = 2*ry + (int)MODEL->anchors[comp][P][1];
                            int dind = MODEL->defind[comp][P];
                            int offset = CascVol.LOFFDT[plevel] + dind*MODEL->featdimsprod[plevel]
                                         + (probex-padx)*MODEL->featdims[plevel][0] + (probey-pady);
                            int px = *(CascVol.DXAM[0] + offset) + padx;
                            int py = *(CascVol.DYAM[0] + offset) + pady;
                            double x1 = (px-2*padx)*scale;
                            double y1 = (py-2*pady)*scale;
                            double x2 = x1 + MODEL->partfilterdims[P][1]*scale - 1;
                            double y2 = y1 + MODEL->partfilterdims[P][0]*scale - 1;
                            coords.push_back(x1+1);
                            coords.push_back(y1+1);
                            coords.push_back(x2+1);
                            coords.push_back(y2+1);
                        }
                        // record component number and score
                        coords.push_back(comp+1);
                        coords.push_back(score);
                    }
                } // end loop over root y
            }   // end loop over root x
        }     // end loop over pyramid levels
    }         // end loop over components

    // searchtimer->toc();
    // searchtimer->mexPrintTimer();

    // width calculation:
    //  4 = detection window x1,y1,x2,y2
    //  4*numparts = x1,y1,x2,y2 for each part
    //  2 = component #, total score
    // (NOTE: assumes that all components have the same number of parts)
    int width = 4 + 4*MODEL->numparts[0] + 2;

    for (int i = 0; i < MODEL->numcomponents; i++)
        delete [] pcascore[i];
    delete [] pcascore;
    cleanup(MODEL,CascVol,CascNVol);

    return width;
}
