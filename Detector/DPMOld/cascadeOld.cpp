//============================================================================
// Author      : F. De Smedt @ EAVISE
// Copyright   : This code is written for the publication of  "Open Framework for Combined Pedestrian detection". This code is for research and educational purposes only. For a different license, please contact the contributors directly. More information about the license can be fount in license.txt
//============================================================================



#include "NewCascadeModelOld.h"
#include <vector>
#include <cmath>
#include "modelOld.h"
#include <iostream>
#include <stdio.h>

#include "cascadeOld.h"

using namespace std;


static const int S = 5;
static int padx, pady;

// square an int
static inline int square(int x)
{
    return x*x;
}

// compute convolution value of filter B on data A at a single
// location (x, y)
static inline float conv(int x, int y,
                         const float *A, const int *A_dims,
                         const float *B, const int *B_dims,
                         int num_features)
{
    float val = 0;
    const float *A_src = A + x*A_dims[0] + y;
    const float *B_off = B;
    int A_inc = A_dims[0]*A_dims[1];


    for (int f = 0; f < num_features; f++)
    {
        const float *A_off = A_src;
        for (int xp = 0; xp < B_dims[1]; xp++)
        {
            // valid only for filters with <= 20 rows
            switch(B_dims[0])
            {
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
//L -> Level (of root)
//pca -> indicates if this is a pca-filter or not
//filterind -> indicates in which internal model we are (component of model)
static inline float rconv(int L, int filterind, int x, int y, int pca,const CascadeArrays_NonVolatileOld &MM, VolatileModelOld *VMod)
{
    const int *A_dims = VMod->featdims[L];

    const float *A = VMod->feat[pca][L];
    const int *B_dims = MM.MODEL->rootfilterdims[filterind];

    const float *B = MM.MODEL->rootfilters[filterind];
    int num_features = VMod->numfeatures;
    // compute convolution
    return conv(x, y, A, A_dims, B, B_dims, num_features);
}


// compute convolution of a filter and distance transform of over the resulting
// values using memorized convolutions and deformation pruning
static inline float pconvdt(int L, int probex, int probey, int filterind, int defindex, int xstart, int xend, int ystart, int yend, int pca, float defthresh,const CascadeArrays_NonVolatileOld &MM,CascadeArrays_VolatileOld &Volatile, VolatileModelOld *VMod)
{
    //get arguments of function
    const int *A_dims = VMod->featdims[L];
    const float *A = VMod->feat[pca][L];
    const int *B_dims = MM.MODEL->partfilterdims[filterind];
    const float *B = MM.MODEL->partfilters[pca][filterind];
    int num_features = (pca == 1 ? VMod->pcadim : VMod->numfeatures);
    float *ptrbase = Volatile.PCONV[pca] + Volatile.LOFFCONV[L] + filterind*VMod->featdimsprod[L];

//loop over window
    for (int x = xstart; x <= xend; x++)
    {
        float *ptr = ptrbase + x*VMod->featdims[L][0] + ystart-1;//minus one is compensated by the increment as the first step in the iteration
        for (int y = ystart; y <= yend; y++)
        {
            ptr++;
            // skip if already computed
            if (*ptr > -INFINITY)
                continue;

            // check for deformation pruning
            float defcost = MM.DXDEFCACHE[defindex][probex-x+S] + MM.DYDEFCACHE[defindex][probey-y+S];
            if (defcost < defthresh) //if below threshhold, it is not even needed to calculate the actual filter response
                continue;
            // compute convolution
            //Is directly put in PCONV
            *ptr = conv(x, y, A, A_dims, B, B_dims, num_features);
        }
    }

    // do distance transform over the region.
    // the region is small enough that brute force DT
    // is the fastest method.
    float max = -INFINITY;
    int xargmax = 0;
    int yargmax = 0;
//search for the best filter response in combination with its deformation cost (big cost is better ...)
    for (int x = xstart; x <= xend; x++)
    {
        float *ptr = ptrbase + x*VMod->featdims[L][0] + ystart-1;
        for (int y = ystart; y <= yend; y++)
        {
            ptr++;
            float val = *ptr + MM.DXDEFCACHE[defindex][probex-x+S]
                        + MM.DYDEFCACHE[defindex][probey-y+S];
            if (val > max)
            {
                max = val;
                xargmax = x;
                yargmax = y;
            }
        }
    }
    int offset = defindex*VMod->featdimsprod[L]
                 + probex*VMod->featdims[L][0]
                 + probey;

    // record max and argmax for DT
    *(Volatile.DXAM[pca] + Volatile.LOFFDT[L] + offset) = xargmax;
    *(Volatile.DYAM[pca] + Volatile.LOFFDT[L] + offset) = yargmax;
    *(Volatile.DT[pca] + Volatile.LOFFDT[L] + offset) = max; //record the maximum of this surrounding
    return max;
}

// lookup or compute the score of a part at a location
//L is the level in the pyramid we are looking on for the filter
static inline float partscore(int L, int defindex, int pfind, int x, int y, int pca, float defthresh,const CascadeArrays_NonVolatileOld &MM,CascadeArrays_VolatileOld &Volatile,VolatileModelOld *VMod)
{
    // remove virtual padding
    x -= padx;
    y -= pady;

    // check if already computed...
    int offset = defindex*VMod->featdimsprod[L]
                 + x*VMod->featdims[L][0]
                 + y;

    float *ptr = Volatile.DT[pca] + Volatile.LOFFDT[L] + offset;

    if (*ptr > -INFINITY) //already computed
        return *ptr;

    // ...nope, define the bounds of the convolution and
    // distance transform region
    int xstart = x-S;
    xstart = (xstart < 0 ? 0 : xstart); //boundary checking
    int xend = x+S;

    int ystart = y-S;
    ystart = (ystart < 0 ? 0 : ystart); //boundary checking
    int yend = y+S; //check comes later

    const int *A_dims = VMod->featdims[L];
    const int *B_dims = MM.MODEL->partfilterdims[pfind];
    yend = (B_dims[0] + yend > A_dims[0]) ? yend = A_dims[0] - B_dims[0] : yend; //Boundary checking
    xend = (B_dims[1] + xend > A_dims[1]) ? xend = A_dims[1] - B_dims[1]  : xend; //Boundary checking

    // do convolution and distance transform in region
    // [xstart, xend] x [ystart, yend]
    //calculation of filter
    return pconvdt(L, x, y,
                   pfind, defindex,
                   xstart, xend,
                   ystart, yend,
                   pca, defthresh,MM,Volatile,VMod);
}


void CleanupNonVolatileDataOld(CascadeArrays_NonVolatileOld &Mod){
	for (int i = 0; i < Mod.MODEL->numdefparams; i++)
	    {
	        delete [] Mod.DXDEFCACHE[i];
	        delete [] Mod.DYDEFCACHE[i];
	    }

	    delete [] Mod.DXDEFCACHE;
	    delete [] Mod.DYDEFCACHE;

}

CascadeArrays_NonVolatileOld InitNonVolatileDataOld(NewCascadeModelOld *CModel){
	CascadeArrays_NonVolatileOld M;
	    	M.MODEL = CModel->MODEL;

	    	M.S = 5;

	    // cache of precomputed deformation costs
	    M.DXDEFCACHE = new float*[M.MODEL->numdefparams];
	    M.DYDEFCACHE = new float*[M.MODEL->numdefparams];
	    for (int i = 0; i < M.MODEL->numdefparams; i++)
	    {
	        const float *def = M.MODEL->defs[i];
	        M.DXDEFCACHE[i] = new float[2*M.S+1];
	        M.DYDEFCACHE[i] = new float[2*M.S+1];
	        for (int j = 0; j < 2*M.S+1; j++)
	        {
	            M.DXDEFCACHE[i][j] = -def[0]*square(j-M.S) - def[1]*(j-M.S);
	            M.DYDEFCACHE[i][j] = -def[2]*square(j-M.S) - def[3]*(j-M.S);
	        }
	    }

	    return M;
}



// initialize global data
static void initOld(NewCascadeModelOld *model, PyramidOld *pyra, int s,const CascadeArrays_NonVolatileOld &MM,CascadeArrays_VolatileOld &Volatile,VolatileModelOld *VMod)
{


    // allocate memory for storing convolution and
    // distance transform data pyramids
    int N    = s;//(int)mxGetScalar(prhs[8]); //s argment (last)
    Volatile.PCONV[0] = new float[N];
    Volatile.PCONV[1] = new float[N];
    Volatile.DT[0]    = new float[N];
    Volatile.DT[1]    = new float[N];
    /*Set everything to -INFINITY*/
    for (int i = 0; i < N; i++)
    {
    	Volatile.PCONV[0][i] = -INFINITY;
    	Volatile.PCONV[1][i] = -INFINITY;
    	Volatile.DT[0][i]    = -INFINITY;
    	Volatile.DT[1][i]    = -INFINITY;
    }

    // each data pyramid (convolution and distance transform)
    // is stored in a 1D array.  since pyramid levels have
    // different sizes, we build an array of offset values
    // in order to index by level.  the last offset is the
    // total length of the pyramid storage array.
    Volatile.LOFFCONV = new int[VMod->numlevels+1];
    Volatile.LOFFDT = new int[VMod->numlevels+1];
    Volatile.LOFFCONV[0] = 0;
    Volatile.LOFFDT[0] = 0;
    for (int i = 1; i < VMod->numlevels+1; i++)
    {
    	Volatile.LOFFCONV[i] = Volatile.LOFFCONV[i-1] + MM.MODEL->numpartfilters*VMod->featdimsprod[i-1];
    	Volatile.LOFFDT[i]   = Volatile.LOFFDT[i-1]   + MM.MODEL->numdefparams*VMod->featdimsprod[i-1];
    }

    for (int p = 0; p < 2; p++)
    {
        // allocate memory (left uninitialized intentionally)
    	Volatile.DXAM[p] = new int[Volatile.LOFFDT[VMod->numlevels]];
    	Volatile.DYAM[p] = new int[Volatile.LOFFDT[VMod->numlevels]];
    }

}

// free global data
static void cleanup(CascadeArrays_VolatileOld &Volatile)
{
    delete [] Volatile.LOFFCONV;
    delete [] Volatile.LOFFDT;

    for (int i = 0; i < 2; i++)
    {
        delete [] Volatile.DXAM[i];
        delete [] Volatile.DYAM[i];
    }

    delete [] Volatile.PCONV[0];
    delete [] Volatile.PCONV[1];
    delete [] Volatile.DT[0];
    delete [] Volatile.DT[1];
}



//void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
int cascadeOld(NewCascadeModelOld *model, PyramidOld *pyra,float*** rootscores,/* int numrootlocs,*/ int padxN, int padyN, int s, int ***SizesC/*rootscoredims*/,vector<float> &coords,const CascadeArrays_NonVolatileOld &MM)
{

	VolatileModelOld *VMod = new VolatileModelOld(pyra);
	CascadeArrays_VolatileOld Volatile;

    initOld(model, pyra, s,MM,Volatile,VMod);

    float *scales      = pyra->scales;//mxGetPr(prhs[5]);
    padx = padxN;
    pady = padyN;
    // we need to keep track of the PCA scores for each PCA filter.
    // allocate some memory for storing these values.
    float **pcascore = new float*[model->numcomponents];
    for (int c = 0; c < model->numcomponents; c++)
        pcascore[c] = new float[MM.MODEL->numparts[c]+1];


    int nlevels = VMod->numlevels-model->interval;

    // run over the components of the detector
    for (int comp = 0; comp < model->numcomponents; comp++)
    {
        for (int plevel = 0; plevel < nlevels; plevel++)
        {
            // root filter pyramid level
            int rlevel = plevel+model->interval;
            float *mxA = rootscores[comp][rlevel];

            int dim[2] = {SizesC[comp][rlevel][0],SizesC[comp][rlevel][1]};//mxGetDimensions(mxA);
            const float *rtscore = mxA;

            /*Run over image*/
            for (int rx = ceil(padx/2.0); rx < dim[1] - ceil(padx/2.0); rx++)
            {
                for (int ry = ceil(pady/2.0); ry < dim[0] - ceil(pady/2.0); ry++)
                {
                    // get stage 0 score (PCA root + component offset)
                    float score = *(rtscore + rx*dim[0] + ry);

                    // record score of PCA filter ('score' has the component offset added
                    // to it, so we subtract it here to get just the PCA filter score)
                    pcascore[comp][0] = score - MM.MODEL->offsets[comp];

                    // cascade stages 1 through 2*numparts+2
                    int stage = 1;
                    int numstages = 2*MM.MODEL->numparts[comp]+2;
                    for (; stage < numstages; stage++)
                    {

                        // check for hypothesis pruning
                        if (score < MM.MODEL->t[comp][2*stage-1])
                        {
                            break;
                        }

                        // pca == 1 if we're placing pca filters
                        // pca == 0 if we're placing "full"/non-pca filters
                        int pca = (stage < MM.MODEL->numparts[comp]+1 ? 1 : 0); //the first x stages are PCA, a simplified filter
                        // get the part# used in this stage
                        // root parts have index -1, non-root parts are indexed 0:numparts
                        int part = MM.MODEL->partorder[comp][stage];

                        if (part == -1)   //Calculate result Root-filter
                        {

                            // we just finished placing all PCA filters, now replace the PCA root
                            // filter with the non-PCA root filter

                            // compute convolution value for a root filter at a fixed location
                            //Result of the root filter -> MODEL->interval deeper in pyramid
                            float rscore = rconv(rlevel, comp, rx, ry, pca,MM,VMod);
                            score += rscore - pcascore[comp][0];
                        }
                        else     //we go for a part filter
                        {

                            // place a non-root filter (either PCA or non-PCA)
                            int px = 2*rx + (int)MM.MODEL->anchors[comp][part][0];
                            int py = 2*ry + (int)MM.MODEL->anchors[comp][part][1];

                            // lookup the filter and deformation model used by this part
                            //
                            // which filter to use for this part, indicated by filterind (in the component)
                            int filterind = MM.MODEL->pfind[comp][part];
                            int defind = MM.MODEL->defind[comp][part]; //the deformation indication
                            float defthresh = MM.MODEL->t[comp][2*stage] - score; //which threshhold to reach ...
                            float ps = partscore(plevel, defind, filterind, px, py, pca, defthresh,MM,Volatile,VMod);

                            if (pca == 1)
                            {
                                // record PCA filter score and update hypothesis score with ps
                                pcascore[comp][part+1] = ps;
                                score += ps;
                            }
                            else
                            {
                                // update hypothesis score by replacing the PCA filter score with ps
                                score += ps - pcascore[comp][part+1]; // if no pca-filter we distract the result of the pca-filters so far
                            }
                        }
                    }
                    // check if the hypothesis passed all stages with a final score over
                    // the global threshold
                    //if (stage == numstages && score >= MM.MODEL->thresh)
                    if (stage == numstages && score >= model->thresh)
                    {
                        // compute and record image coordinates of the detection window
                        float scale = MM.MODEL->sbin/scales[rlevel];
                        float x1 = (rx-padx)*scale;
                        float y1 = (ry-pady)*scale;
                        float x2 = x1 + MM.MODEL->rootfilterdims[comp][1]*scale - 1;
                        float y2 = y1 + MM.MODEL->rootfilterdims[comp][0]*scale - 1;
                        // add 1 for matlab 1-based indexes
                        coords.push_back(x1+1);
                        coords.push_back(y1+1);
                        coords.push_back(x2+1);
                        coords.push_back(y2+1);
                        // compute and record image coordinates of the part filters
                        scale = MM.MODEL->sbin/scales[plevel];
                        for (int P = 0; P < MM.MODEL->numparts[comp]; P++)
                        {
                            int probex = 2*rx + (int)MM.MODEL->anchors[comp][P][0];
                            int probey = 2*ry + (int)MM.MODEL->anchors[comp][P][1];
                            int dind = MM.MODEL->defind[comp][P];
                            int offset = Volatile.LOFFDT[plevel] + dind*VMod->featdimsprod[plevel]
                                         + (probex-padx)*VMod->featdims[plevel][0] + (probey-pady);
                            int px = *(Volatile.DXAM[0] + offset) + padx;
                            int py = *(Volatile.DYAM[0] + offset) + pady;
                            float x1 = (px-2*padx)*scale;
                            float y1 = (py-2*pady)*scale;
                            float x2 = x1 + MM.MODEL->partfilterdims[P][1]*scale - 1;
                            float y2 = y1 + MM.MODEL->partfilterdims[P][0]*scale - 1;
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


    // width calculation:
    //  4 = detection window x1,y1,x2,y2
    //  4*numparts = x1,y1,x2,y2 for each part
    //  2 = component #, total score
    // (NOTE: assumes that all components have the same number of parts)
    int width = 4 + 4*MM.MODEL->numparts[0] + 2;

    for (int i = 0; i < MM.MODEL->numcomponents; i++)
        delete [] pcascore[i];
    delete [] pcascore;
    cleanup(Volatile);

    delete VMod;


    return width;
}
