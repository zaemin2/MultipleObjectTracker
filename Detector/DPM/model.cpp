//#include "mex.h"
#include "model.h"
#include "CModel.h"
#include <iostream>


#include "DPMPCAPyramid.h"

using namespace std;


// see: model.h for descriptions of each class field.

// handy accessors

// return field from struct a
/*static inline const mxArray *F(const mxArray *a, const char *field) {
  return mxGetField(a, 0, field);
}

// return pointer to field from struct a
static inline double *Fpr(const mxArray *a, const char *field) {
  return mxGetPr(F(a, field));
}

// return scalar of field from struct a
static inline double Fsc(const mxArray *a, const char *field) {
  return mxGetScalar(F(a, field));
}

// return field from struct in cell of struct array a
static inline const mxArray *CF(const mxArray *a, int cell, const char *field) {
  return F(mxGetCell(a, cell), field);
}
*/
void Model::initmodel(const CModel *model) {
    thresh        = model->thresh;
    interval      = model->interval;
    numcomponents = model->numcomponents;
    sbin          = model->sbin;
    numpartfilters            = model->SizePartfilters;;//(int)(mxGetDimensions(partinfos)[1]);
    numdefparams              = model->SizeDefs;//(int)(mxGetDimensions(definfos)[1]);

    numparts        = new int[numcomponents];
    anchors         = new double**[numcomponents];
    defs            = new double*[numdefparams];
    rootfilters     = new double*[numcomponents];
    partfilters[0]  = new double*[numpartfilters];
    partfilters[1]  = new double*[numpartfilters];
    rootfilterdims  = new int[numcomponents][3];
    partfilterdims  = new int[numpartfilters][3];
    pfind           = new int*[numcomponents];
    defind          = new int*[numcomponents];

    for (int i = 0; i < numpartfilters; i++) {
        double *w        = model->partfilters[i].w;//F(partinfo, "w");
        partfilters[0][i]       = w;

        partfilterdims[i][0] = model->partfilters[i].SizeW[2];//       = (mwSize*)mxGetDimensions(w); //HARDCODED
        partfilterdims[i][1] = model->partfilters[i].SizeW[1];//6;//       = (mwSize*)mxGetDimensions(w); //HARDCODED
        partfilterdims[i][2] = model->partfilters[i].SizeW[0];;//       = (mwSize*)mxGetDimensions(w); //HARDCODED


        w = model->partfilters[i].wpca;//F(partinfo, "wpca");
        partfilters[1][i] = w;
    }

    for (int i = 0; i < numdefparams; i++) {
        defs[i]                 = model->defs[i].w;//Fpr(definfo, "w");
    }
    partorder  = new int*[numcomponents];
    offsets = new double[numcomponents];
    t = new double*[numcomponents];

    for (int i = 0; i < numcomponents; i++) {

        double *w      		  = model->rootfilters[i].w;

        numparts[i]           = 8;//mxGetDimensions(parts)[1];
        rootfilters[i]        = w;
        rootfilterdims[i][0]  = model->rootfilters[i].size[0]; //HARDCODED
        rootfilterdims[i][1]  = model->rootfilters[i].size[1]; //HARDCODED
        rootfilterdims[i][2]  = 32; //HARDCODED
        anchors[i]            = new double*[numparts[i]];
        pfind[i]              = new int[numparts[i]];
        defind[i]             = new int[numparts[i]];
        offsets[i]            = model->offsets[i].w;;
        partorder[i]          = new int[2*numparts[i]+2];
        double *ord           = model->cascade.order[i];//   mxGetPr(mxGetCell(orderinfo, i));
        t[i]                  = model->cascade.t[i];//mxGetPr(mxGetCell(mxt, i));

        for (int j = 0; j < numparts[i]; j++) {
            int dind                = model->components[i].parts[j].defindex-1;//(int)mxGetScalar(CF(parts, j, "defindex")) - 1;
            int pind                = model->components[i].parts[j].partindex-1;//(int)mxGetScalar(CF(parts, j, "partindex")) - 1;

            anchors[i][j] = new double[2];
            anchors[i][j][0] = model->defs[dind].anchor[0];
            anchors[i][j][1] = model->defs[dind].anchor[1];
            pfind[i][j] = pind;
            defind[i][j] = dind;
        }
        // subtract 1 so that non-root parts are zero-indexed
        for (int j = 0; j < 2*numparts[i]+2; j++) {
            partorder[i][j] = (int)ord[j] - 1;
        }
    }
}

void Model::initpyramid(const DPMPyramid &DPyr, const DPMPcaPyramid &PCAPyr) {
    numlevels    = DPyr.getNumLayers();
// cout << "numlevels = " << numlevels << endl;

    featdims     = new int*[numlevels];
    featdimsprod = new int[numlevels];
    feat[0]      = new const double*[numlevels];
    feat[1]      = new const double*[numlevels];
    for (int l = 0; l < numlevels; l++) {
        double *mxA  = DPyr.getLayer(l)->getFeatures();
        featdims[l] = new int[3];

        featdims[l][0] = DPyr.getLayer(l)->getHeight();
        featdims[l][1] = DPyr.getLayer(l)->getWidth();
        featdims[l][2] = DPyr.getLayer(l)->getDepth();


        featdimsprod[l]     = featdims[l][0]*featdims[l][1];

        feat[0][l]          =  mxA;
        mxA                 = PCAPyr.getLayer(l)->getFeatures();
        feat[1][l]          = mxA;
    }
    numfeatures = DPyr.getLayer(0)->getDepth();//mxGetDimensions(mxGetCell(pyramid, 0))[2];
    pcadim = PCAPyr.getLayer(0)->getDepth();//mxGetDimensions(mxGetCell(projpyramid, 0))[2];

}

Model::~Model() {
    for (int i = 0; i < numcomponents; i++) {
        delete [] partorder[i];
        for(int j=0; j<numparts[i]; j++)
            delete [] anchors[i][j];
        delete [] anchors[i];
        delete [] defind[i];
        delete [] pfind[i];
    }
    delete [] partorder;
    delete [] t;
    delete [] numparts;
    delete [] offsets;
    delete [] defind;
    delete [] pfind;
    delete [] anchors;
    delete [] defs;
    delete [] rootfilters;
    delete [] rootfilterdims;
    delete [] partfilters[0];
    delete [] partfilters[1];
    delete [] partfilterdims;
    for(int i=0; i<numlevels; i++)
        delete [] featdims[i];
    delete [] featdims;
    delete [] featdimsprod;
    delete [] feat[0];
    delete [] feat[1];
}
