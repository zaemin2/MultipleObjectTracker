//============================================================================
// Author      : F. De Smedt @ EAVISE
// Copyright   : This code is written for the publication of  "Open Framework for Combined Pedestrian detection". This code is for research and educational purposes only. For a different license, please contact the contributors directly. More information about the license can be fount in license.txt
//============================================================================



#ifndef PREMODEL_H
#define PREMODEL_H

#include <stdio.h>
#include "PyramidOld.h"

class VolatileModelOld{
public:

    int pcadim;
    // dimenions of the HOG features
    int numfeatures;
    // component indexed array of root filters
    // feature pyramid data
    int numlevels;
    // dimensions of each feature pyramid level
    int **featdims;
    // number of positions in each feature pyramid level
    int *featdimsprod;
    // feature pyramid levels
    // feat[0] holds non-PCA HOG features
    // feat[1] holds PCA of HOG features
    float **feat[2];

    VolatileModelOld(PyramidOld *pyra){
    	 numlevels    = pyra->lenghtFeatures;
    	  featdims     = new int*[numlevels];
    	  featdimsprod = new int[numlevels];
    	  feat[0]      = new float*[numlevels];
    	  feat[1]      = new float*[numlevels];
    	  for (int l = 0; l < numlevels; l++) {
    	    float *mxA  = pyra->featold[l];
    	    featdims[l] = new int[3];
    	    featdims[l][0] = pyra->featsizesold[l][0];
    	    featdims[l][1] = pyra->featsizesold[l][1];
    	    featdims[l][2] = pyra->featsizesold[l][2];

    	    featdimsprod[l]     = featdims[l][0]*featdims[l][1];

    	    feat[0][l]          =  mxA;
    	    // projected pyramid
    	    mxA                 = pyra->feat[l];
    	    feat[1][l]          = mxA;
    	  }
    	  numfeatures = pyra->featsizesold[0][2];//mxGetDimensions(mxGetCell(pyramid, 0))[2];
    	  pcadim = pyra->featsizes[0][2];//mxGetDimensions(mxGetCell(projpyramid, 0))[2];

    }
    ~VolatileModelOld(){
    	  for(int i=0;i<numlevels;i++)
    		delete [] featdims[i];
    	  delete [] featdims;
    	  delete [] featdimsprod;
    	  delete [] feat[0];
    	  delete [] feat[1];
    }
};

class ModelOld
{
public:
    // model data

    // size of HOG feature cell (e.g., 8 pixels)
    int sbin;
    // number of dimensions used for the PCA filter projection
  //  int pcadim;
    // dimenions of the HOG features
  //  int numfeatures;
    // component indexed array of root filters
    float **rootfilters;
    // sizes of root filters
    int (*rootfilterdims)[3];
    // array of arrays of part filters
    // partfilters[0] holds non-PCA filters
    // partfilters[1] holds PCA filters
    float **partfilters[2];
    // dimensions of part filters
    int (*partfilterdims)[3];
    // component indexed offset (a.k.a. bias) values
    float *offsets;
    // number of components in the model
    int numcomponents;
    // number of parts per component
    int *numparts;
    // global detection threshold
    float thresh;
    // component indexed arrays of part orderings
    int **partorder;
    // component indexed arrays of pruning thresholds
    float **t;
    // ideal relative positions for each deformation model
    float ***anchors;
    // array of deformation models
    float **defs;

    // maps from (component,part#) -> part filter or deformation model index
    // this enables supporting models with parts and deformation models that are
    // shared between components (not currently used)

    // map: pfind[component][part#] => part filter index
    int **pfind;
    // map: defind[component][part#] => def param index
    int **defind;

    // pooled part filter and def model counts
    int numpartfilters;
    int numdefparams;

    // number of levels per octave in feature pyramid
    int interval;

    // root PCA filter score + offset (stage 0 computed in cascade_detect.m)
    int numrootlocs;

    ModelOld(){};

    ~ModelOld();

};

#endif
