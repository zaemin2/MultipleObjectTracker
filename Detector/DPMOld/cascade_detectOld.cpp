//============================================================================
// Author      : F. De Smedt @ EAVISE
// Copyright   : This code is written for the publication of  "Open Framework for Combined Pedestrian detection". This code is for research and educational purposes only. For a different license, please contact the contributors directly. More information about the license can be fount in license.txt
//============================================================================

/*cascade_detect.cpp*/
#include <vector>
#include <iostream>
#include "projectpyramideOld.h"
#include "fconvOld.h"
#include "cascadeOld.h"
//#include "CascadeModel.h"
#include "NewCascadeModelOld.h"

using namespace std;

int cascade_detectOld(PyramidOld *pyra, float thresh, vector<float> &dets, vector<float> &boxes, NewCascadeModelOld *NModel,const CascadeArrays_NonVolatileOld &NonVolatile) {

NewCascadeModelOld *model = NModel;
    int numrootfilters = NModel->numcomponents;
    //float* rootfilters[numrootfilters];
	float** rootfilters = new float*[numrootfilters];
    for(int i=0; i<numrootfilters; i++)
    {
        int S[2] = {model->rootfilters[i].size[0], model->rootfilters[i].size[1]};
        rootfilters[i] = (float*)malloc(sizeof(float)*S[0]*S[1]*6);
        /*fill rootfilters with data of the model*/
        memcpy(rootfilters[i],model->rootfilters[i].wpca,sizeof(float)*S[0]*S[1]*6);
    }


    projectpyramideOld(pyra,NModel);


    int numrootlocs = 0;
    int nlevels = pyra->lenghtFeatures;
    float***  rootscores = (float***)malloc(sizeof(float*)*NModel->numcomponents);
    int ***  rootscoresdims = (int***)malloc(sizeof(int*)*NModel->numcomponents);

    for(int Q=0; Q<NModel->numcomponents; Q++) {
        rootscores[Q] = (float**)malloc(sizeof(float*)*nlevels);
        rootscoresdims[Q] = (int**)malloc(sizeof(int*)*nlevels);
    }

    int s=0;
    for(int i=0; i<nlevels; i++) {
        s = s + pyra->featsizes[i][0]*pyra->featsizes[i][1];
        //int SizeC[NModel->numcomponents][2];
		int **SizeC = new int*[NModel->numcomponents];
		for (int i = 0; i < NModel->numcomponents; i++) {
			SizeC[i] = new int[2];
		}

        if(i >= NModel->interval) {

            float **scores = fconvOld(pyra->feat[i],rootfilters,1,numrootfilters,pyra->featsizes[i][1],pyra->featsizes[i][0],pyra->featsizes[i][2],model->rootfilters,SizeC);


            for(int c=0; c<NModel->numcomponents; c++) {
                //int u = Model.components(0,c).rootindex-1;
                int u = NModel->components[c].rootindex-1;
                //int v = Model.components(0,c).offsetindex-1;
                int v = NModel->components[c].offsetindex-1;
                rootscores[c][i] = (float* )malloc(SizeC[c][0]*SizeC[c][1]*sizeof(float));
                rootscoresdims[c][i] = (int*)malloc(sizeof(int)*2);
                rootscoresdims[c][i][0] = SizeC[c][0];
                rootscoresdims[c][i][1] = SizeC[c][1];

                for(int Q=0; Q<SizeC[c][0]*SizeC[c][1]; Q++) {
                    rootscores[c][i][Q] = scores[u][Q] + NModel->offsets[v].w;
                }
                numrootlocs = numrootlocs + SizeC[c][0]*SizeC[c][1];
            }

            /*Free scores*/
            for(int Q=0; Q<(numrootfilters); Q++) {
                free(scores[Q]);
            }
            free(scores);

        }
    }

//free the rootfilters
for(int i=0; i<numrootfilters; i++)
    {
        free(rootfilters[i]);
    }


    s = s*NModel->numcomponents*NUMPARTS;


    NModel->thresh = thresh;

	int Width = cascadeOld(NModel,pyra,rootscores,pyra->padx,pyra->pady,s,rootscoresdims,boxes,NonVolatile);


    for(int Q=0; Q<boxes.size()/Width; Q++) {
        dets.push_back(boxes[Q*Width]);
        dets.push_back(boxes[Q*Width+1]);
        dets.push_back(boxes[Q*Width+2]);
        dets.push_back(boxes[Q*Width+3]);
        dets.push_back(boxes[Q*Width+Width-2]);
        dets.push_back(boxes[Q*Width+Width-1]);
    }


    /*Free rootscores and rootscoresdims*/
    for(int i=0; i<NModel->numcomponents; i++) {
        for(int j=NModel->interval; j<pyra->lenghtFeatures; j++) {
            free(rootscores[i][j]); //???
            free(rootscoresdims[i][j]); //???
        }
        free(rootscores[i]);
        free(rootscoresdims[i]);
    }
    free(rootscores);
    free(rootscoresdims);
    /*Free Feature Pyramid*/
    for(int i=0; i<pyra->lenghtFeatures; i++) {
        free(pyra->featold[i]);
        free(pyra->featsizesold[i]);
        free(pyra->feat[i]);
        free(pyra->featsizes[i]);
    }
    free(pyra->featold);
    free(pyra->feat);
    free(pyra->featsizes);
    free(pyra->featsizesold);
    free(pyra->scales);
    return Width;
}
