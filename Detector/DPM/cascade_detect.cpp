/*cascade_detect.cpp*/
#include <vector>
#include <iostream>
#include "DPMFunctions.h"
#include "DPMPCAPyramid.h"

using namespace std;

int cascade_detect(const CModel *model,double thresh,vector<double> &dets,vector<double> &boxes, const DPMPyramid &DPyr) {

    int numrootfilters = model->numcomponents;
    //double* rootfilters[numrootfilters];
	double** rootfilters = new double*[numrootfilters];
    for(int i=0; i<numrootfilters; i++) {
        int S[2] = {model->rootfilters[i].size[0], model->rootfilters[i].size[1]};

        rootfilters[i] = (double*)malloc(sizeof(double)*S[0]*S[1]*6);
        memcpy(rootfilters[i],model->rootfilters[i].wpca,sizeof(double)*S[0]*S[1]*6);
    }

    /*Create the PCA-features, which are dependent on the model to evaluate*/
    DPMPcaPyramid PCAPyr(DPyr,model);

    int numrootlocs = 0;
    int nlevels = DPyr.getNumLayers();//->lenghtFeatures;
    //int nlevels = pyra->lenghtFeatures;
    double***  rootscores = (double***)malloc(sizeof(double*)*model->numcomponents);
    int ***  rootscoresdims = (int***)malloc(sizeof(int*)*model->numcomponents);


    for(int Q=0; Q<model->numcomponents; Q++) {
        rootscores[Q] = (double**)malloc(sizeof(double*)*nlevels);
        rootscoresdims[Q] = (int**)malloc(sizeof(int*)*nlevels);
    }


    int s=0;
    for(int i=0; i<nlevels; i++) {
        s = s + PCAPyr.getLayer(i)->getWidth()*PCAPyr.getLayer(i)->getHeight();
        //int SizeC[model->numcomponents][2];
		int **SizeC = new int*[model->numcomponents];
		for (int i = 0; i < model->numcomponents; i++) {
			SizeC[i] = new int[2];
		}

        if(i >= DPyr.getInterval()) {
            double **scores = fconv(PCAPyr.getLayer(i)->getFeatures(),rootfilters,1,numrootfilters,PCAPyr.getLayer(i)->getWidth(),PCAPyr.getLayer(i)->getHeight(),PCAPyr.getLayer(i)->getDepth(),model->rootfilters,SizeC);


            for(int c=0; c<model->numcomponents; c++) {
                int u = model->components[c].rootindex-1;
                int v = model->components[c].offsetindex-1;
                rootscores[c][i] = (double* )malloc(SizeC[c][0]*SizeC[c][1]*sizeof(double));
                rootscoresdims[c][i] = (int*)malloc(sizeof(int)*2);
                rootscoresdims[c][i][0] = SizeC[c][0];
                rootscoresdims[c][i][1] = SizeC[c][1];

                for(int Q=0; Q<SizeC[c][0]*SizeC[c][1]; Q++) {
                    rootscores[c][i][Q] = scores[u][Q] + model->offsets[v].w;
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

    s = s*model->SizePartfilters; //lenght model->partfilters
    //model->thresh = thresh;

    int Width = cascade(model,rootscores,numrootlocs,DPyr.getPadx(),DPyr.getPady(),s,rootscoresdims,boxes,DPyr, PCAPyr);

    for(int Q=0; Q<boxes.size()/Width; Q++) {
        dets.push_back(boxes[Q*Width]);
        dets.push_back(boxes[Q*Width+1]);
        dets.push_back(boxes[Q*Width+2]);
        dets.push_back(boxes[Q*Width+3]);
        dets.push_back(boxes[Q*Width+Width-2]);
        dets.push_back(boxes[Q*Width+Width-1]);
    }

    /*Free dynamic memory*/
    /*Free rootfilters*/
    for(int i=0; i<numrootfilters; i++) {
        free(rootfilters[i]);
    }
    /*Free rootscores and rootscoresdims*/
    for(int i=0; i<model->numcomponents; i++) {
        for(int j=DPyr.getInterval(); j<DPyr.getNumLayers(); j++) {
            free(rootscores[i][j]); //???
            free(rootscoresdims[i][j]); //???
        }
        free(rootscores[i]);
        free(rootscoresdims[i]);
    }
    free(rootscores);
    free(rootscoresdims);

    return Width;
}
