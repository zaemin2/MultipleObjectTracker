//============================================================================
// Author      : F. De Smedt @ EAVISE
// Copyright   : This code is written for the publication of  "Open Framework for Combined Pedestrian detection". This code is for research and educational purposes only. For a different license, please contact the contributors directly. More information about the license can be fount in license.txt
//============================================================================



#include "NewCascadeModelOld.h"

void NewCascadeModelOld::initModel(){

	NewCascadeModelOld * model = this;

	MODEL = new ModelOld();

	MODEL->thresh        = model->thresh;
	MODEL->interval      = model->interval;
	MODEL->numcomponents = model->numcomponents;
	MODEL->sbin          = model->sbin;

	MODEL->numpartfilters            = model->numcomponents*NUMPARTS;

	MODEL->numdefparams              = model->numcomponents*NUMPARTS;

	MODEL->numparts        = new int[numcomponents];
	MODEL->anchors         = new float**[numcomponents];
	MODEL->defs            = new float*[MODEL->numdefparams];
	MODEL->rootfilters     = new float*[MODEL->numcomponents];
	MODEL->partfilters[0]  = new float*[MODEL->numpartfilters];
	MODEL->partfilters[1]  = new float*[MODEL->numpartfilters];
	MODEL->rootfilterdims  = new int[MODEL->numcomponents][3];
	MODEL->partfilterdims  = new int[MODEL->numpartfilters][3];
	MODEL->pfind           = new int*[MODEL->numcomponents];
	MODEL->defind          = new int*[MODEL->numcomponents];


	  for (int i = 0; i < MODEL->numpartfilters; i++) {
	    float *w        = model->partfilters[i].w;//F(partinfo, "w");
	    MODEL->partfilters[0][i]       = w;

	    MODEL->partfilterdims[i][0] = SIZEPARTFILTER;
	    MODEL->partfilterdims[i][1] = SIZEPARTFILTER;
	    MODEL->partfilterdims[i][2] = FULL;


	   w = model->partfilters[i].wpca;
	   MODEL->partfilters[1][i] = w;
	  }

	  for (int i = 0; i < MODEL->numdefparams; i++) {
		  MODEL->defs[i]                 = model->defs[i].w;
	  }
	  MODEL->partorder  = new int*[numcomponents];
	  MODEL->offsets = new float[numcomponents];
	  MODEL->t = new float*[numcomponents];

	  for (int i = 0; i < numcomponents; i++) {

	    float *w      		  = model->rootfilters[i].w;

	    MODEL->numparts[i]           = 8;
	    MODEL->rootfilters[i]        = w;

	    MODEL->rootfilterdims[i][0]  = model->rootfilters[i].size[0];
	    MODEL->rootfilterdims[i][1]  = model->rootfilters[i].size[1];
	    MODEL->rootfilterdims[i][2]  = 32;
	    MODEL->anchors[i]            = new float*[MODEL->numparts[i]];
	    MODEL->pfind[i]              = new int[MODEL->numparts[i]];
	    MODEL->defind[i]             = new int[MODEL->numparts[i]];
	    MODEL->offsets[i]            = model->offsets[i].w;
	    MODEL->partorder[i]          = new int[2*MODEL->numparts[i]+2];
	    int *ord              = model->cascade.order[i].order;
	    MODEL->t[i]                  = model->cascade.t[i].t;

	    for (int j = 0; j < MODEL->numparts[i]; j++) {
	      int dind                = model->components[i].parts[j].defindex-1;
	      int pind                = model->components[i].parts[j].partindex-1;


	      MODEL->anchors[i][j] = new float[2];
	      MODEL->anchors[i][j][0] = model->defs[dind].anchor[0];
	      MODEL->anchors[i][j][1] = model->defs[dind].anchor[1];
	      MODEL->pfind[i][j] = pind;
	      MODEL->defind[i][j] = dind;
	    }
	    // subtract 1 so that non-root parts are zero-indexed

	    for (int j = 0; j < 2*MODEL->numparts[i]+2; j++){
	    	MODEL->partorder[i][j] = (int)ord[j] - 1;
	     }
	  }

	std::cout << "Initialised " << std::endl;
}




