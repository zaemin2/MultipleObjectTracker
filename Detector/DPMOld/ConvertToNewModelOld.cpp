//============================================================================
// Author      : F. De Smedt @ EAVISE
// Copyright   : This code is written for the publication of  "Open Framework for Combined Pedestrian detection". This code is for research and educational purposes only. For a different license, please contact the contributors directly. More information about the license can be fount in license.txt
//============================================================================

#include <iostream>

#include "NewCascadeModelOld.h"
#include "CascadeModelOld.h"

void PrintModelOld(NewCascadeModelOld *model){
	std::cout << "Printing the model: " << std::endl;
//COEFF
for(int row=0;row<5;row++){

for(int col=0;col<5;col++){
	std::cout << model->coeff[col*32+row] << "\t";
}
std::cout << std::endl;
}

std::cout << "ROOTFILTERS W" << std::endl;
//Rootfilters
for(int f=0;f<32;f++){
std::cout << "FeatureLayer: " << f << std::endl;
for(int row=0;row<5;row++){

for(int col=0;col<5;col++){
	std::cout << model->rootfilters[1].w[f*15*5+col*15+row] << "\t";
}
std::cout << std::endl;
}
std::cout << std::endl << std::endl;
}
std::cout << "ROOTFILTERS W" << std::endl;
//Rootfilters
for(int f=0;f<6;f++){
std::cout << "FeatureLayer: " << f << std::endl;
for(int row=0;row<5;row++){

for(int col=0;col<5;col++){
	std::cout << model->rootfilters[1].wpca[f*15*5+col*15+row] << "\t";
}
std::cout << std::endl;
}
std::cout << std::endl << std::endl;
}


std::cout << "PARTFILTERS WPCA" << std::endl;
//Rootfilters
for(int f=0;f<6;f++){
std::cout << "FeatureLayer: " << f << std::endl;
for(int row=0;row<5;row++){

for(int col=0;col<5;col++){
        std::cout << model->partfilters[1].wpca[f*6*6+col*6+row] << "\t";
}
std::cout << std::endl;
}
std::cout << std::endl << std::endl;
}

//OFFSETS

        std::cout << model->offsets[1].w << std::endl;
        for(int I=0;I<4;I++)
		std::cout << model->defs[0].w[I] << "\t";

	std::cout << std::endl;


//cascade
for(int I=0;I<18;I++)
        std::cout << model->cascade.order[1].order[I] << "\t";
std::cout << std::endl;


for(int I=0;I<36;I++)
        std::cout << model->cascade.t[0].t[I] << "\t";
std::cout << std::endl;


//END
}


NewCascadeModelOld::~NewCascadeModelOld(){
	std::cout << "Deleting the new model" << std::endl;

	for(int i=0;i<numcomponents;i++){
		RootfilterOld* filter = &(rootfilters[i]);
		delete [] filter->w;
		delete [] filter->wpca;
	}

	delete []rootfilters;
	
	delete []offsets;
	delete []components;

	for(int i=0;i<numcomponents*NUMPARTS;i++){
		PartfilterOld* filter = &(partfilters[i]);
		delete []filter->w;
		delete []filter->wpca;
	}
	delete []partfilters;
	delete []defs;	

	delete []cascade.order;
	delete []cascade.t;
}

// Convert to the new model-format which allows faster convolution
void ConvertToNewModelOld(CascadeModelOld *Model, NewCascadeModelOld &NModel){

//fill the easy ones
NModel.sbin = Model->getSbin();

NModel.thresh = Model->getThreshold();
NModel.maxsize[0] = Model->getMaxSizeY();
NModel.maxsize[1] = Model->getMaxSizeX();


NModel.minsize[0] = Model->getMinSizeY();
NModel.minsize[1] = Model->getMinSizeX();

NModel.interval = Model->getInterval();
NModel.numblocks = Model->numblocks;
NModel.Name = Model->getName();
NModel.numcomponents = Model->getNumcomponents();
NModel.year = Model->year;
NModel.note = Model->note;

//coeff
for(int y=0; y<FULL; y++) {
            for(int x=0; x<PCA; x++) {
                NModel.coeff[x*FULL+y] = Model->coeff(y,x);
            }
        }

NModel.rootfilters = new RootfilterOld[NModel.numcomponents];
for(int i=0;i<NModel.numcomponents;i++){
	RootfilterOld* filter = &(NModel.rootfilters[i]);
	
	filter->size[0] = Model->rootfilters(0,i).size.first;
	filter->size[1] = Model->rootfilters(0,i).size.second;

	filter->w = new DECTYPE[filter->size[0]*filter->size[1]*FULL];	

	for(int I=0; I<FULL; I++) {
	int rows = filter->size[0];
	int cols = filter->size[1];
          for(int y=0; y<rows; y++) {
                 for(int x2=0; x2<cols; x2++) {
                     filter->w[I*rows*cols+x2*rows+y] = Model->rootfilters(0,i).w(I)(y,x2);
                 }
             }
         }

	filter->wpca = new DECTYPE[filter->size[0]*filter->size[1]*PCA];	
	for(int I=0; I<PCA; I++) {
	int rows = filter->size[0];
	int cols = filter->size[1];
          for(int y=0; y<rows; y++) {
                 for(int x2=0; x2<cols; x2++) {
                     filter->wpca[I*rows*cols+x2*rows+y] = Model->rootfilters(0,i).wpca(I)(y,x2);
                 }
             }
         }



	filter->blocklabel = Model->rootfilters(0,i).blocklabel;
}

NModel.offsets = new OffsetOld[NModel.numcomponents];
for(int i=0;i<NModel.numcomponents;i++){
	OffsetOld* oset = &(NModel.offsets[i]);

	oset->w = Model->offsets(0,i).w;
	oset->blocklabel = Model->offsets(0,i).blocklabel;
}

NModel.components = new ComponentOld[NModel.numcomponents];
for(int i=0;i<NModel.numcomponents;i++){
	ComponentOld* comp = &(NModel.components[i]);

	comp->rootindex = Model->components(0,i).rootindex;
	comp->offsetindex = Model->components(0,i).offsetindex;

	for(int p=0;p<NUMPARTS;p++){
		PartOld *pa = &(comp->parts[p]);

		pa->partindex = Model->components(0,i).Parts(0,p).partindex;
		pa->defindex = Model->components(0,i).Parts(0,p).defindex;
	}	
}

NModel.partfilters = new PartfilterOld[NModel.numcomponents*NUMPARTS];
for(int i=0;i<NModel.numcomponents*NUMPARTS;i++){
	PartfilterOld *filter = &(NModel.partfilters[i]);

	filter->w = new DECTYPE[SIZEPARTFILTER*SIZEPARTFILTER*FULL];
	for(int I=0; I<FULL; I++) {
        int rows = SIZEPARTFILTER;
        int cols = SIZEPARTFILTER;
         for(int y=0; y<rows; y++) {
                 for(int x2=0; x2<cols; x2++) {
                     filter->w[I*rows*cols+x2*rows+y] = Model->partfilters(0,i).w(I)(y,x2);
                 }
             }
         }

	filter->wpca = new DECTYPE[SIZEPARTFILTER*SIZEPARTFILTER*PCA];
	for(int I=0; I<PCA; I++) {
        int rows = SIZEPARTFILTER;
        int cols = SIZEPARTFILTER;
          for(int y=0; y<rows; y++) {
                 for(int x2=0; x2<cols; x2++) {
                     filter->wpca[I*rows*cols+x2*rows+y] = Model->partfilters(0,i).wpca(I)(y,x2);
                 }
             }
         }



	filter->blocklabel = Model->partfilters(0,i).blocklabel;
}

NModel.defs = new DefOld[NModel.numcomponents*NUMPARTS]; 
for(int i=0;i<NModel.numcomponents*NUMPARTS;i++){
	DefOld *d = &(NModel.defs[i]);

	d->blocklabel = Model->defs(0,i).blocklabel;
	d->anchor[0] = Model->defs(0,i).anchor.first;
	d->anchor[1] = Model->defs(0,i).anchor.second;

	for(int x2=0; x2<4; x2++) {
                d->w[x2] = Model->defs(0,i).w(0,x2);
	}

}

//cascade
NModel.cascade.thresh = Model->cascade.thresh;

NModel.cascade.order = new OrderOld[NModel.numcomponents];

for(int i=0;i<NModel.numcomponents;i++){
	OrderOld *o = &(NModel.cascade.order[i]);
        for(int x3=0; x3<(1+NUMPARTS)*2; x3++) {
               o->order[x3] =  Model->cascade.order(0,i)(x3);
        }
}


NModel.cascade.t = new TOld[NModel.numcomponents];

for(int i=0;i<NModel.numcomponents;i++){
	TOld *t = &(NModel.cascade.t[i]);
	for(int x3=0; x3<2*(2*(NUMPARTS+1)); x3++) {
               t->t[x3] = Model->cascade.t(0,i)(x3);
        }	
}


}
