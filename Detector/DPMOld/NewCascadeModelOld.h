//============================================================================
// Author      : F. De Smedt @ EAVISE
// Copyright   : This code is written for the publication of  "Open Framework for Combined Pedestrian detection". This code is for research and educational purposes only. For a different license, please contact the contributors directly. More information about the license can be fount in license.txt
//============================================================================



#ifndef __NCASCADEMODEL
#define __NCASCADEMODEL

#include <iostream>
#include <string>

#include "modelOld.h"

#define DECTYPE float
#define FULL 32
#define PCA 6

#define NUMPARTS 8
#define SIZEPARTFILTER 6


//class Model;


struct BBoxpredOld{
	DECTYPE x1[19];
	DECTYPE y1[19];
	DECTYPE x2[19];
	DECTYPE y2[19];
};

struct RootfilterOld{
	int size[2];
	DECTYPE *w; // size[0]*size[1]*FULL
	DECTYPE *wpca; // size[0]*size[1]*PCA
	int blocklabel;
};

struct OffsetOld{
	DECTYPE w;
	int blocklabel;
};

struct PartOld{
	int partindex;
	int defindex;
};


struct ComponentOld{
	int rootindex;
	int offsetindex;
	PartOld parts[NUMPARTS];
};
struct PartfilterOld{
	DECTYPE *w;
	DECTYPE *wpca;
	int blocklabel;

};


struct DefOld{
	DECTYPE w[4];
	int blocklabel;
	int anchor[2];
};

struct OrderOld{
	int order[2*(NUMPARTS+1)*2]; // (rootmodel + NUMPARTS)*(2) -> 1 PCA and 1 full 
};

struct TOld{
	DECTYPE t[2*(NUMPARTS+1)*2];
};

struct CascadeOld{
	DECTYPE thresh;
	OrderOld *order; // (rootmodel + NUMPARTS)*(2) -> 1 PCA and 1 full 
	TOld *t; // 18 from order * 2 -> 1 deformation threshold and 1 score threshold
};



class NewCascadeModelOld{

public:


~NewCascadeModelOld();

int sbin;
DECTYPE thresh;
int maxsize[2];
int minsize[2];

int interval;

int numblocks;
std::string Name;

int numcomponents;


DECTYPE coeff[FULL*PCA];

std::string year;
std::string note;

BBoxpredOld *bboxpred; //numcomponents x 1
RootfilterOld *rootfilters; //1xnumcomponents
OffsetOld *offsets; // 1 x numcomponents
ComponentOld *components; // 1xnumcomponents
PartfilterOld *partfilters; // 1 x SIZEPARTFILTER*NUMPARTS
DefOld *defs; // 1 x SIZEPARTFILTER*NUMPARTS
CascadeOld cascade;

ModelOld *MODEL;

void initModel();

};



#endif
