/*LoadCascadeModel.cpp*/
#include "CModel.h"
#include <iostream>
#include <stdlib.h>


#include <fstream>


using namespace std;

CModel LoadCascadeModel(double thresh, std::string filename)
{
    ifstream infile;
    infile.open(filename.c_str(), ios::binary | ios::in);

    double *t;
    CModel C;

    C.thresh = thresh;


    //sbin
    infile.read(reinterpret_cast<char*>(&C.sbin),sizeof(C.sbin));

    std::cout << "Sbin: " << C.sbin << std::endl;

    // numcomponents
    infile.read(reinterpret_cast<char*>(&C.numcomponents),sizeof(C.numcomponents));
    // numpartfilters
    infile.read(reinterpret_cast<char*>(&C.SizePartfilters),sizeof(C.SizePartfilters));

    C.SizeDefs = C.SizePartfilters;

    //Size coeff
    infile.read(reinterpret_cast<char*>(&C.SizeCoeff[0]),sizeof(int));
    infile.read(reinterpret_cast<char*>(&C.SizeCoeff[1]),sizeof(int));

    //size bbox
    infile.read(reinterpret_cast<char*>(&C.SizeBboxpred),sizeof(int));


    infile.read(reinterpret_cast<char*>(&C.maxsize[0]),sizeof(int));
    infile.read(reinterpret_cast<char*>(&C.maxsize[1]),sizeof(int));
    infile.read(reinterpret_cast<char*>(&C.minsize[0]),sizeof(int));
    infile.read(reinterpret_cast<char*>(&C.minsize[1]),sizeof(int));



    infile.read(reinterpret_cast<char*>(&C.interval),sizeof(int));


    infile.read(reinterpret_cast<char*>(&C.numblocks),sizeof(int));


    //Cascade sizes
    infile.read(reinterpret_cast<char*>(&C.cascade.SizeOrder[0]),sizeof(int));
    infile.read(reinterpret_cast<char*>(&C.cascade.SizeOrder[1]),sizeof(int));

    infile.read(reinterpret_cast<char*>(&C.cascade.SizeT[0]),sizeof(int));
    infile.read(reinterpret_cast<char*>(&C.cascade.SizeT[1]),sizeof(int));

    C.coeff = (double*) malloc(sizeof(double)*C.SizeCoeff[0]*C.SizeCoeff[1]);
    infile.read(reinterpret_cast<char*>(C.coeff),sizeof(double)*C.SizeCoeff[0]*C.SizeCoeff[1]);

    C.bboxpred = (BoxPred*) malloc(sizeof(BoxPred)*C.SizeBboxpred);

    for(int b=0; b<C.SizeBboxpred; b++) {
        infile.read(reinterpret_cast<char*>(C.bboxpred[b].x1),sizeof(double)*19);
        infile.read(reinterpret_cast<char*>(C.bboxpred[b].y1),sizeof(double)*19);
        infile.read(reinterpret_cast<char*>(C.bboxpred[b].x2),sizeof(double)*19);
        infile.read(reinterpret_cast<char*>(C.bboxpred[b].y2),sizeof(double)*19);
    }

    std::cout << "numcomponents: " << C.numcomponents << std::endl;

    //Rootfilters
    C.rootfilters = (Rootfilter*) malloc(sizeof(Rootfilter)*C.numcomponents);

    for(int r=0; r<C.numcomponents; r++) {
        infile.read(reinterpret_cast<char*>(&(C.rootfilters[r].size[0])),sizeof(int));
        infile.read(reinterpret_cast<char*>(&(C.rootfilters[r].size[1])),sizeof(int));

        C.rootfilters[r].w = (double*) malloc(sizeof(double)*C.rootfilters[r].size[0]*C.rootfilters[r].size[1]*32);
        t = C.rootfilters[r].w;
        infile.read(reinterpret_cast<char*>(t),sizeof(double)*C.rootfilters[r].size[0]*C.rootfilters[r].size[1]*32);

        C.rootfilters[r].wpca = (double*) malloc(sizeof(double)*C.rootfilters[r].size[0]*C.rootfilters[r].size[1]*6);
        t = C.rootfilters[r].wpca;
        infile.read(reinterpret_cast<char*>(t),sizeof(double)*C.rootfilters[r].size[0]*C.rootfilters[r].size[1]*6);

        infile.read(reinterpret_cast<char*>(&C.rootfilters[r].blocklabel),sizeof(int));
    }

    //Offsets
    C.offsets = (Offset*) malloc(sizeof(Offset)*C.numcomponents);
    for(int r=0; r<C.numcomponents; r++) {

        infile.read(reinterpret_cast<char*>(&(C.offsets[r].w)),sizeof(double));
        infile.read(reinterpret_cast<char*>(&(C.offsets[r].blocklabel)),sizeof(int));
    }



    C.components = (Component*) malloc(sizeof(Component)*C.numcomponents);
    for(int r=0; r<C.numcomponents; r++) {
        infile.read(reinterpret_cast<char*>(&(C.components[r].rootindex)),sizeof(int));
        infile.read(reinterpret_cast<char*>(&(C.components[r].offsetindex)),sizeof(int));
        int numparts = C.SizePartfilters/C.numcomponents;
        for(int p=0; p<numparts; p++) {
            infile.read(reinterpret_cast<char*>(&(C.components[r].parts[p].partindex)),sizeof(int));
            infile.read(reinterpret_cast<char*>(&(C.components[r].parts[p].defindex)),sizeof(int));
        }
    }

    // Partfilters
    C.partfilters = (Partfilter*)malloc(sizeof(Partfilter)*C.SizePartfilters);
    for(int p=0; p<C.SizePartfilters; p++) {
        infile.read(reinterpret_cast<char*>(&(C.partfilters[p].blocklabel)),sizeof(int));
        infile.read(reinterpret_cast<char*>(&(C.partfilters[p].SizeW[0])),sizeof(int));
        infile.read(reinterpret_cast<char*>(&(C.partfilters[p].SizeW[1])),sizeof(int));
        infile.read(reinterpret_cast<char*>(&(C.partfilters[p].SizeW[2])),sizeof(int));

        int S = C.partfilters[p].SizeW[0]*C.partfilters[p].SizeW[1]*C.partfilters[p].SizeW[2];

        C.partfilters[p].w = (double*) malloc(sizeof(double)*S);
        infile.read(reinterpret_cast<char*>(C.partfilters[p].w),sizeof(double)*S);

        infile.read(reinterpret_cast<char*>(&(C.partfilters[p].SizeWpca[0])),sizeof(int));
        infile.read(reinterpret_cast<char*>(&(C.partfilters[p].SizeWpca[1])),sizeof(int));
        infile.read(reinterpret_cast<char*>(&(C.partfilters[p].SizeWpca[2])),sizeof(int));

        S = C.partfilters[p].SizeWpca[0]*C.partfilters[p].SizeWpca[1]*C.partfilters[p].SizeWpca[2];

        C.partfilters[p].wpca = (double*) malloc(sizeof(double)*S);
        infile.read(reinterpret_cast<char*>(C.partfilters[p].wpca),sizeof(double)*S);
    }


    //Defs
    C.defs = (Def*) malloc(sizeof(Def)*C.SizePartfilters);
    for(int p=0; p<C.SizePartfilters; p++) {
        infile.read(reinterpret_cast<char*>(&(C.defs[p].blocklabel)),sizeof(int));
        infile.read(reinterpret_cast<char*>(C.defs[p].w),sizeof(double)*4);
        infile.read(reinterpret_cast<char*>(C.defs[p].anchor),2*sizeof(int));
    }



    //Cascade
    C.cascade.order = (double**)malloc(sizeof(double*)*C.numcomponents);
    C.cascade.t = (double**)malloc(sizeof(double*)*C.numcomponents);

    for(int i=0; i<C.numcomponents; i++) {
        C.cascade.order[i] = (double*)malloc(sizeof(double)*18);
        infile.read(reinterpret_cast<char*>(C.cascade.order[i]),sizeof(double)*18);
    }

    for(int i=0; i<C.numcomponents; i++) {
        C.cascade.t[i] = (double*)malloc(sizeof(double)*36);
        infile.read(reinterpret_cast<char*>(C.cascade.t[i]),sizeof(double)*36);
    }

    for(int i=0; i<18; i++) {
        std::cout << C.cascade.order[0][i] << std::endl;
    }


    for(int i=0; i<36; i++) {
        std::cout << C.cascade.t[0][i] << std::endl;
    }

    return C;
}
