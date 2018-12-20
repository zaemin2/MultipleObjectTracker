/*CModel.h*/

#ifndef __CModel
#define __CModel

#include <string>
#include <cmath>


using namespace std;


struct BoxPred
{
    double x1[19];
    double y1[19];
    double x2[19];
    double y2[19];
};

struct Offset
{
    double w;
    int blocklabel;
};

struct Rootfilter
{
    int size[2];
    double* w; //double[size[0]][size[1]][32]
    double* wpca; //double[size[0]*][size[1]][6]
    int blocklabel;
};

struct Part
{
    int partindex;
    int defindex;
};

struct Component
{
    int rootindex;
    int offsetindex;
    Part parts[8];
};

struct Partfilter
{
    int SizeW[3];
    int SizeWpca[3];

    double *w;
    double *wpca;
    int blocklabel;
};

struct Def
{
    double w[4];
    int blocklabel;
    int anchor[2];
};

struct Cascade
{
    int SizeOrder[2];
    int SizeT[2];
    double **order;
    double **t;
    double thresh;

};

struct CModel
{
    int sbin;
    double thresh;
    int maxsize[2];
    int minsize[2];
    int interval;
    int numblocks;
    string klasse;
    int numcomponents;
    double *coeff;
    string year;
    string note;

    int SizeBboxpred;
    int SizePartfilters;
    int SizeDefs;
    int SizeCoeff[2];

    BoxPred *bboxpred;
    Rootfilter *rootfilters;
    Offset *offsets;
    Component *components;
    Partfilter *partfilters;
    Def *defs;

    Cascade cascade;


    int getPadx() const {
        return std::ceil(maxsize[1]);
    }

    int getPady() const {
        return std::ceil(maxsize[0]);
    }

    int getInterval() const {
        return this->interval;
    }

};

#endif
