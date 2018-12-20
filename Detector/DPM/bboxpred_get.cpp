#include <iostream>
#include <string>
#include <vector>
//#include "cv.h"
//#include "highgui.h"
#include "CModel.h"
int bboxpred_input(vector<double> dets, vector<double> boxes, vector<int> cinds, vector<double> &A,int W);

using namespace std;
void bboxpred_get(vector<double> &bbox, vector<double> &parts,const CModel *M,vector<double> dets, vector<double> boxes,int W) {

    bbox.clear();
    parts.clear();
    vector<int> cinds;
    vector<double> A;
    double dx1,dx2,dy1,dy2;
    double x1,x2,y1,y2,w,h;

    /*search max*/
    int maxc=0;
    for(int i=0; i<boxes.size()/W; i++) {
        if(boxes[i*W+W-2] > maxc)
            maxc = boxes[i*W+W-2];
    }

    for(int c=1; c<=maxc; c++) {
        cinds.clear();
        /*search elements where ==c*/
        for(int i=0; i<boxes.size()/W; i++) {
            if(boxes[i*W+W-2] == c)
                cinds.push_back(i);
        }
        if(cinds.size()==0)
            continue;
        A.clear();
        int WidthA = bboxpred_input(dets,boxes,cinds, A,W);

        for(int Q=0; Q<cinds.size(); Q++) {
            dx1=dx2=dy1=dy2=0;
            for(int B=0; B<WidthA; B++) {
                dx1+=A[Q*WidthA+B]*M->bboxpred[c-1].x1[B];
                dx2+=A[Q*WidthA+B]*M->bboxpred[c-1].x2[B];
                dy1+=A[Q*WidthA+B]*M->bboxpred[c-1].y1[B];
                dy2+=A[Q*WidthA+B]*M->bboxpred[c-1].y2[B];
            }

            w = dets[cinds[Q]*6+2]-dets[cinds[Q]*6];
            h = dets[cinds[Q]*6+3]-dets[cinds[Q]*6+1];
            bbox.push_back(dets[cinds[Q]*6]+w*dx1);
            bbox.push_back(dets[cinds[Q]*6+1]+h*dy1);
            bbox.push_back(dets[cinds[Q]*6+2]+w*dx2);
            bbox.push_back(dets[cinds[Q]*6+3]+h*dy2);
            bbox.push_back(boxes[cinds[Q]*W+W-2]);
            bbox.push_back(boxes[cinds[Q]*W+W-1]);

            for(int E=0; E<W; E++)
                parts.push_back(boxes[cinds[Q]*W+E]);
        }
    }
}
