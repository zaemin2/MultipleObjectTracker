#include <iostream>
#include <string>
#include <vector>

using namespace std;
int bboxpred_input(vector<double> dets, vector<double> boxes, vector<int> cinds, vector<double> &A,int W) {

    double w,h,rx,ry,px,py;
    A.clear();
    int sizeA = (W-2)/2+1;
    for(int i=0; i<cinds.size(); i++) {
        w = dets[cinds[i]*6+2]-dets[cinds[i]*6];
        h = dets[cinds[i]*6+3]-dets[cinds[i]*6+1];
        rx = dets[cinds[i]*6] + (dets[cinds[i]*6+2]-dets[cinds[i]*6])/2;
        ry = dets[cinds[i]*6+1]+(dets[cinds[i]*6+3]-dets[cinds[i]*6+1])/2;
        for(int j=0; j<W-2; j+=4) {
            px = boxes[cinds[i]*W+j] + (boxes[cinds[i]*W+j+2] - boxes[cinds[i]*W+j])/2;
            py = boxes[cinds[i]*W+j+1] + (boxes[cinds[i]*W+j+3] - boxes[cinds[i]*W+j+1])/2;
            A.push_back((px-rx)/w);
            A.push_back((py-ry)/h);
        }
        /*Add bias feature*/
        A.push_back(1);
    }

    return sizeA;
}
