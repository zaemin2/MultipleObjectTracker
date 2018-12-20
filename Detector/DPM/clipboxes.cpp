#include <iostream>
#include <vector>
//#include "cv.h"
//#include "highgui.h"

using namespace std;

double max(double a, double b) {
    if(a>b) return a;
    return b;
}

double min(double a, double b) {
    if(a<=b) return a;
    return b;
}

void clipboxes(int W, int H, vector<double> &dets, vector<double> &parts, int Width) {
    if(dets.size() > 0) {
        for(int i=0; i<dets.size()/6/*Size of a row in dets*/; i++) {
            dets[i*6] = max(dets[i*6],1);
            dets[i*6+1] = max(dets[i*6+1],1);
            dets[i*6+2] = min(dets[i*6+2],W);
            dets[i*6+3] = min(dets[i*6+3],H);

            double w = dets[i*6+2]-dets[i*6]+1;
            double h = dets[i*6+3]-dets[i*6+1]+1;
            if(w<=0 || h <= 0) {
                for(int BB=0; BB<6; BB++)
                    dets.erase(dets.begin()+i*6-BB);
                if(parts.size() > 0) {
                    for(int Q=0; Q<Width; Q++)
                        parts.erase(parts.begin()+i*Width);
                }
            }
        }
    }

}
