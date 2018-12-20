/*
 * ACFFeaturePyramid.cpp
 *
 *  Created on: Feb 3, 2015
 *      Author: fds
 */

#include "ACFFeaturePyramid.h"
#include <cmath>
//#include <ctgmath>


ACFFeaturePyramid::ACFFeaturePyramid(const cv::Mat &Image, int scalesPerOct, cv::Size minSize, int minheight, int maxheight, const ACFDetector &ADet): scalesPerOct(scalesPerOct), minSize(minSize) {

    this->approxAmount = scalesPerOct-1;

// Based on technique used in DPM
    int interval = scalesPerOct; 
    double sc = pow(2.0,(1.0/interval));
	int modelheight = ADet.getHeight();	

   float startscale = static_cast<float>(modelheight)/minheight;
    float stopsc = static_cast<float>(modelheight)/maxheight;


   int nScales = 1 + floor(log(startscale/stopsc)/log(sc));

//set the size of the feature-pyramid at the nScales amount
    Layers.resize(nScales);
    std::vector<int> approxReal(nScales); // to store the layer to use for approximation...

// Generate scales
    for(int s=0; s<nScales; s++) {
	float sc3 = startscale*1.0/std::pow(sc,(s));
        scales.push_back(sc3);
    }

    float sz1 = Image.rows;
    float sz2 = Image.cols;

    float d0,d1;

    if(sz1<sz2) {
        d0=sz1;
        d1=sz2;
    }
    else {
        d0=sz2;
        d1=sz1;
    }

// Should come from model ...
    float shrink = ADet.getShrinking();
    float eps = 2.2204e-16;

for(int i=0;i<nScales;i++){
        float s = scales[i];
        float s0 = (round((float)d0*s/shrink)*shrink-0.25*shrink)/d0;
        float s1 = (round((float)d0*s/shrink)*shrink+0.25*shrink)/d0;

        std::vector<double> ss,sst;
        for(double sss=0; sss<=1-eps; sss+=0.01) {
            float stemp = sss*(s1-s0)+s0;
            ss.push_back(stemp);
            float es0 = stemp*d0;
            float es0T=es0-round(es0/shrink)*shrink;
            if(es0T < 0)
                es0 = -es0T;
            else
                es0 = es0T;

            float es1=d1*stemp;
            float es1T=es1-round(es1/shrink)*shrink;


            if(es1T < 0)
                es1 = -es1T;
            else
                es1 = es1T;

            //save the max of both
            sst.push_back(std::max(es0,es1));
        }

        // find the minimum
        int index = 0;
        float val = sst[0];
        for(int iGG=1; iGG<sst.size(); iGG++) {
            if(sst[iGG] < val) {
                val = sst[iGG];
                index = iGG;
            }
        }
        scales[i] = ss[index];
    }

    std::vector<bool> kp(scales.size());
    for(int i=0; i<scales.size()-1; i++) {
        if(scales[i] == scales[i+1])
            kp[i] = false;
        else
            kp[i] = true;
    }
    kp[scales.size()-1] = true;

//prune when scale already exists
    for(int i=scales.size()-1; i>=0; i--) {
        if(!kp[i])
            scales.erase(scales.begin()+i);
    }

    std::vector<float> scalesh, scalesw;

    for(int i=0; i<scales.size(); i++) {
        float h = round(sz1*scales[i]/shrink)*shrink/sz1;
        float w = round(sz2*scales[i]/shrink)*shrink/sz2;

        scalesh.push_back(h);
        scalesw.push_back(w);
    }

    std::vector<int> isR;
    for(int i=1; i<=nScales; i+=approxAmount+1) {
        isR.push_back(i);
    }

    std::vector<int> isA(nScales);
    for(int i=1; i<=nScales; i++) {
        isA[i-1] = i;
    }

    for(int i=0; i<isR.size(); i++)
        isA[isR[i]-1] = -1;

    std::vector<int> j;
    j.push_back(0);
    for(int i=0; i<isR.size()-1; i++) {
        j.push_back(std::floor((isR[i]+isR[i+1])/2.0));
    }
    j.push_back(nScales);


    std::vector<int> isN(nScales);
    for(int i=1; i<=nScales; i++) {
        isN[i-1] = i;
    }

    for(int i=0; i<isR.size(); i++) {
        for(int Q = j[i]+1; Q<=j[i+1]; Q++)
            isN[Q-1] = isR[i];
    }

//Real Scales
    for(int i=0; i<isR.size(); i++) {
        int ii = isR[i]-1; //go to C-indexing

        float sc = scales[ii];
        float w1 = round(sz2*sc/shrink)*shrink;
        float h1 = round(sz1*sc/shrink)*shrink;

        cv::Mat Scaled;
        cv::resize(Image,Scaled,cv::Size(w1,h1));

        Layers[ii] = new ChannelFeatures(Scaled, ADet.getShrinking());
    }


    for(int i=0; i<Layers.size(); i++) {
        if(isA[i] == -1) //Real layer
            continue;

        int iR = isN[i]-1;
        Layers[i] =  Layers[iR]->ApproxChannel(Image, ADet.getShrinking(),scales[i],scales[iR]);

    }

//Smoothing
    for(int l=0; l<Layers.size(); l++) {
        Layers[l]->SmoothChannels();
    }
}

ACFFeaturePyramid::ACFFeaturePyramid(const cv::Mat &Image, int scalesPerOct,cv::Size minSize, const ACFDetector &ADet): scalesPerOct(scalesPerOct), minSize(minSize) {

    this->approxAmount = scalesPerOct-1;
    int nOctUp=0;

    int nScales = std::floor(static_cast<float>(scalesPerOct*(nOctUp + log2f(std::min((float)Image.cols/minSize.width, (float)Image.rows/minSize.height)))+1));

//set the size of the feature-pyramid at the nScales amount
    Layers.resize(nScales);
    std::vector<int> approxReal(nScales); // to store the layer to use for approximation...

// Generate scales
    for(int s=0; s<nScales; s++) {
        float sc = std::pow(2.0,(static_cast<float>(-s)/scalesPerOct+nOctUp));
        scales.push_back(sc);
    }

    float sz1 = Image.rows;
    float sz2 = Image.cols;

    float d0,d1;

    if(sz1<sz2) {
        d0=sz1;
        d1=sz2;
    }
    else {
        d0=sz2;
        d1=sz1;
    }

// Should come from model ...
    float shrink = ADet.getShrinking();
    float eps = 2.2204e-16;

for(int i=0;i<nScales;i++){
        float s = scales[i];
        float s0 = (round((float)d0*s/shrink)*shrink-0.25*shrink)/d0;
        float s1 = (round((float)d0*s/shrink)*shrink+0.25*shrink)/d0;

        std::vector<double> ss,sst;
        for(double sss=0; sss<=1-eps; sss+=0.01) {
            float stemp = sss*(s1-s0)+s0;
            ss.push_back(stemp);
            float es0 = stemp*d0;
            float es0T=es0-round(es0/shrink)*shrink;
            if(es0T < 0)
                es0 = -es0T;
            else
                es0 = es0T;

            float es1=d1*stemp;
            float es1T=es1-round(es1/shrink)*shrink;


            if(es1T < 0)
                es1 = -es1T;
            else
                es1 = es1T;

            //save the max of both
            sst.push_back(std::max(es0,es1));
        }

        // find the minimum
        int index = 0;
        float val = sst[0];
        for(int iGG=1; iGG<sst.size(); iGG++) {
            if(sst[iGG] < val) {
                val = sst[iGG];
                index = iGG;
            }
        }
        scales[i] = ss[index];
    }

    std::vector<bool> kp(scales.size());
    for(int i=0; i<scales.size()-1; i++) {
        if(scales[i] == scales[i+1])
            kp[i] = false;
        else
            kp[i] = true;
    }
    kp[scales.size()-1] = true;

//prune when scale already exists
    for(int i=scales.size()-1; i>=0; i--) {
        if(!kp[i])
            scales.erase(scales.begin()+i);
    }

    std::vector<float> scalesh, scalesw;

    for(int i=0; i<scales.size(); i++) {
        float h = round(sz1*scales[i]/shrink)*shrink/sz1;
        float w = round(sz2*scales[i]/shrink)*shrink/sz2;

        scalesh.push_back(h);
        scalesw.push_back(w);
    }

    std::vector<int> isR;
    for(int i=1; i<=nScales; i+=approxAmount+1) {
        isR.push_back(i);
    }

    std::vector<int> isA(nScales);
    for(int i=1; i<=nScales; i++) {
        isA[i-1] = i;
    }

    for(int i=0; i<isR.size(); i++)
        isA[isR[i]-1] = -1;

    std::vector<int> j;
    j.push_back(0);
    for(int i=0; i<isR.size()-1; i++) {
        j.push_back(std::floor((isR[i]+isR[i+1])/2.0));
    }
    j.push_back(nScales);


    std::vector<int> isN(nScales);
    for(int i=1; i<=nScales; i++) {
        isN[i-1] = i;
    }

    for(int i=0; i<isR.size(); i++) {
        for(int Q = j[i]+1; Q<=j[i+1]; Q++)
            isN[Q-1] = isR[i];
    }

//Real Scales
    for(int i=0; i<isR.size(); i++) {
        int ii = isR[i]-1; //go to C-indexing

        float sc = scales[ii];
        float w1 = round(sz2*sc/shrink)*shrink;
        float h1 = round(sz1*sc/shrink)*shrink;

        cv::Mat Scaled;
        cv::resize(Image,Scaled,cv::Size(w1,h1));

        Layers[ii] = new ChannelFeatures(Scaled, ADet.getShrinking());
    }


    for(int i=0; i<Layers.size(); i++) {
        if(isA[i] == -1) //Real layer
            continue;

        int iR = isN[i]-1;
        Layers[i] =  Layers[iR]->ApproxChannel(Image, ADet.getShrinking(),scales[i],scales[iR]);

    }

//Smoothing
    for(int l=0; l<Layers.size(); l++) {
        Layers[l]->SmoothChannels();
    }
}



// Constructor for single scale ...
ACFFeaturePyramid::ACFFeaturePyramid(const cv::Mat &Image, float sc,cv::Size minSize, const ACFDetector &ADet): scalesPerOct(scalesPerOct), minSize(minSize) {
    this->approxAmount = 0;

    int nScales = 1;

//set the size of the feature-pyramid at the nScales amount
    Layers.resize(nScales);

    for(int s=0; s<nScales; s++) {
        scales.push_back(sc);
        cv::Mat Scaled;
        //Rescale the image
        if(sc != 1.0)
            cv::resize(Image,Scaled,cv::Size(),sc,sc);
        else
            Scaled = Image.clone();

        Layers[s] = new ChannelFeatures(Scaled, ADet.getShrinking());
    }

//Smoothing
    for(int l=0; l<Layers.size(); l++) {
        Layers[l]->SmoothChannels();
    }
}

// A constructor the create a singe-layer pyramid from the approximated features
ACFFeaturePyramid::ACFFeaturePyramid(int scalesPerOct,cv::Size minSize, float sc, ChannelFeatures* Features):minSize(minSize), scalesPerOct(scalesPerOct), approxAmount(1){
        this->scales.push_back(sc);
        this->Layers.push_back(Features);
}



ACFFeaturePyramid::~ACFFeaturePyramid() {
    for(int l=0; l<Layers.size(); l++)
        delete Layers[l];
}

