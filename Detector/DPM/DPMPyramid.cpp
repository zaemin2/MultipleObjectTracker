/*
 * DPMPyramid.cpp
 *
 *  Created on: Mar 20, 2015
 *      Author: fds
 */

#include "DPMPyramid.h"

#include <iostream>


int amin(int i[2],int n) {
    if(i[0]<i[1])
        return i[0];
    return i[1];
}


///////////////////

DPMPyramid::DPMPyramid(const cv::Mat &Frame, int padx, int pady, int interval, int minheight, int maxheight, int modelheight):padx(padx), pady(pady) {
    /*Standard sbin for DPM*/
    int sbin = 8;

    /*The default rescaling (interval gives the amount of layers per octave)*/
    double sc = pow(2.0,(1.0/interval)); // Note the space between layers is smaller!!!

    // Search for startscale
    float startscale = static_cast<float>(modelheight)/minheight;
    float stopsc = static_cast<float>(modelheight)/maxheight;

    int imsize[2] = {static_cast<int>(Frame.rows*startscale), static_cast<int>(Frame.cols*startscale)};

    int max_scale = 1 + floor(log(startscale/stopsc)/log(sc));
    interval = std::min(interval,max_scale);
    
   this->interval = interval;  
    /*Amount of layers*/ 
    int lenghtFeatures = max_scale+interval;
    
    /*Now we know the size of the pyramid*/
    scales.resize(lenghtFeatures);
    sbins.resize(lenghtFeatures);
    imscale.resize(lenghtFeatures);


        // The first interval
        for(int l=0;l<interval;l++){
                float Scale = startscale*2.0/std::pow(sc,(l));
                scales[l] = startscale*2.0/std::pow(sc,(l));
                imscale[l] = startscale*1.0/std::pow(sc,(l));
                sbins[l] = sbin/2; //calculated with smaller sbin instead of upscaling the image
        }
        for(int l=interval;l<lenghtFeatures;l++){
                scales[l] = 0.5*scales[l-interval];//startscale*1.0/std::pow(sc,());
                imscale[l] = 0.5*scales[l-interval];//startscale*1.0/std::pow(sc,(l));
                sbins[l] = sbin; //calculated with smaller sbin instead of upscaling the image
        }

    for(int s=0; s<scales.size(); s++) {
        cv::Mat Scaled;
        cv::resize(Frame,Scaled,cv::Size(),imscale[s],imscale[s]);
        this->Features.push_back(new DPMFeatures(Scaled,sbins[s], padx, pady));
    }

}

DPMPyramid::DPMPyramid(const cv::Mat &Frame, int padx, int pady, int interval):padx(padx), pady(pady), interval(interval) {

    /*Standard sbin for DPM*/
    int sbin = 8;

    /*The default rescaling (interval gives the amount of layers per octave)*/
    double sc = pow(2.0,(1.0/interval));


    int imsize[2] = {Frame.rows, Frame.cols};
    int max_scale = 1 + floor(log(amin(imsize,2)/(5.0*sbin))/log(sc));

    /*Amount of layers*/
    int lenghtFeatures = max_scale+interval;

    /*Now we know the size of the pyramid*/
    scales.resize(lenghtFeatures);
    sbins.resize(lenghtFeatures);
    imscale.resize(lenghtFeatures);
    //fill in the scales and sbins
    for(int i=0; i<interval; i++) {
        scales[i] = 2.0/std::pow(sc,(i));
        imscale[i] = 1.0/std::pow(sc,(i));
        sbins[i] = sbin/2; //calculated with smaller sbin instead of upscaling the image

        scales[i+interval] = 1.0/std::pow(sc,(i));
        imscale[i+interval] = 1.0/std::pow(sc,(i));
        sbins[i+interval] = sbin; //calculated with smaller sbin instead of upscaling the image
        for(int j=i+interval; j<max_scale; j+=interval) {
            imscale[j+interval] = 0.5*scales[j];
            scales[j+interval] = 0.5*scales[j];
            sbins[j+interval] = sbin;
        }
    }

    // option for parallelisation
    for(int s=0; s<scales.size(); s++) {
        cv::Mat Scaled;
        cv::resize(Frame,Scaled,cv::Size(),imscale[s],imscale[s]);
        this->Features.push_back(new DPMFeatures(Scaled,sbins[s], padx, pady));
    }
}

DPMPyramid::~DPMPyramid() {
    //delete the features
    for(int l=0; l<Features.size(); l++) {
        delete Features[l];
    }
}
