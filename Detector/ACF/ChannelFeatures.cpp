/*
 * ChannelFeatures.cpp
 *
 *  Created on: Jan 15, 2015
 *      Author: fds
 */

#include "ChannelFeatures.h"
#include <cstring>



float ChannelFeatures::getFeatureValue(int channel, int location) const {
    if(channel >= this->getnChannels() || location >= this->Channelheight*this->Channelwidth) {
        std::cerr << "Requesting feature from out of range!!" << std::endl;
        std::cerr << "channel: " << channel << std::endl;
        std::cerr << "location: " << location << std::endl;
        exit(1);
    }

    return this->Features[channel][location];
}

//Crop the image on a multiple of "clip"
cv::Mat clipImage(const cv::Mat &Im, int clip) {
    //Clip the image on a multiple of the shrinking-factor
    int overX = Im.cols%clip;
    int overY = Im.rows%clip;
    cv::Rect ROI(0,0,Im.cols-overX, Im.rows-overY);

    return Im(ROI).clone();
}




//Calculate the features based on a default configuration
ChannelFeatures::ChannelFeatures(const cv::Mat &Image,int shrinking) {

    cv::Mat clippedImage = clipImage(Image,shrinking);

    //Set correct destination width-height
    this->Channelheight = clippedImage.rows/shrinking; 
    this->Channelwidth = clippedImage.cols/shrinking; 

    // Create the Channels
    ColorChannel LUV(clippedImage);

    GradMagChannel GMag(LUV);
    GradHistChannel GHC(GMag);

    //The addChannelFeatures-function will resize the channels when necessary
    this->addChannelFeatures(LUV);
    this->addChannelFeatures(GMag);
    this->addChannelFeatures(GHC);
}



ChannelFeatures::~ChannelFeatures() {
    // free the features
    for(int c=0; c<this->Features.size(); c++) {
        free(Features[c]);
    }
}

