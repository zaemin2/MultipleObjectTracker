//============================================================================
// Author      : F. De Smedt @ EAVISE
// Copyright   : This code is written for the publication of  "Open Framework for Combined Pedestrian detection". This code is for research and educational purposes only. For a different license, please contact the contributors directly. More information about the license can be fount in license.txt
//============================================================================

#ifndef CHNFTRSMODEL_H
#define CHNFTRSMODEL_H

#include "../Core/detectormodel.h"
#include <iostream>
#include <vector>
#include <fstream>
#include "../Channel/channelold.h"

#include "../Core/featurelayer.h"
#include "../ICF/chnftrsfeaturelayer.h"

#include "rapidxml-1.13/rapidxml.hpp"
#include "rapidxml-1.13/rapidxml_print.hpp"

#include <algorithm>


#include <iostream>

#include "../Channel/channelold.h"

#include "../ICF/chnftrsfeaturelayer.h"

#include "../Core/filelocator.h"
#include "../Core/detectormodel.h"

#include "../Core/nms.h"

#include "../Channel/channellist.h"

using namespace std;
using namespace rapidxml;

/*!
	This class is to represent a model of a ChnFtrs-bsed detector. It is initialised with a model-file which contains the sizes, colors, channels. used features, ... of a ChnFtrs-based detector. A model describes one rigid model. By using the ApplyModel-function, the model can be evaluated on a set of feature-layers.

*/

class ChnFtrsModel : public DetectorModel
{
public:
    ChnFtrsModel(std::string ModelFile);


    ChnFtrsModel(): DetectorModel(41, 100) {

    }


    virtual ~ChnFtrsModel();
    std::vector<Detection*> applyModel(FeatureLayer *FeatLay, int offsetX, int offsetY,float ScaleFactorX, float ScaleFactorY, bool Soft, float threshold, int padding=0);

    void setShrinking(int shrink) {
        this->m_shrinking = shrink;
    }

    int getShrinking() ;

    std::vector<ChannelOld*> getChannels();

    float calculateSumGeneral(FeatureLayer *FeatLay,int offsetX, int offsetY, bool Soft);

//! set the width of the outer model
    void setPwidth(int w) {
        this->m_padwidth = w;
    }
//! set the heaight of the outer model
    void setPheight(int h) {
        this->m_padheight = h;
    }

//! returns the width of the outer model
    int getPwidth() {
        return this->m_padwidth;
    }

//! returns the height of the outer model
    int getPheight() {
        return this->m_padheight;
    }

//! returns the number of stages of the model
    int getNumStages(){
    	return this->m_numstages;
    }

protected:
    ChannelList m_chlist;

//! holds indeces to follow through a stage evaluation, normally these are the same in every stage since these represent the decision stump tree
    std::vector<std::vector<int> > Child;

//! holds the indeces of the features to use, no longer used when features are inside the model
    std::vector<std::vector<int> > Fid;

//! The thresholds that should be reached for each feature
    std::vector<std::vector<float> > Thresholds;

//! stores the values to be added/subtracted at the end of a stage (deoendent on the leaf reached)
    std::vector<std::vector<float> > Values;

//! which features to evaluate
    std::vector<std::vector<Feature> > Features;

//! stores the pruning thresholds of the model
    std::vector<float> SoftThresholds;

//! the channels used on the model
    std::vector<ChannelOld*> m_channels;

//! function used in evaluating each stage, returns the index to follow
    int getIndex(int stage,std::vector<ChannelOld*> channels, FeatureLayer *FeatLay, int offsetx, int offsety);

//! sort the detections on score
    void sortDetections(std::vector<Detection*> &Dets);

//! convert the index of a feature (channel-number) to the actual index. Needed since a channel can have multiple channel-images
    int ConvertIndex(std::vector<ChannelOld*> &Channels, int &index);

private:
    /*Functions for loading the channels*/


//! width of the outer model
    int m_padwidth;
//! height of the outer model
    int m_padheight;
//! number of stages in the model
    int m_numstages;

//! shinking used
    int m_shrinking;


};

#endif // CHNFTRSMODEL_H
