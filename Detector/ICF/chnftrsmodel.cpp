//============================================================================
// Author      : F. De Smedt @ EAVISE
// Copyright   : This code is written for the publication of  "Open Framework for Combined Pedestrian detection". This code is for research and educational purposes only. For a different license, please contact the contributors directly. More information about the license can be fount in license.txt
//============================================================================

#include "../ICF/chnftrsmodel.h"

#include "../Channel/LoadChannels.hpp"


int ChnFtrsModel::getShrinking() {
        return m_shrinking;
    }


std::vector<ChannelOld*> ChnFtrsModel::getChannels() {
        return m_channels;
    }

void ChnFtrsModel::sortDetections(std::vector<Detection*> &Dets) {
        std::sort(Dets.begin(),Dets.end(),compareByScore);

    }


/*!
Free all the channels
        \author F. De Smedt
        \date 2014

*/
ChnFtrsModel::~ChnFtrsModel() {
        for(int C=0; C<m_channels.size(); C++)
          delete m_channels[C];

}

/*!
	Calculate the score on a certin position (given by offsetX and offsetY).
	\param FeatLay Featurelayers to calculate the score on
	\param offsetX X-position in the image
	\param offsetY Y-position in the image
	\param Soft do softcascade to perform early rejection of the searching process
	\return The score of this feature, -15000 when pruned

        \author F. De Smedt
        \date 2014

*/
float ChnFtrsModel::calculateSumGeneral(FeatureLayer *FeatLay,int offsetX, int offsetY, bool Soft) {
    float hs = 0;
    for(int s=0; s<this->Fid.size(); s++) {
        hs += this->Values[s][getIndex(s,this->getChannels(), FeatLay, offsetX, offsetY)];

        if(Soft && hs < this->SoftThresholds[s])
            return -15000;
    }

    return hs;
}

/*!
	Since we have to handle multi-channel images,this function allows to tranform the channel-number read from the features to the actual channel and the offset (channel) inside

        \author F. De Smedt
        \date 2014

*/
int ChnFtrsModel::ConvertIndex(std::vector<ChannelOld*> &Channels, int &index) {

    int I=index;
    int chan=0;
    int offset;
    for(int i=0; i<Channels.size(); i++) {
        int S = Channels[i]->getChannelAmount();
        if(I < S) {
            offset = I;
            index = i;
            break;
        }
        I -= S;
    }
    return offset;
}


/*!
	Search for the index to use. This function is used to walk through the possible scores (hs-values). As long as there are features to evaluate this function will go dpper into the tree.

        \author F. De Smedt
        \date 2014

*/
int ChnFtrsModel::getIndex(int stage,  std::vector<ChannelOld*> channels, FeatureLayer *FeatLay, int offsetx, int offsety) {
    int k=0;
    while(this->Child[stage][k]) {
        /*as long as there are children, there should be features to evaluate*/
        Feature Feat = this->Features[stage][k];

        /*Get the correct index*/
        int index = Feat.getChannel();
        int Oset = ConvertIndex(channels, index);
        /*evaluate the feature on the image*/
        float Score = channels[index]->giveScore(Feat,FeatLay->FeatLayers[Feat.getChannel()],offsetx,offsety,Oset,CHNFTRS);
        if(Score < this->Thresholds[stage][k])
            k = this->Child[stage][k]-1;
        else
            k = this->Child[stage][k];
    }
    /*Return the index. This index will be used to get the score in this weak classifier*/
    return k;
}

/*!
	Load a model file to initialise the model to evaluate on an image
	\todo add the name of the model

        \author F. De Smedt
        \date 2014

*/
ChnFtrsModel::ChnFtrsModel(std::string ModelFile): DetectorModel(64,128)
{
    xml_document<> doc;
    xml_node<> * root_node;
    std::ifstream theFile (ModelFile.c_str());
    std::vector<char> buffer((std::istreambuf_iterator<char>(theFile)), std::istreambuf_iterator<char>());
    buffer.push_back('\0');
    doc.parse<0>(&buffer[0]);
    root_node = doc.first_node("CascadeModel");

    xml_node<> * Nums = root_node->first_node("NumberWeaks");
    this->m_numstages = atoi(Nums->value());
    this->setWidth(atoi(root_node->first_node("Width")->value()));
    this->setHeight(atoi(root_node->first_node("Height")->value()));
    //color of model
    xml_node<> *Color_node = root_node->first_node("Color");
    int Blue = atoi(Color_node->first_node("B")->value());
    int Green = atoi(Color_node->first_node("G")->value());
    int Red = atoi(Color_node->first_node("R")->value());
    cv::Scalar col(Blue,Green,Red);
    this->setColor(col);


    xml_node<> * Shri = root_node->first_node("Shrinking");
    this->m_shrinking = atoi(Shri->value());

    xml_node<> *Childs_node = root_node->first_node("Childs");
    for(xml_node<> *Child_node = Childs_node->first_node("Child"); Child_node; Child_node = Child_node->next_sibling()) {
        std::vector<int> Cs;
        Cs.clear();

        for(xml_node<> *Field_node = Child_node->first_node("Field"); Field_node; Field_node = Field_node->next_sibling()) {
            Cs.push_back(atof(Field_node->value()));
        }
        this->Child.push_back(Cs);

    }

    xml_node<> *Fids_node = root_node->first_node("Fids");
    for(xml_node<> *Child_node = Fids_node->first_node("Fid"); Child_node; Child_node = Child_node->next_sibling()) {
        std::vector<int> Cs;
        std::vector<Feature> Fs;
        Cs.clear();
//        Fs.clear();

        for(xml_node<> *Field_node = Child_node->first_node("Field"); Field_node; Field_node = Field_node->next_sibling()) {
            int V = atoi(Field_node->value());
            Cs.push_back(V);



            //        Fs.push_back(features[V]);
        }
        this->Fid.push_back(Cs);
        //      this->Features.push_back(Fs);
    }

    xml_node<> *Thresh_node = root_node->first_node("Thrs");
    for(xml_node<> *Child_node = Thresh_node->first_node("threshold"); Child_node; Child_node = Child_node->next_sibling()) {
        std::vector<float> Cs;
        Cs.clear();

        for(xml_node<> *Field_node = Child_node->first_node("Field"); Field_node; Field_node = Field_node->next_sibling()) {
            Cs.push_back(atof(Field_node->value()));
        }
        this->Thresholds.push_back(Cs);
    }
    xml_node<> *Value_node = root_node->first_node("hss");
    for(xml_node<> *Child_node = Value_node->first_node("value"); Child_node; Child_node = Child_node->next_sibling()) {
        std::vector<float> Cs;
        Cs.clear();

        for(xml_node<> *Field_node = Child_node->first_node("Field"); Field_node; Field_node = Field_node->next_sibling()) {
            Cs.push_back(atof(Field_node->value()));
        }
        this->Values.push_back(Cs);
    }

    xml_node<> *ValueT_node = root_node->first_node("Softcascade");
    for(xml_node<> *Child_node = ValueT_node->first_node("threshold"); Child_node; Child_node = Child_node->next_sibling()) {
        this->SoftThresholds.push_back(atof(Child_node->value()));
    }

    /*features*/
    xml_node<> *Feature_node = root_node->first_node("Features");
    if(Feature_node == NULL){
    	std::cout << "NO FEATURES INSIDE" << std::endl;
    }
    else{
    for(xml_node<> *Child_node = Feature_node->first_node("Field"); Child_node; Child_node = Child_node->next_sibling()) {
        std::vector<Feature> Fs;

        Fs.clear();

        for(xml_node<> *F_node = Child_node->first_node("Feature"); F_node; F_node = F_node->next_sibling()) {
            //Cs.push_back(atof(Field_node->value()));
            Feature FFF;

            FFF.setX(atoi(F_node->first_node("x-pos")->value()));
            FFF.setY(atoi(F_node->first_node("y-pos")->value()));
            FFF.setWidth(atoi(F_node->first_node("width")->value()));
            FFF.setHeight(atoi(F_node->first_node("height")->value()));
            FFF.setChannel(atoi(F_node->first_node("channel")->value()));

            Fs.push_back(FFF);
        }

        this->Features.push_back(Fs);
    }
    }

    /*currently only default channels are used. These names should come from the model-file*/
    std::vector<std::string> Names;
    //Names.push_back("luv");
    //Names.push_back("hog");

    /*ChannelNames*/
        xml_node<> *Channel_node = root_node->first_node("Channels");
        if(Channel_node == NULL){
        	std::cout << "Channel names are missing, we use the default channels" << std::endl;
            Names.push_back("luv");
            Names.push_back("hog");

        }
        else{
        for(xml_node<> *Child_node = Channel_node->first_node("Channel"); Child_node; Child_node = Child_node->next_sibling()) {
        	Names.push_back(Child_node->value());
        }
        }


    m_channels = LoadChannels(Names);

  //  m_chlist.FillChannels(Names);

    /*PWidth && PHeight*/
    xml_node<> *PWidth_node = root_node->first_node("PWidth");
    xml_node<> *PHeight_node = root_node->first_node("PHeight");
    if(PWidth_node != NULL && PHeight_node != NULL){
        this->setPwidth(atof(PWidth_node->value()));
        this->setPheight(atof(PHeight_node->value()));

    }
    else{
    	std::cout << "No outer dimentions given" << std::endl;
    }


}

/*!
	Apply the model on an image (the features of this image)
	\param FeatLay The image features
	\param offsetX the x-location
	\param offsetY the y-location
	\param ScaleFactorX the scale factor for x-direction
	\param ScaleFactorY the scale factor for y-direction
	\param Soft Perform softcascade pruning or not
	\param threshold threshold to prune on
	\param padding The padded added to the image. this is used to correct the detection position to the position on the original image
       	\return A vector of detections (normaly this is only size 1 since only one position is evaluated)

        \author F. De Smedt
        \date 2014

*/
std::vector<Detection*> ChnFtrsModel::applyModel(FeatureLayer *FeatLay, int offsetX, int offsetY,float ScaleFactorX, float ScaleFactorY,bool Soft, float threshold, int padding) {
    vector<Detection*> D;
    float score = calculateSumGeneral(FeatLay,offsetX/m_shrinking,offsetY/m_shrinking,Soft);
    score = this->normaliseScore(score);
    if(score >= threshold) {
        Detection* DD = new Detection();
        int ExtraX = (this->getPwidth() - this->getWidth())/2;
        int ExtraY = (this->getPheight() - this->getHeight())/2;


        DD->setX((offsetX-1-padding+ExtraX)/ScaleFactorX);
        DD->setY((offsetY-1-padding+ExtraY)/ScaleFactorY);
        DD->setWidth(this->getWidth()/ScaleFactorX);
        DD->setHeight(this->getHeight()/ScaleFactorY);
        DD->setScore(score);
        DD->setColor(this->getModelColor());
        DD->setModelName(this->getModelName());
        D.push_back(DD);
    }

    return D;
}


