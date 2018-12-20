//============================================================================
// Author      : F. De Smedt @ EAVISE
// Copyright   : This code is written for the publication of  "Open Framework for Combined Pedestrian detection". This code is for research and educational purposes only. For a different license, please contact the contributors directly. More information about the license can be fount in license.txt
//============================================================================

#include "../ICF/chnftrsdetector.h"
#include "../ICF/Extra.h"
#include "../Core/ScaleSpacePyramid.h"

ChnFtrsDetector::ChnFtrsDetector() {
    m_density = 4;
    m_shrinking = 4;

    // used for Caltech experiment, is dependend on the dataset
    this->mean = 27.8808;
    this->std = 30.5226;
    this->confidence = 0.18089;

    // model file to use
    std::string modelfile = "Models/ICF/NN3.xml_";
    m_models.push_back(new ChnFtrsModel(modelfile.c_str()));
}

/*!
	Create a CHNFTRS detector based on a vector of model-files. Each file will result in a independend CHNFTR model. The features will be calculated only once per ChnFtrsDetector
        \author F. De Smedt
        \date 2014

*/
ChnFtrsDetector::ChnFtrsDetector(std::vector<std::string> ModelFiles)
{
    /*
    set the desity of the evaluation windows. 4 means that for each 4 pixels a window will be evaluated on the feature image*/
    m_density = 4;
    /*
     Each model-file should lead to a new ChnFtrs model. There should be a control that the feature-space is identical.*/
    for(int i=0; i<ModelFiles.size(); i++)
        m_models.push_back(new ChnFtrsModel(ModelFiles[i]));


    /*A control that the shrinking factors are the same*/
    for(int i=0; i<m_models.size(); i++) {
        if(i==0)
            this->m_shrinking = m_models[i]->getShrinking();
        else if(m_models[i]->getShrinking() != this->m_shrinking) {
            std::cerr << "DIFFERENT SHRINKING-FACTOR" << std::endl;
            exit(2);
        }
    }
}


/*!
	Calculate the features (still a free function for experimentation purposes)
*/
std::vector<cv::Mat> getFeaturesFromChannels(std::vector<ChannelOld*> Chans,cv::Mat &image,int shrinking=1) {
    std::vector<cv::Mat> Features;
    std::vector<cv::Mat> FeatTemp;
    std::vector<cv::Mat> FeatOriginal; //used if shrinking is on
    FeatTemp.clear();


    if(image.channels() != 3) {
        std::cout << "Amount of channels in image is NOT ok, we expect a 3-channel image as input!" << std::endl;
        exit(3);
    }

    for(int c=0; c<Chans.size(); c++) {
        Chans[c]->getFeatOfChannel(image,FeatTemp, FeatOriginal,shrinking);
    }

    for(int c=0; c<FeatTemp.size(); c++) {
        std::vector<cv::Mat> F;
        cv::split(FeatTemp[c],F);
        std::copy(F.begin(),F.end(),back_inserter(Features));
    }

    return Features;
}


ChnFtrsFeatureLayer* ChnFtrsDetector::CalculateFeatureImages(const cv::Mat& image) const {

    ChnFtrsFeatureLayer *FeatLay = new ChnFtrsFeatureLayer();

    //floor the image size to a multiple of the shrinking size
    int nW = (image.cols/this->m_shrinking)*this->m_shrinking;
    int nH = (image.rows/this->m_shrinking)*this->m_shrinking;

    cv::Mat Cutted = image(cv::Rect(0,0,nW,nH)).clone();
    /*transform the image to float-format RGB-image, this is necessary for the channels*/
    cv::Mat imageRGB,imageConv;
    cv::cvtColor(Cutted,imageRGB,CV_BGR2RGB);
    imageRGB.convertTo(imageConv, CV_32FC3,1/255.0);

    /*Calculate the actual feature images and the integral images of these. Add these integral images, forming the features to evaluate on, to the FeatLay (class to keep the features in)*/
    std::vector<cv::Mat> GI =  getFeaturesFromChannels(m_models[0]->getChannels(),imageConv,m_models[0]->getShrinking());
    for(int i=0; i<GI.size(); i++) {
        cv::Mat Integral(GI[i].rows, GI[i].cols, CV_32FC1);
        cv::integral(GI[i],Integral,cv::noArray(),CV_32F);
        FeatLay->FeatLayers.push_back(Integral);
    }

    return FeatLay;
}

void ChnFtrsDetector::findDetection(ChnFtrsFeatureLayer* FeatLay ,std::vector<Detection*> &Detections, float ScaleFactorX, float ScaleFactorY, bool pruning, float threshold, int padding) const {

    if(FeatLay->FeatLayers.size() == 0 || FeatLay->FeatLayers[0].empty()) {
        std::cerr << "Error in searching for detections!!" << std::endl;
        exit(1);
    }

    std::vector<Detection*> D;

    int imagewidth = FeatLay->FeatLayers[0].cols*this->m_models[0]->getShrinking();
    int imageheight = FeatLay->FeatLayers[0].rows*this->m_models[0]->getShrinking();

    /*Evaluate each model on this image*/
    for(int M=0; M < m_models.size(); M++) {
        /*Loop over all positions in the image (Could possibly be parallelised)*/
        for(int X=0; X<imagewidth-m_models[M]->getPwidth(); X+=m_density) {
            for(int Y=0; Y<imageheight-m_models[M]->getPheight(); Y+=m_density) {
                D = m_models[M]->applyModel(FeatLay, X, Y,ScaleFactorX,ScaleFactorY,pruning,threshold, padding);
                std::copy(D.begin(),D.end(),back_inserter(Detections));
            }
        }
    }
}



/*!
	Run the detector on the image, interface for external use
*/

DetectionList ChnFtrsDetector::applyDetector(const cv::Mat &Frame) const {

	int minsize = 100;
	int maxsize = Frame.rows;

    DetectionList DL;


    float threshold = -100;
    float UpScale = 2.0;

    float rescaleratio = 1.05;
    ScaleSpacePyramid SPP(Frame,rescaleratio,cv::Size(41,100),UpScale);
    std::vector<std::vector<Detection*> > Dss(SPP.getNumLayers());
    std::vector<Detection*> Ds;

    // Perform detections on all the layers in parallel
    //#pragma omp parallel for
    for(int L=0; L<SPP.getNumLayers(); L++) {
        cv::Mat Iv = SPP.getImage(L);

        //Apply the detector per layer
        Dss[L] = this->applyDetector(Iv,threshold,SPP.getLayerScale(L),SPP.getLayerScale(L),true);

        //Alter the detections based on rescale-ratio
        for(int V=0; V<Dss[L].size(); V++) {
            Dss[L][V]->setX(Dss[L][V]->getX());
            Dss[L][V]->setY(Dss[L][V]->getY());
            Dss[L][V]->setWidth(Dss[L][V]->getWidth());
            Dss[L][V]->setHeight(Dss[L][V]->getHeight());
        }
    }

    //Group the detections over the layers
    for(int U=0; U<Dss.size(); U++) {
        Ds.insert(Ds.end(),Dss[U].begin(),Dss[U].end());
    }
    // add the detections to the detectionlist to return
    for(int d=0; d<Ds.size(); d++) {
        Ds[d]->setColor(cv::Scalar(255,0,0));
        DL.addDetection(Ds[d]->getX(),Ds[d]->getY(),Ds[d]->getWidth(),Ds[d]->getHeight(),Ds[d]->getScore());
        delete Ds[d];
    }

    DL.resizeDetections(UpScale);


    DL.setDetectorNames(this->getName());

    return DL;
}

std::vector<Detection*> ChnFtrsDetector::applyDetector(const cv::Mat &image2, float threshold, float ScaleFactorX, float ScaleFactorY, bool pruning) const {

    std::vector<Detection*> D;

    bool GoOn=false;
    int padding = 0;

    if(image2.channels() != 3)
    {
        std::cerr << "A 3-channel image is expected" << std::endl;
        exit(1);
    }

    // Check if there are models still fitting in the image
    for(int M=0; M<m_models.size(); M++) {
        if(image2.cols > m_models[M]->getWidth() || image2.rows > m_models[M]->getWidth()) {
            GoOn=true;
            break;
        }
    }

    if(!GoOn)
        return D;

    ChnFtrsFeatureLayer *FeatLay = CalculateFeatureImages(image2) ;

    std::vector<Detection*> AllDetections;
    findDetection(FeatLay , AllDetections, ScaleFactorX, ScaleFactorY, pruning, threshold, padding);

    //Free the memory of the features
    delete FeatLay;

    //return the founded detections
    return AllDetections;
}


/*!
	Destructor of the detector, free all the models

*/
ChnFtrsDetector::~ChnFtrsDetector() {

    for(int i=0; i<m_models.size(); i++)
        delete m_models[i];
}
