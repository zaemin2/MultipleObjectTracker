//============================================================================
// Author      : F. De Smedt @ EAVISE
// Copyright   : This code is written for the publication of  "Open Framework for Combined Pedestrian detection". This code is for research and educational purposes only. For a different license, please contact the contributors directly. More information about the license can be fount in license.txt
//============================================================================



#include "dpmdetectorOld.h"
#include "../Core/ScaleSpacePyramid.h"

// Some free functions used by DPM
void ConvertToNewModelOld(CascadeModelOld *Model, NewCascadeModelOld &NModel);
PyramidOld featpyramidcpuOld(cv::Mat &im, NewCascadeModelOld *Model, int level, int method);
int cascade_detectOld(PyramidOld *pyra, float thresh, std::vector<float> &dets, std::vector<float> &boxes, NewCascadeModelOld *NModel,const CascadeArrays_NonVolatileOld &NonVolatile);

DPMDetectorOld::DPMDetectorOld(){

	// Model-file to use
	std::string modelfile = "Models/DPMOld/inriaperson_cascadeModel.txt";

	//Create the model
	Model = new CascadeModelOld(modelfile.c_str());

	// Convert to new form of the model
	ConvertToNewModelOld(Model,CModel);

	// set interval on 1 since we work on single layer pyramids
	CModel.interval = 1;
	CModel.initModel();
	NonVolatile = InitNonVolatileDataOld(&CModel);

	// used for Caltech experiments (are dependend on the dataset)
	this->mean = 0.16903;
	this->std = 0.14121;
	this->complementarity = 1;
	this->confidence = 0.18089;

}

DPMDetectorOld::~DPMDetectorOld() {
}


// Run the detector on Frame
DetectionList DPMDetectorOld::applyDetector(const cv::Mat &Frame) const{
	std::vector<Detection*> toReturn;

	// The DetectionList that will be returned
	DetectionList DL;

	/*// Used for accuracy results
	float Upscale = 2.5;
	float threshold = -5;
	 */

	float Upscale = 1.0;
	float threshold = -0.5;


    std::vector<Detection*> Ds_DPM;
    std::vector<Detection*> Ds;

    // Rescale ratio used
    double sc = pow(2.0,(1.0/10));
    ScaleSpacePyramid SP(Frame, sc,cv::Size(CModel.maxsize[1]*CModel.sbin,CModel.maxsize[0]*CModel.sbin), Upscale);

    // reserve for each layer
    std::vector<std::vector<Detection*> > Dss_DPM(SP.getNumLayers());

#pragma omp parallel for
        for(int L=0; L<SP.getNumLayers(); L++) {
    	cv::Mat Iv = SP.getImage(L);

            std::vector<float> dets,detsT;
            std::vector<float> boxes,boxesT;
            std::vector<int> NonMax;
            // Create Feature pyramid
            PyramidOld   P = featpyramidcpuOld(Iv,(NewCascadeModelOld*) &CModel, L,3);

            boxes.clear();
            dets.clear();

            // Perform model evaluation on the feature pyramid
            int W = cascade_detectOld(&P,threshold,dets,boxes,(NewCascadeModelOld*) &CModel,NonVolatile);

            // Convert detections to correct format
                for(int Q=0; Q<dets.size(); Q+=6) {
                    float w = (dets[Q+2]-dets[Q+0]);
                    float h = (dets[Q+3]-dets[Q+1]);
                    float x = dets[Q];
                    float y = dets[Q+1];
                    float score = dets[Q+5];
                    Dss_DPM[L].push_back(new Detection(x,y,w,h,dets[Q+5]));
                }

// Resize the detections based on UpScale
        for(int V=0;V<Dss_DPM[L].size();V++){
                                Dss_DPM[L][V]->setX(Dss_DPM[L][V]->getX()/Upscale);
                                Dss_DPM[L][V]->setY(Dss_DPM[L][V]->getY()/Upscale);
                                Dss_DPM[L][V]->setWidth(Dss_DPM[L][V]->getWidth()/Upscale);
                                Dss_DPM[L][V]->setHeight(Dss_DPM[L][V]->getHeight()/Upscale);
                                Dss_DPM[L][V]->setColor(cv::Scalar(255,255,128));
                        }
        }

        for(int U=0; U<Dss_DPM.size(); U++) {
             toReturn.insert(toReturn.end(),Dss_DPM[U].begin(),Dss_DPM[U].end());
         }


        // add to the detections list
	 for(int d=0;d<toReturn.size();d++){
		 DL.addDetection(toReturn[d]->getX(),toReturn[d]->getY(),toReturn[d]->getWidth(),toReturn[d]->getHeight(),toReturn[d]->getScore());
		 delete toReturn[d];
	 }
	 return DL;
}
