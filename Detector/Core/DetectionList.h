//============================================================================
// Author      : F. De Smedt @ EAVISE
// Copyright   : This code is written for the publication of  "Open Framework for Combined Pedestrian detection". This code is for research and educational purposes only. For a different license, please contact the contributors directly. More information about the license can be fount in license.txt
//============================================================================


#ifndef DETECTIONLIST_H_
#define DETECTIONLIST_H_

#include <vector>
#include "detection.h"
#include "FileWriter.h"
#include <string>

// Class to protect from memory-leaks when not releasing detections
class DetectionList {
    // Some classes that should have direct access to the detections
    friend class NonMaximumSuppression;
    friend class Combinator;

public:
    DetectionList();
    virtual ~DetectionList();

    // Add a DetectionList
    void addList(const DetectionList &DL, cv::Scalar C = cv::Scalar(0,0,255)) {
        for(int d=0; d<DL.getSize(); d++) {
            this->addDetection(DL.Ds[d]->getX(),DL.Ds[d]->getY(),DL.Ds[d]->getWidth(),DL.Ds[d]->getHeight(),DL.Ds[d]->getScore(),C);
        }
    }

// Move detections
    void moveDetections(int x, int y){
                for(int d=0;d<Ds.size();d++){
                        Ds[d]->setX(Ds[d]->getX()+x);
                        Ds[d]->setY(Ds[d]->getY()+y);
                }
        }

    // Normalise the scores based on Mean and Std (standard normalisation)
    void normaliseScore(float mean, float std) {
        for(int d=0; d<this->Ds.size(); d++) {
            Ds[d]->setScore((Ds[d]->getScore()-mean)/std);
        }
    }

    DetectionList ThresholdList(float score) {
        DetectionList DL;


        for(int d=0; d<this->Ds.size(); d++) {
            if(Ds[d]->getScore() >= score)
                //std::cout << "With NAME: " << Ds[d]->getDetectorName() << std::endl;
                DL.addDetection(Ds[d]->getX(),Ds[d]->getY(),Ds[d]->getWidth(),Ds[d]->getHeight(),Ds[d]->getScore(), Ds[d]->getDetectorName());
        }

        return DL;
    }

    void setDetectorNames(std::string D) {
        for(int d=0; d<this->Ds.size(); d++) {
            Ds[d]->setDetectorName(D);
        }
    }


    bool TopScore(Detection &D) {
        Detection DL;
        float score = -99999;
        int index = -999;
        for(int d=0; d<this->Ds.size(); d++) {
            float s = Ds[d]->getScore();
            if(s > score) {
                index = d;
                score = s;
            }
        }

        if(index >= 0) {
            D = Detection(Ds[index]->getX(), Ds[index]->getY(),  Ds[index]->getWidth(),  Ds[index]->getHeight(),  Ds[index]->getScore());
            return true;
        }
        return false;
    }

    void addDetection(float x, float y, float w, float h, float score);
    void addDetection(float x, float y, float w, float h, float score, std::string name);
    void addDetection(float x,float y, float w, float h, float score, cv::Scalar C);

    // obtain the number of detections in the list
    int getSize() const {
        return this->Ds.size();
    }

    // Draw the detections on Frame
    void Draw(cv::Mat &Frame) {
        for(int d=0; d<Ds.size(); d++) {
            //if(Ds[d]->getScore() > 20)
            cv::rectangle(Frame,cv::Point(Ds[d]->getX(), Ds[d]->getY()), cv::Point(Ds[d]->getX()+Ds[d]->getWidth(), Ds[d]->getY()+Ds[d]->getHeight()), Ds[d]->getColor(),3);
			cv::putText(Frame, std::to_string(Ds[d]->getScore()), cv::Point(Ds[d]->getX(), Ds[d]->getY()), 1, 1, cv::Scalar(0, 0, 255));
		}
    }

	void getDetections(std::vector<cv::Rect2d>& detections, double th) {
		for (int d = 0; d<Ds.size(); d++) {
			cv::Rect2d r(Ds[d]->getX(), Ds[d]->getY(), Ds[d]->getWidth(), Ds[d]->getHeight());
			if (Ds[d]->getScore() > th)
			{
				detections.push_back(r);
			}
		}
	}
	void getDetections(std::vector<cv::Rect2d>& detections) {
		for (int d = 0; d<Ds.size(); d++) {
			cv::Rect2d r(Ds[d]->getX(), Ds[d]->getY(), Ds[d]->getWidth(), Ds[d]->getHeight());
			detections.push_back(r);
		}
	}

    // Resize the detections (required when working on an upscaled image)
    void resizeDetections(float scale) {
        for(int d=0; d<this->Ds.size(); d++) {
            Ds[d]->resize(scale);
        }
    }

    // Write detections in XML-format to a file coupled by FW
    void WriteDetections(FileWriter *FW, std::string filename,float rescale = 1.0) const {
        for(int d=0; d<this->Ds.size(); d++) {
            Ds[d]->setFilename(filename);
            FW->writeDetection(Ds[d]);
        }
    }


    // Write detections in txt-format to a file coupled by FW
    void WriteDetectionstxt(FileWriter *FW, std::string filename,float rescale = 1.0) const {
        for(int d=0; d<this->Ds.size(); d++) {
            Ds[d]->setFilename(filename);
            FW->writeDetectiontxt(Ds[d]);
        }
    }


    DetectionList( const DetectionList& DL ) {
        Ds.clear();
        for(int d=0; d<DL.getSize(); d++) {
            this->addDetection(DL.Ds[d]->getX(),DL.Ds[d]->getY(),DL.Ds[d]->getWidth(),DL.Ds[d]->getHeight(),DL.Ds[d]->getScore());
        }

    }


    DetectionList& operator=( const DetectionList& DL ) {
        this->Ds.clear();
        for(int d=0; d<DL.getSize(); d++) {
            this->addDetection(DL.Ds[d]->getX(),DL.Ds[d]->getY(),DL.Ds[d]->getWidth(),DL.Ds[d]->getHeight(),DL.Ds[d]->getScore());
        }

        return *this;
    }

    void PrintList() const {
        for(int d=0; d<Ds.size(); d++) {
            std::cout << "Score: " << Ds[d]->getScore() << std::endl;
        }
    }


    void CorrectAR() {
        AlterDetectionsAR(Ds,0.41,2);
    }

private:
    // The detections
    std::vector<Detection*> Ds;
    // the path to the image
    std::string path;

    void AlterDetectionsAR(std::vector<Detection*> &Dets, float AR, int method) {

        if(method == 1) { //constant width
            for(int D=0; D<Dets.size(); D++) {
                Detection *Det = Dets[D];

                float centerX = Det->getX()+Det->getWidth()/2;
                float centerY = Det->getY()+Det->getHeight()/2;

                float nW = Det->getWidth();
                float nH = Det->getWidth()/AR;

                Det->setX(centerX-nW/2);
                Det->setY(centerY-nH/2);
                Det->setWidth(nW);
                Det->setHeight(nH);
            }
        }
        else if(method == 2) { //constant height
            for(int D=0; D<Dets.size(); D++) {
                Detection *Det = Dets[D];

                float centerX = Det->getX()+Det->getWidth()/2;
                float centerY = Det->getY()+Det->getHeight()/2;

                float nW = Det->getHeight()*AR;
                float nH = Det->getHeight();

                Det->setX(centerX-nW/2);
                Det->setY(centerY-nH/2);
                Det->setWidth(nW);
                Det->setHeight(nH);

            }
        }
        else if(method == 3) { //area
            for(int D=0; D<Dets.size(); D++) {
                Detection *Det = Dets[D];

                float centerX = Det->getX()+Det->getWidth()/2;
                float centerY = Det->getY()+Det->getHeight()/2;

                float nH = sqrt((Det->getHeight()*Det->getWidth())/AR);
                float nW = nH*AR;

                Det->setX(centerX-nW/2);
                Det->setY(centerY-nH/2);
                Det->setWidth(nW);
                Det->setHeight(nH);

            }
        }
    }

};

#endif /* DETECTIONLIST_H_ */
