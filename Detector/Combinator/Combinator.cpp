//============================================================================
// Author      : F. De Smedt @ EAVISE
// Copyright   : This code is written for the publication of  "Open Framework for Combined Pedestrian detection". This code is for research and educational purposes only. For a different license, please contact the contributors directly. More information about the license can be fount in license.txt
//============================================================================

#include "Combinator.h"
#include "../DPMOld/dpmdetectorOld.h"
#include "../DPM/dpmdetector.h"
#include "../ICF/chnftrsdetector.h"
#include "../HOG/HOGDetector.h"
#include "../ACF/ACFDetector.h"

#include "helper.hpp"
#include "HasOverlap.h"

/*Convert an integer to a string*/
std::string I2Str(int index) {
    std::stringstream SS;
    SS << index;
    return SS.str();
}

std::string CreateName(int index, std::string Detector) {
    return Detector + I2Str(index);
}


struct SCouple {
    SCouple(std::string first, std::string second): first(first),second(second) {}

    std::string first, second;
};

template <typename Graph, typename NameMap>
inline std::map<std::string, typename boost::graph_traits<Graph>::vertex_descriptor>
AddCouple(Graph& g, NameMap nm, std::string index1, std::string index2)
{
    typedef typename boost::graph_traits<Graph>::vertex_descriptor Vertex;
    std::map<std::string, Vertex> verts;
    Vertex u = add_named_vertex(g, nm, index1, verts);
    Vertex v = add_named_vertex(g, nm, index2, verts);
    add_edge(u, v, g);

    return verts;
}


template <typename Graph, typename NameMap>
inline std::map<std::string, typename boost::graph_traits<Graph>::vertex_descriptor>
AddCoupleList(Graph& g, NameMap nm, std::vector<SCouple> &CoupleList)
{
    typedef typename boost::graph_traits<Graph>::vertex_descriptor Vertex;
    std::map<std::string, Vertex> verts;

    for(int i=0; i<CoupleList.size(); i++) {

        Vertex u = add_named_vertex(g, nm, CoupleList[i].first, verts);
        Vertex v = add_named_vertex(g, nm, CoupleList[i].second, verts);
        add_edge(u, v, g);


    }
    return verts;
}

std::vector<std::vector<DetectionNode> > GroupDetections(std::vector<DetectionNode> &Nodes) {
    std::vector<bool> Found(Nodes.size(),false);
    std::vector<std::vector<DetectionNode> > Groups;
    std::vector<std::vector<int> > Indeces;

    Graph g;
    NameMap nm(get(&Actor::name, g));

    clique_adder vis(Indeces);

    std::vector<SCouple> PairList;

    // look for "same detections"
    for(int d=0; d<Nodes.size(); d++) {
        for(int d2=d+1; d2<Nodes.size(); d2++) {
            if(Nodes[d].Det->getDetectorName() == Nodes[d2].Det->getDetectorName())
                continue;

            if(HasOverlap(Nodes[d].Det, Nodes[d2].Det) >= 0.5) {
                SCouple Stemp(I2Str(d),I2Str(d2));
                PairList.push_back(Stemp);
                Found[d] =true;
                Found[d2] =true;
            }
        }
    }

    AddCoupleList(g,nm,PairList);
    Indeces.clear();
    // Do the actual search for groups
    bron_kerbosch_all_cliques(g, vis);

    // add the "groups" to return them
    for(int I=0; I<Indeces.size(); I++) {
        std::vector<DetectionNode> Pair;
        for(int I2=0; I2<Indeces[I].size(); I2++) {
            Pair.push_back(Nodes[Indeces[I][I2]]);
        }
        Groups.push_back(Pair);
    }

    // Add the single detections as a separate group
    for(int l=0; l<Found.size(); l++) {
        if(!Found[l]) {
            std::vector<DetectionNode> Single;
            Single.push_back(Nodes[l]);
            Groups.push_back(Single);

        }
    }

    return Groups;
}

Combinator::Combinator():CP("Caltech") {
    //this->Detectors.push_back(new DPMDetectorOld());
//    this->Detectors.push_back(new HOGDetector());
    this->Detectors.push_back(new DPMDetector());
    //this->Detectors.push_back(new ChnFtrsDetector());
    this->Detectors.push_back(new ACFDetector());
    //this->Detectors.push_back(new ACFDetector());

    // Perform The Combinator instead of Pool-Combination
    this->doCombinator = true;
}

Combinator::~Combinator() {
    // TODO Auto-generated destructor stub
}


DetectionList Combinator::applyCombinator(const cv::Mat &Frame) const {
    DetectionList DL;

    float offset = -1.9519; // taken from the maximum over the minimum of all detectors

    NonMaximumSuppression NMS;

    std::vector<Detection*> Ds;
    std::vector<DetectionNode> DNodes;


    // obtain NMS-detections of detectors
    for(int D=0; D<this->Detectors.size(); D++) {

        std::string NameDet = this->Detectors[D]->getName();
        DetectionList D1 = this->Detectors[D]->applyDetector(Frame);
        D1.CorrectAR();


        // Normalise scores
        D1.normaliseScore(CP.getMean(Detectors[D]->getName()), CP.getStd(Detectors[D]->getName()));
        
        DetectionList NMax1 = NMS.dollarNMS(D1);
        DetectionList NonMax = NMS.standardNMS(NMax1,0.5);

// The minimum possible score without getting negative results ...!
        DetectionList Fil = NonMax.ThresholdList(offset);
        //Fil.PrintList();

        for(int D2=0; D2<Fil.getSize(); D2++) {
            assert(Fil.Ds[D2]->getDetectorName() == NameDet && "The detectornames should be the same");
            DNodes.push_back(DetectionNode(Fil.Ds[D2], D, D2));
        }
    }

    // Group Detections, each group contains detections of "same object"
    std::vector<std::vector<DetectionNode> > Groups = GroupDetections(DNodes);

    std::vector<std::string> AllDetectors;
    for(int d=0; d<this->Detectors.size(); d++)
        AllDetectors.push_back(Detectors[d]->getName());

    std::vector<float> AllComp;
    std::map<std::string, float> AllCompMap;

    if(AllDetectors.size() > 1) {
        AllComp = CP.getComplementarity(AllDetectors);
    }
    else {
        AllComp.push_back(1); // when a single detection is used as "Combination"
    }
// Convert to a map
    for(int d=0; d<this->Detectors.size(); d++) {
        AllCompMap[Detectors[d]->getName()] = AllComp[d];
    }

    for(int G=0; G<Groups.size(); G++) {
        float score = 0;
        float X=0,Y=0,W=0,H=0;

        int counter=0;
    
	//Obtain complementarity
        std::vector<std::string> Detectors;
        for(int G1=0; G1<Groups[G].size(); G1++) {
            Detectors.push_back(Groups[G][G1].Det->getDetectorName());
        }

        std::vector<float> Complementarities = CP.getComplementarity(Detectors);

        for(int G1=0; G1<Groups[G].size(); G1++) {
            Detection *DD = Groups[G][G1].Det;
            std::string DetectorName = DD->getDetectorName();

            float complementarity;
            if(Groups[G].size() == 1) {
                complementarity = AllCompMap[DetectorName];
            }
            else {
                complementarity = Complementarities[G1];
            }

            float confidence = CP.getConfidence(DetectorName);

            assert(confidence >  0  && "Confidence should be bigger as 0");
            assert(complementarity >  0  && "Compementarity should be bigger as 0");

            float DScore = (DD->getScore()-offset) * confidence * complementarity;

            assert(DScore >  0  && "Score of a single detector should be bigger as 0");
            score += DScore;
            X+=DD->getX();
            Y+=DD->getY();
            W+=DD->getWidth();
            H+=DD->getHeight();
            counter++;
        }
        Ds.push_back(new Detection(X/counter, Y/counter, W/counter, H/counter,score));
    }

    // Add the detections to a DetectionList to return
    for(int DD=0; DD<Ds.size(); DD++) {
        DL.addDetection(Ds[DD]->getX(),Ds[DD]->getY(),Ds[DD]->getWidth(),Ds[DD]->getHeight(),Ds[DD]->getScore());
        delete Ds[DD];
    }


    DL.setDetectorNames("TheCombinator");
    return DL;
}

DetectionList Combinator::applyPoolCombination(const cv::Mat &Frame) const {
    DetectionList DL;

    for(int D=0; D<this->Detectors.size(); D++) {
        DetectionList D1 = this->Detectors[D]->applyDetector(Frame);
        D1.CorrectAR();
        D1.normaliseScore(CP.getMean(Detectors[D]->getName()), CP.getStd(Detectors[D]->getName()));

        /*Add the detections to the list*/
        DL.addList(D1);
    }
    DL.setDetectorNames("PoolCombinator");
    return DL;
}


DetectionList Combinator::applyDetector(const cv::Mat &Frame) const {

    if(this->doCombinator) {
        /*Perform the combination technique of The Combinator*/
        return this->applyCombinator(Frame);
    }
    else {
        /*Perform Pool Combination*/
        return this->applyPoolCombination(Frame);
    }
}
