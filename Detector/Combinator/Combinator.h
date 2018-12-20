//============================================================================
// Author      : F. De Smedt @ EAVISE
// Copyright   : This code is written for the publication of  "Open Framework for Combined Pedestrian detection". This code is for research and educational purposes only. For a different license, please contact the contributors directly. More information about the license can be fount in license.txt
//============================================================================

#ifndef COMBINATOR_H_
#define COMBINATOR_H_


#include <map>
#include <algorithm>
#include <string>
#include <sstream>

#include <boost/graph/properties.hpp>
#include <boost/graph/undirected_graph.hpp>
#include <boost/graph/bron_kerbosch_all_cliques.hpp>
#include "../Core/NonMaximumSuppression.h"
#include "../Core/detector.h"


#include "../Combinator/DetectorProperties.h"

using namespace std;
using namespace boost;


struct Actor
{
    std::string name;
};

typedef undirected_graph<Actor> Graph;
typedef graph_traits<Graph>::vertex_descriptor Vertex;
typedef graph_traits<Graph>::edge_descriptor Edge;

typedef property_map<Graph, string Actor::*>::type NameMap;

struct DetectionNode {
    Detection *Det; //The detection
    int DetectorIndex; //Forms the index to obtain the confidence & complementarity
    int DetectionIndex;

    DetectionNode(Detection* D, int index, int index2): DetectorIndex(index), DetectionIndex(index2)
    {
        Det = new Detection(D);
    }

    ~DetectionNode() {
        delete Det;
    }

    DetectionNode( const DetectionNode& D ) {
        Det = new Detection(D.Det);
        DetectorIndex = D.DetectorIndex;
        DetectionIndex = D.DetectionIndex;
    }

};

// To add the nodes belonging together in the same group
struct clique_adder
{
    clique_adder(std::vector<std::vector<int> >& Gs)
        : Groups(Gs)
    { }

    template <typename Clique, typename Graph>
    void clique(const Clique& c, const Graph& g)
    {
        // Iterate over the clique and print each vertex within it.
        typename Clique::const_iterator i, end = c.end();

        std::vector<int> Link;
        for(i = c.begin(); i != end; ++i) {
            int index;
            istringstream buffer(g[*i].name);
            buffer >> index;
            Link.push_back(index);
        }
        std::cout << std::endl;
        Groups.push_back(Link);
    }
    std::vector<std::vector<int> > &Groups;
};

class Combinator: public Detector {
private:
    std::vector<Detector*> Detectors;
    bool doCombinator;

    DetectionList applyCombinator(const cv::Mat &Frame) const;
    DetectionList applyPoolCombination(const cv::Mat &Frame) const;


// used to store all data about the combination -> confidence, complementarity, mean score, std score
    CombinationProperties CP;

public:

    std::string getName() const {
        return "Combinator";
    }

    Combinator();
    virtual ~Combinator();

    DetectionList applyDetector(const cv::Mat &Frame) const;
};

#endif /* COMBINATOR_H_ */
