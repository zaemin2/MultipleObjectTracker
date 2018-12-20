//============================================================================
// Author      : F. De Smedt @ EAVISE
// Copyright   : This code is written for the publication of  "Open Framework for Combined Pedestrian detection". This code is for research and educational purposes only. For a different license, please contact the contributors directly. More information about the license can be fount in license.txt
//============================================================================

#include "Eigen/Dense"
#include "Eigen/Core"
#include "Eigen/Sparse"
#include <string>
#include <ostream>


#ifndef __CASCADEMODEL
#define __CASCADEMODEL


class Types {
public:
    typedef float Scalar;
    static const int featuredepth = 32;
    static const int pcafeaturedepth = 6;
    typedef Eigen::Array<Scalar, 32, 1> Cell;
    typedef Eigen::Array<Scalar, 6, 1> PCACell;
    typedef Eigen::Array<Scalar, 1,Eigen::Dynamic> Order;
    typedef Eigen::Array<Scalar, Eigen::Dynamic,1> T;
};

class BboxOld {
public:
    Eigen::Matrix<Types::Scalar,19,1> x1;
    Eigen::Matrix<Types::Scalar,19,1> y1;
    Eigen::Matrix<Types::Scalar,19,1> x2;
    Eigen::Matrix<Types::Scalar,19,1> y2;
};

class RootfilterCOld {
public:
    std::pair<int,int> size;
    Eigen::Array<Eigen::Matrix<Types::Scalar,Eigen::Dynamic,Eigen::Dynamic>,Types::featuredepth,1> w;
    Eigen::Array<Eigen::Matrix<Types::Scalar,Eigen::Dynamic,Eigen::Dynamic>,Types::pcafeaturedepth,1> wpca;
    int blocklabel;
};

class OffsetMatOld {
public:
    Types::Scalar w;
    int blocklabel;
};

class PartCOld {
public:
    int partindex;
    int defindex;
};

class ComponentCOld {
public:
    int rootindex;
    int offsetindex;
    Eigen::Matrix<PartCOld,1, Eigen::Dynamic, Eigen::RowMajor> Parts;
};

class PartfilterCOld {
public:
    Eigen::Array<Eigen::Matrix<Types::Scalar,Eigen::Dynamic,Eigen::Dynamic>,Types::featuredepth,1> w;
    Eigen::Array<Eigen::Matrix<Types::Scalar,Eigen::Dynamic,Eigen::Dynamic>,Types::pcafeaturedepth,1> wpca;
    int blocklabel;
};

class DefCOld {
public:
    Eigen::Matrix<Types::Scalar, 1,Eigen::Dynamic> w;
    int blocklabel;
    std::pair<int,int> anchor;
};

class CascadeCOld {
public:
    Types::Scalar thresh;
    Eigen::Matrix<Types::T,1,Eigen::Dynamic,Eigen::RowMajor> t;
    Eigen::Matrix<Types::Order,1,Eigen::Dynamic,Eigen::RowMajor> order;

};



class CascadeModelOld {
public:
    CascadeModelOld(std::string modelfile);
    friend std::ostream &  operator<<(std::ostream & os, const CascadeModelOld & model);
    std::string getName() {
        return this->ModelClass;
    }
    int getMaxSizeX() {
        return this->maxsize.second;
    }
    int getMaxSizeY() {
        return this->maxsize.first;
    }

    int getMinSizeX() {
        return this->minsize.second;
    }
    int getMinSizeY() {
        return this->minsize.first;
    }

    int getSbin() {
        return this->sbin;
    }
    int getInterval() {
        return this->interval;
    }
    int getNumcomponents() {
        return this->numcomponents;
    }
    Types::Scalar getThreshold() {
        return this->thresh;
    }
    void setThreshold(Types::Scalar thresh) {
        this->thresh = thresh;
    }

    Eigen::Matrix<BboxOld,Eigen::Dynamic,1,Eigen::ColMajor> bboxpred;
    Eigen::Matrix<RootfilterCOld,1, Eigen::Dynamic, Eigen::RowMajor> rootfilters;

    Eigen::Matrix<OffsetMatOld,1, Eigen::Dynamic, Eigen::RowMajor> offsets;
    Eigen::Matrix<ComponentCOld,1, Eigen::Dynamic, Eigen::RowMajor> components;
    Eigen::Matrix<PartfilterCOld,1, Eigen::Dynamic, Eigen::RowMajor> partfilters;
    Eigen::Matrix<DefCOld,1, Eigen::Dynamic, Eigen::RowMajor> defs;

    CascadeCOld cascade;
    Eigen::Matrix<Types::Scalar, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor> coeff;

//private:
    int sbin;
    Types::Scalar thresh;
    std::pair<int, int> maxsize;
    std::pair<int, int> minsize;
    int interval;
    int numblocks;
    std::string ModelClass;
    int numcomponents;

    std::string year;
    std::string note;

};

#endif
