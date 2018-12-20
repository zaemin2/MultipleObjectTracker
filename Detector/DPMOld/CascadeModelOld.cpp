//============================================================================
// Author      : F. De Smedt @ EAVISE
// Copyright   : This code is written for the publication of  "Open Framework for Combined Pedestrian detection". This code is for research and educational purposes only. For a different license, please contact the contributors directly. More information about the license can be fount in license.txt
//============================================================================

#include <iostream>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <utility>

#include "CascadeModelOld.h"

//using namespace std;




CascadeModelOld::CascadeModelOld(std::string modelfile) {

    std::string temporaryString;
    int T1, T2;
    int rows, columns,columns2;
//Open model file
    std::ifstream fin;
    fin.open(modelfile.c_str());
    if(!fin.good()) {
        std::cout << "Their is an error in opening the model file " << modelfile << std::endl;
    }
    else {
        //sbin
        fin >> temporaryString;
        fin >> this->sbin;

        //thresh
        fin >> temporaryString;
        fin >> this->thresh;

        //interval
        fin >> temporaryString;
        fin >> this->interval;

        //thresh
        fin >> temporaryString;
        fin >> this->numblocks;

        //ModelClass
        fin >> temporaryString;
        fin >> this->ModelClass;

        //numcomponents
        fin >> temporaryString;
        fin >> this->numcomponents;

        //year
        fin >> temporaryString;
        fin >> this->year;

        //note
        fin >> temporaryString;
        fin >> this->note;

        //maxsize
        fin >> temporaryString;
        fin >> T1 >> T2;
        this->maxsize = std::make_pair(T1,T2);

        //minsize
        fin >> temporaryString;
        fin >> T1 >> T2;
        this->minsize = std::make_pair(T1,T2);

        //coeff
        fin >> temporaryString;
        fin >> rows >> columns;
        this->coeff.resize(rows,columns);
        for(int y=0; y<rows; y++) {
            for(int x=0; x<columns; x++) {
                fin >> this->coeff(y,x);
            }
        }



        //bboxpred
        fin >> temporaryString;
        fin >> rows;
        this->bboxpred.resize(rows,1);

        for(int y=0; y<rows; y++) {
            //for each bboxpred structure
            //x1
            getline(fin,temporaryString); //clear last tab
            getline(fin,temporaryString);
            for(int i=0; i<19; i++) {
                fin >> this->bboxpred(y,0).x1(i,0);
            }

            //y1
            getline(fin,temporaryString); //clear last tab
            getline(fin,temporaryString);
            for(int i=0; i<19; i++) {
                fin >> this->bboxpred(y,0).y1(i,0);
            }

            //x2
            getline(fin,temporaryString); //clear last tab
            getline(fin,temporaryString);
            for(int i=0; i<19; i++) {
                fin >> this->bboxpred(y,0).x2(i,0);
            }
            //y2
            getline(fin,temporaryString); //clear last tab
            getline(fin,temporaryString);
            for(int i=0; i<19; i++) {
                fin >> this->bboxpred(y,0).y2(i,0);
            }

        }


        //rootfilters
        fin >> temporaryString;
        fin >> columns2;
        this->rootfilters.resize(1,columns2);

        for(int x=0; x<columns2; x++) {
            //w
            getline(fin,temporaryString); //clear last tab
            getline(fin,temporaryString); //comment

            fin >> rows >> columns; //read size w
            this->rootfilters(0,x).size = std::make_pair(rows,columns);

            getline(fin,temporaryString); // ending tab
            getline(fin,temporaryString); // comment line

            for(int i=0; i<32; i++) {
                this->rootfilters(0,x).w(i).resize(rows, columns);
                for(int y=0; y<rows; y++) {
                    for(int x2=0; x2<columns; x2++) {
                        fin >> this->rootfilters(0,x).w(i)(y,x2);
                    }
                }
            }


            //wpca
            getline(fin,temporaryString); //clear last tab
            getline(fin,temporaryString);


            for(int i=0; i<6; i++) {
                this->rootfilters(0,x).wpca(i).resize(rows, columns);
                for(int y=0; y<rows; y++) {
                    for(int x2=0; x2<columns; x2++) {

                        fin >> this->rootfilters(0,x).wpca(i)(y,x2);
                    }
                }
            }

            //blocklabel
            getline(fin,temporaryString); //clear last tab
            getline(fin,temporaryString); //comment
            fin >> this->rootfilters(0,x).blocklabel;

        }

        //offsets
        fin >> temporaryString;
        fin >> columns2;
        this->offsets.resize(1,columns2);
        for(int x=0; x<columns2; x++) {

            getline(fin,temporaryString); //remove newline
            getline(fin,temporaryString); // comment line

            fin >> this->offsets(0,x).w;


            getline(fin,temporaryString); //remove newline
            getline(fin,temporaryString); // comment line

            fin >> this->offsets(0,x).blocklabel;

        }

        //components
        fin >> temporaryString;
        fin >> columns2;
        this->components.resize(1,columns2);
        for(int x=0; x<columns2; x++) {
            //rootindex
            getline(fin,temporaryString); //remove newline
            getline(fin,temporaryString); // comment line
            fin >> this->components(0,x).rootindex;

            //offsetindex

            getline(fin,temporaryString); //remove newline
            getline(fin,temporaryString); // comment line

            fin >> this->components(0,x).offsetindex;

            //part
            int sizeParts;
            getline(fin,temporaryString); //remove newline
            getline(fin,temporaryString); // comment line
            fin >> sizeParts;
            this->components(0,x).Parts.resize(1,sizeParts);
            for(int p=0; p<sizeParts; p++) {
                getline(fin,temporaryString); //remove newline
                getline(fin,temporaryString); // comment line
                fin >> this->components(0,x).Parts(0,p).partindex;

                getline(fin,temporaryString); //remove newline
                getline(fin,temporaryString); // comment line
                fin >> this->components(0,x).Parts(0,p).defindex;
            }

        }

        //partfilters
        fin >> temporaryString;
        fin >> columns2;
        this->partfilters.resize(1,columns2);
        for(int x=0; x<columns2; x++) {
            //w
            getline(fin,temporaryString); //remove newline
            getline(fin,temporaryString); // comment line
            fin >> rows >> columns; //read size w

            for(int i=0; i<32; i++) {
                this->partfilters(0,x).w(i).resize(rows,columns);
                for(int y=0; y<rows; y++) {
                    for(int x2=0; x2<columns; x2++) {

                        fin >> this->partfilters(0,x).w(i)(y,x2);
                    }
                }
            }

            //wpca
            getline(fin,temporaryString); //remove newline
            getline(fin,temporaryString); // comment line
            fin >> rows >> columns; //read size w
            for(int i=0; i<6; i++) {
                this->partfilters(0,x).wpca(i).resize(rows,columns);
                for(int y=0; y<rows; y++) {
                    for(int x2=0; x2<columns; x2++) {

                        fin >> this->partfilters(0,x).wpca(i)(y,x2);
                    }
                }
            }

            //blocklabel
            getline(fin,temporaryString); //remove newline
            getline(fin,temporaryString); // comment line
            fin >> this->partfilters(0,x).blocklabel; //read size w
        }


        //defs
        fin >> temporaryString;
        fin >> columns2;
        this->defs.resize(1,columns2);
        for(int x=0; x<columns2; x++) {
            //w
            getline(fin,temporaryString); //remove newline
            getline(fin,temporaryString); // comment line
            fin >> columns;
            this->defs(0,x).w.resize(1,columns);
            for(int x2=0; x2<columns; x2++) {
                fin >> this->defs(0,x).w(0,x2);
            }
            //blocklabel
            getline(fin,temporaryString); //remove newline
            getline(fin,temporaryString); // comment line
            fin >> this->defs(0,x).blocklabel;
            //anchor
            getline(fin,temporaryString); //remove newline
            getline(fin,temporaryString); // comment line
            fin >> T1 >> T2;
            this->defs(0,x).anchor = std::make_pair(T1,T2);
        }

        //cascade
        fin >> temporaryString;
        fin >> columns2;
        //this->cascade.resize(1,columns2);
        for(int x=0; x<columns2; x++) {
            //order
            getline(fin,temporaryString); //remove newline
            getline(fin,temporaryString); // comment line
            fin >> columns;
            this->cascade.order.resize(1,columns);

            for(int x2=0; x2<columns; x2++) {
                int columns3;
                fin >> columns3;
                this->cascade.order(0,x2).resize(1,columns3);
                for(int x3=0; x3<columns3; x3++) {
                    fin >> this->cascade.order(0,x2)(x3);
                }
            }

            //t
            getline(fin,temporaryString); //remove newline
            getline(fin,temporaryString); // comment line
            fin >> columns;
            this->cascade.t.resize(1,columns);

            for(int x2=0; x2<columns; x2++) {
                int columns3;
                fin >> columns3;
                this->cascade.t(0,x2).resize(columns3,1);
                for(int x3=0; x3<columns3; x3++) {
                    fin >> this->cascade.t(0,x2)(x3);
                }
            }
            //thresh
            getline(fin,temporaryString); //remove newline
            getline(fin,temporaryString); // comment line
            fin >>  this->cascade.thresh;
        }


        fin.close();
    }
}



std::ostream &  operator<<(std::ostream & os, const CascadeModelOld & model)
{
    os << "dit is een test" << std::endl;

    //sbin
    os << "sbin: " << model.sbin << std::endl;

    //thresh
    os << "thresh: " << model.thresh << std::endl;

    //interval
    os<< "interval: " << model.interval << std::endl;

    //class
    os << "class: " << model.ModelClass << std::endl;

    //numcomponents
    //numcomponents

    os <<"numcomponents: " << model.numcomponents << std::endl;

    //year
    os << "year: " << model.year << std::endl;

    //note
    os << "note: " << model.note << std::endl;

    //maxsize
    os << "maxsize: " <<model.maxsize.first << "  " <<  model.maxsize.second << std::endl;

    //minsize
    os << "minsize: " <<model.minsize.first << "  " <<  model.minsize.second << std::endl;

    //coeff
    os << "coeff: size " << model.coeff.rows() << "  " << model.coeff.cols() << std::endl;
    for(int y=0; y<model.coeff.rows(); y++) {
        for(int x=0; x<model.coeff.cols(); x++) {
            os << model.coeff(y,x)<< "\t";
        }
        os << std::endl;
    }

    //bboxpred
    os << "bboxpred: size " << model.bboxpred.rows() << std::endl;
    for(int y=0; y<model.bboxpred.rows(); y++) {
        //for each bboxpred structure
        //x1
        os << "x1 from bboxpred " << y << ": " << std::endl;
        for(int i=0; i<19; i++) {
            os << model.bboxpred(y,0).x1(i,0) << '\t';
        }
        os << std::endl;
        //y1
        os << "y1 from bboxpred " << y << ": " << std::endl;
        for(int i=0; i<19; i++) {
            os << model.bboxpred(y,0).y1(i,0) << '\t';
        }
        os << std::endl;

        //x2
        os << "x2 from bboxpred " << y << ": " << std::endl;
        for(int i=0; i<19; i++) {
            os << model.bboxpred(y,0).x2(i,0) << '\t';
        }
        os << std::endl;

        //y2
        os << "y2 from bboxpred " << y << ": " << std::endl;
        for(int i=0; i<19; i++) {
            os << model.bboxpred(y,0).y2(i,0) << '\t';
        }
        os << std::endl;
    }
    //rootfilers

    os << "rootfilters: size " << model.rootfilters.cols() << std::endl;
    for(int x=0; x<model.rootfilters.cols(); x++) {
        os << "size rootfilter " << x << ": " << model.rootfilters(0,x).size.first << "  " << model.rootfilters(0,x).size.second << std::endl;
        os << "w: size " << model.rootfilters(0,x).w(0).rows() << " " << model.rootfilters(0,x).w(0).cols() << " " << 32 << std::endl;

        for(int i=0; i<32; i++) {
            for(int y=0; y<model.rootfilters(0,x).w.rows(); y++) {
                for(int x2=0; x2<model.rootfilters(0,x).w.cols(); x2++) {

                    os << model.rootfilters(0,x).w(y,x2)(i) << "\t";
                }
                os << std::endl;
            }
            os << std::endl;
        }
        //wpca
        os << "wpca: size " << model.rootfilters(0,x).wpca(0).rows() << " " << model.rootfilters(0,x).wpca(0).cols() << " " << 6 << std::endl;
        for(int i=0; i<6; i++) {
            for(int y=0; y<model.rootfilters(0,x).wpca.rows(); y++) {
                for(int x2=0; x2<model.rootfilters(0,x).wpca.cols(); x2++) {

                    os << model.rootfilters(0,x).wpca(y,x2)(i) << "\t";
                }
                os << std::endl;
            }
            os << std::endl;
        }
        //blocklabel
        os << "blocklabel: " << model.rootfilters(0,x).blocklabel << std::endl;

    }


    //offsets

    os << "offsets: size " << model.offsets.cols() << std::endl;


    for(int x=0; x<model.offsets.cols(); x++) {

        os << "w from offset " << x << ": " << model.offsets(0,x).w << std::endl;
        os << "blocklabel from offset " << x << ": " << model.offsets(0,x).blocklabel << std::endl;

    }


    //components
    os << "components: size " << model.components.cols() << std::endl;
    for(int x=0; x<model.components.cols(); x++) {
        //  for(int x=0;x<1;x++){
        //rootindex
        os << "rootindex from component " << x << ": " << model.components(0,x).rootindex <<  std::endl;

        //rootindex
        os << "offsetindex from component " << x << ": " << model.components(0,x).offsetindex <<  std::endl;

        //part

        os << "size of parts is : " << model.components(0,x).Parts.cols() << std::endl;

        for(int p=0; p<model.components(0,x).Parts.cols(); p++) {

            os << "Partindex from part " << p << ": " << model.components(0,x).Parts(0,p).partindex << std::endl;
            os << "Defindex from part " << p << ": " << model.components(0,x).Parts(0,p).defindex << std::endl;

        }

    }

    //partfilters
    os << "partfilters: size " << model.partfilters.cols() << std::endl;
    for(int x=0; x<model.partfilters.cols(); x++) {
        // for(int x=0;x<1;x++){
        //w

        os << "size of w from partfilter " << x << ": " << model.partfilters(0,x).w(0).rows() << "  " << model.partfilters(0,x).w(0).cols() << std::endl;

        for(int i=0; i<32; i++) {
            for(int y=0; y<model.partfilters(0,x).w.rows(); y++) {
                for(int x2=0; x2<model.partfilters(0,x).w.cols(); x2++) {
                    os << model.partfilters(0,x).w(i)(x2,y) << "\t";
                }
                os << std::endl;
            }
            os << std::endl;
        }


        //wpca
        os << "size of wpca from partfilter " << x << ": " << model.partfilters(0,x).wpca(0).rows() << "  " << model.partfilters(0,x).wpca(0).cols() << std::endl;

        for(int i=0; i<6; i++) {
            for(int y=0; y<model.partfilters(0,x).wpca.rows(); y++) {
                for(int x2=0; x2<model.partfilters(0,x).wpca.cols(); x2++) {
                    os << model.partfilters(0,x).wpca(i)(x2,y) << "\t";
                }
                os << std::endl;
            }
            os << std::endl;
        }

        //blocklabel

        os << "blocklabel from partfilter " << x << ": " <<model.partfilters(0,x).blocklabel << std::endl;
    }

    //defs
    os << "defs: size " << model.defs.cols() << std::endl;
    for(int x=0; x<model.defs.cols(); x++) {
        //for(int x=0;x<1;x++){
        //w
        os << "size of w from defs " << x << ": " << model.defs(0,x).w.cols() << std::endl;
        os << "w from defs " << x << ":" << std::endl;
        for(int x2=0; x2<model.defs(0,x).w.cols(); x2++) {
            os << model.defs(0,x).w(0,x2) << "\t";
        }
        os << std::endl;
        //blocklabel
        os << "blocklabel from defs " << x << ": " << model.defs(0,x).blocklabel << std::endl;
        //anchor
        os << "anchor from defs " << x << ": " << model.defs(0,x).anchor.first << " " << model.defs(0,x).anchor.second << std::endl;
    }


    //cascade
    //for(int x=0;x<1;x++){
    //order
    os << "size of orders from cascade "  << ": " << model.cascade.order.cols() << std::endl;

    for(int x2=0; x2<model.cascade.order.cols(); x2++) {
        os << "number of items in order " << x2 << " from cascade "  << ": " << model.cascade.order(0,x2).cols() << std::endl;

        for(int x3=0; x3<model.cascade.order(0,x2).cols(); x3++) {
            os << model.cascade.order(0,x2)(x3) << "\t";
        }
        os << std::endl;
    }

    //t
    os << "size of t from cascade " <<  ": " << model.cascade.t.cols() << std::endl;

    for(int x2=0; x2<model.cascade.t.cols(); x2++) {
        os << "number of items in t " << x2 << " from cascade " <<  ": " << model.cascade.t(0,x2).rows() << std::endl;

        for(int x3=0; x3<model.cascade.t(0,x2).rows(); x3++) {
            os << model.cascade.t(0,x2)(x3) << "\t";
        }
        os << std::endl;
    }
    //thresh
    os << "threshold from cascade " <<  ": " << model.cascade.thresh << std::endl;
    return os;
}



