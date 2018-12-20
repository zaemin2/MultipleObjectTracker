#ifndef _H_DETPROP
#define _H_DETPROP

#include <cstdlib>

class CombinationProperties {
public:

    CombinationProperties(std::string Dataset) {

        if(Dataset == "Caltech") {
            ConfidenceMap["HOG"] = 0.102;
            ConfidenceMap["DPM"] = 0.149;
            ConfidenceMap["ICF"] = 0.137;
            ConfidenceMap["ACF"] = 0.154;
            ConfidenceMap["SqrtICF"] = 0.171;


            ComplementarityMap["HOG"]["HOG"] = 0;
            ComplementarityMap["HOG"]["ICF"] = 0.377;
            ComplementarityMap["HOG"]["DPM"] = 0.393;
            ComplementarityMap["HOG"]["ACF"] = 0.400;
            ComplementarityMap["HOG"]["SqrtICF"] = 0.393;

            ComplementarityMap["ICF"]["HOG"] = 0.341;
            ComplementarityMap["ICF"]["ICF"] = 0;
            ComplementarityMap["ICF"]["DPM"] = 0.340;
            ComplementarityMap["ICF"]["ACF"] = 0.239;
            ComplementarityMap["ICF"]["SqrtICF"] = 0.240;

            ComplementarityMap["DPM"]["HOG"] = 0.331;
            ComplementarityMap["DPM"]["ICF"] = 0.315;
            ComplementarityMap["DPM"]["DPM"] = 0;
            ComplementarityMap["DPM"]["ACF"] = 0.323;
            ComplementarityMap["DPM"]["SqrtICF"] = 0.308;


            ComplementarityMap["ACF"]["HOG"] = 0.344;
            ComplementarityMap["ACF"]["ICF"] = 0.205;
            ComplementarityMap["ACF"]["DPM"] = 0.328;
            ComplementarityMap["ACF"]["ACF"] = 0;
            ComplementarityMap["ACF"]["SqrtICF"] = 0.177;


            ComplementarityMap["SqrtICF"]["HOG"] = 0.334;
            ComplementarityMap["SqrtICF"]["ICF"] = 0.211;
            ComplementarityMap["SqrtICF"]["DPM"] = 0.308;
            ComplementarityMap["SqrtICF"]["ACF"] = 0.168;
            ComplementarityMap["SqrtICF"]["SqrtICF"] = 0;

            MeanScore["HOG"] = 1.2905;
            StdScore["HOG"] = 0.4664;

            MeanScore["ICF"] = 21.5234;
            StdScore["ICF"] = 29.9966;

            MeanScore["DPM"] = -0.1175;
            StdScore["DPM"] = 0.3719;


            MeanScore["ACF"] = 29.3558;
            StdScore["ACF"] = 15.5432;

            MeanScore["SqrtICF"] = 52.6231;
            StdScore["SqrtICF"] = 17.8675;
        }
        else {
            std::cerr << "Unknown dataset" << std::endl;
            exit(1);
        }
    }

    float getConfidence(std::string detectorname) const {
        //return ConfidenceMap[detectorname];
        return static_cast<float>(ConfidenceMap.find(detectorname)->second);
    }


    float getMean(std::string detectorname) const {
        //return MeanScore[detectorname];
        return static_cast<float>(MeanScore.find(detectorname)->second);
    }


    float getStd(std::string detectorname) const {
        //return StdScore[detectorname];
        return static_cast<float>(StdScore.find(detectorname)->second);
    }


    // Obtain the complementarity values according to the number of detectors in this combination
    std::vector<float> getComplementarity(std::vector<std::string> Ds) const {
        // generate the average values based on the detectors involved
        std::vector<float> V =  GenerateAverageComp(Ds);
        return GeneralFormula(V);
    }


private:
    // Table to store the (pairwise) complementarities between detectors
    std::map<std::string, std::map<std::string, float> > ComplementarityMap;

    // Table to store the confidences of the detectors
    std::map<std::string, float> ConfidenceMap;

    std::map<std::string, float> MeanScore;
    std::map<std::string, float> StdScore;


// Generate the average complementarity for the detectors which have a detection on a certain location
// The argument is a vector of the detector names involved in this detection
    std::vector<float> GenerateAverageComp(std::vector<std::string> detectors) const {
        // allocate space for all the average values
        std::vector<float> Cs(detectors.size());
        // obtain the number of detectors
        int n = detectors.size();

        for(int d=0; d<detectors.size(); d++) {
            float sum=0;
            for(int d2=0; d2<detectors.size(); d2++) {
                float Val = static_cast<float>(	static_cast<std::map<std::string, float> >(ComplementarityMap.find(detectors[d])->second).find(detectors[d2])->second);

                sum += Val;
            }
            // make the average. Since each detector has a zero complementarity with itself, we use n-1
            Cs[d] = sum/(n-1);
        }

        return Cs;
    }


// Implementation of the general projection function for complementarity values. A pre-caching of all possible combinations may obtain a speed-up!!
    std::vector<float> GeneralFormula(std::vector<float> CompValues) const {
// Based on matlab implementation
        std::vector<float> Result(CompValues.size());
        int n = CompValues.size();

        for(int D=0; D<n; D++) {
            float Ci = CompValues[D];

//First term
            float result = static_cast<float>(1)/n;

// difference term
            float T=0;
            for(int d=0; d<n; d++) { // since Ci is in the first, we skip it by starting from 1
                if(d == D)
                    continue;
                float Cj = CompValues[d];
                T += Ci-Cj;
            }
            result += T/n;

// product term
            T=0;
            for(int d=0; d<n; d++) { // since Ci is in the first, we skip it by starting from 1
                if(d == D)
                    continue;
                float Cj = CompValues[d];
                T += Ci*Cj;
            }
            result += T/n;

//Correction term
            float Sum = 0;
            for(int d=0; d<n; d++)
                Sum+=CompValues[d];

            float S = (Sum-1)*(1-Ci);
            result += S/n;

//Second Prod term
            float prod=1;
            for(int d=0; d<n; d++) {
                float Cj = CompValues[d];
                prod*=(1-Cj);
            }
            result += prod/n;

            Result[D] = result;

        }

        return Result;
    }
};


#endif
