#ifndef _H_DPMFUNC
#define _H_DPMFUNC

#include "CModel.h"

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "DPMPyramid.h"

class DPMPcaPyramid;

void clipboxes(int W, int H, vector<double> &dets, vector<double> &parts, int Width);
void bboxpred_get(vector<double> &bbox, vector<double> &parts,const CModel *M,vector<double> dets, vector<double> boxes,int W);
double *project(double *features, double* coeff, int *featsizes);
double *matmult(double *A,double *B,int colsA,int colsB, int rowsA,int rowsB);
int cascade(const CModel *model ,double*** rootscores, int numrootlocs, int padxN, int padyN, int s, int ***SizesC/*rootscoredims*/,vector<double> &coords,const DPMPyramid &DPyr,const DPMPcaPyramid &PCAPyr);

double **fconv(double* A, double** B, int start, int end, int WA,int HA, int DA,  Rootfilter* rootfilters, int **sizeC);
CModel LoadCascadeModel(double thresh, std::string filename);
double* features(int sbin/*mxsbin*/, int width, int height, int *sizesfeat, const cv::Mat &Scaled);

double *padarray(double *A, int padding[2], int sizes[3]);
int cascade_detect(const CModel *model,double thresh,vector<double> &dets,vector<double> &boxes, const DPMPyramid &DPyr);
#endif
