//============================================================================
// Author      : F. De Smedt @ EAVISE
// Copyright   : This code is written for the publication of  "Open Framework for Combined Pedestrian detection". This code is for research and educational purposes only. For a different license, please contact the contributors directly. More information about the license can be fount in license.txt
//============================================================================



/*featpyramid.cpp*/
#include <iostream>
//#include "Functions.h"
#include <cmath>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "NewCascadeModelOld.h"
#include "featpyramidOld.h"
#include "padarrayOld.h"

using namespace std;
using namespace cv;


void getpadding(NewCascadeModelOld *m, int *padx, int *pady) {
    *padx = ceil(m->maxsize[1]);
    *pady = ceil(m->maxsize[0]);
}

int min(int i[2],int n) {
    if(i[0]<i[1])
        return i[0];
    return i[1];
}

float* featurescpuOld(cv::Mat &im, int sbin, int width, int height, int *sizesfeat);


PyramidOld DownScale(cv::Mat &im, NewCascadeModelOld *Model, int level){

	PyramidOld A;

		int padx, pady;
	    getpadding(Model, &padx, &pady);
	    A.padx = padx;
	    A.pady = pady;


	    int sbin = Model->sbin;
	    int interval = Model->interval;
	    double sc = pow(2.0,(1.0/10));
	    int imsize[2] = {im.rows, im.cols};
	    int max_scale = 1;// + floor(log(min(imsize,2)/(5.0*sbin))/log(sc));

	    A.imsize[0] = imsize[0];
	    A.imsize[1] = imsize[1];

	    A.lenghtFeatures = 2;

	    int Wi,He,WN,HN;
	    int sizesfeat[3];

	    A.feat = (float**)malloc((max_scale+interval)*sizeof(float*));
	    A.featold = (float**)malloc((max_scale+interval)*sizeof(float*));
	    A.scales = (float*)malloc((max_scale+interval)*sizeof(float));
	    A.featsizes = (int**)malloc((max_scale+interval)*sizeof(int*));
	    A.featsizesold = (int**)malloc((max_scale+interval)*sizeof(int*));

	    for(int i=0; i<interval; i++) {
	        Wi = im.cols;
	        He = im.rows;
	        A.featsizes[i] = (int *)malloc(sizeof(int)*3);
	        A.featsizesold[i] = (int *)malloc(sizeof(int)*3);
	        A.featsizes[i+interval] = (int *)malloc(sizeof(int)*3);
	        A.featsizesold[i+interval] = (int *)malloc(sizeof(int)*3);

	        Wi = (int)im.cols;
	        He = (int)im.rows;
	        A.feat[i] = featurescpuOld(im,sbin,Wi,He,A.featsizes[i]);
	        A.scales[i] = 1.0/pow(sc,(level));

	        cv::Mat scaled;
	        Size sz = Size(cvRound(im.cols*(0.5/pow(sc,(i)))), im.rows*(0.5/pow(sc,(i))));
	        resize(im,scaled,sz);

	        Wi = (int)scaled.cols;
	        He = (int)scaled.rows;

	        A.feat[i+interval] =  featurescpuOld(scaled, sbin,Wi,He,A.featsizes[i+interval]);
	        A.scales[i+interval] = 0.5/pow(sc,(level));
	    }
	    int place = 31;
	    for(int i=0; i<max_scale+interval; i++) {
	        int pad[2] = {pady+1, padx+1};
	        A.feat[i] = padarray(A.feat[i], pad, A.featsizes[i]);
	        for(int W=0; W<=pady; W++) {
	            for(int Q=0; Q<A.featsizes[i][0]; Q++) {
	                A.feat[i][ place*A.featsizes[i][0]*A.featsizes[i][1] + Q * A.featsizes[i][1] + W ] = 1;
	            }
	        }
	        for(int W=A.featsizes[i][1]-pady-1; W<A.featsizes[i][1]; W++) {
	            for(int Q=0; Q<A.featsizes[i][0]; Q++) {
	                A.feat[i][ place*A.featsizes[i][0]*A.featsizes[i][1] + Q * A.featsizes[i][1] + W ] = 1;
	            }
	        }
	        for(int W=0; W<A.featsizes[i][1]; W++) {
	            for(int Q=0; Q<=padx; Q++) {
	                A.feat[i][ place*A.featsizes[i][0]*A.featsizes[i][1] + Q * A.featsizes[i][1] + W ] = 1;
	            }
	        }
	        for(int W=0; W<A.featsizes[i][1]; W++) {
	            for(int Q=A.featsizes[i][0]-padx-1; Q<A.featsizes[i][0]; Q++) {
	                A.feat[i][ place*A.featsizes[i][0]*A.featsizes[i][1] + Q * A.featsizes[i][1] + W ] = 1;
	            }
	        }
	    }
	    return A;
}


PyramidOld UpScale(cv::Mat &im, NewCascadeModelOld *Model, int level){

	PyramidOld A;

		int padx, pady;
	    getpadding(Model, &padx, &pady);
	    A.padx = padx;
	    A.pady = pady;


	    int sbin = Model->sbin;
	    int interval = Model->interval;
	    double sc = pow(2.0,(1.0/10));
	    int imsize[2] = {im.rows, im.cols};
	    int max_scale = 1;// + floor(log(min(imsize,2)/(5.0*sbin))/log(sc));

	    A.imsize[0] = imsize[0];
	    A.imsize[1] = imsize[1];

	    A.lenghtFeatures = 2;

	    int Wi,He,WN,HN;
	    int sizesfeat[3];

	    A.feat = (float**)malloc((max_scale+interval)*sizeof(float*));
	    A.featold = (float**)malloc((max_scale+interval)*sizeof(float*));
	    A.scales = (float*)malloc((max_scale+interval)*sizeof(float));
	    A.featsizes = (int**)malloc((max_scale+interval)*sizeof(int*));
	    A.featsizesold = (int**)malloc((max_scale+interval)*sizeof(int*));

	    for(int i=0; i<interval; i++) {
	        Wi = im.cols;
	        He = im.rows;
	        A.featsizes[i] = (int *)malloc(sizeof(int)*3);
	        A.featsizesold[i] = (int *)malloc(sizeof(int)*3);
	        A.featsizes[i+interval] = (int *)malloc(sizeof(int)*3);
	        A.featsizesold[i+interval] = (int *)malloc(sizeof(int)*3);

	        Wi = (int)im.cols;
	        He = (int)im.rows;

	        cv::Mat scaled;
	        Size sz = Size(cvRound(im.cols*(2.0/pow(sc,(i)))), im.rows*(2.0/pow(sc,(i))));
	        resize(im,scaled,sz);


	        A.feat[i] = featurescpuOld(scaled,sbin,scaled.cols, scaled.rows,A.featsizes[i]);
	        A.scales[i] = 2.0/pow(sc,(level));



	        Wi = (int)scaled.cols;
	        He = (int)scaled.rows;

	        A.feat[i+interval] =  featurescpuOld(im, sbin,Wi,He,A.featsizes[i+interval]);
	        A.scales[i+interval] = 1.0/pow(sc,(level));
	    }
	    int place = 31;
	    for(int i=0; i<max_scale+interval; i++) {
	        int pad[2] = {pady+1, padx+1};
	        A.feat[i] = padarray(A.feat[i], pad, A.featsizes[i]);
	        for(int W=0; W<=pady; W++) {
	            for(int Q=0; Q<A.featsizes[i][0]; Q++) {
	                A.feat[i][ place*A.featsizes[i][0]*A.featsizes[i][1] + Q * A.featsizes[i][1] + W ] = 1;
	            }
	        }
	        for(int W=A.featsizes[i][1]-pady-1; W<A.featsizes[i][1]; W++) {
	            for(int Q=0; Q<A.featsizes[i][0]; Q++) {
	                A.feat[i][ place*A.featsizes[i][0]*A.featsizes[i][1] + Q * A.featsizes[i][1] + W ] = 1;
	            }
	        }
	        for(int W=0; W<A.featsizes[i][1]; W++) {
	            for(int Q=0; Q<=padx; Q++) {
	                A.feat[i][ place*A.featsizes[i][0]*A.featsizes[i][1] + Q * A.featsizes[i][1] + W ] = 1;
	            }
	        }
	        for(int W=0; W<A.featsizes[i][1]; W++) {
	            for(int Q=A.featsizes[i][0]-padx-1; Q<A.featsizes[i][0]; Q++) {
	                A.feat[i][ place*A.featsizes[i][0]*A.featsizes[i][1] + Q * A.featsizes[i][1] + W ] = 1;
	            }
	        }
	    }
	    return A;
}


PyramidOld HalfSbin(cv::Mat &im, NewCascadeModelOld *Model, int level){

	PyramidOld A;
	    int padx, pady;
	    getpadding(Model, &padx, &pady);
	    A.padx = padx;
	    A.pady = pady;


	    int sbin = Model->sbin;
	    int interval = Model->interval;
	    double sc = pow(2.0,(1.0/10));
	    int imsize[2] = {im.rows, im.cols};
	    int max_scale = 1;

	    A.imsize[0] = imsize[0];
	    A.imsize[1] = imsize[1];

	    A.lenghtFeatures = 2;

	    int Wi,He,WN,HN;
	    int sizesfeat[3];

	    A.feat = (float**)malloc((max_scale+interval)*sizeof(float*));
	    A.featold = (float**)malloc((max_scale+interval)*sizeof(float*));
	    A.scales = (float*)malloc((max_scale+interval)*sizeof(float));
	    A.featsizes = (int**)malloc((max_scale+interval)*sizeof(int*));
	    A.featsizesold = (int**)malloc((max_scale+interval)*sizeof(int*));

	    for(int i=0; i<interval; i++) {
	        Wi = im.cols;
	        He = im.rows;
	        A.featsizes[i] = (int *)malloc(sizeof(int)*3);
	        A.featsizesold[i] = (int *)malloc(sizeof(int)*3);
	        A.featsizes[i+interval] = (int *)malloc(sizeof(int)*3);
	        A.featsizesold[i+interval] = (int *)malloc(sizeof(int)*3);

	        Wi = (int)im.cols;
	        He = (int)im.rows;
	        A.feat[i] = featurescpuOld(im,sbin/2,Wi,He,A.featsizes[i]);
	        A.scales[i] = 2.0/pow(sc,(level));
	        A.feat[i+interval] =  featurescpuOld(im, sbin,Wi,He,A.featsizes[i+interval]);
	        A.scales[i+interval] = 1.0/pow(sc,(level));
	    }
	    int place = 31;
	    for(int i=0; i<max_scale+interval; i++) {
	        int pad[2] = {pady+1, padx+1};
	        A.feat[i] = padarray(A.feat[i], pad, A.featsizes[i]);
	        for(int W=0; W<=pady; W++) {
	            for(int Q=0; Q<A.featsizes[i][0]; Q++) {
	                A.feat[i][ place*A.featsizes[i][0]*A.featsizes[i][1] + Q * A.featsizes[i][1] + W ] = 1;
	            }
	        }
	        for(int W=A.featsizes[i][1]-pady-1; W<A.featsizes[i][1]; W++) {
	            for(int Q=0; Q<A.featsizes[i][0]; Q++) {
	                A.feat[i][ place*A.featsizes[i][0]*A.featsizes[i][1] + Q * A.featsizes[i][1] + W ] = 1;
	            }
	        }
	        for(int W=0; W<A.featsizes[i][1]; W++) {
	            for(int Q=0; Q<=padx; Q++) {
	                A.feat[i][ place*A.featsizes[i][0]*A.featsizes[i][1] + Q * A.featsizes[i][1] + W ] = 1;
	            }
	        }
	        for(int W=0; W<A.featsizes[i][1]; W++) {
	            for(int Q=A.featsizes[i][0]-padx-1; Q<A.featsizes[i][0]; Q++) {
	                A.feat[i][ place*A.featsizes[i][0]*A.featsizes[i][1] + Q * A.featsizes[i][1] + W ] = 1;
	            }
	        }
	    }

	    return A;
}


PyramidOld featpyramidcpuOld(cv::Mat &im, NewCascadeModelOld *Model, int level, int method) {
    if(method == 1)
    	return HalfSbin(im,Model,level);
    else if(method == 2)
    	return DownScale(im,Model,level);
    else if (method == 3)
    	return UpScale(im,Model,level);
    else
    	std::cout << "PYRAMID TECHNIQUE DOES NOT EXIST!!" << std::endl;
    exit(1);
}



