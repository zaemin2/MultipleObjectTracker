//============================================================================
// Author      : F. De Smedt @ EAVISE
// Copyright   : This code is written for the publication of  "Open Framework for Combined Pedestrian detection". This code is for research and educational purposes only. For a different license, please contact the contributors directly. More information about the license can be fount in license.txt
//============================================================================



#ifndef EXTRA_H
#define EXTRA_H

#include <iostream>
#include <vector>
#include <string>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>

//#include "defines.h"



/*!
	This function returns the number of layers that should be created based on the image-sizes,the rescale-factor and the minimum size of the model that should be able to be evaluated on the image

	\author F. De Smedt
        \date 2014
*/
int giveNumberOfLayers(int imwidth,int imheight, float rescale,int minwidth,int minheight);

/*!
	This function can be used to smooth an image based on a certain ratio.

	\param r radius
	\param image image to smooth
	\return Smoothed image

*/
cv::Mat SmoothImage(int r, cv::Mat &image);


/*!
	Function to pad an image to allow detections on the borders of the image

	\param padding numberof pixels to add at each border
	\param img image to pad
	\return The padded image

*/
cv::Mat PadImage(int padding, cv::Mat &img);


cv::Scalar randomColor( cv::RNG& rng );
#endif // EXTRA_H
