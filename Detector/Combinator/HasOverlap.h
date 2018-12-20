//============================================================================
// Author      : F. De Smedt @ EAVISE
// Copyright   : This code is written for the publication of  "Open Framework for Combined Pedestrian detection". This code is for research and educational purposes only. For a different license, please contact the contributors directly. More information about the license can be fount in license.txt
//============================================================================



#ifndef __H_OVERLAP
#define __H_OVERLAP
#include <iostream>
#include "../Core/detection.h"

/*!
	This function calculates the overlap between two detections.
	\param A The first detection
	\param B The second detection
	\return the amount of overlap (decimal value between 0-1).
*/
float HasOverlap(const Detection *A, const Detection *B);
#endif
