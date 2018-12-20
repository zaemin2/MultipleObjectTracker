//============================================================================
// Author      : F. De Smedt @ EAVISE
// Copyright   : This code is written for the publication of  "Open Framework for Combined Pedestrian detection". This code is for research and educational purposes only. For a different license, please contact the contributors directly. More information about the license can be fount in license.txt
//============================================================================



#include <iostream>
#include "HasOverlap.h"

using namespace std;

/*!
	\file Implementation ofthe HasOverlap function

*/

/*! \brief calculate the overlap between Detection A and Detection B

	This function calculates the amount of overlap between two detections.

*/

float HasOverlap(const Detection *A, const Detection *B) {
    //find the common area between the two detections
    int xx1 = max(A->getX(),B->getX());
    int yy1 = max(A->getY(),B->getY());
    int xx2 = min(A->getX()+A->getWidth(), B->getX()+B->getWidth());
    int yy2 = min(A->getY()+A->getHeight(),B->getY()+B->getHeight());

    int w = xx2-xx1;
    int h = yy2-yy1;
    int commonArea = w*h;

    /*if there is a common area*/
    if(w>0 && h>0)
    {
        int AArea = A->getWidth()*A->getHeight();
        int BArea = B->getWidth()*B->getHeight();
        return min((((float)commonArea/(float)AArea)),(((float)commonArea/(float)BArea)));
    }
    else
    {
        // no overlap
        return 0;
    }

//should never be reached
    return 0.0;
}
